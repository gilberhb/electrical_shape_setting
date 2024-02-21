/*********************************************************
/* Note, this code is designed to run on the Arduino
*  Uno, with a 16MHz clock rate. (ATMega 328p) It won't
*  work if you switch to a different microprocessor or run
*  at a different base clock frequency.
*********************************************************/
const int CS = 4;              //Chip select pin for ADS1255 (Arduino pinout)
const int MOSFET = 8;          //pin which drives the fet (Arduino pinout)
const int HISTORY_SIZE = 100;  //number of data points to keep a history of
const double RDS_ON = 0.001;   //RDS_ON of MOSFET, Ohms
const double R_SENSE = 0.0005; //Value of current sense resistor, Ohms

//15,000 samples per second Datarate
//Note: Don't change this, as it affects
//the settling time in single-shot mode, which
//this code currently uses.
const byte DR = 0xE0;

//ADS 1255 Command definitions
const byte CMD_RREG = 0x10;           //Read register
const byte CMD_RDATA = 0x01;          //Read data
const byte CMD_WREG = 0x50;           //Write Register
const byte CMD_SELFCAL = 0xF0;        //Initiate self-calibration
const byte CMD_SYSOCAL = 0xF3;        //Initiate system offset calibration
const byte CMD_SYNC = 0xFC;           //Wait for wakeup to start conversion
const byte CMD_WAKEUP = 0xFF;         //start conversion
const byte CMD_STANDBY = 0xFD;        //stop data conversion and wait for sync/wakeup

//ADS 1255 Register Addresses
const byte STATUS_REG = 0x00;         //status register, nothing here of great importance
const byte MUX_REG = 0x01;            //mux register, controls the currently active analog channel
const byte ADCON_REG = 0x02;          //control register, used to turn off the clock output
const byte DRATE_REG = 0x03;          //datarate register, controls the ADC's digital filter (settling time)
const byte OFC0_REG = 0x05;           //offset calibration register

const byte CLOCKOFF = 0x00;           //data to put in ADCON to turn the clock off

volatile long ADCval;                 //place the interrupt handler puts the converted value
volatile bool waitForData;            //boolean to allow for waiting for the ISR to run

double val;
unsigned long loopStartTime = 0;

//inputs
double RESISTANCE_CRITICAL = 1.15;    //relative to initial resistance
long   RAMP_TIME_SECONDS = 180;        //time to ramp up (s)
long    SECONDS = 120;                  //time to stay at high resistance

int    LOOP_OFF_TIME = 20000; //milliseconds
double R0 = -1.0;
double I_Max = -100;
double V_Max = -100;
double G_current = 40; //A/V
double E_joules = 0;
double Energy_Target = 0;
long loop_counter = 0;
int loop_off_counter = 0;
const int LOOP_STATE_ON = 1;
const int LOOP_STATE_OFF = 0;
int loopState = LOOP_STATE_OFF;
double R0_meas = -1.0;

inline bool IsDone(long nLoops)
{
  if (nLoops > RAMP_TIME_SECONDS*1000 + SECONDS*1000)
    return true;
  //if(nLoops >= 1)
    //return true;
  else
    return false;
}

inline double TargetResistance(long timeMillis)
{
  if (timeMillis < RAMP_TIME_SECONDS*1000) {
    double R = 1.0 + (double)timeMillis/((double)RAMP_TIME_SECONDS*1000.0) * (RESISTANCE_CRITICAL-1.0);
    return R;
  }
  else
    return RESISTANCE_CRITICAL; 
}

inline void spinWait(int nNops)
{
  //1 clock is equal to 1/8e6 seconds 
  for (int i = 0; i < nNops; ++i) 
    asm("nop");
  return;
}

inline char SPI_MasterTransmit(char cData)
{
  SPDR = cData;
  while(!(SPSR & _BV(SPIF))) {
    ;
  } 
  return SPDR;
}

void Sync()
{
  digitalWrite(CS,LOW);
  SPI_MasterTransmit(CMD_SYNC);
  digitalWrite(CS,HIGH);
}

void Wakeup()
{
  digitalWrite(CS,LOW);
  SPI_MasterTransmit(CMD_WAKEUP); //WAKEUP command
  digitalWrite(CS,HIGH); 
}

void SetClockOff()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(5);
  SPI_MasterTransmit(CMD_WREG | ADCON_REG);
  SPI_MasterTransmit(0x00); //only one register
  delayMicroseconds(7);
  SPI_MasterTransmit(CLOCKOFF);
  delayMicroseconds(5);
  digitalWrite(CS,HIGH);
}

void SetDataRate()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(5);
  SPI_MasterTransmit(CMD_WREG | DRATE_REG);
  SPI_MasterTransmit(0x00); //only one register
  delayMicroseconds(7);
  SPI_MasterTransmit(DR);
  delayMicroseconds(5);
  digitalWrite(CS,HIGH);
}

void SetBufferOff()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(5);
  SPI_MasterTransmit(CMD_WREG | STATUS_REG);
  SPI_MasterTransmit(0x00); //only one register
  delayMicroseconds(7);
  SPI_MasterTransmit(0x00); //turn on the buffer
  delayMicroseconds(5);
  digitalWrite(CS,HIGH);
}

void SetBufferOn()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(5);
  SPI_MasterTransmit(CMD_WREG | STATUS_REG);
  SPI_MasterTransmit(0x00); //only one register
  delayMicroseconds(7);
  SPI_MasterTransmit(0x02); //turn on the buffer
  delayMicroseconds(5);
  digitalWrite(CS,HIGH);
}

void SelfCalibrate()
{
  //long calibrations[10] = {0,0,0,0,0,0,0,0,0,0};
  long average = 0;
  for (int i = 0; i < 50; ++i) {
    digitalWrite(CS,LOW);
    delayMicroseconds(5);
    SPI_MasterTransmit(CMD_SELFCAL); //Calibrate
    delayMicroseconds(5);
    digitalWrite(CS,HIGH);
    delay(50);
    average += ReadOffsetCalibration();
  }
  average /= 50;

  WriteOffsetCalibration(average);
}

void SystemCalibrate()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(5);
  SPI_MasterTransmit(CMD_SYSOCAL); //Calibrate
  delayMicroseconds(5);
  digitalWrite(CS,HIGH);
  delay(1000);
}

char ReadStatusReg()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(5);
  SPI_MasterTransmit(CMD_RREG | STATUS_REG);
  SPI_MasterTransmit(0x00); //read only one register
  delayMicroseconds(7);     //delay
  char reg = SPI_MasterTransmit(0x00);  //transmit data back
  //reg = SPI_MasterTransmit(0x00); 
  //reg = SPI_MasterTransmit(0x00); 
  digitalWrite(CS,HIGH);
  return reg;
}

char ReadMuxReg()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(5);
  SPI_MasterTransmit(CMD_RREG | MUX_REG);
  SPI_MasterTransmit(0x00);
  delayMicroseconds(7);
  char reg = SPI_MasterTransmit(0x00); 
  //reg = SPI_MasterTransmit(0x00); 
  //reg = SPI_MasterTransmit(0x00); 
  digitalWrite(CS,HIGH);
  return reg;
}

void SetChannelA()
{
  SPI_MasterTransmit(CMD_WREG | MUX_REG);
  SPI_MasterTransmit(0x00); //1 register
  SPI_MasterTransmit(0x0F); //sets channel A, single ended mode
}

void SetChannelB()
{
  SPI_MasterTransmit(CMD_WREG | MUX_REG);
  SPI_MasterTransmit(0x00); //1 register
  SPI_MasterTransmit(0x1F); //sets channel B, single ended mode
}

void WaitForGo()
{
  Serial.println("Please press enter to proceed: \n");
  while (!Serial.available()) {} //spin until we have data
  Serial.println("Starting up...");
}

long ReadADC()
{
  digitalWrite(CS,LOW);
  SPI_MasterTransmit(0xFF); //WAKEUP
  digitalWrite(CS,HIGH);
  while (digitalRead(3)==HIGH) {;} //wait for DRDY
  digitalWrite(CS,LOW);
  SPI_MasterTransmit(CMD_RDATA);
  delayMicroseconds(10);
  char b1 = SPI_MasterTransmit(0x00); //most-significant byte
  char b2 = SPI_MasterTransmit(0x00);
  char b3 = SPI_MasterTransmit(0x00); //least-significant byte
  digitalWrite(CS,HIGH);
  //Arduino is Little-Endian, so make sure the long is shifted the right way!
  long result = (((long)b3) & 0x000000FF)  | ((((long)b2) << 8) & 0x0000FF00) | (((long)b1 << 16) & 0xFFFF0000);
  digitalWrite(CS,LOW);
  SPI_MasterTransmit(0xFD);
  digitalWrite(CS,HIGH);
  return result;
}

void PrintBatteryVoltage()
{
  //PORTD &= ~_BV(2); //set pin 0 low
  PORTD &= ~_BV(4); //set pin 4 low
  SetChannelA();
  SPI_MasterTransmit(CMD_SYNC);
  SPI_MasterTransmit(CMD_WAKEUP); //start the conversion
  PORTD |= _BV(4);  //set pin 4 high
  waitForData = true;
  while (waitForData) {;} //wait until we have the data (ISR sets waitForData false)
  //convert using 5.0 volts as the reference full scale
  double voltage1 = ((double)(ADCval))/(8388607.0)*5.00*3.0;
  Serial.print("Battery Voltage: ");
  Serial.print(voltage1,3);
  Serial.print("\n");
}

void PrintCurrentReading()
{
  //PORTD &= ~_BV(2); //set pin 0 low
  PORTD &= ~_BV(4); //set pin 4 low
  SetChannelB();
  SPI_MasterTransmit(CMD_SYNC);
  SPI_MasterTransmit(CMD_WAKEUP); //start the conversion
  PORTD |= _BV(4);  //set pin 4 high
  waitForData = true;
  while (waitForData) {;} //wait until we have the data (ISR sets waitForData false)
  //convert using 5.0 volts as the reference full scale
  double voltage1 = ((double)(ADCval))/(8388607.0)*5.00*G_current;
  Serial.print("Current Measurement: ");
  Serial.print(voltage1,3);
  Serial.print("\n");
}

long ReadOffsetCalibration()
{
  digitalWrite(CS,LOW);
  SPI_MasterTransmit(CMD_RREG | OFC0_REG);
  SPI_MasterTransmit(0x02);  //read three registers
  delayMicroseconds(10);
  char b3 = SPI_MasterTransmit(0x00); //least-significant byte
  char b2 = SPI_MasterTransmit(0x00);
  char b1 = SPI_MasterTransmit(0x00); //most-significant byte
  digitalWrite(CS,HIGH);
  //Arduino is Little-Endian, so make sure the long is shifted the right way!
  long result = (((long)b3) & 0x000000FF)  | ((((long)b2) << 8) & 0x0000FF00) | (((long)b1 << 16) & 0xFFFF0000);
  return result;  
}

void WriteOffsetCalibration(long cal)
{
  byte b1 = (byte)((cal & 0xFFFF0000) >> 16);
  byte b2 = (byte)((cal & 0x0000FF00) >> 8);
  byte b3 = (byte)((cal & 0x000000FF));
  digitalWrite(CS,LOW);
  SPI_MasterTransmit(CMD_WREG | OFC0_REG);
  SPI_MasterTransmit(0x02);  //read three registers
  delayMicroseconds(10);
  SPI_MasterTransmit(b3); //least-significant byte
  SPI_MasterTransmit(b2);
  SPI_MasterTransmit(b1); //most-significant byte
  digitalWrite(CS,HIGH);
  //Arduino is Little-Endian, so make sure the long is shifted the right way!
}

float GetResistanceFromSerial()
{
  Serial.print("Target Resistance: ");
  float target = 0;
  while ((target = Serial.parseFloat()) == 0) {}
  return target;
}

float GetR0FromSerial()
{
  Serial.print("Room temp resistance (-1 to read at startup): ");
  float R = 0;
  while ((R = Serial.parseFloat()) == 0) {}
  return R;
}

bool GetOk()
{
  char yn = 0;
  //force the user to input a 'y' or a 'n'
  while (Serial.readBytes(&yn,1) == 0 || (yn != 'y' && yn != 'n') ) {}
  //flush the serial buffer
  delay(1000);
  while (Serial.available()) { Serial.read(); }
  if (yn == 'y') return true;
  else return false;
}

float GetR0FromUser()
{
  float R;
  R = GetR0FromSerial();
  while ((R < 0.05 || R > 2) && R != -1)
  {
    Serial.println("Room temp resistance must be in the range [0.05,2] or equal to -1"); 
    R = GetR0FromSerial();
  }
  
  if (R > 0) {
    Serial.print("You have selected the room temperature resistance of ");
    Serial.print(R,6);
    Serial.print("\n");
    Serial.print("Is this what you selected? [y/n]: ");
  } else if (R == -1)
  {
    Serial.print("You have selected to read the room temp resistance at startup.");
    Serial.print("Is this what you selected? [y/n]: ");
  }

  if (GetOk())
  {
    return R;
  } else {
    return GetR0FromUser(); //recurse
  }
}

float GetCriticalResistance()
{
  float target;
  target = GetResistanceFromSerial();
  while (target < 1.01 || target > 1.24)
  {
    Serial.println("Target resistance must be in the range [1.01,1.24]"); 
    target = GetResistanceFromSerial();
  }
  
  Serial.print("You have selected the target resistance of ");
  Serial.print(target,6);
  Serial.print("\n");
  Serial.print("Is this what you selected? [y/n]: ");
  if (GetOk())
  {
    return target;
  } else {
    return GetCriticalResistance(); //recurse
  }
}

void setup()
{
  MCUSR ^= MCUSR;                //clear all status registers
  WDTCSR &= ~(1 << WDE);         //turn off watchdog reset
  pinMode(MOSFET,OUTPUT);        //set the FET driver pin as output
  digitalWrite(MOSFET,LOW);      //make sure that the FET is off when we are waiting for user input
  Serial.begin(115200);
  
  SPCR = (1<<SPE)|(1<<MSTR)|(1 << SPR0);//set spi base clock speed and turn on SPI master mode
  SPSR = (1 << SPI2X); //multiply spi clock by 2 (for a resulting 2 MHz SPI clock rate)
  SPCR |= _BV(CPHA);   //set the data polarity for SPI
  //SPCR |= _BV(CPOL);
  
  pinMode(2,OUTPUT);             //This pin used for debugging purposes
  pinMode(CS,OUTPUT);            //Set chip select to output
  pinMode(MOSI,OUTPUT);          //set SPI Mosi to output
  pinMode(MISO,INPUT);           //set SPI Miso to input
  pinMode(SCK,OUTPUT);           //set SPI clock to output
  pinMode(SS,OUTPUT);            //set the Arduino's slave select pin to output to avoid surprises
  pinMode(3,INPUT);              //set pin 3 (ADS1255 DRDY) to input
  digitalWrite(SCK,LOW);         //serial clock low
  digitalWrite(CS,HIGH);         //chip select high
  digitalWrite(SS,HIGH);         //make sure the Arduino doesn't get accidentally reset to SPI slave
  delayMicroseconds(1000);
  //Wakeup();                      //wakeup the ADC
  digitalWrite(CS,LOW);
  SPI_MasterTransmit(CMD_STANDBY);   //tell the ADC to standby and wait for further instructions
  digitalWrite(CS,HIGH);
  delayMicroseconds(1000);
  SetClockOff();                 //turn off the ADC clock output (don't need it)
  delayMicroseconds(1000);
  SetDataRate();                 //set the ADC digital filter to 15 kHz
  //delayMicroseconds(1000);
  delayMicroseconds(1000);
  SetBufferOff();
  
  delay(50);
  Serial.println("Calibrating ADC...");
  SelfCalibrate();               //calibrate the ADC
  long offset_cal = ReadOffsetCalibration();
  //delay(100);
  //SystemCalibrate();
  //WriteOffsetCalibration(); //use the self-calibration routine
  delayMicroseconds(1000);
  
  Serial.println("ADC Offset Calibration: ");
  Serial.print(offset_cal, DEC);
  Serial.print(" counts\n");
  
  //ZeroHistoryArrays();
  
  EICRA |= 0b00001000;        //Set up interrupt on pin 3 to execute on falling signal
  EIMSK |= 0x02;              //enable interrupt on Arduino pin 3 (INT1)
  
  //PrintBatteryVoltage();
  //PrintCurrentReading();
  R0 = GetR0FromUser();
  RESISTANCE_CRITICAL = GetCriticalResistance();
  //Energy_Target = GetEnergyTarget(); //Get the energy target  
  WaitForGo();

  sei();                      //enable global interrupts
  
  //Enable the watchdog!
  WDTCSR |= (1 << WDE); //system reset mode on watchdog timer!
  digitalWrite(MOSFET, HIGH); //turn on the FET, and head into the loop
  loopState = LOOP_STATE_ON;
}

void Finish()
{
  cli();                    //disable interrupts
  MCUSR &= ~(1<<WDRF);       //clear watchdog reset flag
  WDTCSR |= (1 << WDCE) | (1 << WDE);    //enable watchdog changes
  WDTCSR = 0x00;            //turn off watchdog

  digitalWrite(MOSFET,LOW); //turn off the mosfet

  PORTD |= _BV(4);
  SPI_MasterTransmit(CMD_STANDBY); //turn off the ADC
  PORTD &= ~_BV(4);
  sei();
  Serial.print("Initial Resistance (measured): ");
  Serial.print(R0_meas,8);
  Serial.print("\n");
  
  Serial.print("Maximum Current: ");
  Serial.print(I_Max,8);
  Serial.print("\n");
  
  Serial.print("Maximum Voltage: ");
  Serial.print(V_Max,8);
  Serial.print("\n");
  Serial.println("Finished.");
  
  Serial.print("Metered Energy: ");
  Serial.print(E_joules,3);
  Serial.print(" J\n");
  //Serial.println("Voltage Waveform");
  //for (int i = 0; i < HISTORY_SIZE; ++i)
  //  Serial.println(Voltage_History[i],5);
  //Serial.println("Current Waveform");
  //for (int i = 0; i < HISTORY_SIZE; ++i)
  //  Serial.println(Current_History[i],5);
  //enable the onboard watchdog timer to reset the board
  
  delay(1000); //wait for a second to make sure the transmission completes
  WDTCSR |= (1 << WDE);  //let watchdog reset the board
  while (1) {;}             //kill the arduino 
}

void loopOn()
{
  digitalWrite(MOSFET, HIGH);
  PORTD &= ~_BV(4); //set pin 4 low
  SetChannelA();
  SPI_MasterTransmit(CMD_SYNC);
  SPI_MasterTransmit(CMD_WAKEUP); //start the conversion
  PORTD |= _BV(4);  //set pin 4 high
  waitForData = true;
  while (waitForData) {;} //wait until we have the data (ISR sets waitForData false)
  //convert using 5.0 volts as the reference full scale
  double voltage1 = ((double)(ADCval))/(8388607.0)*5.00;

  PORTD &= ~_BV(4); //set pin 4 low
  SetChannelB();
  SPI_MasterTransmit(CMD_SYNC);
  SPI_MasterTransmit(CMD_WAKEUP); //start the conversion
  PORTD |= _BV(4);  //set pin 4 high
  waitForData = true;
  while (waitForData) {;} //wait until we have the data
  //convert using 5.0 volts as the reference full scale
  double voltage2 = ((double)(ADCval))/(8388607.0)*5.00;
 /*********************DONE GETTING DATA FROM THE ADC************************/  
  //long result = ReadADC();
  double battVoltage = voltage1*3.0; //convert the measured voltage to the battery voltage
  double current = voltage2*G_current; //gain of INA139 chip based on 100k resistor load
  //compute the power, subtracting off approximate losses due to mosfet RDS_ON
  // and the Sense Resistor.  The cabling is assumed to have negligible resistance.
  double power = battVoltage*current; 
  
  if (R0_meas < 0) {
    R0_meas = battVoltage/current;
  }

  if (R0 == -1)
  {
    R0 = R0_meas;
  }
  
  if (R0 > 0) {
    if (battVoltage/current/R0 > TargetResistance(loop_counter)) {
      loopState = LOOP_STATE_OFF;
      digitalWrite(MOSFET, LOW); 
    }
  }
  E_joules += power*0.001; //Euler integrator -- accumulated energy
                           //is approximated as the measured applied power times
                           // the amount of time that has passed since the last
                           // computation, which is approximately 0.001 seconds
  if (battVoltage > V_Max) V_Max = battVoltage; //keep track of the largest voltage we've seen
  if (current > I_Max) I_Max = current;  //keep track of the largest current we've seen
}

void loopOff()
{
  loop_off_counter++;
  if (loop_off_counter > LOOP_OFF_TIME) { //50 ms off time
    loop_off_counter = 0;
    loopState = LOOP_STATE_ON;
  }
}

void loop()
{
  /***********************GET DATA FROM THE ADC*********************/
  /* According to the oscilloscope, this section of code requires
   * approximately 600 us to run, leaving about 400us of time after
   * this loop before we need to start again in a 1kHz loop        */
   /****************************************************************/
  loopStartTime = micros();
  PORTD &= ~_BV(2); //set pin 3 low
 
  asm("WDR;");      //kick the watchdog
  
  if (loopState == LOOP_STATE_OFF) {
    loopOff();
  } else {
    loopOn();
  }

  PORTD |= _BV(2);         //set pin 3 high
  if (IsDone(loop_counter)) {
    Finish();
  }
  //Voltage_History[(loop_counter/10) % HISTORY_SIZE] = battVoltage;
  //Current_History[(loop_counter/10) % HISTORY_SIZE] = current;
  loop_counter++;
  //
  // By the time the uC reaches this point, it will have used about
  // 680 us of the loop time, which is 1000 us.  If anything else needs
  // to be added, it better be quick.
  //
  while ((micros()-loopStartTime) < 1000) { //wait until the next loop should begin
    ; 
  }
  
}

//This ISR pulls the data off
//the ADS1255 when the chip pulls DRDY
//low to signal that data is available
ISR(INT1_vect)
{
  PORTD &= ~_BV(4);                        //pull chip select low

  SPI_MasterTransmit(CMD_RDATA);
  //This block of 84 nops works out to a 6 us delay
  delayMicroseconds(6);   //measured a 6.7 us delay between the last falling sclk and the
                          //next rising sclk of the next transmission
  char b1 = SPI_MasterTransmit(0x00);      //read most-significant byte
  char b2 = SPI_MasterTransmit(0x00);      //read mid-byte
  char b3 = SPI_MasterTransmit(0x00);      //read least-significant byte
  SPI_MasterTransmit(CMD_STANDBY);         //tell the ADC to standby before it floods us with more interrupts
  PORTD |= _BV(4);                         //pull chip select high
  
  //Store the result in ADCval as a signed long integer (note that the sign bit is taken from the MSB)
  //the cast is to long, rather than unsigned long, because we want to preserve the sign that the ADC
  //gives us, and don't want to mess with the two's complement format that the number is already in
  ADCval = (((long)b3) & 0x000000FF)  | ((((long)b2) << 8) & 0x0000FF00) | (((long)b1 << 16) & 0xFFFF0000);
  waitForData = false; //we can stop waiting for this ISR to run when we get back to loop()
}

//This ISR handles the overflows of Timer 2, which is
//used as a watchdog to make sure that if the ADC stops communicating
//and the code stalls whil waiting, this code will be able to stop
//the system from creating a potentially dangerous situation where
//the mosfet is left on.
//ISR(TIMER2_OVF_vect)
//{
//  watchdog++;
//  if (watchdog >= 5) {      //a value here is approximately the number of milliseconds
//                            //that have passed since the watchdog was last reset
//    digitalWrite(MOSFET,LOW); //turn off!
//    digitalWrite(0,HIGH);     //signal a problem 
//    //Enter the infinite loop to die
//    while(true) {;} 
//  }
//}

