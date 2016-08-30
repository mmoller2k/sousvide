
//-------------------------------------------------------------------
//
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//
// 2-Button UI Update and start delay timer
// by Michael Moller
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the Adafruit RGB/LCD Shield
//#include <Wire.h>
//#include <Adafruit_MCP23017.h>
//#include <Adafruit_RGBLCDShield.h>
#include <LiquidCrystal.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************
#if 0 //new pinout
#define RelayPin 6
#define ONE_WIRE_BUS 0
#define pin_dRS A0
#define pin_dE A1
#define pin_d4 A2
#define pin_d5 A4
#define pin_d6 A3
#define pin_d7 A5

#define pin_Up 3
#define pin_Down 4

#else //old pinout
// Output Relay
#define RelayPin A5

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 4
//#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 3

/* micro
#define pin_dRS 2
#define pin_dE 3
#define pin_d4 4
#define pin_d5 5
#define pin_d6 6
#define pin_d7 7
*/

#define pin_dRS 8
#define pin_dE 9
#define pin_d4 10
#define pin_d5 11
#define pin_d6 12
#define pin_d7 13

#define pin_Up A0
#define pin_Down A1 
//#define WITH_SERIAL
#endif

#define BUTTON_UP 1
#define BUTTON_DOWN 2
#define BUTTON_LEFT 4
#define BUTTON_RIGHT 8
#define BUTTON_SELECT 16

//#define pin_Left 14
//#define pin_Right 15
//#define pin_Select A0

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
uint8_t ModeNo=0;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;
boolean relay_flag=LOW;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

//Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
LiquidCrystal lcd=LiquidCrystal(pin_dRS,pin_dE,pin_d4,pin_d5,pin_d6,pin_d7);
// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define BUTTON_SHIFT BUTTON_SELECT
#define KEY_SLOW 350
#define KEY_FAST 100

unsigned long lastInput = 0; // last button press

byte degree[8] = // define the degree symbol 
{ 
 B00110, 
 B01001, 
 B01001, 
 B00110, 
 B00000,
 B00000, 
 B00000, 
 B00000 
}; 

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;
unsigned Twait=0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_W, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

enum tunemodeState { TEMP=0, MODE };
tunemodeState tuneMode = TEMP;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
#ifdef WITH_SERIAL  
   //Serial.begin(9600);
#endif
   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start

   // Set up Ground & Power for the sensor from GPIO pins

   // pinMode(ONE_WIRE_GND, OUTPUT);
   //digitalWrite(ONE_WIRE_GND, LOW);

   //pinMode(ONE_WIRE_PWR, OUTPUT);
   //digitalWrite(ONE_WIRE_PWR, HIGH);

   pinMode(pin_Up, INPUT_PULLUP);
   pinMode(pin_Down, INPUT_PULLUP);
   //pinMode(pin_Left, INPUT_PULLUP);
   //pinMode(pin_Right, INPUT_PULLUP);
   //pinMode(pin_Select, INPUT_PULLUP);

   // Initialize LCD DiSplay 

   lcd.begin(8, 2);
   lcd.createChar(1, degree); // create degree symbol from the binary
   lcd.clear();
   
   //lcd.setBacklight(VIOLET);
   lcd.print(F("Adafruit"));
   lcd.setCursor(0, 1);
   lcd.print(F("SousVide"));

   // Start up the DS18B20 One Wire Temperature Sensor

   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sens Err"));
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);

   delay(3000);  // Splash screen

   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);
   //lcd.clear();

#if 1
  // UNO: Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
#else
  // Micro: Run timer1 interrupt every 30 ms 
  TCCR1A = 0;
  TCCR1B = 0<<CS32 | 1<<CS31 | 0<<CS30;

  //Timer1 Overflow Interrupt Enable
  TIMSK1 |= 1<<TOIE1;
#endif
 
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

char * float2s(float f)
{
 return float2s(f, 2);
}

char * float2s(float f, unsigned int digits)
{
 int index = 0;
 static char s[16];                    // buffer to build string representation
 // handle sign
 if (f < 0.0)
 {
   s[index++] = '-';
   f = -f;
 }
 // handle infinite values
 if (isinf(f))
 {
   strcpy(&s[index], "INF");
   return s;
 }
 // handle Not a Number
 if (isnan(f))
 {
   strcpy(&s[index], "NaN");
   return s;
 }

 if(digits>0){
   long multiplier = pow(10, digits);
   long whole = long(f);                     // single digit
   long part = long((f-whole)*multiplier+0.5);   // # digits
   char format[16];
   sprintf(format, "%%ld.%%0%dld", digits);
   sprintf(&s[index], format, whole, part);
 }
 else{
  sprintf(&s[index], "%ld", (long)(f+0.5)); 
 }
 return s;
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   uint8_t buttons = 0;
   // wait for button release before changing state
   do{
    buttons=ReadButtons();
    if(buttons != 0){
      delay(KEY_SLOW);
    }
    if(((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN)) && millis()-lastInput > 2000){
      if (abs(Input - Setpoint) < 0.5)  // Should be at steady-state
      {
         StartAutoTune();
      }
      else{
        lcd.clear();
        lcd.print(" Not");
        lcd.setCursor(0, 1);
        lcd.print("Stable.");
      }
      opState=RUN;
      while(ReadButtons())delay(KEY_FAST);
    }
   }while((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN));

   switch (opState)
   {
   case OFF:
      Off();
      break;
   case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
   case TUNE_W:
      TuneWait();
      break;
   case TUNE_P:
      TuneP();
      break;
   case TUNE_I:
      TuneI();
      break;
   case TUNE_D:
      TuneD();
      break;
   }
}

// **************
// Initial State 
// **************
void Off()
{
   myPID.SetMode(MANUAL);
   //lcd.setBacklight(0);
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.clear();
   //lcd.print(F("Adafruit"));
   //lcd.setCursor(0, 1);
   //lcd.print(F("SousVide"));
   uint8_t buttons = 0;
/*   
   while(!(buttons & (BUTTON_RIGHT)))
   {
      buttons = ReadButtons();
   }
*/   
   delay(500);
   // Prepare to transition to the RUN state
   sensors.requestTemperatures(); // Start an asynchronous temperature reading

   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

void showSetpoint(void)
{
  lcd.clear();
  lcd.print("Setpoint");
  lcd.setCursor(1,1);
  lcd.print(float2s(Setpoint,1));
  lcd.write(1);
  lcd.print(F("C "));
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// Both to change mode
// ************************************************
void Tune_Sp()
{
   //lcd.setBacklight(TEAL);
   //lcd.print(F("SetTemp:"));
   uint8_t buttons = 0;
   int Tk=KEY_SLOW;
   float increment = 0.5;

   if(tuneMode == MODE){
     Tune_Mode();
     return;
   }

   showSetpoint();

   while(true)
   {
      buttons = ReadButtons();

/*      
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_LEFT)
      {
         opState = RUN;
         return;
      }
      if (buttons & BUTTON_RIGHT)
      {
         opState = TUNE_P;
         return;
      }
*/
      if(!buttons){
        Tk=KEY_SLOW;
        increment = 0.5;
      }
      else if ((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN)){
        tuneMode = MODE;
        Show_Mode();
        return;
      }
      else if (buttons & BUTTON_UP)
      {
         Setpoint += increment;
         if(Setpoint>99){
          Setpoint=99;
         }
         showSetpoint();
         delay(Tk);
         Tk=KEY_FAST;
         increment=1;
      }
      else if (buttons & BUTTON_DOWN)
      {
         Setpoint -= increment;
         if(Setpoint<25){
          Setpoint=25;
         }
         showSetpoint();
         delay(Tk);
         Tk=KEY_FAST;
         increment=1;
      }
    
      if (!buttons && ((millis() - lastInput) > 2000))  // return to RUN after 2 seconds idle
      {
         opState = RUN;
         return;
      }
      //lcd.setCursor(0,1);
      //lcd.print(Setpoint);
      //lcd.print(" ");
      DoControl();
   }
}

/*
	49	Rare
	56.5	Med Rare
	63.5	Medium
	74	Welldone
	85	Vegetable
*/

void Show_Mode()
{
  const float Tm[] = {49.0f,56.5f,63.5f,74.0f,85.0f};
  const char *Desc[] = {"Rare", "Med Rare", "Medium", "Welldone", "Vegetable"};

  Setpoint = Tm[ModeNo];
  lcd.clear();
  //lcd.print(" ");
  lcd.print(float2s(Setpoint,1));
  lcd.write(1);
  lcd.print(F("C"));
  lcd.setCursor(0,1);
  lcd.print(Desc[ModeNo]);
}

// ************************************************
// Mode Entry State
// UP/DOWN to change mode
// Both to change setpoint
// ************************************************
void Tune_Mode()
{
   uint8_t buttons = 0;
   Show_Mode();

   while(true)
   {
      buttons = ReadButtons();
      if ((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN)){
        tuneMode = TEMP;
        showSetpoint();
        return;
      }
      else if (buttons & BUTTON_UP)
      {
         ModeNo++;
         ModeNo%=5;
         Show_Mode();
         delay(KEY_SLOW);
      }
      else if (buttons & BUTTON_DOWN)
      {
         ModeNo+=4;
         ModeNo%=5;
         Show_Mode();
         delay(KEY_SLOW);
      }
    
      if (!buttons && ((millis() - lastInput) > 2000))  // return to RUN after 6 seconds idle
      {
         opState = RUN;
         tuneMode = TEMP;
         return;
      }
      DoControl();
   }	
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// Both for Ki
// ************************************************
void TuneP()
{
    int Tk=KEY_SLOW;
    float increment = 1.0;
   //lcd.setBacklight(TEAL);
   lcd.clear();
   lcd.print(F("Set Kp"));
    lcd.setCursor(0,1);
    lcd.print(Kp);

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      if(!buttons){
        Tk=KEY_SLOW;
      }
      else if ((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN)){
        opState = TUNE_I;
        return;
      }
      else if (buttons & BUTTON_UP)
      {
         Kp += increment;
        lcd.setCursor(0,1);
        lcd.print(Kp);
        lcd.print("    ");
         delay(Tk);
         Tk=KEY_FAST;
      }
      else if (buttons & BUTTON_DOWN)
      {
         Kp -= increment;
        lcd.setCursor(0,1);
        lcd.print(Kp);
        lcd.print("    ");
         delay(Tk);
         Tk=KEY_FAST;
      }
      if (!buttons && ((millis() - lastInput) > 6000))  // return to RUN after 6 seconds idle
      {
         opState = RUN;
         return;
      }
      DoControl();
   }
   Kp=(int)Kp;
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// Both for Kd
// ************************************************
void TuneI()
{
   int Tk=KEY_SLOW;
   float increment = 0.1;
   //lcd.setBacklight(TEAL);
   lcd.clear();
   lcd.print(F("Set Ki"));
   lcd.setCursor(0,1);
   lcd.print(Ki);

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      if(!buttons){
        Tk=KEY_SLOW;      
      }
      else if ((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN)){
        opState = TUNE_D;
        return;
      }
      else if (buttons & BUTTON_UP)
      {
         Ki += increment;
        lcd.setCursor(0,1);
        lcd.print(Ki);
        lcd.print("    ");
         delay(Tk);
         Tk=KEY_FAST;
      }
      else if (buttons & BUTTON_DOWN)
      {
         Ki -= increment;
        lcd.setCursor(0,1);
        lcd.print(Ki);
        lcd.print("    ");
         delay(Tk);
         Tk=KEY_FAST;
      }
      if (!buttons && ((millis() - lastInput) > 6000))  // return to RUN after 6 seconds idle
      {
         opState = RUN;
         return;
      }
      DoControl();
   }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// Both to exit
// ************************************************
void TuneD()
{
    int Tk=KEY_SLOW;
    float increment = 0.01;
   //lcd.setBacklight(TEAL);
   lcd.clear();
   lcd.print(F("Set Kd"));
   lcd.setCursor(0,1);
   lcd.print(Kd);

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      if(!buttons){
        Tk=KEY_SLOW;      
      }
      else if ((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN)){
        opState = RUN;
        return;
      }
      else if (buttons & BUTTON_UP)
      {
         Kd += increment;
          lcd.setCursor(0,1);
          lcd.print(Kd);
          lcd.print("    ");
         delay(Tk);
         Tk=KEY_FAST;
      }
      else if (buttons & BUTTON_DOWN)
      {
         Kd -= increment;
          lcd.setCursor(0,1);
          lcd.print(Kd);
          lcd.print("    ");
         delay(Tk);
         Tk=KEY_FAST;
      }
      if (!buttons && ((millis() - lastInput) > 6000))  // return to RUN after 6 seconds idle
      {
         opState = RUN;
         return;
      }
      DoControl();
   }
}

void TuneWait()
{
    int Tk=KEY_SLOW;
    int increment = 15*60; //15 minutes
   //lcd.setBacklight(TEAL);
   lcd.clear();
   lcd.print(F(" Delay"));
   lcd.setCursor(0,1);
   showTwait();

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      if(!buttons){
        Tk=KEY_SLOW;      
      }
      else if ((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN)){
        opState = TUNE_P;
        return;
      }
      else if (buttons & BUTTON_UP)
      {
         Twait += increment;
         Twait -= Twait%increment;
         showTwait();
         delay(Tk);
         Tk=KEY_FAST;
      }
      else if (buttons & BUTTON_DOWN)
      {
         if(Twait>=increment){
          Twait -= increment;
         }
         else Twait=0;
         Twait -= Twait%increment;
         showTwait();
         delay(Tk);
         Tk=KEY_FAST;
      }
      if (!buttons && ((millis() - lastInput) > 6000))  // return to RUN after 6 seconds idle
      {
         opState = RUN;
         return;
      }
      DoControl();
   }
}

// ************************************************
// PID COntrol State
// Up/Down to adjust Setpoint
// Both short - Kp
// Both long - Autotune
// ************************************************
void Run()
{
   float increment=0.5;
   static int lc=0;
   // set up the LCD's number of rows and columns: 
   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   uint8_t buttons = 0;
   while(true)
   {
      //setBacklight();  // set backlight based on state

      buttons = ReadButtons();
      if ((buttons & BUTTON_UP) && (buttons & BUTTON_DOWN)){
        opState = TUNE_W;
        return;
      }
      else if (buttons & (BUTTON_UP | BUTTON_DOWN))
      {
         opState = SETP;
         return;
      }

      DoControl();
      
      if (millis() - lastInput > KEY_FAST)  {
        //lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" ");
        lcd.print(float2s(Input,1));
        lcd.write(1);
        lcd.print(F("C"));
        
        float pct = map(Output, 0, WindowSize, 0, 1000);
        lcd.setCursor(1,1);

        if(tuning){
          lcd.setCursor(0,1);
          lcd.print("Tuning..");
        }
        else if(Twait==0){
          myPID.SetMode(AUTOMATIC);
          lcd.setCursor(0,1);
          if(relay_flag){
            lcd.print(F("*    "));
          }
          else{
            lcd.print(F("     "));
          }
          lcd.setCursor(2,1);
          if(pct<1000){
            lcd.print(" ");
          }
          lcd.print(float2s(pct/10,0));
          //lcd.print(Output);
          lcd.print("% ");
          if(relay_flag){
            lcd.print("*");
          }
          else{
            lcd.print(" ");
          }
        }
        else{
          myPID.SetMode(MANUAL);
          onTime = Output = 0;
          showTwait();
        }
  
      }

#ifdef WITH_SERIAL      
      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        Serial.print(Input);
        Serial.print(",");
        Serial.println(Output);
        lastLogTime = millis();
      }
#endif
      lc++;
      if(lc>=10){
        if(Twait)Twait--;
        lc=0;
      }
      delay(65); //approx 100ms loop time
   }
}

void showTwo(int n)
{
  if(n<10)lcd.print("0");
  lcd.print(n%100);
}

void showTwait(void)
{
  int h = Twait/3600;
  int m = (Twait%3600)/60;
  int s = Twait%60;
  lcd.setCursor(0,1);
  showTwo(h);
  lcd.print(":");
  showTwo(m);
  lcd.print(":");
  showTwo(s);
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  //onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
     onTime = Output; 
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
     relay_flag = HIGH;
  }
  else
  {
     digitalWrite(RelayPin,LOW);
     relay_flag = LOW;
  }
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
#if 0  
   if (tuning)
   {
      lcd.setBacklight(VIOLET); // Tuning Mode
   }
   else if (abs(Input - Setpoint) > 1.0)  
   {
      lcd.setBacklight(RED);  // High Alarm - off by more than 1 degree
   }
   else if (abs(Input - Setpoint) > 0.2)  
   {
      lcd.setBacklight(YELLOW);  // Low Alarm - off by more than 0.2 degrees
   }
   else
   {
      lcd.setBacklight(WHITE);  // We're on target!
   }
#endif   
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
   lcd.clear();
   lcd.print("AutoTune");
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
#if 0  
uint8_t ReadButtons()
{
  uint8_t buttons = lcd.readButtons();
  if (buttons != 0)
  {
    lastInput = millis();
  }
  return buttons;
}
#else
uint8_t ReadButtons()
{
  //uint8_t ba[]={pin_Select, pin_Right, pin_Left, pin_Down, pin_Up};
  uint8_t ba[]={pin_Down, pin_Up};
  uint8_t i, new_buttons=0;
  static uint8_t buttons=0;
  for(i=0;i<2;i++){
    new_buttons <<= 1;
    new_buttons |= !digitalRead(ba[i]);
  }
  if (new_buttons != buttons)
  {
    lastInput = millis();
  }
  buttons = new_buttons;
  //lcd.setCursor(0, 0);
  //lcd.print(buttons,HEX);
  return buttons;
}
#endif

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.2;
   }
   if (isnan(Kd))
   {
     Kd = 0.0;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

