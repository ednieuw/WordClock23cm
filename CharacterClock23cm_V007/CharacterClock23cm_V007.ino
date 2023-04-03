// =============================================================================================================================
/* 
This Arduino code controls the Nano Every connected to the 23cm word clock with SK6812 LEDs.
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module BLE HM10 and JDY23 works fines
- DCF77 module DCF-2
- LCD
- HC12 transceiver


A HM-10 or JDY-23 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module.
The DCFNOINT subroutine is  a non-interrupt driven DCF77 decoding routine. Using both together improves decoding efficiency

Arduino Uno, Nano with SK6812 LEDs: Program size must stay below approx 23572, 1129 bytes bytes with 144 LEDs in ATMEGA 32KB 
With a Arduino Nano Every with 48kb memory this is not a problem
************************************************************************************
To set the time with the rotary button:
One press on rotary: UUR is flashing -> turn to set hour
Second press on rotary: HET IS WAS flashes --> turn to set minute
the following presses are various display modes with the WHEELCOLOR choice as second last 
Last press on rotary: many letters turns on --> turn to set intensity level of the LEDs
Press again of wait for one minute to return to normal operation
Displaymodes (Qn):
DEFAULTCOLOUR = 0; Yellow text.  HET Green to Red hour IS WAS Green to Red Minute  
HOURLYCOLOUR  = 1; Every hour other colour text. HET Green to Red hour IS WAS Green to Red Minute         
WHITECOLOR    = 2; All white text
OWNCOLOUR     = 3; All own colour
OWNHETISCLR   = 4; All own colour. HET Green to Red hour IS WAS Green to Red Minute
WHEELCOLOR    = 5; Rotating colour of rainbow changing every minute
************************************************************************************
 Author .: Ed Nieuwenhuys
 
Changes.: V001 Derived from Character_Colour_Clock_V080.
Changes.: V002 Clock & display are working
Changes.: V003 Cleaning up redundant code
Changes.: V004 Compiles with MegaCoreX and Nano Every, MegaCoreX does not write to Bluetooth. Wire conflicht. Wait for upgrade
Changes.: V005 Removed LedsOff(); in Displaytime(). Caused flickering. 
               Comparable with Character_Colour_Clock_V084
               Repaired RotaryEncoderCheck(0 and ProcessKeyPressTurn(0. Comparable with Character_Clock_V117.ino
Changes.: V006 Cleaning up code. LDR reads not divided by 4 anymore and are now between 0 and 1024
Changes.: V007 

 */
// ===============================================================================================================================
//                                                                                            //
//------------------------------------------------------------------------------
// ARDUINO Definition of installed modules
// You have to choose between 
  // - WS2812 and SK6812
  // - NEOpixel or LIBSK6812
  // - define how many LEDs in the strip
// If you do not define the MOD_DS3231 the internal, not precise, clock is used. 
//------------------------------------------------------------------------------
// ------------------>   Define only one type of LED strip 
//#define LED2812
#define LED6812                                    // Choose between LED type RGB=LED2812 or RGBW=LED6812
const byte NUM_LEDS      = 76;                     // How many leds in  strip?
// ------------------>   Define only NEOPIXEL or LIBSK6812 library
//#define NEOPIXEL                                 // Adafruit Neopixel for WS2812 or SK6812 LEDs
#define LIBSK6812                                  // SK6812 library. Only with SK6812 LEDs  (saves 614 bytes with NEOPIXEL)
// ------------------>  Define which modules are present.  
#define BLUETOOTHMOD                               // Use  this define if Bluetooth needs other pins (default( pin 6&7) than pin 0 and pin 1
#define MOD_DS3231                                 // The DS3231 module is installed, if not the Arduino internal clock is used

//#define HC12MOD                                  // Use HC12 time transreceiver Long Range Wireless Communication Module in Bluetooth slot
//#define DCF77MOD                                 // DCF77 receiver installed
//#define LCDMOD                                   // For LCD support
//#define HT16K33tijd                              // 4-digit HT16K33 time display installed https://www.adafruit.com/product/879 Adafruit GFX library 

#define ROTARYMOD                                  // Rotary encoder installed
//#define KEYPAD3x4                                // Use a 3x4 keypad with 7 wires
//#define KEYPAD3x1                                // Use a 3x1 keypad with 4 wires   
//#define ONEWIREKEYPAD3x1                         // Use a 3x1 keypad with one wire   
//#define ONEWIREKEYPAD3x4                         // Use a 3x4 keypad with one wire
   
//------------------------------------------------------------------------------
// ARDUINO Includes defines and initialisations
//------------------------------------------------------------------------------
//                                                                                            //
#include <TimeLib.h>                               // https://github.com/PaulStoffregen/Time 
#include <Wire.h>                                  // Arduino standard library
#include "RTClib.h"                                // https://github.com/adafruit/RTClib 
//#include <avr/wdt.h>                             // For reset function
//#include <avr/pgmspace.h>                        // Arduino
#include <EEPROM.h>
                     #if defined(ROTARYMOD) || defined(ONEWIREKEYPAD3x1) || defined(ONEWIREKEYPAD3x4)
#define ROTKEYMOD                                  // not Implemented yet ************ Make this #ifdef easier to read
                     #endif
                     #ifdef DCF77MOD
#define DCFTH                                      // Use the Arduino DCF77 library with interrupts.  
#define DCFNOINT                                   // Use the Tiny DCF algorithm in this program. 
                     #endif                   

                     #ifdef NEOPIXEL
#include <Adafruit_NeoPixel.h>                      // https://github.com/adafruit/Adafruit_NeoPixel   for LED strip WS2812 or SK6812
                     #endif  // NEOPIXEL
                     #ifdef LIBSK6812
#include <EdSoft_SK6812.h>                         // https://github.com/ednieuw/EdSoft_SK6812
                     #endif // LIBSK6812
                     #ifdef LCDMOD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>                     // https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
                     #endif  //LCDMOD              // https://github.com/adafruit/Adafruit_NeoPixel 
                     #ifdef HT16K33tijd
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"                  //https://learn.adafruit.com/adafruit-led-backpack/overview?view=all
                     #endif //HT16K33tijd                     
                     #if defined(KEYPAD3x4) || defined(KEYPAD3x1)
#include "Adafruit_Keypad.h"                       // https://github.com/adafruit/Adafruit_Keypad//                 
                     #endif //KEYPAD 
                     #ifdef HC12MOD
#include <SoftwareSerial.h>                        // For HC12 transceiver
                     #endif  //HC12MOD 
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>                        // Arduino for Bluetooth communication
                     #endif //BLUETOOTHMOD
                     #ifdef ROTARYMOD
#include <Encoder.h>                               // http://www.pjrc.com/teensy/td_libs_Encoder.html 
                     #endif //ROTARYMOD
                     #ifdef DCFTH
#include "DCF77.h"                                 // http://playground.arduino.cc/Code/DCF77                                                                                         //
                     #endif  //DCFTH
                     #ifdef DCFNOINT
#include "EdSoft_DCF77.h"                          // https://github.com/ednieuw/EdSoft_DCF77
                     #endif  //DCFNOINT
//------------------------------------------------------------------------------
// Definitions to under which LEDs the words are placed.
// Start counting at 0 (zero)
//------------------------------------------------------------------------------
#define HET     ColorLeds("Het",      0,   2, LetterColor);   
#define IS      ColorLeds("is",       3,   4, LetterColor); Is = true;
#define WAS     ColorLeds("was",      5,   7, LetterColor); Is = false;
#define MVIJF   ColorLeds("vijf",    11,  13, LetterColor); 
#define MTIEN   ColorLeds("tien",     8,  10, LetterColor); 
#define KWART   ColorLeds("kwart",   14,  17, LetterColor);
#define VOOR    ColorLeds("voor",    18,  20, LetterColor);
#define OVER    ColorLeds("over",    27,  29, LetterColor);
#define PRECIES ColorLeds("precies", 21,  26, LetterColor);
#define HALF    ColorLeds("half",    30, 32, LetterColor);
#define ELF     ColorLeds("elf",     33, 35, LetterColor);
#define MIDDER  ColorLeds("midder",  48, 52, LetterColor);
#define VIJF    ColorLeds("vijf",    39, 41, LetterColor);
#define TWEE    ColorLeds("twee",    36, 38, LetterColor);
#define EEN     ColorLeds("een",     42, 44, LetterColor);
#define VIER    ColorLeds("vier",    45, 47, LetterColor);
#define TIEN    ColorLeds("tien",    53, 55, LetterColor);
#define TWAALF  ColorLeds("twaalf",  48, 52, LetterColor);
#define DRIE    ColorLeds("drie",    56, 58, LetterColor);
#define NEGEN   ColorLeds("negen",   59, 62, LetterColor);
#define ACHT    ColorLeds("acht",    66, 68, LetterColor);
#define NACHT   ColorLeds("nacht",   73, 75, LetterColor);
#define ZES     ColorLeds("zes",     63, 65, LetterColor);
#define ZEVEN   ColorLeds("zeven",   69, 72, LetterColor);
#define NOEN    ColorLeds("noen",    52, 56, LetterColor);
#define UUR     ColorLeds("uur",     73, 75, LetterColor);
#define EDSOFT  ColorLeds("EdSoft",   0,  0, LetterColor);

//                                                                                            //
//------------------------------------------------------------------------------
// PIN Assigments
//------------------------------------------------------------------------------ 
enum DigitalPinAssignments {      // Digital hardware constants ATMEGA 328 ----
 RX           = 0,                // Connects to Bluetooth TX
 TX           = 1,                // Connects to Bluetooth RX
 DCF_PIN      = 2,                // DCFPulse on interrupt  pin
 encoderPinA  = 3,                // right (labeled DT on decoder)on interrupt  pin
 clearButton  = 4,                // switch (labeled SW on decoder)
 LED_PIN      = 5,                // Pin to control colour SK6812/WS2812 LEDs
 BT_TX        = 6,                // Connects to Bluetooth RX
 BT_RX        = 7,                // Connects to Bluetooth TX
 HC_12TX      = 6,                // HC-12 TX Pin  // note RX and TX are reversed compared with a BT Module
 HC_12RX      = 7,                // HC-12 RX Pin 
 encoderPinB  = 8,                // left (labeled CLK on decoder)no interrupt pin  
 secondsPin   = 9,                // LED_BUILTIN
 HeartbeatLED = 11,               //   // PIN10        = 10,               // PIN 10 
 DCFgood      = 10,               // DCF-signal > 50    
 PIN11        = 11,               // PIN 11
 PIN12        = 12,               // PIN 12             // led = 12,          // MKR1010
 DCF_LED_Pin  = 13,                // DCF signal
 };
 
enum AnaloguePinAssignments {     // Analogue hardware constants ----
 EmptyA0      = A0,                // Empty
 EmptyA1      = A1,                // Empty
 PhotoCellPin = A2,                // LDR pin
 OneWirePin   = A3,                // OneWirePin
 SDA_pin      = A4,                // SDA pin
 SCL_pin      = A5,                // SCL pin
 EmptyA6     =  A6,                // Empty
 EmptyA7     =  A7};               // Empty

//------------------------------------------------------------------------------
// COLOURS
//------------------------------------------------------------------------------   
const byte DEFAULTCOLOUR = 0;
const byte HOURLYCOLOUR  = 1;          
const byte WHITECOLOR    = 2;
const byte OWNCOLOUR     = 3;
const byte OWNHETISCLR   = 4;
const byte WHEELCOLOR    = 5;
byte ResetDisplayChoice  = DEFAULTCOLOUR; 
                         #ifdef LED2812
const uint32_t white  = 0xFFFFFF, lgray  = 0x666666;               // R, G and B on together gives white light
const uint32_t gray   = 0x333333, dgray  = 0x222222;                
                         #endif  //LED2812
                         #ifdef LED6812    
const uint32_t white  = 0xFF000000, lgray  = 0x66000000;           // The SK6812 LED has a white LED that is pure white
const uint32_t dgray  = 0x22000000, gray   = 0x33000000;
                         #endif  //LED6812  
//------------------------------------------------------------------------------
const uint32_t black    = 0x000000, darkorange    = 0xFF8C00, red    = 0xFF0000, chartreuse = 0x7FFF00;
const uint32_t brown    = 0x503000, cyberyellow   = 0xFFD300, orange = 0xFF8000; 
const uint32_t yellow   = 0xFFFF00, cadmiumyellow = 0xFFF600, chromeyellow = 0xFFA700;
const uint32_t green    = 0x00FF00, brightgreen   = 0x66FF00, apple  = 0x80FF00, grass  = 0x00FF80;  
const uint32_t amaranth = 0xE52B50, edamaranth    = 0xFF0050, amber  = 0xFF7E00;
const uint32_t marine   = 0x0080FF, darkviolet    = 0x800080, pink   = 0xFF0080, purple = 0xFF00FF; 
const uint32_t blue     = 0x0000FF, cerulean      = 0x007BA7, sky    = 0x00FFFF, capri  = 0x00BFFF;
const uint32_t edviolet = 0X7500BC, frenchviolet  = 0X8806CE, coquelicot = 0xFF3800;
const uint32_t greenblue= 0x00F2A0, hotmagenta    = 0xFF00BF, dodgerblue = 0x0073FF, screamingreen = 0x70FF70;
//------------------------------------------------------------------------------
// LED
//------------------------------------------------------------------------------
//                                                                 //
const byte BRIGHTNESS    = 32;                                     // BRIGHTNESS 0 - 255
                       #ifdef LED6812    
                             #ifdef NEOPIXEL   
Adafruit_NeoPixel LEDstrip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);  //NEO_RGBW
                             #endif //NEOPIXEL
                             #ifdef LIBSK6812
EdSoft_SK6812 LEDstrip(NUM_LEDS, LED_PIN);                          // Initialyse SK6812 library
                            #endif  //LIBSK6812  
                       #endif  //LED6812  
                            #ifdef LED2812
Adafruit_NeoPixel LEDstrip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //NEO_RGB NEO_GRB
                            #endif  //LED2812
bool     LEDsAreOff            = false;                             // If true LEDs are off except time display
bool     NoTextInColorLeds     = false;                             // Flag to control printing of the text in function ColorLeds()
byte     BrightnessCalcFromLDR = BRIGHTNESS;                        // Initial brightness value The intensity send to the LEDs (0-255)
int      Previous_LDR_read     = 512;                               // The actual reading from the LDR + 4x this value /5
uint32_t MINColor      = chromeyellow;
uint32_t SECColor      = chromeyellow;  
uint32_t DefaultColor  = chromeyellow;   
uint32_t LetterColor   = chromeyellow;   
uint32_t OwnColour     = 0X002345DD;        // Blueish
uint32_t WhiteColour   = white;
uint32_t WheelColor    = blue;
uint32_t HourColor[] ={  white,      darkviolet, cyberyellow, capri,         amber,         apple,
                         darkorange, cerulean,   edviolet,    cadmiumyellow, green,         edamaranth,
                         red,        yellow,     coquelicot,  pink,          apple,         hotmagenta,
                         green,      greenblue,  brightgreen, dodgerblue,    screamingreen, blue,
                         white,      darkviolet, chromeyellow};        

     
//------------------------------------------------------------------------------
// KY-040 ROTARY
//------------------------------------------------------------------------------ 
                          #ifdef ROTARYMOD                         
Encoder myEnc(encoderPinA, encoderPinB);                         // Use digital pin  for encoder
                          #endif  //ROTARYMOD      
long     Looptime          = 0;
byte     RotaryPress       = 0;                                  // Keeps track display choice and how often the rotary is pressed.
uint32_t RotaryPressTimer  = 0;
byte     NoofRotaryPressed = 0;

//------------------------------------------------------------------------------
// LDR PHOTOCELL
//------------------------------------------------------------------------------
//                                                                                            //
const byte SLOPEBRIGHTNESS  = 80;                                // Steepness of with luminosity of the LED increases
const int  MAXBRIGHTNESS    = 999;                               // Maximun value in bits  for luminosity of the LEDs (1 - 1023)
const byte LOWBRIGHTNESS    = 5;                                 // Lower limit in bits of Brightness ( 0 - 255)   
byte       TestLDR          = 0;                                 // If true LDR inf0 is printed every second in serial monitor
int        OutPhotocell;                                         // stores reading of photocell;
int        MinPhotocell     = 999;                               // stores minimum reading of photocell;
int        MaxPhotocell     = 1;                                 // stores maximum reading of photocell;
uint32_t   SumLDRreadshour  = 0;
uint32_t   NoofLDRreadshour = 0;

//------------------------------------------------------------------------------
// CLOCK
//------------------------------------------------------------------------------                                 
static  uint32_t msTick;                                          // the number of millisecond ticks since we last incremented the second counter
int     count; 
int     Delaytime = 200;
byte    lastday = 0, lastminute = 0, lasthour = 0, sayhour = 0;
bool    ChangeTime           = false;
bool    ChangeLightIntensity = false;
bool    Demo                 = false;
bool    Zelftest             = false;
bool    Is                   = true;                              // Toggle of displaying Is or Was
bool    ZegUur               = true;                              // Say or not say Uur in NL clock
tmElements_t I;                                                   // Initialyse a time struct I for internal times used to display the time in the clock

//------------------------------------------------------------------------------
// DS3231 CLOCK MODULE
//------------------------------------------------------------------------------
//                                                                                            //
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
        #ifdef MOD_DS3231
RTC_DS3231 RTCklok;    //RTC_DS1307 RTC; 
        #else
RTC_Millis RTCklok;   
        #endif  //
DateTime Inow;

//------------------------------------------------------------------------------
// BLUETOOTH
//------------------------------------------------------------------------------                                     
                           #ifdef BLUETOOTHMOD                    // Bluetooth ---------------------
SoftwareSerial Bluetooth(BT_RX, BT_TX);                           // BT_RX <=> TXD on BT module, BT_TX <=> RXD on BT module
                           #endif  //BLUETOOTHMOD 
//------------------------------------------------------------------------------
// HC-12 Long Range Wireless Communication Module
//------------------------------------------------------------------------------
                     #ifdef HC12MOD
SoftwareSerial HC12(HC_12TX, HC_12RX);                            // HC-12 TX Pin, HC-12 RX Pin
                     #endif  //HC12MOD 
                                          
//------------------------------------------------------------------------------
// DCF-2 DCF77 MODULE
//------------------------------------------------------------------------------
byte   DCF_signal            = 10;                                 // is a proper time received?
byte   DCF_counts            = 0;                                  // Valid DCF receptions per hour
                    #ifdef DCF77MOD    
bool   DCFEd                 = false;                              // Flag to detect if DCFNOINT library received a good time
bool   DCFTh                 = false;                              // Flag to detect if DCF77 library received a good time
bool   DCFlocked             = false;                              // Time received from DCF77.
                    #endif // DCF77MOD
                    #ifdef DCFTH 
                    #if defined ATmega644_1284 ||  defined ARDUINO_AVR_NANO_EVERY || defined ARDUINO_AVR_ATmega4809
#define DCF_INTERRUPT 2                                            // DCF Interrupt number associated with DCF_PIN
                    #else
#define DCF_INTERRUPT 0                                            // Check for other boards !!! this works for Uno, nano etc
                    #endif

const   byte DCF_INVERTED  = LOW;                                 // HIGH (HKW) or LOW (Reichelt). Some DCF modules invert the signal
DCF77   DCF = DCF77(DCF_PIN, DCF_INTERRUPT,DCF_INVERTED);
                    #endif  //DCFTH                   
                    #ifdef DCFNOINT
//------------------------------------------------------------------------------
// DCF77 DCFNOINT MODULE
//------------------------------------------------------------------------------
const byte DCF_Inverted     = LOW;                                 // HIGH (HKW) or LOW (Reichelt). Some DCF modules invert the signal
EdSoft_DCF77 DCFNoInt = EdSoft_DCF77(DCF_PIN, DCF_Inverted);
                    #endif  //DCFNOINT                
                    #ifdef LCDMOD
//------------------------------------------------------------------------------
// LCD Module
//------------------------------------------------------------------------------
//LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);// 0x27 is the I2C bus address for an unmodified backpack
LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7);
                    #endif 
                       #ifdef HT16K33tijd
//--------------------------------------------
// HT16K33 IC2 display
//--------------------------------------------
Adafruit_7segment Tijd_displayIC2 = Adafruit_7segment();
                      #endif //HT16K33tijd                   
//------------------------------------------------------------------------------
// Webserver
//------------------------------------------------------------------------------
//                                                                                            //
                     #ifdef KEYPAD3x1
//------------------------------------------------------------------------------
// KEYPAD 3x1
//          -------- GND
//  R Y G   -------- Pin 2
//          -------- Pin 3
//          -------- Pin 4
//------------------------------------------------------------------------------
String        KeypadString;
unsigned long KeyLooptime;
const byte    ROWS = 3;                                             // No of rows
const byte    COLS = 1;                                             // No of columns
char Keys[ROWS][COLS] = {  {'G'},  {'Y'},  {'R'} };                 // Define the symbols on the buttons of the keypads
byte rowPins[ROWS] = {2,3,4};                                       // Connect to the row pinouts of the keypad
byte colPins[COLS] = {15};                                          // Connect to a nonsense pin
Adafruit_Keypad Keypad3x1 = Adafruit_Keypad(makeKeymap(Keys), rowPins, colPins, ROWS, COLS); 
                     #endif //KEYPAD3x1                       
                     #ifdef KEYPAD3x4
//------------------------------------------------------------------------------
// KEYPAD3*4
//------------------------------------------------------------------------------
String        KeypadString;
unsigned long KeyLooptime;
const byte    ROWS = 4;                                              // No of rows 
const byte    COLS = 3;                                              // No of columns 
byte          rowPins[ROWS] = {12,11,10,9};                          // Connect to the row pinouts of the keypad 
byte          colPins[COLS] = {8,7,6};                               // Connect to the column pinouts of the keypad
                                                                     // Wire under * to pin 6. Wire under # to pin 12
char   hexaKeys[ROWS][COLS] = {                                      // Define the symbols on the buttons of the keypads
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'} };
Adafruit_Keypad Keypad3x4 = Adafruit_Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
                      #endif //KEYPAD3x4
byte KeyInputactivated = false;                                     // if pressing the keys stop other events like EveryMinuteUpdate()   

//----------------------------------------
// Common
//----------------------------------------
// 
bool   UseDCF                = false;                              // Use the DCF-receiver or not
char   sptext[100];                                                // For common print use
uint16_t  MilliSecondValue  = 1000;                                // The duration of a second  minus 1 ms. Used in Demo mode
static uint32_t last_time = 0;                                     // Heartbeat defines
struct EEPROMstorage {                                             // Data storage in EEPROM to maintain them after power loss
  byte     LightReducer;
  byte     LowerBrightness;
  int      UpperBrightness;
  byte     DisplayChoice;
  byte     TurnOffLEDsAtHH;
  byte     TurnOnLEDsAtHH;
  int      NVRAMmem[24];                                           // Memory in DS3231 Clock
  uint32_t OwnColour;                                              // Self defined colour for clock display
  uint32_t EdMinTijd;                                              // Counter of valid time but invalid date
  uint32_t EdMin;                                                  // Counter of total valid date/times since start for DCFNOINT 
  uint32_t ThMin;                                                  // Counter of totalvalid date/times since start for DCF77
  uint32_t EdTh;                                                   // Valid date/times since start counters for DCFNOINT and DCF77
  uint32_t EdThMin;                                                // Valid times per minute counters for DCFNOINT and DCF77
  uint32_t ThWrong;                                                // Different times reported DCFNOINT and DCF77
  uint32_t EdWrong;                                                // Different times reported DCFNOINT and DCF77
  uint32_t ValidTimes;                                             // Counter of total recorded valid times
  uint32_t MinutesSinceStart;                                      // Counter of recorded valid and not valid times
  int  Checksum;
} Mem;// {0};
//------------------------------------------------------------------------------
// Menu
//------------------------------------------------------------------------------  
//0        1         2         3         4         5
//12345678901234567890123456789012345678901234567890  
 char menu[][42] = {                       // menu[][nn]  nn is largest length of sentence in the menu 
 "Woordklok 23CM",
 "Enter time as: hhmmss (132145)",
                    #ifdef DCFNOINT
 "A Debug DCF-signal",
                     #endif  //DCFNOINT
 "D D15122017 is date 15 December 2017",   //
                     #ifdef DCF77MOD
// "G DCF-signalinfo in display",
 "H Use DCF-receiver",
                     #endif //DCF77MOD
 "L (L5) Min light intensity(0-255 bits)", //
 "M (M90)Max light intensity(1-999)",
 "N (N2208)Turn OFF LEDs between Nhhhh",   //
 "0 Turn ON/OFF LEDs", 
 "P (P00234F8A) own colour (n=0-F)",
 "Q Display Choice (Q0-6) Q=Choices)",
 "I For this info",
 "R Reset to default settings",
 "S Self test",
 "T Thhmmss is time (T031500)",
 "W Test LDR reading every second",       //
 "X (X50) Demo mode. ms delay (0-9999)",
 "Y (Y50) Slope light intensity(1-250)",
 "Ed Nieuwenhuys March 2023" };
//  -------------------------------------   End Definitions  ---------------------------------------
//                                                                                            //
//------------------------------------------------------------------------------
// ARDUINO Loop
//------------------------------------------------------------------------------
void loop()
{
                      InputDevicesCheck();                                                    // Check for input from input devices
 if (Demo)            Demomode();
 else if (Zelftest)   Selftest();   
 else                 EverySecondCheck();  
}  
//------------------------------------------------------------------------------
// ARDUINO Setup
//------------------------------------------------------------------------------
//                                                                                            //
void setup()
{                                                            
 Serial.begin(9600);                                                                          // Setup the serial port to 9600 baud       
//  int32_t Tick = millis();                                                                  // Start the timer 
//  while (!Serial)                                                                           // Wait until serial port is started 
//     {if ((millis() - Tick) >1000) break;}                                                  // Prevents hanging if serial monitor/port is not connected  Tekstprintln("*********\nSerial started"); 
 Tekstprintln("\n*********\nSerial started"); 
 Wire.begin();                                                                                // Start communication with I2C / TWI devices     
 #if defined(MEGACOREX)
      Tekstprintln("Compiled with MegaCoreX board"); 
 #else 
     Tekstprintln("Compiled with Arduino MegaAVR board");   
#endif
                          #ifdef DCF77MOD
 pinMode(DCF_LED_Pin,  OUTPUT);                                                               // For showing DCF-pulse with a LED
 pinMode(DCF_PIN,      INPUT_PULLUP);                                                         // Use Pin 2 for DCF input
 pinMode(DCFgood,      OUTPUT ); 
 digitalWrite(DCFgood,LOW);
                         #endif //DCF77MOD
 pinMode(secondsPin,   OUTPUT );  
 pinMode(HeartbeatLED, OUTPUT );      
                          #ifdef MOD_DS3231
 RTCklok.begin();
 Tekstprintln("RTC DS3231 enabled");// start the RTC-module
                          #else 
 RTCklok.begin(DateTime(F(__DATE__), F(__TIME__)));                                           // If no RTC module is installed use the ATMEGAchip clock
 Tekstprintln("Internal clock enabled");   
                          #endif  //MOD_DS3231 
                          #ifdef BLUETOOTHMOD 
 Bluetooth.begin(9600);  
 Tekstprintln("Bluetooth enabled");
                          #endif  //BLUETOOTHMOD
                          #ifdef ROTARYMOD   
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP); 
 Tekstprintln("Rotary enabled"); 
 myEnc.write(0);                                                                              // Clear Rotary encode buffer
                          #endif  //ROTARYMOD 
                          #ifdef KEYPAD3x1
 Keypad3x1.begin();                                                                           // Start the 3x4 keypad
 Tekstprint("3*1 keypad enabled");  
                          #endif  //KEYPAD3x1
                          #ifdef KEYPAD3x4
 Keypad3x4.begin();                                                                           // Start the 3x4 keypad
 Tekstprint("4*3 keypad enabled");  

                          #endif  //KEYPAD3x4
                          #ifdef ONEWIREKEYPAD  
 Tekstprint("4*3keypad onewire enabled");                                                     // The one wire keypad is enabled
                          #endif  //ONEWIREKEYPAD
                            #ifdef NEOPIXEL 
 LEDstrip.begin();                                                                            // Start communication to LED strip
 LEDstrip.setBrightness(BRIGHTNESS);                                                          // Set brightness of LEDs
 ShowLeds(); 
 Tekstprintln("LIB NEOPIXEL");   
                          #endif //NEOPIXEL 
                          #ifdef LIBSK6812
 LEDstrip.setBrightness(BRIGHTNESS);                                                          // Set brightness of LEDs
 Tekstprintln("LIBSK6812");                                                                   // Initialyse SK6812 library
                          #endif //LIBSK6812                                                  // Initialize all pixels to 'off' 
                          #ifdef LED6812    
 Tekstprintln("LEDs SK6812 enabled");
                          #endif  //LED6812  
                          #ifdef LED2812
 Tekstprintln("LEDs WS2812 enabled");
                          #endif  //LED2812 
                          #ifdef HC12MOD
 HC12.begin(9600);               // Serial port to HC12
 Tekstprintln("HC-12 time receiver enabled");
                          #endif  //HC12MOD                           
//                                                                                            //
                          #ifdef LCDMOD
 lcd.begin (16,2); // for 16 x 2 LCD module                                                   // Activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH);
 Tekstprintln("LCD enabled");
                          #endif  //LCDMOD
                           #ifdef HT16K33tijd
 Tijd_displayIC2.begin(0x70);
 Tijd_displayIC2.setBrightness(3);                                                            // Set the display brightness (0-7):
 Tekstprintln("4-digit time HT16K33 display installed"); 
                          #endif //HT16K33tijd                         
 UseDCF = false;                                                                              // Flag that indicates if the DCF receiver must be turned on or off
                          #ifdef DCFTH
 DCF.Start();                                                                                 // Start the DCF-module
 Tekstprintln("DCFTh enabled");
 UseDCF = true; 
                          #endif  //DCFTH
                          #ifdef DCFNOINT
 //DCFmsTick = millis();
 Tekstprintln("DCFNOINT enabled");
 UseDCF = true; 
                          #endif  //DCFNOINT
                          #if defined(ARDUINO_ARCH_RP2040)
 Tekstprintln("Compiled for ARDUINO NANO RP2020");
                          #endif
                          #ifdef ARDUINO_AVR_NANO_EVERY
 Tekstprintln("Compiled for ARDUINO AVR NANO EVERY");
                          #endif
                          #if defined(__AVR_ATmega328P__) 
 Tekstprintln("Compiled with ATmega328P emulation");
                          #endif
                          #if defined(__AVR_ATmega4809__) 
 Tekstprintln("Compiled with ATmega4809 emulation");
                          #endif                               
 DateTime now = RTCklok.now();
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));                             // Following line sets the RTC to the date & time this sketch was compiled
   RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
 EEPROM.get(0,Mem);                                                                           // Get the data from EEPROM
 if( Mem.Checksum != 25065) Reset();                                                          // If the checksum is incorrect the data were not set to default values
 Mem.LightReducer    = constrain(Mem.LightReducer,1,250);                                     // 
 Mem.LowerBrightness = min(Mem.LowerBrightness, 250);                                         // 
 Mem.UpperBrightness = min(Mem.UpperBrightness, 1023); 
 Mem.DisplayChoice   = min(Mem.DisplayChoice, WHEELCOLOR);                                       // 
 if ( Mem.OwnColour == 0 ) Mem.OwnColour = 0X002345DD;                                        // If memory is empty then store default value, blue  
 EEPROM.put(0,Mem);                                                                           // update EEPROM if some data are out of the constrains     
 Previous_LDR_read = analogRead(PhotoCellPin);                                                // to have a start value
 MinPhotocell      = Previous_LDR_read;                                                       // Stores minimum reading of photocell;
 MaxPhotocell      = Previous_LDR_read;                                                       // Stores maximum reading of photocell;                                            
// Selftest();                                                                                // Play the selftest
 GetTijd(0);                                                                                  // Get the time and store it in the proper variables
 SWversion();                                                                                 // Display the version number of the software
 Displaytime();                                                                               // Turn on the LEDs with proper time and display choice
 Print_RTC_tijd();
 while (Serial.available())  Serial.read();                                                   // Flush the serial input buffer. When a Bluetooth module is connected to RX/TX pin 0&1 garbage can set the time to 00:00:00
 msTick = Looptime = millis();                                                                // Used in KY-040 rotary for debouncing and seconds check, Start of DCF 1 second loop
 } 
//                                                                                            //
//------------------------------------------------------------------------------
// ARDUINO Reset to default settings
//------------------------------------------------------------------------------
void Reset(void)
{
 Tekstprintln("\nReset to default settings"); 
 Mem.Checksum         = 25065; 
 Mem.LightReducer     = SLOPEBRIGHTNESS;                                                      // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 Mem.UpperBrightness  = MAXBRIGHTNESS;                                                        // Upper limit of Brightness in bits ( 1 - 1023)
 Mem.LowerBrightness  = LOWBRIGHTNESS;                                                        // Lower limit of Brightness in bits ( 0 - 255)
 Mem.DisplayChoice    = ResetDisplayChoice; 
 Mem.TurnOffLEDsAtHH  = Mem.TurnOnLEDsAtHH = 0;
 Mem.OwnColour        = 0X002345DD;                                                           // Blue 
 Mem.EdMinTijd        = 0;                                                                    // Counter of total valid times since start for DCFNOINT 
 Mem.EdMin            = 0;                                                                    // Counter of total valid date/times since start for DCFNOINT 
 Mem.ThMin            = 0;                                                                    // Counter of totalvalid date/times since start for DCF77
 Mem.EdTh             = 0;                                                                    // Valid date/times since start counters for DCFNOINT and DCF77
 Mem.EdThMin          = 0;                                                                    // Valid times per minute counters for DCFNOINT and DCF77
 Mem.ThWrong          = 0;                                                                    // Different times reported DCFNOINT and DCF77
 Mem.EdWrong          = 0;                                                                    // Different times reported DCFNOINT and DCF77
 Mem.ValidTimes       = 0;                                                                    // Counter of total recorded valid times
 Mem.MinutesSinceStart= 0;                                                                    // Counter of recorded valid and not valid times
 Previous_LDR_read    = analogRead(PhotoCellPin);                                             // to have a start value
 MinPhotocell         = Previous_LDR_read;                                                    // Stores minimum reading of photocell;
 MaxPhotocell         = Previous_LDR_read;                                                    // Stores maximum reading of photocell;                                            
 TestLDR              = 0;                                                                    // If true LDR display is printed every second
 ChangeTime           = false;
 ChangeLightIntensity = false;
 Demo                 = false;
 Zelftest             = false;
 Is                   = true;                                                                 // toggle of displaying Is or Was
 ZegUur               = true;                                                                 // Say or not Uur in NL clock
 for (int i=0;i<24;i++) Mem.NVRAMmem[i] = 3;                                                  // Reset LDR readings 
 EEPROM.put(0,Mem);                                                                           // Update EEPROM       
// Selftest();                                                                                // Play the selftest
 GetTijd(0);                                                                                  // Get the time and store it in the proper variables
 SWversion();                                                                                 // Display the version number of the software
 Displaytime();
 Print_RTC_tijd();
}
//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Version info
//------------------------------------------------------------------------------
void SWversion(void) 
{ 
 #define FILENAAM (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
 unsigned int i;
 PrintLine(40);
 for (i = 0; i < sizeof(menu) / sizeof(menu[0]); Tekstprintln(menu[i++]));
 PrintLine(48);
                     #ifdef DCF77MOD    
 sprintf(sptext,"DCF reception per hour");                                           Tekstprintln(sptext); 
                    #else
 sprintf(sptext,"LDR average bits measured per hour");                             Tekstprintln(sptext); 
                    # endif  //DCF77MOD
 for (i=0;i<12;i++) { sprintf(sptext," %02d ",i );             Tekstprint(sptext); } Tekstprintln("");
 for (i=0;i<12;i++) { sprintf(sptext,"%03d ",Mem.NVRAMmem[i]); Tekstprint(sptext); } Tekstprintln("");
 for (i=12;i<24;i++){ sprintf(sptext,"%03d ",Mem.NVRAMmem[i]); Tekstprint(sptext); } Tekstprintln("");
 PrintLine(48);
 sprintf(sptext,"Brightness Min: %3d bits  Max: %3d bits Slope:%3d%%",
                                  Mem.LowerBrightness, Mem.UpperBrightness, Mem.LightReducer);  Tekstprintln(sptext);
 sprintf(sptext,"LDR read   Min:%4d bits  Max: %3d bits",MinPhotocell, MaxPhotocell);           Tekstprintln(sptext);
 sprintf(sptext,"Number of LEDs: %d   Palette: %d", NUM_LEDS, Mem.DisplayChoice);               Tekstprintln(sptext);  
 sprintf(sptext,"LEDs off between: %02d - %02d",Mem.TurnOffLEDsAtHH, Mem.TurnOnLEDsAtHH);       Tekstprintln(sptext);
 sprintf(sptext,"Software: %s",FILENAAM);                                                       Tekstprintln(sptext); 
 GetTijd(1);  
 PrintLine(40);
}
void PrintLine(byte Lengte)
{
 for (int n=0; n<Lengte; n++) Tekstprint("-"); Tekstprintln(""); //{Serial.print(F("-"));} Serial.println(); 
}

//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Update routine done every second
//------------------------------------------------------------------------------
void EverySecondCheck(void)
{
 static bool LEDhigh = false;  
 uint32_t msLeap = millis() - msTick;                                                         // A Digitalwrite() is very time consuming. 
 if (msLeap >970 && !LEDhigh) {digitalWrite(secondsPin,HIGH); LEDhigh = true;}                // Do not update the LED unnecessary often
 if(UseDCF) if(msLeap==(msLeap>>5)<<5) digitalWrite(DCF_LED_Pin,!digitalRead(DCF_PIN));       // Write received signal to the DCF LED every 32 loops (>>5 <<5)
 if (msLeap > 999)                                                                            // Every second enter the loop
  {
   msTick = millis();                                                                         // Set the colour per second of 'IS' and 'WAS' 
   GetTijd(0);                                                                                // Update I.Second, I.Minute, I.Hour, I.Day, I.Month, I.Year
   LEDhigh = false;
   digitalWrite(secondsPin,LOW);                                                              // Turn ON the second on pin     
                             #ifdef LCDMOD
   Print_tijd_LCD();
                             #endif  //LCDMOD    
//   if(I.Second == 30) DimLeds(true);                                                        // Text LED intensity control + seconds tick print every 30 seconds    
//   else DimLeds(false);                                                                     // Read the LDR and set intensity for the LEDs
                   #ifdef DCF77MOD
    if(DCFNoInt.getDCFDebugStatus()) Tekstprintln(DCFNoInt.getDebugInfo());                   // If a DCF information report line is available then print it
                   #endif // DCF77MOD
   DimLeds(TestLDR);                                                                          // Every second an intensitiy check and update from LDR reading 
   }
 // heartbeat();                                                                            // Only heartbeat with ATMEGA1284 or Nano Every
  if (I.Minute != lastminute)   EveryMinuteUpdate();                                          // Enter the every minute routine after one minute
 }

//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Update routine done every minute
//------------------------------------------------------------------------------
void EveryMinuteUpdate(void)
 {
  lastminute = I.Minute;  
  GetTijd(0);                                                                                 // Turn the LEDs on     
  DimLeds(true);                                                                              // measure the LDR and print the results
  Displaytime();                                                                              // Print the time in words
  Print_RTC_tijd();
                           #ifdef HT16K33tijd
 sprintf(sptext,"%02d:%02d",Inow.hour(),Inow.minute());
 for (int i=0; i<5; i++)  Tijd_displayIC2.writeDigitNum(i, sptext[i]- '0');
 Tijd_displayIC2.drawColon(true); 
 Tijd_displayIC2.writeDisplay();
                           #endif //HT16K33tijd
                    #ifdef DCFNOINT
  if(Mem.MinutesSinceStart > 0X0FFFFFF0)                                                      // Reset counters before they overflow
    { Mem.EdMinTijd = Mem.EdMin = Mem.ThMin = Mem.EdThMin = Mem.ValidTimes = Mem.MinutesSinceStart = 0; }
                     #endif  //DCFNOINT
 if (I.Hour != lasthour) EveryHourUpdate(); 
 }
//                                                                                            //
 //------------------------------------------------------------------------------
// CLOCK Update routine done every hour
//------------------------------------------------------------------------------
void EveryHourUpdate(void)
{
 if(I.Hour != lasthour) 
   {
    lasthour = min(I.Hour, 23);
    if(I.Hour == Mem.TurnOffLEDsAtHH) {LEDsAreOff = true;  DimLeds(false);}                   // is it time to turn off the LEDs?
    if(I.Hour == Mem.TurnOnLEDsAtHH)  LEDsAreOff = false;                                     // or on?
                    #ifdef DCF77MOD    
    Mem.NVRAMmem[lasthour] = DCF_counts;                                                      //
                    #else
    Mem.NVRAMmem[lasthour] =(int)(SumLDRreadshour / NoofLDRreadshour);                   // Update the average LDR readings per hour
                    # endif  //DCF77
//    EEPROM.put(0,Mem);                   
    SumLDRreadshour  = 0;
    NoofLDRreadshour = 0;
    DCF_counts       = 0;                                                                     // Valid DCF receptions per hour
    }
 if (I.Day != lastday) EveryDayUpdate(); 
}

//------------------------------------------------------------------------------
// CLOCK Update routine done every day
//------------------------------------------------------------------------------
void EveryDayUpdate(void)
{
 if(I.Day != lastday) 
   {
    lastday = I.Day; 
    Previous_LDR_read = analogRead(PhotoCellPin);                                             // to have a start value
    MinPhotocell      = Previous_LDR_read;                                                    // Stores minimum reading of photocell;
    MaxPhotocell      = Previous_LDR_read;                                                    // Stores maximum reading of photocell;
    EEPROM.put(0,Mem);                                                                        // This function uses EEPROM.update() to perform the write, so does not rewrites the value if it didn't change
                    #ifdef DCFNOINT
    DCFlocked = false;                                                                        // RTC time can be updated from DCF time  Tekstprintln("DCFlock is unlocked ");
   // Tekstprintln("DCFLock unlocked in EveryDayUpdate");
                    #endif  //DCFNOINT
    }
}
//------------------------------------------------------------------------------
// CLOCK check for input from devices
// This fubction is called from with the loop()
//------------------------------------------------------------------------------
//                                                                                            //
void InputDevicesCheck(void)
{
 SerialCheck();
                           #ifdef DCF77MOD
 if(UseDCF)  DCF77Check();                                                                    // More reads for DCFNoInt inproves accurary of signal
                           #endif  //DCF77MOD
                           #ifdef ROTARYMOD      
 RotaryEncoderCheck(); 
                           #endif  //ROTARYMOD
                           #ifdef KEYPAD3x4   
 Keypad3x4Check(); 
                           #endif  //KEYPAD3x4
                           #ifdef KEYPAD3x1   
 Keypad3x1Check(); 
                           #endif  //KEYPAD3x1
                           #ifdef ONEWIREKEYPAD3x4   
 OnewireKeypad3x4Check(); 
                           #endif  //ONEWIREKEYPAD3x4
                           #ifdef ONEWIREKEYPAD3x1   
 OnewireKeypad3x1Check(); 
                           #endif  //ONEWIREKEYPAD3x1
                           #ifdef HC12MOD
 HC12Check();
                           #endif  //HC12MOD 
                          #ifdef DCFNOINT
 if(UseDCF)  DCF77Check();                                                                    // More reads for DCFNoInt inproves accurary of signal
                           #endif  //DCFNOINT
 
                           #ifdef BLUETOOTHMOD   
 BluetoothCheck(); 
                           #endif  //BLUETOOTHMOD

                           #ifdef DCFNOINT
 if(UseDCF)  DCF77Check();                                                                    // More reads for DCFNoInt inproves accurary of signal
                           #endif  //DCFNOINT
}
//------------------------------------------------------------------------------
// CLOCK common print routines
//------------------------------------------------------------------------------
//                                                                                            //
void Tekstprint(char const *tekst)
{
 Serial.print(tekst);    
                          #ifdef BLUETOOTHMOD   
 Bluetooth.print(tekst);  
                          #endif  //BLUETOOTHMOD
}

void Tekstprintln(char const *tekst)
{
 strcpy(sptext,tekst);
 strcat(sptext,"\n");          //sprintf(sptext,"%s\n",tekst);
 Tekstprint(sptext);    
}

//------------------------------------------------------------------------------
// CLOCK Heart beat in LED
//------------------------------------------------------------------------------
void Heartbeat() 
{
 static byte hbval   = 128;                                                                   // Heartbeat initial intensity
 static byte hbdelta = 10;                                                                    // Determines how fast heartbeat is
 unsigned long now   = millis();
 if ((now - last_time) < 40)    return;
 last_time = now;
 if (hbval > 230 || hbval < 20 ) hbdelta = -hbdelta; 
 hbval += hbdelta;
 analogWrite(HeartbeatLED, hbval);
}
//------------------------------------------------------------------------------
// CLOCK Demo mode
//------------------------------------------------------------------------------
//                                                                                            //
void Demomode(void)
{
 if ( millis() - msTick == 10)   digitalWrite(secondsPin,LOW);                                // Turn OFF the second on pin 13
 if ( millis() - msTick >= MilliSecondValue)                                                  // Flash the onboard Pin 13 Led so we know something is happening
 {    
  msTick = millis();                                                                          // second++; 
  digitalWrite(secondsPin,HIGH);                                                              // Turn ON the second on pin 13
  I.Second = 60-I.Minute;
  if( ++I.Minute >59) { I.Minute = 0; I.Second = 0; I.Hour++;}
  if( I.Hour >12)                                                                             // If hour is after 12 o'clock 
   {    
    I.Hour = 9;                   
    if (++ Mem.DisplayChoice > WHEELCOLOR)  Mem.DisplayChoice = 0;                            // Start with a following DISPLAYCHOICE AT Nine oclock
   }
  DimLeds(false);
  Displaytime();
  Tekstprintln("");
  SerialCheck();
 }
}

//------------------------------------------------------------------------------
// CLOCK check for serial input
//------------------------------------------------------------------------------
void SerialCheck(void)
{
 String SerialString = "";
 while (Serial.available())
  {
   char c = Serial.read();  delay(3); 
   if (c>31 && c<127) SerialString += c;                            // allow input from Space - Del
   else c = 0;
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);  // Rework ReworkInputString();
 SerialString = "";
}
                           #ifdef HC12MOD
//------------------------------------------------------------------------------
// CLOCK check for HC-12 input
//------------------------------------------------------------------------------  
//                                                                                            //                          
void HC12Check(void)
{
 String HC12String = ""; 
 HC12String.reserve(64);
 HC12String ="";
 HC12.listen();                                                                               // When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (HC12.available()>0)                                                                   // If HC-12 has data
   {       
    char c = HC12.read();     
    if (c>31 && c<127)  HC12String += c;
    else c = 0;                                                                               // Allow only input from Space - Del
    delay(3);
   }
 HC12String += "\n";
 if (HC12String.length()>0) 
   {
    Serial.print("Received HC-12: "); Serial.println(HC12String);
    ReworkInputString(HC12String);
    }                  
}                         
                           #endif  //HC12MOD
                           #ifdef BLUETOOTHMOD
//------------------------------------------------------------------------------
// CLOCK check for Bluetooth input
//------------------------------------------------------------------------------                           
void BluetoothCheck(void)
{ 
 String  BluetoothString = "";
 char c = 0;
 Bluetooth.listen();                                                                          //  When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (Bluetooth.available()) 
  { 
   c = Bluetooth.read(); //  Serial.print(c);
   if (c>31 && c<127) BluetoothString += c;
   else c = 0;
   delay(3);
  }
 if (BluetoothString.length()>0) 
   {   
    ReworkInputString(BluetoothString);                                                       // Rework ReworkInputString();
    BluetoothString = "";
   }
}
                           #endif  //BLUETOOTHMOD
                           #ifdef ONEWIREKEYPAD3x4
//------------------------------------------------------------------------------
// KEYPAD check for Onewire Keypad input
//------------------------------------------------------------------------------
//                                                                                            //
void OnewireKeypad3x4Check(void)
{
 byte keyvalue;
 char Key;
 int sensorValue = analogRead(OneWirePin);                                                    // read the value from the sensor:
 switch(sensorValue)
  {
    case   0 ... 100:  keyvalue = 13; break;   // noise
    case 101 ... 132:  keyvalue = 12; Key = '*'; break;   // * 
    case 133 ... 154:  keyvalue =  0; Key = '0'; break;   // 0 
    case 155 ... 216:  keyvalue = 11; Key = '#'; break;   // # 
    case 217 ... 281:  keyvalue =  7; Key = '7'; break;   // 7 
    case 282 ... 318:  keyvalue =  4; Key = '4'; break;   // 4 
    case 319 ... 349:  keyvalue =  1; Key = '1'; break;   // 1 
    case 350 ... 390:  keyvalue =  8; Key = '8'; break;   // 8 
    case 391 ... 463:  keyvalue =  5; Key = '5'; break;   // 5 
    case 464 ... 519:  keyvalue =  2; Key = '2'; break;   // 2 
    case 520 ... 619:  keyvalue =  9; Key = '9'; break;   // 9 
    case 620 ... 848:  keyvalue =  6; Key = '6'; break;   // 6 
    case 849 ... 1023: keyvalue =  3; Key = '3'; break;   // 3
  }
 if(keyvalue<13) { Serial.println(Key); delay(300); }
  if (Key == 12)   // *                                                                       // Pressing a * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString ="";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                                                      // Turn all LEDs green
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>=0 && Key<10))
   {
    delay(20); 
    KeypadString += Key;                                                                      // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                                          // Turn all LEDs red
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if (KeypadString.length()>5)                                                                 // If six numbers are entered rework this to a time hhmmss
   {       
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock settings resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                                                        // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }    
   }
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000) ) 
   {  
    KeyInputactivated = false;                                                                // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
}
                           #endif  //ONEWIREKEYPAD3x4  
                           #ifdef ONEWIREKEYPAD3x1
//--------------------------------------------
// KEYPAD check for Onewire 
// Keypad input with 5V and 1.1, 4.7, 4.7, 4.7 kOhm resistors
//--------------------------------------------
void OnewireKeypad3x1Check(void)
{
 char keyvalue, Key;
 int sensorValue = analogRead(OneWirePin);                                                    // Read the value from the sensor:
 switch(sensorValue)
   {
    case   0 ... 385:  keyvalue = 99;            break;                                       // Noise
    case 386 ... 635:  keyvalue = -1; Key = 'G'; break;                                       // G 
    case 636 ... 910:  keyvalue =  0; Key = 'Y'; break;                                       // Y 
    case 911 ... 1024: keyvalue =  1; Key = 'R'; break;                                       // R 
   }
 if(keyvalue<2) 
    { 
     Serial.print(sensorValue); Serial.println(Key); 
     if (Key == 'R') ProcessKeyPressTurn(1);                                                  // Pressing Red increases hour or minute. 
     if (Key == 'G') ProcessKeyPressTurn(-1);                                                 // Pressing Green decreases hour or minute. 
     if (Key == 'Y') ProcessKeyPressTurn(0);                                                  // Pressing Yellow activates the keyboard input. 
     delay(200);     
    }
}
                           #endif //ONEWIREKEYPAD3x1
                           #ifdef KEYPAD3x4
 //------------------------------------------------------------------------------
// KEYPAD check for Keypad input
//------------------------------------------------------------------------------ 
//                                                                                            //                          
void Keypad3x4Check(void)
{ 
 char Key = 0;
 Keypad3x4.tick(); 
 while(Keypad3x4.available())
  {
   keypadEvent e = Keypad3x4.read();  
   if(e.bit.EVENT == KEY_JUST_PRESSED)
     {                                                                                        // Serial.println(F(" pressed"));  
      delay(20);
     }  
   else  if(e.bit.EVENT == KEY_JUST_RELEASED) 
     {
      Key = (char) e.bit.KEY;                                                                 // Serial.print(Key);  Serial.println(F(" released"));
      Keypad3x4.clear();
      delay(20);
     }
   }
 if (Key == 42)   // *                                                                        // Pressing * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString = "";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                                                      // Turn all LEDs green
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>47 && Key<58))
   {
    delay(20); 
    KeypadString += Key;                                                                      // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                                          // Turn all LEDs red
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if(KeypadString.length()>5)                                                                  // If six numbers are entered rework this to a time hhmmss
   {  
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock setting resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                                                        // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }
   }
  if (Key == 35)   // #                                                                       // Pressing # changes palettes. 
   { 
    KeypadString ="";
    Mem.DisplayChoice++;
    Mem.DisplayChoice = min(Mem.DisplayChoice, 9);
    Displaytime();
   }
   
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000)                                 // Stop keyboard entry after 30 seconds
   {  
    KeyInputactivated = false;                                                                // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
} 
                           #endif  //KEYPAD3x4   
//                                                                                            //
                           #ifdef KEYPAD3x1
 //------------------------------------------------------------------------------
// KEYPAD check for Keypad input
//------------------------------------------------------------------------------                           
void Keypad3x1Check(void)
{ 
 char Key = 0;
 Keypad3x1.tick(); 
 while(Keypad3x1.available())
  {
   keypadEvent e = Keypad3x1.read();  
   Serial.print((char)e.bit.KEY);
   Key = (char) e.bit.KEY;   
   if(e.bit.EVENT == KEY_JUST_PRESSED) {Serial.println(F(" pressed"));}  
   if(e.bit.EVENT == KEY_JUST_RELEASED){Serial.println(F(" released"));}
   }
 if (Key == 'G') ProcessKeyPressTurn(-1);                                                     // Pressing Yellow activates the keyboard input. 
 if (Key == 'Y') ProcessKeyPressTurn( 0);                                                     // Pressing Yellow activates the keyboard input. 
 if (Key == 'R') ProcessKeyPressTurn( 1);                                                     // Pressing Yellow activates the keyboard input. 
 Keypad3x1.clear();                                                                           // and clear the keyboard buffer
 delay(200);
} 
                           #endif  //KEYPAD3x1      

                           #ifdef ROTARYMOD
//------------------------------------------------------------------------------
// KY-040 ROTARY check if the rotary is moving  
//------------------------------------------------------------------------------
void RotaryEncoderCheck(void)
{
 int ActionPress = 999;
 if (digitalRead(clearButton) == LOW )          ProcessKeyPressTurn(0);                       // Set the time by pressing rotary button
 else if (ChangeTime)    
  {   
   ActionPress = myEnc.read();                                                                // If the knob is turned store the direction (-1 or 1)
   if (ActionPress == 0) {  ActionPress = 999;  ProcessKeyPressTurn(ActionPress);  }          // Sent 999 = nop 
   if (ActionPress == 1 || ActionPress == -1 )  ProcessKeyPressTurn(ActionPress);             // Process the ActionPress
  } 
 myEnc.write(0);                                                                              // Set encoder pos back to 0
}
                           #endif  //ROTARYMOD  
//------------------------------------------------------------------------------
// CLOCK
// KY-040 or Membrane 3x1 processing input
// encoderPos < 1 left minus 
// encoderPos = 0 attention and selection choice
// encoderPos > 1 right plus
//------------------------------------------------------------------------------
//                                                                                            //
void ProcessKeyPressTurn(int encoderPos)
{
if ((unsigned long) (millis() - RotaryPressTimer) > 60000)                                    // After 60 sec after shaft is pressed time of light intensity can not be changed 
   {
    if (ChangeTime || ChangeLightIntensity)                         
      {
        Tekstprintln("<-- Changing time is over -->");
        NoofRotaryPressed = 0;
      }
    ChangeTime            = false;
    ChangeLightIntensity  = false;
   }  
 if (ChangeTime || ChangeLightIntensity)                                                      // If shaft is pressed time of light intensity can be changed
   {
    if ( encoderPos!=999 && ( (millis() - Looptime) > 250))                                   // If rotary turned avoid debounce within 0.25 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos == 1)                                                                     // Increase  
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(5); }                                  // If time < 60 sec then adjust light intensity factor
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                                                        // Change hours
              {if( ++I.Hour >23) { I.Hour = 0; } }      
           if (NoofRotaryPressed == 2)                                                        // Change minutes
              {  I.Second = 0;
               if( ++I.Minute >59) { I.Minute = 0; if( ++I.Hour >23) { I.Hour = 0; } }   }
           } 
        }    
      if (encoderPos == -1)                                                                   // Decrease
       {
       if (ChangeLightIntensity)   { WriteLightReducer(-5); }    // If time < 60 sec then adjust light intensity factor
       if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                                                        // Change hours
            { if( I.Hour-- ==0) { I.Hour = 23; } }      
           if (NoofRotaryPressed == 2)                                                        // Change minutes
            { I.Second = 0;
             if( I.Minute-- == 0) { I.Minute = 59; if( I.Hour-- == 0) { I.Hour = 23; } }  }
          }          
        } 
      SetRTCTime();  
      Print_RTC_tijd();
      Looptime = millis();       
     }                                                
   }
 if (encoderPos == 0 )                                                                        // Set the time by pressing rotary button
   { 
    delay(250);
    ChangeTime            = false;
    ChangeLightIntensity  = false;
    RotaryPressTimer      = millis();                                                         // Record the time the shaft was pressed.
    if(++NoofRotaryPressed >10 ) NoofRotaryPressed = 0;
    switch (NoofRotaryPressed)                                                                // No of times the rotary is pressed
      {
       case 1:  ChangeTime = true;            BlinkUUR(3, 20);      break;                    // Change the hours
       case 2:  ChangeTime = true;            BlinkHETISWAS(3, 20); break;                    // Change the hours        
       case 3:  ChangeLightIntensity = true;  BlinkLetters(5, 10);  break;                    // Turn on all LEDs and change intensity 
       case 4:  Mem.DisplayChoice = DEFAULTCOLOUR;                  break;
       case 5:  Mem.DisplayChoice = HOURLYCOLOUR;                   break;        
       case 6:  Mem.DisplayChoice = WHITECOLOR;                     break;
       case 7:  Mem.DisplayChoice = OWNCOLOUR;                      break;
       case 8:  Mem.DisplayChoice = OWNHETISCLR;                    break;
       case 9:  Mem.DisplayChoice = WHEELCOLOR;                     break;    
       case 10:                                                     break; 
       case 11:
       case 12:
       case 13:                                                     break;
       case 14: Reset();                                            break;                     
       default: NoofRotaryPressed = 0; 
                Mem.DisplayChoice = DEFAULTCOLOUR;
                ChangeTime = ChangeLightIntensity  = false;  
                Selftest();        
                break;                         
      }
    Serial.print(F("NoofRotaryPressed: "));   Serial.println(NoofRotaryPressed);   
    sprintf(sptext,"Display choice stored: Q%d", Mem.DisplayChoice);    Tekstprintln(sptext);  
    Looptime = millis();     
    Displaytime();                                                                            // Turn on the LEDs with proper time
   }
 }

//                                                                                            //
//------------------------------------------------------------------------------
//  CLOCK Blink UUR
//------------------------------------------------------------------------------
void BlinkUUR(int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); UUR; Laatzien(); delay(Delayms);} 
}
//------------------------------------------------------------------------------
//  CLOCK Blink HET IS WAS
//------------------------------------------------------------------------------
void BlinkHETISWAS (int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); HET; IS; WAS; Laatzien(); delay(Delayms);} 
}
//------------------------------------------------------------------------------
//  CLOCK Blink Letters
//------------------------------------------------------------------------------
void BlinkLetters (int NoofBlinks, int Delayms)
{
 for(int n=0 ; n<=NoofBlinks; n++) 
    { 
      LedsOff(); 
      Laatzien(); delay(Delayms); 
      ColorLeds("",  36, 59, 0XFF0000FF);  
      Laatzien(); delay(Delayms);
    } 
}
//------------------------------------------------------------------------------
// CLOCK Self test sequence
//------------------------------------------------------------------------------
void Selftest(void)
{ 
 GetTijd(1);                                                                                  // Prints time in Serial monitor
 LedsOff(); 
 HET;   Laatzien(); IS;    Laatzien(); WAS;    Laatzien(); PRECIES; Laatzien(); MTIEN;  Laatzien();  MVIJF; Laatzien();    
 KWART; Laatzien(); VOOR;  Laatzien(); OVER;   Laatzien(); HALF;    Laatzien(); MIDDER; Laatzien(); VIJF;   Laatzien();
 TWEE;  Laatzien(); EEN;   Laatzien(); VIER;   Laatzien(); TIEN;    Laatzien(); TWAALF; Laatzien(); DRIE;   Laatzien();
 NEGEN; Laatzien(); NACHT; Laatzien(); ACHT;   Laatzien(); ZES;     Laatzien(); ZEVEN;  Laatzien();  ELF;   Laatzien(); 
 NOEN;  Laatzien(); UUR;   Laatzien(); EDSOFT; Laatzien();
 Tekstprintln("*");    
 Play_Lights();    
 Zelftest = false; 
 Displaytime();                                                                               // Turn on the LEDs with proper time
}

//------------------------------------------------------------------------------
// CLOCK Say the time and load the LEDs 
// with the proper colour and intensity
//------------------------------------------------------------------------------
void Displaytime(void)
{ 
 SetMinuteColour();                                                                           // Set the colour per second of 'IS' and 'WAS'       
 Dutch(); 
 ShowLeds();                                                                                  // and turn on the LEDs
}
//--------------------------- Time functions --------------------------
//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK utility function prints time to serial
//------------------------------------------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%02d:%02d:%02d",I.Hour,I.Minute,I.Second);
 Tekstprintln(sptext);
}
                    #ifdef LCDMOD
//------------------------------------------------------------------------------
// LCD Print time to LCD display
//------------------------------------------------------------------------------
void Print_tijd_LCD(void)
{
 lcd.home (); // set cursor to 0,0
 sprintf(sptext,"%02d:%02d:%02d",Inow.hour(),Inow.minute(),Inow.second());  lcd.print(sptext);
 sprintf(sptext," LDR%d   ",analogRead(PhotoCellPin));                      lcd.print(sptext);
 lcd.setCursor (0,1);        // go to start of 2nd line
 sprintf(sptext,"%02d-%02d-%04d",Inow.day(),Inow.month(),Inow.year());      lcd.print(sptext);
 sprintf(sptext," DCF%d   ",DCF_signal);                                    lcd.print(sptext);
}
                    #endif  //LCDMOD


//----------------------- RTC DS3231 functions  -------------------------
//                                                                                            //
//------------------------------------------------------------------------------
// DS3231 Adjust time in RTC with DCF time from tmElements_t Dtime struct
//------------------------------------------------------------------------------
void AdjustRTC(int TimeDiff, tmElements_t Dtime, char const *tekst) 
{
 sprintf(sptext,"%d sec difference RTC:%02d:%02d:%02d %7s:%02d:%02d:%02d ----> time updated ",
     TimeDiff, I.Hour, I.Minute,I.Second,tekst, Dtime.Hour, Dtime.Minute, Dtime.Second); 
 Tekstprintln(sptext);    
 RTCklok.adjust(DateTime(Dtime.Year, Dtime.Month, Dtime.Day, Dtime.Hour, Dtime.Minute, 0));   // Update time here if DCFNOINT is not used otherwise both receiver must received the same date-time
}

//------------------------------------------------------------------------------
// DS3231 Return time from RTC module in seconds since 1-1-2000
//------------------------------------------------------------------------------
time_t GetSecondsSince2000TimeRTC(void)
{
 DateTime RTCtm = RTCklok.now();                                                              // This give the time from 1970
 time_t RTCtime = RTCtm.unixtime() - SECONDS_FROM_1970_TO_2000 - SECS_PER_DAY;                // These are the seconds since 2000
 //  Serial.print("===> RTCtime: "); Serial.println(RTCtime);
 //  sprintf(sptext,"  RTC OK --> %02d:%02d %02d-%02d-%04d ",
 //      RTCtm.hour(), RTCtm.minute(), RTCtm.day(), RTCtm.month(), RTCtm.year());   Tekstprintln(sptext); 
 return(RTCtime);                                                                             // Seconds since 1-1-2000
}
//------------------------------------------------------------------------------
// DS3231 Get time from DS3231
//------------------------------------------------------------------------------
void GetTijd(byte printit)
{
 Inow    = RTCklok.now();
 I.Hour   = min(Inow.hour()  , 24);
 I.Minute = min(Inow.minute(), 59);
 I.Second = min(Inow.second(), 59);
 I.Day    = Inow.day();
 I.Month  = Inow.month();
 I.Year   = Inow.year()-2000;
 I.Wday   = Inow.dayOfTheWeek();
// if (I.Hour > 24) { I.Hour = random(12)+1; I.Minute = random(60)+1; I.Second = 30;}          // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}


//------------------------------------------------------------------------------
// DS3231 utility function prints time to serial
//------------------------------------------------------------------------------
void Print_RTC_tijd(void)
{
 Inow = RTCklok.now();
 sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",
     Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
}
//------------------------------------------------------------------------------
// DS3231 Set time in module and print it
//------------------------------------------------------------------------------
//                                                                                            //
void SetRTCTime(void)
{ 
 I.Hour   = min((byte) I.Hour  , 24);
 I.Minute = min((byte) I.Minute, 59); 
 I.Second = min((byte) I.Second, 59); 
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), I.Hour, I.Minute, I.Second));
 GetTijd(0);                                                                                  // Synchronize time with RTC clock
 Displaytime();
 Print_tijd();
}
//                                                                                            //
                           #if defined DCFNOINT || defined DCFTH
//------------------------------------------------------------------------------
// DCF check for DCF input.
// This subroutine assumes that both DCF-receivers
// are defined with #define DCFTH and DCFNOINT
//------------------------------------------------------------------------------
//                                                                                            //
void DCF77Check(void)
{
 time_t static LastDCFNoIntSecs=0, LastDCF77Seconds=0;
 DCFEd = true;                                                                                // Necessary for DCFlocking 
 DCFTh = true;                                                                                // If both DCFEd and DCFTH are true RTC time is set and locked for change until unlocked
                              #ifdef DCFTH
 time_t  TimeDCF77Seconds = GetSecondsSince2000DCF77();                                       // Seconds since 1-1-2000
 DCFTh = false;
 if(TimeDCF77Seconds) 
   {
    LastDCF77Seconds = TimeDCF77Seconds;                                                      // Remember the recorded DCF time
    Mem.ThMin++;                                                                              // Add  
    DCFTh = true;                                                                             // Used to check if DCF77 and DCFNOINT receive both a good time    
   }
                             #endif  //DCFTH   
                             #ifdef DCFNOINT 
  time_t TimeDCFNoIntSecs = GetSecondsSince2000DCFNoInt();                                    // Decode the signals and retrieve seconds since 1-1-2000 if we have received a proper time after 59 secs
  DCFEd = false;
  if(TimeDCFNoIntSecs)
    {
     LastDCFNoIntSecs = TimeDCFNoIntSecs;                                                     // Remember the recorded DCF time
     Mem.EdMin++;                                                                             // in Seconds. Differences less than 1.5 hour with RTC  are changed unchecked 
     DCFEd = true;                                                                            // Used to check if DCF77 and DCFNoInt receive both a good time 
    }          
                             #endif  //DCFNOINT 
  if(LastDCFNoIntSecs && LastDCF77Seconds) 
        CheckValidityDCF(LastDCFNoIntSecs,LastDCF77Seconds);                                  // Compare the last recorded DCF times and update RTC if necessary
}
//                                                                                            //
//------------------------------------------------------------------------------
// DCF77 DCFNoInt Check and count valid & invalid 
// DCF decodings,  update RTC if necessary and
// DCFlocked=true to prevent updating RTC.
//------------------------------------------------------------------------------
void CheckValidityDCF(time_t NoIntSecs, time_t DCF77Secs)
{

if (DCFEd && DCFTh) { DCF_signal++;  Mem.ValidTimes++;    DCF_counts++;}
 else  DCF_signal--;  
 DCF_signal = constrain(DCF_signal,1,99);
 DCF_signal>25?digitalWrite(DCFgood,HIGH):digitalWrite(DCFgood,LOW) ;

 if(DCFEd && DCFTh)                                                                           // Check for invalid reported DCF date/times
   {  
    time_t TimeRTCSecs = GetSecondsSince2000TimeRTC(); 
    time_t SecDiffDCF  = abs(NoIntSecs - DCF77Secs);
    if(DCFlocked && DCFEd && (abs(NoIntSecs - TimeRTCSecs >10)))
       {
        Mem.EdWrong++;                                                                        // The DCFNoInt was false  
        PrintTijd("DCFNoInt: ",NoIntSecs,1);
        PrintTijd("     RTC: ",TimeRTCSecs,1);
       }     
    if(DCFlocked && DCFTh && (abs(DCF77Secs - TimeRTCSecs >10)))
      { 
        Mem.ThWrong++;                                                                        // The DCF77 was false. IF DCFlocked then RTC is te right time 
        PrintTijd("   DCF77: ",DCF77Secs,1);
        PrintTijd("     RTC: ",TimeRTCSecs,1);
      }
  //  Serial.print("===>   RTCtime: "); Serial.println(TimeRTCSecs);    Serial.print("===> DCF77time: "); Serial.println(DCF77Secs);    Serial.print("===> NoInttime: "); Serial.println(NoIntSecs);
    if(SecDiffDCF==0)                                                                         // Allow one second difference
      {
       tmElements_t DCF;                                                                      // Initialyse a time struct
       breakTime(NoIntSecs, DCF);                                                             // Store the time_t DCFtime in struct DCFnitm (no interrupt time)
       if(!DCFlocked) AdjustRTC(SecDiffDCF, DCF, "Both DCF");
       Mem.EdThMin++;                                                                         // Count the times both DCFalgorithm received a good time. 
       DCFlocked = true;                                                                      // Lock changing the time because the two DCF algoritm wee identical until unlocked
 //    Tekstprintln("DCFlock = true"); 
      } 
   }                                  
}      
//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Print time from seconds since 1-1-2000  
//       and return time struct
//------------------------------------------------------------------------------
 tmElements_t PrintTijd(char const *tekst, time_t Seconds2000, bool printit)  
 {
  tmElements_t TijdReturn; 
  breakTime(Seconds2000,TijdReturn );  
  if(printit) 
   {
    sprintf(sptext,"%s,%02d:%02d:%02d %02d-%02d-%04d/%01d", 
           tekst,TijdReturn.Hour, TijdReturn.Minute, TijdReturn.Second, TijdReturn.Day, TijdReturn.Month, TijdReturn.Year+2000, TijdReturn.Wday);  Tekstprintln(sptext);
   }
 return(TijdReturn); 
 }  
//                                                                                            //
                             #ifdef DCFTH 
//------------------------------------------------------------------------------
// DCF77  Return time in seconds since 1-1-2000 
//  with Arduino DCF77 library
//------------------------------------------------------------------------------
time_t GetSecondsSince2000DCF77(void)
{
 time_t DCFtime = DCF.getTime() ;                                                             // Check if new DCF77 time is available
 if (DCFtime!=0)
  {
   setTime(DCFtime);                                                                          // Needed for DCF77 lib  
   DCFtime -= (SECONDS_FROM_1970_TO_2000 + SECS_PER_DAY);                                     // Seconds seems one day off. Why?
 //  Serial.print("===> DCFtime: "); Serial.println(DCFtime);
 //  sprintf(sptext,"  DCF77 Lib OK --> %02d:%02d %02d-%02d-%04d ",DCFtm.Hour, DCFtm.Minute, DCFtm.Day, DCFtm.Month, DCFtm.Year+1970);   Tekstprintln(sptext); 
  }
 return(DCFtime);                                                                             // Seconds since 1-1-2000
}
                             #endif  //DCFTH 
                             #ifdef DCFNOINT
time_t GetSecondsSince2000DCFNoInt(void)
{
  return(DCFNoInt.getTime(I));
}
                             #endif  //DCFNOINT
//                                                                                            //     
                             #endif  //defined DCFNOINT || defined DCFTH

// --------------------Colour Clock Light functions -----------------------------------
//------------------------------------------------------------------------------
//  LED Set color for LEDs in strip and print tekst
//------------------------------------------------------------------------------
void ColorLeds(char const *Tekst, int FirstLed, int LastLed, uint32_t RGBWColor)
{ 
 Stripfill(RGBWColor, FirstLed, ++LastLed - FirstLed );                                        //
 if (!NoTextInColorLeds && strlen(Tekst) > 0 )
     {sprintf(sptext,"%s ",Tekst); Tekstprint(sptext); }                                      // Print the text  
}
//--------------------------------------------
//  LED Set color for one LED
//--------------------------------------------
void ColorLed(int Lednr, uint32_t RGBWColor)
{   
 Stripfill(RGBWColor, Lednr, 1 );
}
//------------------------------------------------------------------------------
//  LED Clear display settings of the LED's
//------------------------------------------------------------------------------
void LedsOff(void) 
{ 
 Stripfill(0, 0, NUM_LEDS );  
}
//------------------------------------------------------------------------------
// LED Turn On and Off the LED's after Delaytime is milliseconds
//------------------------------------------------------------------------------
void Laatzien()
{ 
 ShowLeds();
 delay(Delaytime);
 LedsOff(); 
 InputDevicesCheck();                                                                         // Check for input from input devices
}

//------------------------------------------------------------------------------
//  LED Push data in LED strip to commit the changes
//------------------------------------------------------------------------------
void ShowLeds(void)
{
 LEDstrip.show();
}
//------------------------------------------------------------------------------
//  LED Set brighness of LEDs
//------------------------------------------------------------------------------  
void SetBrightnessLeds(byte Bright)
{
 LEDstrip.setBrightness(Bright);                                           // Set brightness of LEDs   
 ShowLeds();
}
//--------------------------------------------
//  LED Fill the strip array for LEDFAB library
//--------------------------------------------
void Stripfill(uint32_t RGBWColor, int FirstLed, int NoofLEDs)
{   
 LEDstrip.fill(RGBWColor, FirstLed, NoofLEDs);
}
//--------------------------------------------
//  LED Strip Get Pixel Color 
//--------------------------------------------
uint32_t StripGetPixelColor(int Lednr)
{
return(LEDstrip.getPixelColor(Lednr));
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED convert HSV to RGB  h is from 0-360, s,v values are 0-1
//  r,g,b values are 0-255
//------------------------------------------------------------------------------
uint32_t HSVToRGB(double H, double S, double V) 
{
  int i;
  double r, g, b, f, p, q, t;
  if (S == 0)  {r = V;  g = V;  b = V; }
  else
  {
    H >= 360 ? H = 0 : H /= 60;
    i = (int) H;
    f = H - i;
    p = V * (1.0 -  S);
    q = V * (1.0 - (S * f));
    t = V * (1.0 - (S * (1.0 - f)));
    switch (i) 
    {
     case 0:  r = V;  g = t;  b = p;  break;
     case 1:  r = q;  g = V;  b = p;  break;
     case 2:  r = p;  g = V;  b = t;  break;
     case 3:  r = p;  g = q;  b = V;  break;
     case 4:  r = t;  g = p;  b = V;  break;
     default: r = V;  g = p;  b = q;  break;
    }
  }
return FuncCRGBW((int)(r*255), (int)(g*255), (int)(b*255), 0 );                                // R, G, B, W 
}
//------------------------------------------------------------------------------
//  LED function to make RGBW color
//------------------------------------------------------------------------------ 
uint32_t FuncCRGBW( uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t White)
{ 
 return ( (White<<24) + (Red << 16) + (Green << 8) + Blue );
}
//------------------------------------------------------------------------------
//  LED functions to extract RGBW colors
//------------------------------------------------------------------------------ 
 uint8_t Cwhite(uint32_t c) { return (c >> 24);}
 uint8_t Cred(  uint32_t c) { return (c >> 16);}
 uint8_t Cgreen(uint32_t c) { return (c >> 8); }
 uint8_t Cblue( uint32_t c) { return (c);      }
//                                                                                            //
//------------------------------------------------------------------------------
//  LED Set second color
//  Set the colour per second of 'IS' and 'WAS'
//------------------------------------------------------------------------------
void SetMinuteColour(void)
{
 MINColor = FuncCRGBW(15 + I.Minute * 4, 255 - I.Minute * 4,0,0); 
 SECColor = FuncCRGBW(15 + I.Second * 4, 255 - I.Second * 4,0,0 );   
                                                                                              // Light up  IS or WAS with the proper colour  
 switch (Mem.DisplayChoice)
  {
   case DEFAULTCOLOUR: LetterColor = DefaultColor;                                     break; // Yellow text with changing MIN and SEC  
   case HOURLYCOLOUR : LetterColor = HourColor[I.Hour];                                break; // A colour every hour
   case WHITECOLOR   : LetterColor = MINColor = SECColor = WhiteColour;                break; // all white
   case OWNCOLOUR    : LetterColor = Mem.OwnColour;                                    break; // own colour
   case OWNHETISCLR  : LetterColor = Mem.OwnColour; MINColor = SECColor = LetterColor; break; // own colour except HET IS WAS  
   case WHEELCOLOR   : LetterColor = MINColor = SECColor = Wheel((I.Minute*4));        break; // Colour of all letters changes per second
  }
 NoTextInColorLeds  = true;                                                                   // Flag to control printing of the text IS en WAS in serial
 if(Is) {IS;} 
 else {WAS;} 
 NoTextInColorLeds  = false;                                                                  // Flag to control printing of the text IS en WAS in serial
 ShowLeds();                                                                                  // Updating IS and WAS with ShowLeds is done here to avoid updating all letters every second with Displaytime function
//    Serial.print("SecColor: ");    Serial.println(SECColor,HEX);  
}

//------------------------------------------------------------------------------
//  LED Set second for 4 language clock
//------------------------------------------------------------------------------
void SetColours(void)
{  
 switch (Mem.DisplayChoice)
  {
   case DEFAULTCOLOUR: LetterColor   = DefaultColor;            break;                       // HET IS WAS changing   
   case HOURLYCOLOUR : LetterColor   = HourColor[I.Hour];       break;                       // A colour every hour   
   case WHITECOLOR   : LetterColor   = WhiteColour;             break;                       // All white
   case OWNCOLOUR    : LetterColor   = Mem.OwnColour;           break;   
   case OWNHETISCLR  : LetterColor   = Mem.OwnColour;           break;                       // Own colour HET IS WAS changing  
   case WHEELCOLOR   : LetterColor   = Wheel((I.Minute*4));     break;                       // Colour of all letters changes per second
  }
}

//--------------------------------------------
//  LED Dim the leds measured by the LDR and print values
// LDR reading are between 0 and 1024. The Brightness send to the LEDs is between 0 and 255
// If the LDR reading (0-1024) is immedialtly divided by 4 the results will become too low in the dark
// causing visible flikkering between changes (ie 1 and 2, ..., 4 and 5) 
//--------------------------------------------
void DimLeds(bool print) 
{                                                                                                       
  int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;                     // Read lightsensor (0-1024)and avoid rapid light intensity changes
  Previous_LDR_read = LDR_read;                                                              // by using the previous reads
  OutPhotocell = (int)( (Mem.LightReducer * sqrt(1023*(long)LDR_read))/100);                 // Linear --> hyperbolic with sqrt. (0-1024)
  MinPhotocell = min(MinPhotocell, LDR_read);                                                // Lowest LDR measurement (0-1024)
  MaxPhotocell = max(MaxPhotocell, LDR_read);                                                // Highest LDR measurement (0-1024)
  OutPhotocell = constrain(OutPhotocell, Mem.LowerBrightness, Mem.UpperBrightness);          // Keep result between lower and upper boundery ( 0-1024)
  SumLDRreadshour += LDR_read;    NoofLDRreadshour++;                                        // For statistics LDR readings per hour
  if(print)
  {
   sprintf(sptext,"LDR:%3d Avg:%3d",analogRead(PhotoCellPin),LDR_read); Tekstprint(sptext);
   sprintf(sptext," (%3d-%3d)",MinPhotocell,MaxPhotocell);              Tekstprint(sptext);
   sprintf(sptext," Out:%3d", OutPhotocell);                            Tekstprint(sptext);
   sprintf(sptext,"=%2d%%",(int)(BrightnessCalcFromLDR / 10));          Tekstprint(sptext);
                 #ifdef MOD_DS3231
   sprintf(sptext," Temp:%2dC ",get3231Temp()-3);                       Tekstprint(sptext);   // Correct the reported temperature 
                 #endif // MOD_DS3231
   Print_tijd();  
  }
 if(LEDsAreOff) OutPhotocell = 0;
 SetBrightnessLeds((byte) (OutPhotocell/4));                                                  // Set brighness of the LEDs (0-255)
}
//------------------------------------------------------------------------------
//  LED Turn On en Off the LED's
//------------------------------------------------------------------------------
void Play_Lights()
{
 for(int i=0; i<NUM_LEDS; i++) { ColorLeds("",i,i,chromeyellow); ShowLeds(); }
 WhiteOverRainbow(0, 0, 5 );
 WhiteOverRainbow(0,0, 5 );  // wait, whiteSpeed, whiteLength
 LedsOff();
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED Wheel
//  Input a value 0 to 255 to get a color value.
//  The colours are a transition r - g - b - back to r.
//------------------------------------------------------------------------------
uint32_t Wheel(byte WheelPos) 
{
 WheelPos = 255 - WheelPos;
 if(WheelPos < 85)   { return FuncCRGBW( 255 - WheelPos * 3, 0, WheelPos * 3, 0);  }
 if(WheelPos < 170)  { WheelPos -= 85;  return FuncCRGBW( 0,  WheelPos * 3, 255 - WheelPos * 3, 0); }
 WheelPos -= 170;      
 return FuncCRGBW(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}

//------------------------------------------------------------------------------
//  LED RainbowCycle
// Slightly different, this makes the rainbow equally distributed throughout
//------------------------------------------------------------------------------
void RainbowCycle(uint8_t wait) 
{
  uint16_t i, j;
  for(j=0; j<256 * 5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< NUM_LEDS; i++) 
      ColorLeds("",i,i,Wheel(((i * 256 / NUM_LEDS) + j) & 255));
      ShowLeds();
      delay(wait);
  }
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED WhiteOverRainbow
//------------------------------------------------------------------------------
void WhiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength ) 
{
  if(whiteLength >= NUM_LEDS) whiteLength = NUM_LEDS - 1;
  int head = whiteLength - 1;
  int tail = 0;
  int loops = 1;
  int loopNum = 0;
  static unsigned long lastTime = 0;
  while(true)
  {
    for(int j=0; j<256; j++) 
     {
      for(int i=0; i<NUM_LEDS; i++) 
       {
        if((i >= tail && i <= head) || (tail > head && i >= tail) || (tail > head && i <= head) )
              ColorLeds("",i,i,0XFFFFFF );
        else  
              ColorLeds("",i,i,Wheel(((i * 256 / NUM_LEDS) + j) & 255));
       }
      if(millis() - lastTime > whiteSpeed) 
       {
        head++;        tail++;
        if(head == NUM_LEDS) loopNum++;
        lastTime = millis();
      }
      if(loopNum == loops) return;
      head %= NUM_LEDS;
      tail %= NUM_LEDS;
      ShowLeds();
      delay(wait);
    }
  }  // end while
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED In- or decrease light intensity value i.e. Slope
//------------------------------------------------------------------------------
void WriteLightReducer(int amount)
{
 int value = Mem.LightReducer + amount;                                                       // Prevent byte overflow by making it an integer before adding
 Mem.LightReducer = (byte) min(value,  255);                                                         // May not be larger than 255
 sprintf(sptext,"Max brightness: %3d%%",Mem.LightReducer);
 Tekstprintln(sptext);
}

// --------------------End Light functions 
//------------------------------------------------------------------------------
//  CLOCK Constrain a string with integers
// The value between the first and last character in a string is returned between the  low and up bounderies
//------------------------------------------------------------------------------
int SConstrainInt(String s,byte first,byte last,int low,int up){return constrain(s.substring(first, last).toInt(), low, up);}
int SConstrainInt(String s,byte first,          int low,int up){return constrain(s.substring(first).toInt(), low, up);}
//------------------------------------------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//------------------------------------------------------------------------------
//                                                                                            //
void ReworkInputString(String InputString)
{
 InputString.trim();                                                                          // Remove trailing spaces
 if (InputString.length()>10) return;                                                         // If string is too long for some reason
 if (InputString[0] > 47 && InputString[0] <123)                                              // If the first charater is a letter
  {
  sprintf(sptext,"**** Length fault ****");                                                   // Default message 
  Serial.println(InputString);
  switch (InputString[0]) 
   {
                    #ifdef DCFNOINT
    case 'A':
    case 'a':
            if (InputString.length() == 1)
              {
               if(UseDCF)
                 {
                 DCFNoInt.toggleDCFDebugInfo();                                                 // Show DCF debug info is serial output  
                 sprintf(sptext,"Toggle DCF debug info");
                 }
               else sprintf(sptext,"Turn DCF-receiver ON for info with option H");
              }
             break;  
                    #endif //DCFNOINT
    case 'C':
    case 'c':
             if(InputString.length() == 1)
               {
                Mem.LightReducer    = SLOPEBRIGHTNESS;
                Mem.UpperBrightness = MAXBRIGHTNESS; 
                Mem.LowerBrightness = LOWBRIGHTNESS;
                Mem.DisplayChoice = Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = Mem.OwnColour = 0;
                Mem.EdMinTijd = Mem.EdMin = Mem.ThMin = Mem.EdTh = Mem.EdThMin = Mem.ThWrong = 0;
                Mem.EdWrong = Mem.ValidTimes = Mem.MinutesSinceStart = 0;
                for (int i=0;i<24;i++) Mem.NVRAMmem[i] = 2;                                       // Reset LDR readings 
                sprintf(sptext,"Data were cleared");   
               }
              if(InputString.length() == 3)
               {
                for (unsigned int i=0 ; i<EEPROM.length(); i++) { EEPROM.write(i, 0); }
                Tekstprintln("EEPROM data were erased"); 
                strcpy(sptext,""); 
                setup();
               }
               break;
//                                                                                            //
    case 'D':
    case 'd':  
            if (InputString.length() == 9 )
             {
              int Jaar;
              I.Day   = (byte) SConstrainInt(InputString,1,3,0,31);
              I.Month = (byte) SConstrainInt(InputString,3,5, 0, 12); 
              Jaar   =        SConstrainInt(InputString,5,9, 2000, 3000); 
              RTCklok.adjust(DateTime(Jaar, I.Month, I.Day, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",Inow.hour(),Inow.minute(),Inow.second(),I.Day,I.Month,Jaar);
             }
            break;
                     #ifdef DCF77MOD
    case 'H':                                                                                 // Toggle DCF Signal on Display
    case 'h':
            if (InputString.length() == 1)
              {
               if(DCFNoInt.getDCFDebugStatus() && UseDCF) DCFNoInt.toggleDCFDebugInfo();     // If DCF debuginfo lines are printed turn it off
               UseDCF = 1 - UseDCF;
               sprintf(sptext,"Use DCF-receiver:%s",UseDCF ? "On" : "Off");
               }            
             break; 
                     #endif //DCF77MOD
    case 'I':
    case 'i': 
            if (InputString.length() == 1)
            {
             SWversion();
             sptext[0] = 0;                                                                   // Clear sptext
            }
            break;
    case 'L':                                                                                 // Lowest value for Brightness
    case 'l':
             if (InputString.length() < 5)
               {      
                Mem.LowerBrightness = (byte) SConstrainInt(InputString,1,0,255);
                sprintf(sptext,"Lower brightness changed to: %d bits",Mem.LowerBrightness);
               }
             break;  
    case 'M':                                                                                 // factor to multiply brighness (0 - 255) with 
    case 'm':
            if (InputString.length() < 6)
               {    
                Mem.UpperBrightness = (int) SConstrainInt(InputString,1,1,1023);
                sprintf(sptext,"Upper brightness changed to: %d bits",Mem.UpperBrightness);
               }
              break;
    case 'N':
    case 'n':
             if (InputString.length() == 1 )         Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = 0;
             if (InputString.length() == 5 )
              {
               Mem.TurnOffLEDsAtHH = SConstrainInt(InputString,1,3,0,23);
               Mem.TurnOnLEDsAtHH  = SConstrainInt(InputString,3,5,0,23); 
              }
              sprintf(sptext,"LEDs are OFF between %2d:00 and %2d:00", Mem.TurnOffLEDsAtHH,Mem.TurnOnLEDsAtHH );
             break;
    case 'O':
    case 'o':
             if(InputString.length() == 1)
               {
                LEDsAreOff = !LEDsAreOff;
                sprintf(sptext,"LEDs are %s", LEDsAreOff?"OFF":"ON" );
                if(LEDsAreOff)  DimLeds(false);                                               // Turn the LEDs off
                else {DimLeds(true); Displaytime();}                                          // Turn the LEDs on                
               }
              break;                                                                   
    case 'P':
    case 'p':  
             if (InputString.length() == 9 )
               {

                LetterColor = Mem.OwnColour = HexToDec(InputString.substring(1,9));           // Display letter color 
                sprintf(sptext,"Own colour stored0X%lX", Mem.OwnColour);
                Displaytime();
               }
             else Tekstprintln("Enter Pwwrrggbb");            
             break;
    case 'q':
    case 'Q':  
             if (InputString.length() == 1 )
               {
                Tekstprintln("  Q0= Yellow colour, HETISWAS changing");
                Tekstprintln("  Q1= Hourly colour");
                Tekstprintln("  Q2= All white");
                Tekstprintln("  Q3= All Own colour");
                Tekstprintln("  Q4= Own colour, HETISWAS changing");
                Tekstprintln("  Q5= Wheel colour");
                strcpy(sptext,"");
               }
             if (InputString.length() == 2 )
               {
                Mem.DisplayChoice = min((byte) InputString.substring(1,2).toInt(),5); 
                sprintf(sptext,"Display choice: Q%d", Mem.DisplayChoice);
                lastminute = 99;                                                              // Force a minute update
               }
             break;     
//                                                                                            //
    case 'R':
    case 'r':
            if (InputString.length() == 1)
              {
               Reset();                                                                       // Reset all settings 
               Tekstprintln("\n**** Reset to default settings ****"); 
               strcpy(sptext,""); 
              }
            break;
    case 'S':
    case 's':
             if (InputString.length() == 1)
               {   
                Zelftest = 1 - Zelftest; 
                sprintf(sptext,"Zelftest: %d",Zelftest);
                Displaytime();                                                               // Turn on the LEDs with proper time
               }                                
             break; 
    case 'T':
    case 't':
            if(InputString.length() >= 7)  // T125500
              {              
              I.Hour   = (byte) SConstrainInt(InputString,1,3,0,23);
              I.Minute = (byte) SConstrainInt(InputString,3,5,0,59); 
              I.Second = (byte) SConstrainInt(InputString,5,7,0,59); 
              sprintf(sptext,"Time set");
              SetRTCTime();
                                  #ifdef DCFNOINT
              DCFlocked = false;                                                              // RTC time can be updated from DCF time  
              sprintf(sptext,"DCFlock unlocked in Thhmmss "); 
                                  #endif  //DCFNOINT
              }
              break; 
    case 'W':
    case 'w':
             if (InputString.length() >1) break;   
             TestLDR = 1 - TestLDR;                                                           // If TestLDR = 1 LDR reading is printed every second instead every 30s
             sprintf(sptext,"TestLDR: %s",TestLDR? "On" : "Off");
             break;        
    case 'X':
    case 'x':    
             MilliSecondValue = 1000;                                                         // Clock runs at normal speed minus 1 ms 
              if (InputString.length() >1 && InputString.length() < 6 )
                MilliSecondValue = InputString.substring(1,5).toInt();                
             Demo = 1 - Demo;                                                                 // Toggle Demo mode
             sprintf(sptext,"Demo mode: %d MillisecondTime=%d",Demo,MilliSecondValue);
             break;  
    case 'Y':                                                                                 // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'y':
             if (InputString.length() < 5)
               {    
                Mem.LightReducer = (byte) SConstrainInt(InputString,1,1,255);
                sprintf(sptext,"Slope brightness changed to: %d%%",Mem.LightReducer);
               }
              break;      
    case '0':
    case '1':
    case '2':        
             if (InputString.length() == 6 )                                                  // For compatibility input with only the time digits
              {
               I.Hour   = (byte) SConstrainInt(InputString,0,2,0,23);
               I.Minute = (byte) SConstrainInt(InputString,2,4,0,59); 
               I.Second = (byte) SConstrainInt(InputString,4,6,0,59);
               sprintf(sptext,"Time set");  
               SetRTCTime();
                    #ifdef DCFNOINT
              sprintf(sptext,"DCFlock unlocked in Thhmmss ");  
              DCFlocked = false;                                                              // RTC time can be updated from DCF time  Tekstprintln("DCFlock is unlocked ");
                    #endif  //DCFNOINT
               }
     default:
             break;
    }
  }
 Tekstprintln(sptext); 
 EEPROM.put(0,Mem);                                                                           // Update EEPROM  
 Displaytime();                                             
 InputString = "";
 Tekstprintln("");
}
//                                                                                            //
//------------------------------------------------------------------------------
//  CLOCK Convert Hex to uint32
//------------------------------------------------------------------------------
uint32_t HexToDec(String hexString) 
{
 uint32_t decValue = 0;
 int nextInt;
 for (unsigned int i = 0; i < hexString.length(); i++) 
  {
   nextInt = int(hexString.charAt(i));
   if (nextInt >= 48 && nextInt <= 57)  nextInt = map(nextInt, 48, 57, 0, 9);
   if (nextInt >= 65 && nextInt <= 70)  nextInt = map(nextInt, 65, 70, 10, 15);
   if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
   nextInt = min(nextInt, 15);
   decValue = (decValue * 16) + nextInt;
  }
return decValue;
}

//------------------------------------------------------------------------------
//  CLOCK Dutch clock display
//------------------------------------------------------------------------------
void Dutch(void)
{
 LedsOff();
 HET;                                       // HET light is always on
 switch (I.Minute)
 {
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; break;
  case  4: 
  case  5: 
  case  6: IS;  MVIJF; OVER; break;
  case  7: 
  case  8: WAS; MVIJF; OVER; break;
  case  9: 
  case 10: 
  case 11: IS;  MTIEN; OVER; break;
  case 12: 
  case 13: WAS; MTIEN; OVER; break;
  case 14: 
  case 15: 
  case 16: IS;  KWART; OVER; break;
  case 17: 
  case 18: WAS; KWART; OVER; break;
  case 19: 
  case 20: 
  case 21: IS;  MTIEN; VOOR; HALF; break;
  case 22: 
  case 23: WAS; MTIEN; VOOR; HALF; break;
  case 24: 
  case 25: 
  case 26: IS;  MVIJF; VOOR; HALF; break;
  case 27: 
  case 28: WAS; MVIJF; VOOR; HALF; break;
  case 29: IS;  HALF; break;
  case 30: IS;  PRECIES; HALF; break;
  case 31: IS;  HALF; break;
  case 32: 
  case 33: WAS; HALF; break;
  case 34: 
  case 35: 
  case 36: IS;  MVIJF; OVER; HALF; break;
  case 37: 
  case 38: WAS; MVIJF; OVER; HALF; break;
  case 39: 
  case 40: 
  case 41: IS;  MTIEN; OVER; HALF; break;
  case 42: 
  case 43: WAS; MTIEN; OVER; HALF; break;
  case 44: 
  case 45: 
  case 46: IS;  KWART; VOOR; break;
  case 47: 
  case 48: WAS; KWART; VOOR; break;
  case 49: 
  case 50: 
  case 51: IS;  MTIEN; VOOR;  break;
  case 52: 
  case 53: WAS; MTIEN; VOOR;  break;
  case 54: 
  case 55: 
  case 56: IS;  MVIJF; VOOR; break;
  case 57: 
  case 58: WAS; MVIJF; VOOR; break;
  case 59: IS;  break;
}
//if (I.Hour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);
//                                                                                            //
 sayhour = I.Hour;
 if (I.Minute > 18 )  sayhour = I.Hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1: EEN; break;
  case 14:
  case 2: TWEE; break;
  case 15:
  case 3: DRIE; break;
  case 16:
  case 4: VIER; break;
  case 17:
  case 5: VIJF; break;
  case 18:
  case 6: ZES; break;
  case 19:
  case 7: ZEVEN; break;
  case 20:
  case 8: ACHT; break;
  case 21:
  case 9: NEGEN; break;
  case 22:
  case 10: TIEN; break;
  case 23:
  case 11: ELF; break;
  case 0:
  case 12: TWAALF; break;
 } 
 switch (I.Minute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UUR;  break; 
 }
}
                      #ifdef MOD_DS3231
//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int  temp3231;
 
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    // Temp registers (11h-12h) get updated automatically every 64s
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read();                          // 2's complement int portion
    tLSB = Wire.read();                          // fraction portion 
    temp3231 = (tMSB & 0b01111111);               // do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    // only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else   {temp3231 = -273; }  
  return (temp3231);
}
                 #endif // MOD_DS3231
//
//------------------------------------------------------------------------------
// ARDUINO Reset function
//------------------------------------------------------------------------------
void SoftwareReset( uint8_t prescaler) 
{
  prescaler=prescaler;
// wdt_enable( prescaler);    // start watchdog with the provided prescaler
 //while(1) {}
}
//------------------------------------------------------------------------------
// ARDUINO Reset to default settings
//------------------------------------------------------------------------------
void ResetArduino(void)
{
//SoftwareReset( WDTO_60MS); //WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS, WDTO_500MS, WDTO_1S, WDTO_2S, WDTO_4S, and WDTO_8S
}
