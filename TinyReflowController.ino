/*******************************************************************************
  Title: Tiny Reflow Controller
  Version: 2.00
  Date: 03-03-2019
  Company: Rocket Scream Electronics
  Author: Lim Phang Moh
  Website: www.rocketscream.com

  Brief
  =====
  This is an example firmware for our Arduino compatible Tiny Reflow Controller.
  A big portion of the code is copied over from our Reflow Oven Controller
  Shield. We added both lead-free and leaded reflow profile support in this
  firmware which can be selected by pressing switch #2 (labelled as LF|PB on PCB)
  during system idle. The unit will remember the last selected reflow profile.
  You'll need to use the MAX31856 library for Arduino.

  Lead-Free Reflow Curve
  ======================

  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)

  Leaded Reflow Curve (Kester EP256)
  ==================================

  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |           x   |              |                    |
      |         x     |              |                    |
      |       x       |              |                    |
      |     x         |              |                    |
      |   x           |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)

  This firmware owed very much on the works of other talented individuals as
  follows:
  ==========================================
  Brett Beauregard (www.brettbeauregard.com)
  ==========================================
  Author of Arduino PID library. On top of providing industry standard PID
  implementation, he gave a lot of help in making this reflow oven controller
  possible using his awesome library.

  ==========================================
  Limor Fried of Adafruit (www.adafruit.com)
  ==========================================
  Author of Arduino MAX31856 and SSD1306 libraries. Adafruit has been the source
  of tonnes of tutorials, examples, and libraries for everyone to learn.

  ==========================================
  Spence Konde (www.drazzy.com/e/)
  ==========================================
  Maintainer of the ATtiny core for Arduino:
  https://github.com/SpenceKonde/ATTinyCore

  Disclaimer
  ==========
  Dealing with high voltage is a very dangerous act! Please make sure you know
  what you are dealing with and have proper knowledge before hand. Your use of
  any information or materials on this Tiny Reflow Controller is entirely at
  your own risk, for which we shall not be liable.

  Licences
  ========
  This Tiny Reflow Controller hardware and firmware are released under the
  Creative Commons Share Alike v3.0 license
  http://creativecommons.org/licenses/by-sa/3.0/
  You are free to take this piece of code, use it and modify it.
  All we ask is attribution including the supporting libraries used in this
  firmware.

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
  - Adafruit MAX31856 Library:
    >> https://github.com/adafruit/Adafruit_MAX31856
  - Adafruit SSD1306 Library:
    >> https://github.com/adafruit/Adafruit_SSD1306
  - Adafruit GFX Library:
    >> https://github.com/adafruit/Adafruit-GFX-Library

  Revision  Description
  ========  ===========
  2.00      Support V2 of the Tiny Reflow Controller:
            - Based on ATMega328P 3.3V @ 8MHz
            - Uses SSD1306 128x64 OLED
  1.00      Initial public release:
            - Based on ATtiny1634R 3.3V @ 8MHz
            - Uses 8x2 alphanumeric LCD

*******************************************************************************/

// ***** INCLUDES *****
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_MAX31856.h>
#include <PID_v1.h>


// Print on both USB CDC serial and UART
// https://github.com/tmk/WIP/wiki/ESP32#serial
#ifdef ARDUINO_USB_MODE
#if !ARDUINO_USB_CDC_ON_BOOT
HWCDC HWCDCSerial;
#endif
#define serial_begin(baud)     do { Serial0.begin(baud);    HWCDCSerial.begin(); HWCDCSerial.setTxTimeoutMs(0); } while (0)
#define serial_print(...)      do { Serial0.print(__VA_ARGS__);   HWCDCSerial.print(__VA_ARGS__);   } while (0)
#define serial_println(...)    do { Serial0.println(__VA_ARGS__); HWCDCSerial.println(__VA_ARGS__); } while (0)
#define serial_printf(...)     do { Serial0.printf(__VA_ARGS__);  HWCDCSerial.printf(__VA_ARGS__);  } while (0)
#else
#define serial_begin(baud)     Serial.begin(baud)
#define serial_print(...)      Serial.print(__VA_ARGS__)
#define serial_println(...)    Serial.println(__VA_ARGS__)
#define serial_printf(...)     Serial.printf(__VA_ARGS__)
#endif


// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

typedef enum REFLOW_PROFILE
{
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED
} reflowProfile_t;

// ***** CONSTANTS *****
// ***** GENERAL *****
#define USE_MAX31855    // instead of MAX31856

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 200
#define TEMPERATURE_REFLOW_MAX_LF 250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 180
#define TEMPERATURE_REFLOW_MAX_PB 224
#define SOAK_MICRO_PERIOD_PB 10000

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define X_AXIS_START 18 // X-axis starting position

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "TinyReflow",
  "PreHeat",
  "Soak",
  "Reflow",
  "Cool",
  "Done!",
  "Hot!",
  "Error"
};

// ***** PIN ASSIGNMENT *****
// EPS32C3 default assign
// MAX38155(SPI):   4=SCK, 5=SDO,        CS=7
// MAX38156(SPI):   4=SCK, 5=SDO, 6=SDI, CS=7
// SSD1306(I2C):    8=SDA, 9=SCL
unsigned char ssrPin = 0;
//unsigned char ssr2Pin = 1;
unsigned char ledPin = 2;
unsigned char switchLfPbPin = 7;
unsigned char switchStartStopPin = 10;
unsigned char thermocoupleCSPin = 3;
unsigned char buzzerPin = 6;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long updateLcd;
unsigned long timerSoak;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
unsigned long soakMicroPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Reflow profile type
reflowProfile_t reflowProfile;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
switch_t switchValue;
switch_t switchMask;
// Seconds timer
unsigned int timerSeconds;
// Thermocouple fault status
unsigned char fault;
unsigned int timerUpdate;
unsigned char temperature[SCREEN_WIDTH - X_AXIS_START];
unsigned char x;


// PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// LCD interface
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// MAX31856 thermocouple interface
#if defined(USE_MAX31855)
Adafruit_MAX31855 thermocouple(SCK, thermocoupleCSPin, MISO);
#else
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(thermocoupleCSPin);
#endif


// U8g2 Fonts
// https://github.com/olikraus/u8g2/wiki/fntlist8#7-pixel-height
// u8g2_font_6x10_mr        // 5x7 and spacing:1    monospace

// https://github.com/olikraus/u8g2/wiki/fntgrpprofont#profont17
// u8g2_font_profont17_tf   // 9x17? ascent=11 descent=-3 strWith("A")=8
// u8g2_font_profont17_mf   // 9x17? ascent=11 descent=-3 strWith("A")=9 monospace

#define FONT_HEIGHT(mergin)     (oled.getAscent() - oled.getDescent() + (mergin))
// Font line: top=0, bottom=-1
#define LINE(l, mergin)         (((l) < 0 ? SCREEN_HEIGHT : 0) + FONT_HEIGHT(mergin) * (l) + oled.getAscent())
// Font column form right edge
#define FCR(str)     (SCREEN_WIDTH - oled.getStrWidth(str))


void setup()
{
  // Check current selected reflow profile
  unsigned char value = EEPROM.read(PROFILE_TYPE_ADDRESS);
  if ((value == 0) || (value == 1))
  {
    // Valid reflow profile value
    reflowProfile = (reflowProfile_t)value;
  }
  else
  {
    // Default to lead-free profile
    EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
    reflowProfile = REFLOW_PROFILE_LEADFREE;
  }

  // switch pin initialization
  pinMode(switchStartStopPin, INPUT_PULLUP);
  pinMode(switchLfPbPin, INPUT_PULLUP);

  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active high)
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Initialize thermocouple interface
  thermocouple.begin();
#if defined(USE_MAX31855)
#else
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);
#endif

  tone(buzzerPin,4100,100);

  oled.begin();
  // Flip display 180 deg
  //oled.setRotation(2);

  // Start-up splash
  oled.clearBuffer();
  oled.setFont(u8g2_font_profont17_mf);
  oled.drawStr(0,  LINE(0, 2), "TinyReflow");
  oled.drawStr(80, LINE(1, 2), "v2.00");
  oled.sendBuffer();
  delay(3000);

  // Serial communication at 115200 bps
  serial_begin(115200);

  // Turn off LED (active high)
  digitalWrite(ledPin, LOW);
  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
}

void loop()
{
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
#if defined(USE_MAX31855)
    input = thermocouple.readCelsius();
#else
    input = thermocouple.readThermocoupleTemperature();
#endif
    // Check for thermocouple fault
#if defined(USE_MAX31855)
    fault = thermocouple.readError();
#else
    fault = thermocouple.readFault();
#endif

    // If any thermocouple fault is detected
#if defined(USE_MAX31855)
    if ((fault & MAX31855_FAULT_OPEN) ||
        (fault & MAX31855_FAULT_SHORT_GND) ||
        (fault & MAX31855_FAULT_SHORT_VCC))
#else
    if ((fault & MAX31856_FAULT_CJRANGE) ||
        (fault & MAX31856_FAULT_TCRANGE) ||
        (fault & MAX31856_FAULT_CJHIGH) ||
        (fault & MAX31856_FAULT_CJLOW) ||
        (fault & MAX31856_FAULT_TCHIGH) ||
        (fault & MAX31856_FAULT_TCLOW) ||
        (fault & MAX31856_FAULT_OVUV) ||
        (fault & MAX31856_FAULT_OPEN))
#endif
    {
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
      serial_println(F("Error"));
    }
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += SENSOR_SAMPLING_TIME;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      digitalWrite(ledPin, !(digitalRead(ledPin)));
      // Increase seconds timer for reflow curve plot
      timerSeconds++;
      // Send temperature and time stamp to serial
      serial_print(timerSeconds);
      serial_print(F(","));
      serial_print(setpoint);
      serial_print(F(","));
      serial_print(input);
      serial_print(F(","));
      serial_print(output);
      serial_print(F(","));
      serial_println(lcdMessagesReflowStatus[reflowState]);
    }
    else
    {
      // Turn off red LED
      digitalWrite(ledPin, LOW);

      serial_print(input);
      serial_print(F(","));
#if defined(USE_MAX31855)
      serial_println(thermocouple.readInternal());
#else
      serial_println(thermocouple.readCJTemperature());
#endif
    }
  }

  if (millis() > updateLcd)
  {
    // Update LCD in the next 100 ms
    updateLcd += UPDATE_RATE;

    oled.clearBuffer();
    // Reflow state: top left
    oled.setFont(u8g2_font_profont17_mf);
    oled.setCursor(0, LINE(0, 2));
    oled.print(lcdMessagesReflowStatus[reflowState]);

    // Lead Free / Pb: top right
    oled.setFont(u8g2_font_6x10_mr);    // 5x7 and spacing:1
    oled.setCursor(FCR("XX"), LINE(0, 1));
    if (reflowProfile == REFLOW_PROFILE_LEADFREE)
    {
      oled.print(F("LF"));
    }
    else
    {
      oled.print(F("PB"));
    }

    // Temperature markers
    oled.setFont(u8g2_font_6x10_mr);
    oled.setCursor(0, 26);
    oled.print(F("250"));
    oled.setCursor(0, 44);
    oled.print(F("150"));
    oled.setCursor(6, 62);
    oled.print(F("50"));
    // Draw temperature and time axis
    oled.drawLine(18, 18, 18, 63);
    oled.drawLine(18, 63, 127, 63);


    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      oled.setFont(u8g2_font_6x10_mr);
      oled.setCursor(80, LINE(1, 1));
      oled.print(F("TC Error"));
    }
    else
    {
      // Temperature: bottom right
      oled.setFont(u8g2_font_profont17_mf);
      if      (input <= -100) oled.setCursor(FCR("-999.99\260C"), LINE(-1, 1));
      else if (input <= -10)  oled.setCursor(FCR("-99.99\260C"),  LINE(-1, 1));
      else if (input < 0)     oled.setCursor(FCR("-9.99\260C"),   LINE(-1, 1));
      else if (input < 10)    oled.setCursor(FCR("9.99\260C"),    LINE(-1, 1));
      else if (input < 100)   oled.setCursor(FCR("99.99\260C"),   LINE(-1, 1));
      else if (input < 1000)  oled.setCursor(FCR("999.99\260C"),  LINE(-1, 1));
      else                    oled.setCursor(FCR("9999.99\260C"), LINE(-1, 1));
      oled.print(input);
      oled.setCursor(FCR("\260C"), LINE(-1, 1));
      oled.print(F("\260C"));   // degree Celsius
    }

    // Elapsed time
    oled.setFont(u8g2_font_profont17_mf);
    if      (timerSeconds < 10)   oled.setCursor(FCR("9sec"),    LINE(-2, 1));
    else if (timerSeconds < 100)  oled.setCursor(FCR("99sec"),   LINE(-2, 1));
    else if (timerSeconds < 1000) oled.setCursor(FCR("999sec"),  LINE(-2, 1));
    else                          oled.setCursor(FCR("9999sec"), LINE(-2, 1));
    oled.print(timerSeconds);
    oled.setCursor(FCR("sec"), LINE(-2, 1));
    oled.print(F("sec"));

    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // We are updating the display faster than sensor reading
      if (timerSeconds > timerUpdate)
      {
        // Store temperature reading every 3 s
        if ((timerSeconds % 3) == 0)
        {
          timerUpdate = timerSeconds;
          unsigned char averageReading = map(input, 0, 250, 63, 19);
          if (x < (SCREEN_WIDTH - X_AXIS_START))
          {
            temperature[x++] = averageReading;
          }
        }
      }
    }

    unsigned char timeAxis;
    for (timeAxis = 0; timeAxis < x; timeAxis++)
    {
      oled.drawPixel(timeAxis + X_AXIS_START, temperature[timeAxis]);
    }

    // Update screen
    oled.sendBuffer();
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (input >= TEMPERATURE_ROOM)
      {
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      else
      {
        // If switch is pressed to start reflow process
        if (switchStatus == SWITCH_1)
        {
          // Send header for CSV file
          serial_println(F("TinyReflowController build at " __DATE__ " " __TIME__));
          serial_println(F("Time,Setpoint,Input,Output,State"));
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;

          // Initialize reflow plot update timer
          timerUpdate = 0;

          for (x = 0; x < (SCREEN_WIDTH - X_AXIS_START); x++)
          {
            temperature[x] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          x = 0;

          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Load profile specific constant
          if (reflowProfile == REFLOW_PROFILE_LEADFREE)
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_LF;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
            soakMicroPeriod = SOAK_MICRO_PERIOD_LF;
          }
          else
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_PB;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_PB;
            soakMicroPeriod = SOAK_MICRO_PERIOD_PB;
          }
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
          tone(buzzerPin,4100,1000);
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN)
      {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + soakMicroPeriod;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + soakMicroPeriod;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > soakTemperatureMax)
        {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = reflowTemperatureMax;
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (reflowTemperatureMax - 5))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN)
      {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer to indicate completion
        tone(buzzerPin,4100,700);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    case REFLOW_STATE_COMPLETE:
      if (millis() > buzzerPeriod)
      {
        // Turn off buzzer
        tone(buzzerPin,4100,100);
        // Reflow process ended
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM)
      {
        // Ready to reflow
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_ERROR:
      // Check for thermocouple fault
#if defined(USE_MAX31855)
      fault = thermocouple.readError();
#else
      fault = thermocouple.readFault();
#endif

      // If thermocouple problem is still present
#if defined(USE_MAX31855)
    if ((fault & MAX31855_FAULT_OPEN) ||
        (fault & MAX31855_FAULT_SHORT_GND) ||
        (fault & MAX31855_FAULT_SHORT_VCC))
#else
      if ((fault & MAX31856_FAULT_CJRANGE) ||
          (fault & MAX31856_FAULT_TCRANGE) ||
          (fault & MAX31856_FAULT_CJHIGH) ||
          (fault & MAX31856_FAULT_CJLOW) ||
          (fault & MAX31856_FAULT_TCHIGH) ||
          (fault & MAX31856_FAULT_TCLOW) ||
          (fault & MAX31856_FAULT_OVUV) ||
          (fault & MAX31856_FAULT_OPEN))
#endif
      {
        // Wait until thermocouple wire is connected
        reflowState = REFLOW_STATE_ERROR;
      }
      else
      {
        // Clear to perform reflow process
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
  }

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
      tone(buzzerPin,4100,1000);
    }
  }
  // Switch 2 is pressed
  else if (switchStatus == SWITCH_2)
  {
    // Only can switch reflow profile during idle
    if (reflowState == REFLOW_STATE_IDLE)
    {
      // Currently using lead-free reflow profile
      if (reflowProfile == REFLOW_PROFILE_LEADFREE)
      {
        // Switch to leaded reflow profile
        reflowProfile = REFLOW_PROFILE_LEADED;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 1);
      }
      // Currently using leaded reflow profile
      else
      {
        // Switch to lead-free profile
        reflowProfile = REFLOW_PROFILE_LEADFREE;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
      }
      tone(buzzerPin,1000,200);
    }
  }
  // Switch status has been read
  switchStatus = SWITCH_NONE;

  // Simple switch debounce state machine (analog switch)
  switch (debounceState)
  {
    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      switchStatus = SWITCH_NONE;

      switchValue = readSwitch();

      // If either switch is pressed
      if (switchValue != SWITCH_NONE)
      {
        // Keep track of the pressed switch
        switchMask = switchValue;
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      switchValue = readSwitch();
      if (switchValue == switchMask)
      {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {
          // Valid switch press
          switchStatus = switchMask;
          // Proceed to wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // False trigger
      else
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      switchValue = readSwitch();
      if (switchValue == SWITCH_NONE)
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if ((now - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (now - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(ssrPin, LOW);
  }
}

switch_t readSwitch(void)
{
  // Switch connected directly to individual separate pins
  if (digitalRead(switchStartStopPin) == LOW) return SWITCH_1;
  if (digitalRead(switchLfPbPin) == LOW) return SWITCH_2;

  return SWITCH_NONE;
}
