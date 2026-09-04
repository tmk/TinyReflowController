# Tiny Reflow Controller
An all-in-one Arduino compatible reflow controller powered by ATmega328P (V2) or ATtiny1634R (V1). A reincarnation of the Reflow Oven Controller Shield that requires an external Arduino board like Arduino Uno based on user feedbacks over the years. Powered by the ATmega328P/ATtiny1634R coupled with the latest thermocouple sensor interface IC MAX31856 from Maxim, we managed to remove the need of an Arduino board and reduce the overall cost. We also use as much SMD parts in this revision to keep the cost low (manual soldering and left over residue cleaning is time consuming) and leaving only the terminal block and the LCD connector on through hole version. We also managed to streamline all components to run on 3.3V to further simplify the design. All you need is an external Solid State Relay (SSR) (rated accordingly to your oven), K type thermocouple (we recommend those with fiber glass or steel jacket), and an oven of course! You can now select to run a lead-free profile or leaded profile from the selection switch. V2 comes with 0.96" 128*64 OLED LCD to plot the real-time reflow curve and has a built-in serial-USB interface. V2 also has an optional transistor output drive fan if needed. 

- https://github.com/rocketscream/TinyReflowController


# My setup
2026-09-04
## ESP32C3
This uses U8g2 instead of Adafruit GFX/SSD1306 library and MAX31855 instead of MAX31856.

### Pin assign

    SSR:            0=HEATER, 1=FAN(Not used)
    LED             2(Not used)
    MAX38155(SPI):  CS=3, 4=SCK, 5=SDO
    Buzzer          6
    SSD1306:        8=SDA, 9=SCL
    Button          10=Start/Stop, 7=LF/PB

### Required Libraries
#### Arduino PID Library:
- https://github.com/br3ttb/Arduino-PID-Library
#### Adafruit MAX31856 Library:
- https://github.com/adafruit/Adafruit_MAX31856
#### Adafruit MAX31855 Library:
- https://github.com/adafruit/Adafruit-MAX31855-library
#### U8g2:
- https://github.com/olikraus/u8g2

## TODO
- web interface/control
- clock sync with NTP
