
#include <Arduino.h>        // Core Arduino
#include <BatterySocData.h> // Contains Configurable parameters

#include <Wire.h>   //  I2C communication SDA,SCL
#include <Ticker.h> //  Tickers which can call repeating functions. Replaces delay() with non-blocking functions

#ifdef ADS1115_SENSOR
#include <Adafruit_ADS1X15.h> //  ADS1115 16-bit ADC
#endif

#ifdef INA219_SENSOR
#include <Adafruit_INA219.h> //  INA218  12-bit current sensing IC using shunt
#endif

#ifdef LCD_16X2_I2C
#include <LiquidCrystal_I2C.h> //  LCD 16x2
#endif

#ifdef LED_7SEG_TM1637_I2C
#include <TM1637Display.h> //  4 Digit 7 segment display I2C
#endif

// #include <ArduinoJson.h>         //  Easy handling of JSON data in C/C++

#ifndef BatterySoc_h
#define BatterySoc_h

// Varialbles
uint8_t lcd_screen_no = 0;

float battery_voltage = BATTERY_VOLTAGE;                                      //  Present voltage of battery V
float battery_capacity_ah = BATTERY_CAPACITY_AH;                              //  Present Capacity of battery Ah
double battery_capacity_coulomb = BATTERY_CAPACITY_COULOMB;                   //  Present Capacity of battery Col

// TODO : REMOVE IN PRODUCTION / 2 PART
uint32_t battery_capacity_milli_coulomb = BATTERY_CAPACITY_MILLI_COULOMB / 2; // Present Capacity of battery mCol

const uint32_t battery_capacity_milli_coulomb_const = BATTERY_CAPACITY_MILLI_COULOMB; // Present Capacity of battery mCol

float shunt_voltage = 0; // Present voltage across shunt mV

unsigned long last_read_micros = 0;
unsigned long current_read_micros = 0;

bool flag_battery_fully_charged = 0;
bool flag_battery_fully_drained = 0;

// Methods

/*Set Message on LCD screen*/
void lcd_set_msg(String message, uint8_t lcd_row = 0, uint8_t lcd_col = 0, bool clear_screen = true, bool align_centre = true);

/*Setup LCD display*/
void setup_lcd_i2c();

/*Setup 7 Segment display*/
void setup_7seg_i2c();

/*Setup push buttons*/
void setup_push_buttons();

/*Setup Analog Read : For 10K pot */
void setup_analog_read_pot();

/*Setup Analog Read : For V-bat (64V)*/
void setup_analog_read_vbat();

/*Setup ADS1115*/
void setup_ads1115();

/*Setup INA219*/
void setup_ina219();

void print_BatterySocData_h();

/*Only Execute once while power on*/
void setup();

double calculte_milli_coulombs_transffred();

void show_coulomb_lcd();
void show_battery_percent_lcd();

void analog_read_vbat();
void show_vbat_lcd();

void differential_analog_read_ads_vshunt();
void show_vshunt_lcd();

/*Display latest data on LCD*/
void update_lcd_display();

/*Read all sensor data and update respective variables*/
void read_sensor_data();

/*Update all running Tickers/Timers*/
void ticker_loop();

/*Infinite loop*/
void loop();

#endif