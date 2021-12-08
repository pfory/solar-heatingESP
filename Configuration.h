#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <FS.h>          //this needs to be first
#include <Ticker.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <DoubleResetDetector.h>      //https://github.com/khoih-prog/ESP_DoubleResetDetector
#include <PubSubClient.h>
#include <Keypad_I2C.h>
#include <Keypad.h>          // GDY120705
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <timer.h>

//SW name & version
#define     VERSION                       "2.21"
#define     SW_NAME                       "Solar"

#define ota
#define time
#define verbose
#define flowSensor
#define serverHTTP
#define PIR
#define timers

#define CFGFILE "/config.json"


#define CONFIG_PORTAL_TIMEOUT 60 //jak dlouho zustane v rezimu AP nez se cip resetuje
#define CONNECT_TIMEOUT 5 //jak dlouho se ceka na spojeni nez se aktivuje config portal

static const char* const      mqtt_server                    = "192.168.1.56";
static const uint16_t         mqtt_port                      = 1883;
static const char* const      mqtt_username                  = "datel";
static const char* const      mqtt_key                       = "hanka12";
static const char* const      mqtt_base                      = "/home/Corridor/esp07";
static const char* const      mqtt_topic_relay               = "manualRelay";
static const char* const      mqtt_topic_restart             = "restart";
static const char* const      mqtt_topic_netinfo             = "netinfo";
static const char* const      mqtt_config_portal             = "config";
static const char* const      mqtt_config_portal_stop        = "disconfig";
static const char* const      mqtt_topic_tDiffOFF            = "tDiffOFF";
static const char* const      mqtt_topic_tDiffON             = "tDiffON";
static const char* const      mqtt_topic_controlSensor       = "controlSensorBojler";
static const char* const      mqtt_topic_sendSO              = "sorder";
static const char* const      mqtt_topic_so                  = "so";


/*
sensor order
/home/Corridor/esp07/so0 - 7
/home/Corridor/esp07/so1 - 4
/home/Corridor/esp07/so2 - 2
/home/Corridor/esp07/so3 - 1
/home/Corridor/esp07/so4 - 3
/home/Corridor/esp07/so5 - 5
/home/Corridor/esp07/so6 - 6
/home/Corridor/esp07/so7 - 0
kontrola /home/Corridor/esp07/sorder
*/


/*
Version history:
1.5  -            first version with Wemos D1 Mini ESP8266
1.41 - debug serial.print
1.4  - 20.10.2017 doplneny hodiny RTC a zobrazeni uhlu kolektoru na displeji
1.31 - change display to 4x20
1.1  - 13.9.2017  pridany cidla teploty bojler vstup/vystup
1.01 - 28.8.2017  pridani cidla prutoku
1.00 - 28.8.2017  vypinani pouze na zaklade vystupni teploty e solaru na strese 
0.99 - 24.8.2017  mereni z obou panelu
0.95 - 19.6.2016  i2c keypad
0.90 - 05.05.2016 komunikace s jednokou ESP8266
0.82 - 27.1.2016  opraveno zobrazeni teplot od 0 do -0.9 na displeji
0.80 - 9.9.2015   I2C komunikace s powerMeter unit
0.79 - 24.10.2014 zachyceni stavu po resetu
0.78 - 27.9.2014  pridano zobrazeni dnu bez slunce, zap/vyp podsviceni
0.77 - 14.8.2014
0.76 - 7.8.2014
0.75 - 31.7.2014  funkcni pocitani totalSec a totalPower
0.74 - 30.7.2014  add delay after ON, prevent cyclic switch ON-OFF-ON....
0.73 - 29.7.2014  add totalSec, oprava posilani totalPower
0.72 - 14.6.2013  zmena spinani teplot
0.71 - 5.6.2014   optiboot, watchdog
0.70 - 21.5.2014
0.69 - 21.5.2014
0.68 - 20.5.2014
0.67 - 15.6.2014
0.66 - 6.4.2014
0.65 - 26.3.2014
0.60 - 16.3.2014
0.50 - 1.12.2013
0.41 - 20.10.2013
--------------------------------------------------------------------------------------------------------------------------
HW
ESP8266 - Wemos D1 Mini
I2C display
2 Relays module
DALLAS temperature sensor
keyboard

*/

//keypad i2c address
#define I2CADDR                              0x20

//display
#define LCDADDRESS                           0x27
#define LCDROWS                              4
#define LCDCOLS                              20

//All of the IO pins have interrupt/pwm/I2C/one-wire support except D0.
//#define STATUS_LED                           BUILTIN_LED //status LED
#define PIRPIN                               D0 //                           GPIO16
#define RELAYPIN                             D3 //relay 10k Pull-up        GPIO0
#ifdef flowSensor
#define FLOWSENSORPIN                        D6 //flow sensor MISO           GPIO12
#endif
#define ONE_WIRE_BUS                         D7 //MOSI                       GPIO13
//#define RELAY2PIN                          D8 //10k Pull-down, SS          GPIO15
//SDA                                        D2 //                           GPIO4
//SCL                                        D1 //                           GPIO5
//BUILTIN_LED                                D4 //10k Pull-up, BUILTIN_LED   GPIO2
//                                           D5 //SCK                        GPIO14

#define RELAY_ON                            LOW
#define RELAY_OFF                           HIGH

#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES                    10
#endif

//0123456789012345
//15.6 15.8 15.8 V
//1234 0.12 624
#define TEMP1X                               0  //P1 In
#define TEMP1Y                               0
#define TEMP2X                               3  //P1 Out
#define TEMP2Y                               0
#define TEMP3X                               6  //P2 In
#define TEMP3Y                               0
#define TEMP4X                               9  //P2 Out
#define TEMP4Y                               0
#define TEMP5X                              12  //Control
#define TEMP5Y                               0
#define TEMP6X                               9  //Bojler in
#define TEMP6Y                               1
#define TEMP7X                              12  //Bojler out
#define TEMP7Y                               1
#define TEMP8X                              15  //Bojler
#define TEMP8Y                               1
#define POWERX                               0
#define POWERY                               2
#define ENERGYX                              8
#define ENERGYY                              2
#define FLOWX                                1
#define FLOWY                                1
#define RUNMINTODAY_X                       16
#define RUNMINTODAY_Y                        2
#define TIMEX                                0
#define TIMEY                                3
#define POZ0X                                0
#define POZ0Y                                0
#define POZ1Y                                2
#define CONTROLSENSORX                      19
#define CONTROLSENSORY                       0
#define SUNANGLEX                            6
#define SUNANGLEY                            3

#define DISPLAY_MAIN                         0

#define DISPLAY_T_DIFF_ON                    2
#define DISPLAY_T_DIFF_OFF                   3
#define DISPLAY_FLOW                         4
#define DISPLAY_MAX_IO_TEMP                  5
#define DISPLAY_MAX_BOJLER                   6
#define DISPLAY_MAX_POWER_TODAY              7
#define DISPLAY_CONTROL_SENSOR               8
#define RELAY_STATUSX                       17
#define RELAY_STATUSY                        3

#define PRINT_SPACE                          lcd.print(F(" "));

#define DELAY_AFTER_ON                       120000 //1000*60*2; //po tento cas zustane rele sepnute bez ohledu na stav teplotnich cidel
  
#define SAFETY_ON                            86.0 //teplota, pri niz rele vzdy sepne
#define ROOMTEMPON                           25.0 //teplota, pri niz sepne rele kdyz je kontrolni cidlo mistnost
  
#define SEND_DELAY                           60000  //prodleva mezi poslanim dat v ms
#define SHOW_INFO_DELAY                      5000  //
#define SENDSTAT_DELAY                       60000 //poslani statistiky kazdou minutu
#define MEAS_DELAY                           2000  //mereni teplot
#define CALC_DELAY                           1000  //mereni prutoku a vypocet energie kazdou sekundu
#define CONNECT_DELAY                        5000 //ms
  
#define TEMP_ERR                            -127

#include <fce.h>

#endif
