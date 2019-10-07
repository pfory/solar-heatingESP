#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//SW name & version
#define     VERSION                       "1.71"
#define     SW_NAME                       "Solar"

#define ota
#define time
#define verbose
#define flowSensor
#define serverHTTP

#define AUTOCONNECTNAME   HOSTNAMEOTA
#define AUTOCONNECTPWD    "password"

#ifdef ota
#define HOSTNAMEOTA   "solar"
#endif

#ifdef verbose
  #define DEBUG_PRINT(x)                     Serial.print (x)
  #define DEBUG_PRINTDEC(x)                  Serial.print (x, DEC)
  #define DEBUG_PRINTLN(x)                   Serial.println (x)
  #define DEBUG_PRINTF(x, y)                 Serial.printf (x, y)
  #define PORTSPEED 115200             
  #define DEBUG_WRITE(x)                     Serial.write (x)
  #define DEBUG_PRINTHEX(x)                  Serial.print (x, HEX)
  #define SERIAL_BEGIN                       Serial.begin(PORTSPEED)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTDEC(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, y)
  #define DEBUG_WRITE(x)
#endif 

char                  mqtt_server[40]       = "192.168.1.56";
uint16_t              mqtt_port             = 1883;
char                  mqtt_username[40]     = "datel";
char                  mqtt_key[20]          = "hanka12";
char                  mqtt_base[60]         = "/home/Corridor/esp07";
char                  static_ip[16]         = "192.168.1.108";
char                  static_gw[16]         = "192.168.1.1";
char                  static_sn[16]         = "255.255.255.0";

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

LOLIN Wemos D1 pinout
------------------------------------------
TX                                           - Tx
RX                                           - Rx
D0                                           - PIR sensor
D1                                           - I2C display SCL 0x20, keypad 0x27
D2                                           - I2C display SDA 0x20, keypad 0x27
D3                                           - relay 1
D4                                           - LED
D5                                           - 
D6                                           - flow sensor
D7                                           - DALLAS temperature sensors
D8                                           - relay 2
--------------------------------------------------------------------------------------------------------------------------
*/

//keypad i2c address
#define I2CADDR                              0x20

//display
#define LCDADDRESS                           0x27
#define LCDROWS                              4
#define LCDCOLS                              20

//All of the IO pins have interrupt/pwm/I2C/one-wire support except D0.
#define STATUS_LED                           BUILTIN_LED //status LED
#define PIRPIN                               D0 //                           GPIO16
#define RELAY1PIN                            D3 //relay 10k Pull-up          GPIO0
#ifdef flowSensor
#define FLOWSENSORPIN                        D6 //flow sensor MISO           GPIO12
#endif
#define ONE_WIRE_BUS                         D7 //MOSI                       GPIO13
#define RELAY2PIN                            D8 //10k Pull-down, SS          GPIO15
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
#define TEMP5X                              13  //Control
#define TEMP5Y                               0
#define TEMP6X                               9  //Bojler in
#define TEMP6Y                               2
#define TEMP7X                              12  //Bojler out
#define TEMP7Y                               2
#define TEMP8X                              15  //Bojler
#define TEMP8Y                               2
#define POWERX                               0
#define POWERY                               1
#define ENERGYX                              8
#define ENERGYY                              1
#define FLOWX                                1
#define FLOWY                                2
#define MINRUNX                             16
#define MINRUNY                              1
#define TIMEX                                0
#define TIMEY                                3
#define POZ0X                                0
#define POZ0Y                                0
#define POZ1Y                                1
#define CONTROLSENSORX                       19
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


                          
//#define RELAY1X                              19
//#define RELAY1Y                              0
/*#define RELAY2X                            15
#define RELAY2Y                              1
*/                          
                          
#define DAY_INTERVAL                         43200000 //1000*60*60*12; //ms

#define PRINT_SPACE                          lcd.print(F(" "));

#define DS_MEASSURE_INTERVAL                 750 //inteval between meassurements
#define DELAY_AFTER_ON                       120000 //1000*60*2; //po tento cas zustane rele sepnute bez ohledu na stav teplotnich cidel
  
#define SAFETY_ON                            86.0 //teplota, pri niz rele vzdy sepne
  
#define SEND_DELAY                           30000  //prodleva mezi poslanim dat v ms
#define SHOW_INFO_DELAY                      5000  //
#define SENDSTAT_DELAY                       60000 //poslani statistiky kazdou minutu
#define MEAS_DELAY                           5000  //mereni teplot
#define CALC_DELAY                           1000  //mereni prutoku a vypocet energie kazdou sekundu
  
#define T_MIN                                -128.0
#endif
