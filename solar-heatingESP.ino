/*SOLAR-HEATING
--------------------------------------------------------------------------------------------------------------------------
SOLAR - control system for solar unit
Petr Fory pfory@seznam.cz
GIT - https://github.com/pfory/solar-heating
//Wemos D1 R2 & mini  !!!!!!!!!!!! 2M FS !!!!!!!!!!!!!!!!!!!!!!!!!!! - jinak se smaže nastavení z configu
*/

/*TODO
vyresit chybu teplomeru
*/

#include "Configuration.h"

//DALLAS temperature sensors
OneWire onewire(ONE_WIRE_BUS);  // pin for onewire DALLAS bus
DallasTemperature dsSensors(&onewire);
DeviceAddress tempDeviceAddress;

DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
unsigned int numberOfDevices                = 0; // Number of temperature devices found

float sensor[NUMBER_OF_DEVICES];

#ifdef serverHTTP
ESP8266WebServer server(80);
#endif

//navrhar - https://maxpromer.github.io/LCD-Character-Creator/
byte customChar[] = {
  B01110,
  B01010,
  B01110,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

unsigned int volatile pulseCount            = 0;
float tP2In                                 = 0; //input medium temperature to solar panel roof
float tP2Out                                = 0; //output medium temperature to solar panel roof`
float tP1In                                 = 0; //input medium temperature to solar panel drevnik
float tP1Out                                = 0; //output medium temperature to solar panel drevnik
//float tRoom                                 = 0; //room temperature
float tBojler1                              = 0; //boiler temperature
float tBojler2                              = 0; //boiler temperature
//float tControl                              = 0; //temperature which is used as control temperature
float tBojlerIn                             = 0; //boiler input temperature
float tBojlerOut                            = 0; //boiler output temperature
float lMin                                  = 0;
unsigned long energyADay                    = 0; //energy a day in Ws
unsigned long secOnDay                      = 0; //sec in ON state per day
unsigned int  power                         = 0; //actual power in W
float energyDiff                            = 0.f; //difference in Ws
volatile bool showDoubleDot                 = false;
bool firstTempMeasDone                      = false;
unsigned long lastOffOn                     = 0; //zamezuje cyklickemu zapinani a vypinani rele
//unsigned long lastOff                       = 0;  //ms posledniho vypnuti rele
bool dispClear                              = false;

DeviceAddress T1Addr                        = { 0x28, 0xE6, 0xFB, 0x80, 0x04, 0x00, 0x00, 0x14 };
DeviceAddress T2Addr                        = { 0x28, 0x0E, 0xC9, 0x80, 0x04, 0x00, 0x00, 0x9B };
DeviceAddress T3Addr                        = { 0x28, 0xFF, 0x94, 0x27, 0x74, 0x16, 0x04, 0x40 };
DeviceAddress T4Addr                        = { 0x28, 0xFF, 0x78, 0x33, 0x03, 0x17, 0x04, 0xDC };
DeviceAddress T5Addr                        = { 0x28, 0xEA, 0x67, 0x6B, 0x05, 0x00, 0x00, 0x89 };
DeviceAddress T6Addr                        = { 0x28, 0xFF, 0x65, 0xB9, 0x02, 0x17, 0x03, 0xA6 };
DeviceAddress T7Addr                        = { 0x28, 0xFF, 0x6D, 0x2F, 0x73, 0x16, 0x05, 0x2D };
DeviceAddress T8Addr                        = { 0x28, 0xFF, 0xA5, 0x68, 0x74, 0x16, 0x03, 0xF0 };

byte manualRelay                             = 2;
byte relayStatus                             = RELAY_ON;
   
//bool shouldSaveConfig                       = false; //flag for saving data

bool todayClear                             = false;

ADC_MODE(ADC_VCC); //vcc read

#ifdef flowSensor
volatile int      numberOfPulsesFlow        = 0; // Measures flow sensor pulses
void flow () { // Interrupt function
   numberOfPulsesFlow++;
}
#endif    
    
unsigned int displayType                    = DISPLAY_MAIN;
unsigned long showInfo                      = 0; //zobrazeni 4 radky na displeji


LiquidCrystal_I2C lcd(LCDADDRESS,LCDCOLS,LCDROWS);  // set the LCD

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS]                       = {
                                            {'1','2','3','A'},
                                            {'4','5','6','B'},
                                            {'7','8','9','C'},
                                            {'*','0','#','D'}
};
byte rowPins[ROWS]                          = {7,6,5,4}; //connect to the row pinouts of the keypad
byte colPins[COLS]                          = {3,2,1,0}; //connect to the column pinouts of the keypad

//Keypad_I2C keypad = Keypad_I2C( makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR );
Keypad_I2C keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR); 
// #endif

//promenne ulozene v pameti (viz CFGFILE "/config.json")
byte            tDiffON                    = 5; //rozdil vystupni teploty panelu 1 tP1Out nebo panelu 2 tP2Out proti teplote bojleru1 pri kterem dojde ke spusteni cerpadla
byte            tDiffOFF                   = 2; //rozdil vystupni teploty panelu 2 tP2Out proti teplote bojleru1 pri kterem dojde k vypnuti cerpadla

byte            sensorOrder[NUMBER_OF_DEVICES];


byte sunAngle[12]                           = {17,23,32,44,55,62,63,58,48,37,26,19}; //uhel slunce nad obzorem kdyz je na jihu

//MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  char * pEnd;
  long int valL;
  String val =  String();
  DEBUG_PRINT("Message arrived [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("] ");
  for (int i=0;i<length;i++) {
    DEBUG_PRINT((char)payload[i]);
    val += (char)payload[i];
  }
  DEBUG_PRINTLN();
  lcd.clear();
  lcd.print(topic);
  lcd.print(": ");
  lcd.print(val);
  delay(500);
  lcd.clear();
  
  if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_tDiffOFF)).c_str())==0) {
    printMessageToLCD(topic, val);
    DEBUG_PRINT("set tDiffOFF to ");
    tDiffOFF=val.toInt();
    DEBUG_PRINT(tDiffOFF);
    saveConfig();
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_tDiffON)).c_str())==0) {
    printMessageToLCD(topic, val);
    DEBUG_PRINT("set tDiffON to ");
    tDiffON=val.toInt();
    DEBUG_PRINT(tDiffON);
    saveConfig();
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_restart)).c_str())==0) {
    printMessageToLCD(topic, val);
    DEBUG_PRINTLN("RESTART");
    saveConfig();
    ESP.restart();
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_netinfo)).c_str())==0) {
    printMessageToLCD(topic, val);
    DEBUG_PRINTLN("NET INFO");
    sendNetInfoMQTT();    
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_config_portal)).c_str())==0) {
    startConfigPortal();
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_config_portal_stop)).c_str())==0) {
    stopConfigPortal();  
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_relay)).c_str())==0) {
    printMessageToLCD(topic, val);
    DEBUG_PRINT("set manual control relay to ");
    manualRelay = val.toInt();
    if (val.toInt()==1) {
      DEBUG_PRINTLN(F("ON"));
    } else {
      DEBUG_PRINTLN(F("OFF"));
    }
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_DSAddr)).c_str())==0) {
    sendDSAddrMQTT();
  }
}

#ifdef flowSensor
void ICACHE_RAM_ATTR flow();
#endif
//----------------------------------------------------- S E T U P -----------------------------------------------------------
void setup() {
  preSetup();
  
  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, RELAY_ON);

  lcd.init();               // initialize the lcd 
  lcd.backlight();
  lcd.home();                   
  lcd.print(SW_NAME);  
  PRINT_SPACE
  lcd.print(VERSION);
  lcd.createChar(0, customChar);

  pinMode(ONE_WIRE_BUS, INPUT);
#ifdef flowSensor
  //pinMode(FLOWSENSORPIN, INPUT);
  //digitalWrite(FLOWSENSORPIN, HIGH); // Optional Internal Pull-Up
  attachInterrupt(digitalPinToInterrupt(FLOWSENSORPIN), flow, RISING); // Setup Interrupt
/*
  DEBUG_PRINTLN("Flow sensor");
  pinMode(FLOWSENSORPIN, INPUT_PULLUP);
  //digitalWrite(FLOWSENSORPIN, HIGH); // Optional Internal Pull-Up
  attachInterrupt(FLOWSENSORPIN, flow, CHANGE); // Setup Interrupt*/
#endif
#ifdef PIR
  pinMode(PIRPIN, INPUT);
#endif

  dsInit();

  if (!readConfig()) {
    DEBUG_PRINTLN(F("ERROR config corrupted"));
  }

  lcd.setCursor(0,2);
  lcd.print(F("tON:"));  
  lcd.print(tDiffON);
  lcd.print(F(" tOFF:"));  
  lcd.print(tDiffOFF);
  lcd.setCursor(0,3);
  lcd.print(F("Control:"));  
  //lcd.print(controlSensor);
  // if (controlSensorBojler==1) {
    // lcd.print("Bojler");
  // } else {
    // lcd.print("Room");
  // }
  delay(2000);
  lcd.clear();
  
  lcd.print("Connecting to WiFi..");
  
  DEBUG_PRINT(F("tON:"));  
  DEBUG_PRINTLN(tDiffON);
  DEBUG_PRINT(F("tOFF:"));  
  DEBUG_PRINTLN(tDiffOFF);
  DEBUG_PRINT(F("Control:"));  
  //DEBUG_PRINT(controlSensor);
  // if (controlSensorBojler==1) {
    // DEBUG_PRINTLN("Bojler");
  // } else {
    // DEBUG_PRINTLN("Room");
  // }

  keypad.begin();
  //keypad.addEventListener(keypadEvent); //add an event listener for this keypad  
  
  if (numberOfDevices>NUMBER_OF_DEVICES) {
    DEBUG_PRINTLN("ERROR - real number of devices DS18B20 > NUMBER_OF_DEVICES. Change variable NUMBER_OF_DEVICES in Configuration.h file!!!!!!!!");
  }

#ifdef timers
  //setup timers
  DEBUG_PRINTLN("Setup timers.");
  timer.every(SEND_DELAY, sendDataMQTT);
  timer.every(MEAS_DELAY, tempMeas);
  timer.every(CALC_DELAY, calcPowerAndEnergy);
  timer.every(CONNECT_DELAY, reconnect);
#ifdef flowSensor
  timer.every(CALC_DELAY, calcFlow);
#endif
  timer.every(SENDSTAT_DELAY, sendStatisticMQTT);
#ifdef time
  timer.every(CALC_DELAY/2, displayTime);
#endif
#endif

  void * a;
  reconnect(a);
  sendStatisticMQTT(a);
  sendNetInfoMQTT();
  
  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, HIGH);

  drd.stop();

  lcd.clear();
  DEBUG_PRINTLN(F("SETUP END......................."));
}


//----------------------------------------------------- L O O P -----------------------------------------------------------
void loop() {
  firstTempMeasDone ? relay() : void();
  
#ifdef PIR
  if (digitalRead(PIRPIN)==1) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
  }
#endif

  display();
  
  keyBoard();
  
  timer.tick(); // tick the timer
#ifdef serverHTTP
  server.handleClient();
#endif

#ifdef ota
  ArduinoOTA.handle();
#endif

  wifiManager.process();
  client.loop();
  
  //handle ftp server
  //ftpSrv.handleFTP();
#ifdef time
  displayClear();
  nulStat();
#endif

} //loop


//----------------------------------------------------- F U N C T I O N S -----------------------------------------------------------
void printMessageToLCD(char* t, String v) {
  lcd.clear();
  lcd.print(t);
  lcd.print(": ");
  lcd.print(v);
  delay(2000);
  lcd.clear();
}

#ifdef time
void displayClear() {
  if (minute()==0 && second()==0) {
    if (!dispClear) { 
      lcd.clear();
      dispClear = true;
    }
  } else {
    dispClear = false;
  }
}

  //nulovani statistik o pulnoci
void nulStat() {
  if (hour()==0 && !todayClear) {
    todayClear =true;
    energyADay=0;
    secOnDay=0;
    //tMaxOut=TEMP_ERR;
    //tMaxIn=TEMP_ERR;
    //tMaxBojler=TEMP_ERR;
  } else if (hour()>0) {
    todayClear = false;
  }
}
#endif


void changeRelay(byte status) {
  digitalWrite(RELAYPIN, status);
}

bool calcPowerAndEnergy(void *) {
  // float t1;
  // float t2;
  
  if (relayStatus==RELAY_ON) {  //pump is ON
    // if (controlSensorBojler==1) {
      // t1 = tBojlerIn;
      // t2 = tBojlerOut;
    // } else {
      // t1 = tP2Out;
      // t2 = tP1In;
    // }
    if (tBojlerIn>tBojlerOut) {
      secOnDay++;
      power = getPower(tBojlerIn, tBojlerOut); //in W
      //DEBUG_PRINTLN(F(power);
      energyDiff += CALC_DELAY*(float)power/1000.f; //in Ws
      if (energyDiff >= 3600.f) { //Wh
        energyADay  += (unsigned long)energyDiff;
        energyDiff = energyDiff - (long)energyDiff;
      }
    } else {
      power=0;
    }
  } else {
    power=0;
  }
  return true;
}

#ifdef serverHTTP
void handleRoot() {
	#define TEXTLEN 1000
  char temp[TEXTLEN];
#ifdef time  
  printSystemTime();
#endif
  DEBUG_PRINTLN(" Client request");
  digitalWrite(BUILTIN_LED, LOW);
  
	snprintf ( temp, TEXTLEN,
      "<html>\
        <head>\
          <meta charset='UTF-8'>\
        </head>\
        <body>\
          <b>Sensors:</b><br />\
          sensor[0]:%s%d.%02d<br />\
          sensor[1]:%s%d.%02d<br />\
          sensor[2]:%s%d.%02d<br />\
          sensor[3]:%s%d.%02d<br />\
          sensor[4]:%s%d.%02d<br />\
          sensor[5]:%s%d.%02d<br />\
          sensor[6]:%s%d.%02d<br />\
          sensor[7]:%s%d.%02d<br />\
          <br /><br />\
          <b>Temperatures:</b><br />\
          so0-Panel 1 Input:%d.%02d<br />\
          so1-Panel 1 Output:%d.%02d<br />\
          so2-Panel 2 Input:%d.%02d<br />\
          so3-Panel 2 Output:%d.%02d<br />\
          so4-Bojler Input:%d.%02d<br />\
          so5-Bojler Output:%d.%02d<br />\
          so6-Bojler:%d.%02d<br />\
          so7-Room:%d.%02d<br />\
        </body>\
      </html>",
      sensor[0]<0 && sensor[0]>-1 ? "-":"",
      (int)sensor[0], 
      abs((sensor[0] - (int)sensor[0]) * 100),
      sensor[1]<0 && sensor[1]>-1 ? "-":"",
      (int)sensor[1], 
      abs((sensor[1] - (int)sensor[1]) * 100),
      sensor[2]<0 && sensor[2]>-1 ? "-":"",
      (int)sensor[2], 
      abs((sensor[2] - (int)sensor[2]) * 100),
      sensor[3]<0 && sensor[3]>-1 ? "-":"",
      (int)sensor[3], 
      abs((sensor[3] - (int)sensor[3]) * 100),
      sensor[4]<0 && sensor[4]>-1 ? "-":"",
      (int)sensor[4], 
      abs((sensor[4] - (int)sensor[4]) * 100),
      sensor[5]<0 && sensor[5]>-1 ? "-":"",
      (int)sensor[5], 
      abs((sensor[5] - (int)sensor[5]) * 100),
      sensor[6]<0 && sensor[6]>-1 ? "-":"",
      (int)sensor[6], 
      abs((sensor[6] - (int)sensor[6]) * 100),
      sensor[7]<0 && sensor[7]>-1 ? "-":"",
      (int)sensor[7], 
      abs((sensor[7] - (int)sensor[7]) * 100),
      (int)tP1In,
      abs((tP1In - (int)tP1In) * 100),
      (int)tP1Out,
      abs((tP1Out - (int)tP1Out) * 100),
      (int)tP2In,
      abs((tP2In - (int)tP2In) * 100),
      (int)tP2Out,
      abs((tP2Out - (int)tP2Out) * 100),
      (int)tBojlerIn,
      abs((tBojlerIn - (int)tBojlerIn) * 100),
      (int)tBojlerOut,
      abs((tBojlerOut - (int)tBojlerOut) * 100),
      (int)tBojler1,
      abs((tBojler1 - (int)tBojler1) * 100)
      // ,
      // (int)tRoom,
      // abs((tRoom - (int)tRoom) * 100)
);
	server.send ( 200, "text/html", temp );
  digitalWrite(BUILTIN_LED, HIGH);
}
#endif

bool saveConfig() {
  DEBUG_PRINTLN(F("Saving config..."));

  // if SPIFFS is not usable
  if (!SPIFFS.begin() || !SPIFFS.exists(CFGFILE) ||
      !SPIFFS.open(CFGFILE, "w"))
  {
    DEBUG_PRINTLN(F("Need to format SPIFFS: "));
    SPIFFS.end();
    SPIFFS.begin();
    DEBUG_PRINTLN(SPIFFS.format());
  }

  StaticJsonDocument<1024> doc;

  // doc["MQTT_server"]             = mqtt_server;
  // doc["MQTT_port"]               = mqtt_port;
  // doc["MQTT_uname"]              = mqtt_username;
  // doc["MQTT_pwd"]                = mqtt_key;
  // doc["MQTT_base"]               = mqtt_base;
  
  // doc["ip"]                      = WiFi.localIP().toString();
  // doc["gateway"]                 = WiFi.gatewayIP().toString();
  // doc["subnet"]                  = WiFi.subnetMask().toString();

  doc["tDiffON"]                 = tDiffON;
  doc["tDiffOFF"]                = tDiffOFF;
//  doc["controlSensor"]           = controlSensorBojler;
  // for (int i=0; i<numberOfDevices; i++) {
    // //doc["sensorOrder[" + String(i) + "0]"]          = sensorOrder[i];
    // doc["sensorOrder[" + String(i) + "]"]          = sensorOrder[i];
  // }

  lcd.clear();
 
  File configFile = SPIFFS.open(CFGFILE, "w+");
  if (!configFile) {
    DEBUG_PRINTLN(F("Failed to open config file for writing"));
    SPIFFS.end();
    lcd.print("Failed to open config file for writing");
    delay(2000);
    lcd.clear();
    return false;
  } else {
    if (isDebugEnabled) {
      serializeJson(doc, Serial);
    }
    serializeJson(doc, configFile);
    //json.printTo(configFile);
    configFile.close();
    SPIFFS.end();
    DEBUG_PRINTLN(F("\nSaved successfully"));
    lcd.print("Config saved.");
    delay(2000);
    lcd.clear();
    return true;
  }
}

bool readConfig() {
  DEBUG_PRINT(F("Mounting FS..."));

  if (SPIFFS.begin()) {
    DEBUG_PRINTLN(F(" mounted!"));
    if (SPIFFS.exists(CFGFILE)) {
      // file exists, reading and loading
      DEBUG_PRINTLN(F("Reading config file"));
      File configFile = SPIFFS.open(CFGFILE, "r");
      if (configFile) {
        DEBUG_PRINTLN(F("Opened config file"));

        char json[500];
        while (configFile.available()) {
         int l = configFile.readBytesUntil('\n', json, sizeof(json));
         json[l] = 0;
         DEBUG_PRINTLN(json);
        }

        DynamicJsonDocument doc(1024);
        deserializeJson(doc, json);
        tDiffON        = doc["tDiffON"];
        DEBUG_PRINT(F("tDiffON: "));
        DEBUG_PRINTLN(tDiffON);
        tDiffOFF       = doc["tDiffOFF"];
        DEBUG_PRINT(F("tDiffOFF: "));
        DEBUG_PRINTLN(tDiffOFF);
        return true;
      }
      DEBUG_PRINTLN(F("ERROR: unable to open config file"));
    } else {
      DEBUG_PRINTLN(F("ERROR: config file not exist"));
    }
  } else {
    DEBUG_PRINTLN(F(" ERROR: failed to mount FS!"));
  }
  return false;
}

void sendRelayMQTT(byte akce) {
  digitalWrite(BUILTIN_LED, LOW);
  client.publish((String(mqtt_base) + "/relayChange").c_str(), String(akce).c_str());
  delay(1000);
  void * a;
  calcFlow(a);
  client.publish((String(mqtt_base) + "/prutok").c_str(), String(lMin).c_str());
  digitalWrite(BUILTIN_LED, HIGH);
}


bool sendSOMQTT(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Sensor order"));
  for (int i=0; i<NUMBER_OF_DEVICES; i++) {
    client.publish((String(mqtt_base) + "/so" + String(i)).c_str(), String(sensorOrder[i]).c_str());
  }
  digitalWrite(BUILTIN_LED, HIGH);
  return true;
}

bool sendDataMQTT(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Data"));

  client.publish((String(mqtt_base) + "/tP1IN").c_str(), String(tP1In).c_str());
  client.publish((String(mqtt_base) + "/tP1OUT").c_str(), String(tP1Out).c_str());
  client.publish((String(mqtt_base) + "/tP2IN").c_str(), String(tP2In).c_str());
  client.publish((String(mqtt_base) + "/tP2OUT").c_str(), String(tP2Out).c_str());
  client.publish((String(mqtt_base) + "/prutok").c_str(), String(lMin).c_str());
  client.publish((String(mqtt_base) + "/sPumpSolar/status").c_str(), String(relayStatus==RELAY_ON ? 1 : 0).c_str());
  client.publish((String(mqtt_base) + "/tBojler1").c_str(), String(tBojler1).c_str());
  client.publish((String(mqtt_base) + "/tBojler2").c_str(), String(tBojler2).c_str());
  client.publish((String(mqtt_base) + "/tBojlerIN").c_str(), String(tBojlerIn).c_str());
  client.publish((String(mqtt_base) + "/tBojlerOUT").c_str(), String(tBojlerOut).c_str());
 
  digitalWrite(BUILTIN_LED, HIGH);
  return true;
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    DEBUG_WRITE('0');
  }
  DEBUG_PRINT(number);
}

bool tempMeas(void *) {
  DEBUG_PRINT(F("Requesting temperatures..."));
  dsSensors.requestTemperatures(); 
  DEBUG_PRINTLN(F("DONE"));

  tP1In         = dsSensors.getTempC(T1Addr);
  tP1Out        = dsSensors.getTempC(T2Addr);
  tP2In         = dsSensors.getTempC(T7Addr);
  tP2Out        = dsSensors.getTempC(T8Addr);
  tBojlerIn     = dsSensors.getTempC(T4Addr);
  tBojlerOut    = dsSensors.getTempC(T6Addr);
  tBojler1      = dsSensors.getTempC(T5Addr);
  tBojler2      = dsSensors.getTempC(T3Addr);
  
  DEBUG_PRINT(F("P1 In:"));
  DEBUG_PRINTLN(tP1In);
  DEBUG_PRINT(F("P1 Out:"));
  DEBUG_PRINTLN(tP1Out);
  DEBUG_PRINT(F("P2 In:"));
  DEBUG_PRINTLN(tP2In);
  DEBUG_PRINT(F("P2 Out:"));
  DEBUG_PRINTLN(tP2Out);
  DEBUG_PRINT(F("Bojler:"));
  DEBUG_PRINTLN(tBojler1);
  DEBUG_PRINT(F("Bojler In:"));
  DEBUG_PRINTLN(tBojlerIn);
  DEBUG_PRINT(F("Bojler Out:"));
  DEBUG_PRINTLN(tBojlerOut);
  DEBUG_PRINT(F("Control:"));

  
  //obcas se vyskytne chyba a vsechna cidla prestanou merit
  //zkusim restartovat sbernici
  // bool reset=false;
  // for (byte i=0; i<numberOfDevices; i++) {
    // //if (sensor[i]==0.0 || sensor[i]<-100.0) {
    // if (sensor[i]<-100.0) {
      // reset=true;
    // }
  // }
  // if (reset) {
    // dsInit();
  // }
  firstTempMeasDone = true;

  return true;
}

void dispRelayStatus() {
  lcd.setCursor(RELAY_STATUSX,RELAY_STATUSY);
  if (manualRelay==1) {
    lcd.print("MON");
  } else if (manualRelay==0) {
    lcd.print("MOF");
  } else if (relayStatus==RELAY_ON) {
    lcd.print(" ON");
  } else if (relayStatus==RELAY_OFF) {
    lcd.print("OFF");
  }
}

//---------------------------------------------M A I N  C O N T R O L ------------------------------------------------
void relay() {
  if ((tP1In >= SAFETY_ON) || (tP1Out >= SAFETY_ON) || (tP2In >= SAFETY_ON) || (tP2Out >= SAFETY_ON)) {
    relayStatus=RELAY_ON; 
    DEBUG_PRINTLN(F("SAFETY CONTROL!!!!"));
  } else if (manualRelay==2) {
    //-----------------------------------zmena 0-1--------------------------------------------
    if (relayStatus == RELAY_OFF && ((tP1Out - tBojler1) >= tDiffON || (tP2Out - tBojler1) >= tDiffON || (tP1In - tBojler1) >= tDiffON || (tP2In - tBojler1) >= tDiffON)) {
      lastOffOn = millis();
      relayStatus = RELAY_ON;
      changeRelay(relayStatus);
      sendRelayMQTT(1);
    //-----------------------------------zmena 1-0--------------------------------------------
    } else if (relayStatus == RELAY_ON && ((((tP2Out - tBojler1) < tDiffOFF)) && (millis() - DELAY_AFTER_ON >= lastOffOn))) { 
      DEBUG_PRINT(F("millis()="));
      DEBUG_PRINT(millis());
      DEBUG_PRINT(F(" delayAfterON="));
      DEBUG_PRINT(DELAY_AFTER_ON);
      DEBUG_PRINT(F(" lastOffOn="));
      DEBUG_PRINT(lastOffOn);
      DEBUG_PRINT(F(" tP2Out="));
      DEBUG_PRINT(tP2Out);
      // DEBUG_PRINT(F("tControl="));
      // DEBUG_PRINTLN(tControl);
      relayStatus = RELAY_OFF;
      changeRelay(relayStatus);
      sendRelayMQTT(0);
    }
  } else if (manualRelay==1) {
      relayStatus = RELAY_ON;
      changeRelay(relayStatus);
  } else if (manualRelay==0) {
      relayStatus = RELAY_OFF;
      changeRelay(relayStatus);
  }
  dispRelayStatus();
}


//---------------------------------------------D I S P L A Y ------------------------------------------------
void display() {
  if (displayType>=100) { 
    lcd.setCursor(POZ0X,POZ0Y);
  }
  // if (millis() > SHOW_INFO_DELAY + showInfo) {
    // showInfo = millis();
    // lcd.setCursor(0,3);
    // for (byte i=0;i<14;i++) {
      // PRINT_SPACE;
    // }
  // }
  
  if (displayType==DISPLAY_MAIN) {
    //    012345678901234567890
    //00  10  9  0  -0 -9.1  T    
    //01   555W   12.3kWh 124m                    
    //02   2.1l/m  55 45 48  0
    //03                123456
    if (firstTempMeasDone) {
      displayValue(TEMP1X,TEMP1Y, tP1In, 3, 0);
      displayValue(TEMP2X,TEMP2Y, tP1Out, 3, 0);
      displayValue(TEMP3X,TEMP3Y, tP2In, 3, 0);
      displayValue(TEMP4X,TEMP4Y, tP2Out, 3, 0);
      //displayValue(TEMP5X,TEMP5Y, tControl, 3, 1);
      displayValue(TEMP6X,TEMP6Y, tBojlerIn, 3, 0);
      displayValue(TEMP7X,TEMP7Y, tBojlerOut, 3, 0);
      displayValue(TEMP8X,TEMP8Y, tBojler1, 3, 0);
    }
    displayValue(POWERX,POWERY, (int)power, 4, 0);  //W
    lcd.print(F("W"));
    // if ((millis()-lastOff)>=DAY_INTERVAL) {
      // lcd.print(F("Bez slunce      "));
      // lcd.print((millis() - lastOff)/1000/3600);
      // lcd.print(F(" h"));
    // } else {
      //zobrazeni okamziteho vykonu ve W
      //zobrazeni celkoveho vykonu za den v kWh
      //zobrazeni poctu minut behu cerpadla za aktualni den
      //0123456789012345
      // 636 0.1234 720T
      
    displayValue(ENERGYX,ENERGYY, enegyWsTokWh(energyADay), 3, 0);  ////Ws -> kWh (show it in kWh)
    lcd.print(F("kWh"));

    displayValue(FLOWX,FLOWY, lMin, 3, 0);  
    lcd.print(F("l/m"));

    displayValue(RUNMINTODAY_X,RUNMINTODAY_Y, secOnDay / 60, 4, 0);  //min

    lcd.setCursor(CONTROLSENSORX, CONTROLSENSORY);
    //controlSensorBojler==1 ? lcd.print(F("B")) : lcd.print(F("R"));
#ifdef time
    lcd.setCursor(SUNANGLEX,SUNANGLEY);
    lcd.print("Uhel:");
    lcd.print(90- sunAngle[month()-1]);
    lcd.write(byte(0));
#endif    
  //} else if (displayType==DISPLAY_T_DIFF_ON) {
    //displayInfoValue('TDiffON', tDiffON, 'C');
  //} else if (displayType==DISPLAY_T_DIFF_OFF) { 
    //displayInfoValue('TDiffOFF', tDiffOFF, 'C');
  //} else if (displayType==DISPLAY_FLOW) {
    //displayInfoValue('Flow', lMin, 'l/min');
  //} else if (displayType==DISPLAY_MAX_IO_TEMP) {
    //lcd.setCursor(POZ0X,POZ0Y);
    //lcd.print(F("Max IN:"));
    //lcd.print(tMaxIn);
    //lcd.print(F("     "));
    //lcd.setCursor(0,1);
    //lcd.print(F("Max OUT:"));
    //lcd.print(tMaxOut);
    //lcd.print(F("     "));
  //} else if (displayType==DISPLAY_MAX_BOJLER) {
    //displayInfoValue('Max bojler', tMaxBojler, 'C');
  //} else if (displayType==DISPLAY_MAX_POWER_TODAY) { 
    //displayInfoValue('Max power today', maxPower, 'W');
  // } else if (displayType==DISPLAY_CONTROL_SENSOR) {
    // lcd.setCursor(POZ0X,POZ0Y);
    // lcd.print(F("Control sensor"));
    // lcd.setCursor(0,1);
    // lcd.print(F(" ["));
    // lcd.setCursor(0,2);
    // if (controlSensorBojler==1) {
      // lcd.print(tBojler1);
      // lcd.print(F("]   "));
      // lcd.print(F("Bojler"));
    // } else {
      // lcd.print(tRoom);
      // lcd.print(F("]   "));
      // lcd.print(F("Room"));
    // }
  }
}

//---------------------------------------------K E Y B O A R D ------------------------------------------------
void keyBoard() {
  char key = keypad.getKey();
  if (key!=NO_KEY){
    lcd.clear();
    //DEBUG_PRINTLN(key);
    /*
    Keyboard layout
    -----------
    | 1 2 3 A |
    | 4 5 6 B |
    | 7 8 9 C |
    | * 0 # D |
    -----------
   
    INFO
    1 - 
    2 - TDiffON
    3 - TDiffOFF
    4 - prutok
    5 - Max IN OUT temp
    6 - Max bojler
    B - clear display
    7 - Max power today
    8 - Control sensor
    9 - 
    C - RESTART
    * -
    0 - main display
    D - manual/auto
    */
    
    if (key=='1') { 
    }
    else if (key=='2') { 
      displayType=DISPLAY_T_DIFF_ON;
    }
    else if (key=='3') { 
      displayType=DISPLAY_T_DIFF_OFF;
    }
    else if (key=='4') {
      displayType=DISPLAY_FLOW;
    }
    else if (key=='5') { 
      displayType=DISPLAY_MAX_IO_TEMP;
    }
    else if (key=='6') { 
      displayType=DISPLAY_MAX_BOJLER;
    }
    else if (key=='B') {
      lcd.clear();
    }
    else if (key=='7') { 
      displayType=DISPLAY_MAX_POWER_TODAY;
    }
    else if (key=='8') { 
      displayType=DISPLAY_CONTROL_SENSOR;
    }
    else if (key=='9') { 
    }
    if (key=='C') {
      saveConfig();
      ESP.restart();
    }
    if (key=='*') {
      lcd.clear();
    }
    if (key=='0') { 
      displayType=DISPLAY_MAIN;
    }
    if (key=='D') {
      // manualON = !manualON;
      // if (manualON) {
        // manualRelay=1;
      // } else {
        // manualRelay=0;
      // }
      manualRelay++;
      if (manualRelay>2) {
        manualRelay=0;
      }
    }
    key = ' ';
  }
}

void dsInit(void) {
  dsSensors.begin();
  numberOfDevices = dsSensors.getDeviceCount();

  lcd.setCursor(0,1);
  lcd.print(numberOfDevices);
  DEBUG_PRINT(numberOfDevices);
  
  if (numberOfDevices==1) {
    DEBUG_PRINTLN(" sensor found");
    lcd.print(F(" sensor found"));
  } else {
    DEBUG_PRINTLN(" sensor(s) found");
    lcd.print(F(" sensors found"));
  }

  dsSensors.setResolution(12);
  dsSensors.setWaitForConversion(false);
}


void displayInfoValue(char text1, float value, char text2) {
  lcd.setCursor(POZ0X,POZ0Y);
  lcd.print(text1);
  lcd.setCursor(POZ0X,POZ1Y);
  lcd.print(value);
  lcd.print(" ");
  lcd.print(text2);
  lcd.print("          ");
}

float enegyWsTokWh(float e) {
  return e/3600.f/1000.f;
}

unsigned int getPower(float t1, float t2) {
  /*
  Q	0,00006	m3/s
  K	4184000	
  t vstup	56,4	°C
  t vystup	63,7	
  P = Q x K x (t1 - t2)	1832,592	W
  */
  return (lMin / 1000.0 / 60.0) * 4184000.0 * (t1 - t2);
  //return (float)energyKoef*(tBojlerOut-tBojlerIn); //in W
}


#ifdef flowSensor
bool calcFlow(void *) {
  // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
  //numberOfPulsesFlow = 31;
  lMin = (float)numberOfPulsesFlow / (CALC_DELAY / 1000.f) / 7.5f;
  //DEBUG_PRINT(F("Pulsu: "));
  //DEBUG_PRINTLN(numberOfPulsesFlow);
  //Serial.print(lMin, DEC); // Print litres/min
  //DEBUG_PRINTLN(F(" L/min"));
  numberOfPulsesFlow = 0; // Reset Counter
  return true;
}
#endif

void displayValue(int x, int y, float value, byte cela, byte des) {
  char buffer [18];
  if (des==0) {
    value = round(value);
  }
 
  char format[5];
  char cislo[2];
  itoa (cela, cislo, 10);
  strcpy(format, "%");
  strcat(format, cislo);
  strcat(format, "d");

  sprintf (buffer, format, (int)value); // send data to the buffer
  lcd.setCursor(x,y);
  lcd.print(buffer); // display line on buffer

  if (des>0) {
    lcd.print(F("."));
    lcd.print(abs((int)(value*(10*des))%(10*des)));
  }
}

#ifdef time
bool displayTime(void *) {
  lcd.setCursor(TIMEX, TIMEY); //col,row
  char buffer[6];
  if (showDoubleDot) {
    sprintf(buffer, "%02d:%02d", hour(), minute());
  } else {
    sprintf(buffer, "%02d %02d", hour(), minute());
  }
  lcd.print(buffer);
  showDoubleDot = !showDoubleDot;
  return true;
}
#endif

void sendDSAddrMQTT() {
  byte i;
  byte addr[8];
  char buffer[256];
  DynamicJsonDocument doc(1024);
  
  DEBUG_PRINTLN("Looking for 1-Wire devices...");
  while(onewire.search(addr)) {
    DEBUG_PRINTLN("\n\rFound \'1-Wire\' device with address:");
    String s = String(40);
    String s1 = String(2);
    for (i = 0; i < 8; i++) {
      DEBUG_PRINT("0x");
      s += "0x";
      if (addr[i] < 16) {
        DEBUG_PRINT('0');
        s += "0";
      }
      DEBUG_PRINTHEX(addr[i]);
      s1 = String(addr[i], HEX);
      s1.toUpperCase();
      s += s1;
      if (i < 7) {
        DEBUG_PRINT(", ");
        s += ", ";
      }
    }
    doc["Addr"]   = s;
    
    serializeJson(doc, buffer);
    client.publish((String(mqtt_base) + "/DSAddresses").c_str(), buffer);
    
    if (OneWire::crc8( addr, 7) != addr[7]) {
        DEBUG_PRINTLN("CRC is not valid!");
        return;
    }
  }
  
  onewire.reset_search();
  return;
}


bool reconnect(void *) {
  if (!client.connected()) {
    DEBUG_PRINT("Attempting MQTT connection...");
     // Attempt to connect
     if (client.connect(mqtt_base, mqtt_username, mqtt_key, (String(mqtt_base) + "/LWT").c_str(), 2, true, "offline", true)) {
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_tDiffON)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_tDiffOFF)).c_str());
      // client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_controlSensor)).c_str());
      // for (int i=0; i<NUMBER_OF_DEVICES; i++) {
        // client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_so) + String(i)).c_str());
      // }
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_restart)).c_str());
      // client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_sendSO)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_relay)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_netinfo)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_DSAddr)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_config_portal_stop)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_config_portal)).c_str());
      client.publish((String(mqtt_base) + "/LWT").c_str(), "online", true);
      DEBUG_PRINTLN("connected");
    } else {
      DEBUG_PRINT("disconected.");
      DEBUG_PRINT(" Wifi status:");
      DEBUG_PRINT(WiFi.status());
      DEBUG_PRINT(" Client status:");
      DEBUG_PRINTLN(client.state());
    }
  }
  return true;
}

