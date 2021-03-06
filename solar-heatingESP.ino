/*SOLAR-HEATING
--------------------------------------------------------------------------------------------------------------------------
SOLAR - control system for solar unit
Petr Fory pfory@seznam.cz
GIT - https://github.com/pfory/solar-heating
//Wemos D1 R2 & mini  !!!!!!!!!!!! 2M FS !!!!!!!!!!!!!!!!!!!!!!!!!!! - jinak se smaže nastavení z configu
vyřešeno void ICACHE_RAM_ATTR flow(); - POZOR na verzi desky esp8266 2.42+, nefunguje interrupt, až do vyřešení nepřecházet na vyšší verzi
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

#ifdef time
WiFiUDP EthernetUdp;
static const char     ntpServerName[]       = "tik.cesnet.cz";
TimeChangeRule        CEST                  = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule        CET                   = {"CET", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);
unsigned int          localPort             = 8888;  // local port to listen for UDP packets
time_t getNtpTime();
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

DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

unsigned int volatile pulseCount            = 0;
//unsigned long lastRunMin                    = 0;
    
uint32_t heartBeat                          = 0;
   
float tP2In                                 = 0; //input medium temperature to solar panel roof
float tP2Out                                = 0; //output medium temperature to solar panel roof`
float tP1In                                 = 0; //input medium temperature to solar panel drevnik
float tP1Out                                = 0; //output medium temperature to solar panel drevnik
float tRoom                                 = 0; //room temperature
float tBojler                               = 0; //boiler temperature
float tControl                              = 0; //temperature which is used as control temperature
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

//maximal temperatures
float tMaxIn                                = 0; //maximal input temperature (just for statistics)
float tMaxOut                               = 0; //maximal output temperature (just for statistics)
float tMaxBojler                            = 0; //maximal boiler temperature (just for statistics)

   
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

// enum modeDisplay                          {SETUP, INFO};
// modeDisplay displayMode                    = INFO;


//promenne ulozene v pameti (viz CFGFILE "/config.json")
byte            tDiffON                    = 5; //rozdil vystupni teploty panelu 1 tP1Out nebo panelu 2 tP2Out proti teplote bojleru nebo mistnosti tControl pri kterem dojde ke spusteni cerpadla
byte            tDiffOFF                   = 2; //rozdil vystupni teploty panelu 2 tP2Out proti teplote bojleru nebo mistnosti tControl pri kterem dojde k vypnuti cerpadla
byte            controlSensorBojler        = 1; //kontrolni cidlo 1 - Bojler 0 Room

byte            sensorOrder[NUMBER_OF_DEVICES];


byte sunAngle[12]                           = {17,23,32,44,55,62,63,58,48,37,26,19}; //uhel slunce nad obzorem kdyz je na jihu

bool isDebugEnabled() {
#ifdef verbose
  return true;
#endif // verbose
  return false;
}

//for LED status
Ticker ticker;

auto timer = timer_create_default(); // create a timer with default settings
Timer<> default_timer; // save as above


void tick() {
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}
  
//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  DEBUG_PRINTLN("Entered config mode");
  lcd.clear();
  lcd.print("Connect timeout, start config...");
  DEBUG_PRINTLN(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  DEBUG_PRINTLN(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.1, tick);
  drd.stop();
}


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
  
  if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_controlSensor)).c_str())==0) {
    printMessageToLCD(topic, val);
    DEBUG_PRINT("set control sensor to ");
    if (val.toInt()==1) {
      DEBUG_PRINTLN(F("Bojler"));
    } else {
      DEBUG_PRINTLN(F("Room"));
    }
    controlSensorBojler=val.toInt();
    saveConfig();
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_tDiffOFF)).c_str())==0) {
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
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_relay)).c_str())==0) {
    printMessageToLCD(topic, val);
    DEBUG_PRINT("set manual control relay to ");
    manualRelay = val.toInt();
    if (val.toInt()==1) {
      DEBUG_PRINTLN(F("ON"));
    } else {
      DEBUG_PRINTLN(F("OFF"));
    }
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_sendSO)).c_str())==0) {
   //} else if (strcmp(topic, mqtt_topic_sendSO)==0) {
    printMessageToLCD(topic, val);
    DEBUG_PRINT("send sensor order");
    void * a;
    sendSOMQTT(a);
    
  } else {
    for (int i = 0; i<NUMBER_OF_DEVICES; i++) {
      if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_so) + String(i)).c_str())==0) {
        printMessageToLCD(topic, val);
        DEBUG_PRINT("set sensor order ");
        DEBUG_PRINT(i);
        DEBUG_PRINT(" to ");
        sensorOrder[i]=val.toInt();
        DEBUG_PRINTLN(val.toInt());
        saveConfig();  
      }
    }
  }  
}

WiFiClient espClient;
PubSubClient client(espClient);

WiFiManager wifiManager;

#ifdef flowSensor
void ICACHE_RAM_ATTR flow();
#endif
//----------------------------------------------------- S E T U P -----------------------------------------------------------
void setup() {
  SERIAL_BEGIN;
  DEBUG_PRINT(F(SW_NAME));
  DEBUG_PRINT(F(" "));
  DEBUG_PRINTLN(F(VERSION));

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, RELAY_ON);

  pinMode(BUILTIN_LED, OUTPUT);
  ticker.attach(1, tick);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);
  wifiManager.setConnectTimeout(CONNECT_TIMEOUT);


  lcd.init();               // initialize the lcd 
  //lcd.noCursor();
  lcd.backlight();
  lcd.home();                   
  lcd.print(SW_NAME);  
  PRINT_SPACE
  lcd.print(VERSION);
  lcd.createChar(0, customChar);

  if (drd.detectDoubleReset()) {
    DEBUG_PRINTLN("Double reset detected, starting config portal...");
    lcd.clear();
    ticker.attach(0.2, tick);
    lcd.print("DRD, config...");

    if (!wifiManager.startConfigPortal(HOSTNAMEOTA)) {
      DEBUG_PRINTLN("failed to connect and hit timeout");
      lcd.print("con failed, timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      lcd.print("ESP reset!");
      delay(5000);
    }
  }

  pinMode(BUILTIN_LED, OUTPUT);
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

  rst_info *_reset_info = ESP.getResetInfoPtr();
  uint8_t _reset_reason = _reset_info->reason;
  DEBUG_PRINT("Boot-Mode: ");
  DEBUG_PRINTLN(_reset_reason);
  heartBeat = _reset_reason;

  /*
 REASON_DEFAULT_RST             = 0      normal startup by power on 
 REASON_WDT_RST                 = 1      hardware watch dog reset 
 REASON_EXCEPTION_RST           = 2      exception reset, GPIO status won't change 
 REASON_SOFT_WDT_RST            = 3      software watch dog reset, GPIO status won't change 
 REASON_SOFT_RESTART            = 4      software restart ,system_restart , GPIO status won't change 
 REASON_DEEP_SLEEP_AWAKE        = 5      wake up from deep-sleep 
 REASON_EXT_SYS_RST             = 6      external system reset 
  */
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  WiFi.printDiag(Serial);

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
  if (controlSensorBojler==1) {
    lcd.print("Bojler");
  } else {
    lcd.print("Room");
  }
  delay(2000);
  lcd.clear();
  
  lcd.print("Connecting to WiFi..");
  
  if (!wifiManager.autoConnect(AUTOCONNECTNAME, AUTOCONNECTPWD)) { 
    DEBUG_PRINTLN("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  } 

  lcd.print("WiFi connected.");
  sendNetInfoMQTT();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
#ifdef serverHTTP
  server.on ( "/", handleRoot );
  server.begin();
  DEBUG_PRINTLN ( "HTTP server started!!" );
#endif

#ifdef time
  DEBUG_PRINTLN("Setup TIME");
  EthernetUdp.begin(localPort);
  DEBUG_PRINT("Local port: ");
  DEBUG_PRINTLN(EthernetUdp.localPort());
  DEBUG_PRINTLN("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  
  printSystemTime();
#endif

#ifdef ota
  ArduinoOTA.setHostname(HOSTNAMEOTA);

  ArduinoOTA.onStart([]() {
    DEBUG_PRINTLN("Start updating ");
  });
  ArduinoOTA.onEnd([]() {
   DEBUG_PRINTLN("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_PRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_PRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) DEBUG_PRINTLN("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) DEBUG_PRINTLN("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) DEBUG_PRINTLN("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) DEBUG_PRINTLN("Receive Failed");
    else if (error == OTA_END_ERROR) DEBUG_PRINTLN("End Failed");
  });
  ArduinoOTA.begin();
#endif


  DEBUG_PRINT(F("tON:"));  
  DEBUG_PRINTLN(tDiffON);
  DEBUG_PRINT(F("tOFF:"));  
  DEBUG_PRINTLN(tDiffOFF);
  DEBUG_PRINT(F("Control:"));  
  //DEBUG_PRINT(controlSensor);
  if (controlSensorBojler==1) {
    DEBUG_PRINTLN("Bojler");
  } else {
    DEBUG_PRINTLN("Room");
  }

  keypad.begin();
  //keypad.addEventListener(keypadEvent); //add an event listener for this keypad  
  
  if (numberOfDevices>NUMBER_OF_DEVICES) {
    DEBUG_PRINTLN("ERROR - real number of devices DS18B20 > NUMBER_OF_DEVICES. Change variable NUMBER_OF_DEVICES in Configuration.h file!!!!!!!!");
  }

  //setup timers
  DEBUG_PRINTLN("Setup timers.");
  timer.every(SEND_DELAY, sendDataMQTT);
  timer.every(MEAS_DELAY, tempMeas);
  timer.every(CALC_DELAY, calcPowerAndEnergy);
#ifdef flowSensor
  timer.every(CALC_DELAY, calcFlow);
#endif
  timer.every(SENDSTAT_DELAY, sendStatisticMQTT);
#ifdef time
  timer.every(CALC_DELAY/2, displayTime);
#endif

  void * a;
  sendStatisticMQTT(a);

  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, HIGH);

  drd.stop();

  DEBUG_PRINTLN(F("Setup end."));
  lcd.clear();
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

  reconnect();
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
    tMaxOut=TEMP_ERR;
    tMaxIn=TEMP_ERR;
    tMaxBojler=TEMP_ERR;
  } else if (hour()>0) {
    todayClear = false;
  }
}
#endif


void changeRelay(byte status) {
  digitalWrite(RELAYPIN, status);
}

bool calcPowerAndEnergy(void *) {
  float t1;
  float t2;
  
  if (relayStatus==RELAY_ON) {  //pump is ON
    if (controlSensorBojler==1) {
      t1 = tBojlerIn;
      t2 = tBojlerOut;
    } else {
      t1 = tP2Out;
      t2 = tP1In;
    }
    if (t1>t2) {
      secOnDay++;
      power = getPower(t1, t2); //in W
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


void reconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    if (lastConnectAttempt == 0 || lastConnectAttempt + connectDelay < millis()) {
      DEBUG_PRINT("Attempting MQTT connection...");
      // Attempt to connect
      if (client.connect(mqtt_base, mqtt_username, mqtt_key)) {
        DEBUG_PRINTLN("connected");
        // Once connected, publish an announcement...
        client.subscribe((String(mqtt_base) + "/" + "tDiffON").c_str());
        client.subscribe((String(mqtt_base) + "/" + "tDiffOFF").c_str());
        client.subscribe((String(mqtt_base) + "/" + "controlSensorBojler").c_str());
        for (int i=0; i<NUMBER_OF_DEVICES; i++) {
          client.subscribe((String(mqtt_base) + "/" + "so" + String(i)).c_str());
        }
        client.subscribe((String(mqtt_base) + "/" + "restart").c_str());
        client.subscribe((String(mqtt_base) + "/" + "sorder").c_str());
        client.subscribe((String(mqtt_base) + "/" + "manualRelay").c_str());
      } else {
        lastConnectAttempt = millis();
        DEBUG_PRINT("failed, rc=");
        DEBUG_PRINTLN(client.state());
      }
    }
  }
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
      (int)tBojler,
      abs((tBojler - (int)tBojler) * 100),
      (int)tRoom,
      abs((tRoom - (int)tRoom) * 100)
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
  doc["controlSensor"]           = controlSensorBojler;
  for (int i=0; i<numberOfDevices; i++) {
    //doc["sensorOrder[" + String(i) + "0]"]          = sensorOrder[i];
    doc["sensorOrder[" + String(i) + "]"]          = sensorOrder[i];
  }

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
        controlSensorBojler = doc["controlSensor"];
        DEBUG_PRINT(F("control sensor: "));
        controlSensorBojler==1 ? DEBUG_PRINTLN(" bojler") : DEBUG_PRINTLN(" room");
        
        for (int i=0; i<numberOfDevices; i++) {
          sensorOrder[i] = doc["sensorOrder[" + String(i) + "]"];
          DEBUG_PRINT(F("sensorOrder["));
          DEBUG_PRINT(i);
          DEBUG_PRINT(F("]:"));
          DEBUG_PRINTLN(sensorOrder[i]);
        }
       
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
  SenderClass sender;
  sender.add("relayChange", akce);
 
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  digitalWrite(BUILTIN_LED, HIGH);
}


bool sendSOMQTT(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Sensor order"));
  SenderClass sender;
  for (int i=0; i<NUMBER_OF_DEVICES; i++) {
    sender.add("so" + String(i), sensorOrder[i]);
  }

  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  digitalWrite(BUILTIN_LED, HIGH);
  return true;
}

void sendNetInfoMQTT() {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Net info"));

  SenderClass sender;
  sender.add("IP",              WiFi.localIP().toString().c_str());
  sender.add("MAC",             WiFi.macAddress());
  
  DEBUG_PRINTLN(F("Calling MQTT"));
  
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  digitalWrite(BUILTIN_LED, HIGH);
  return;
}


bool sendDataMQTT(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Data"));
  
  SenderClass sender;
  sender.add("tP1IN", tP1In);
  sender.add("tP1OUT", tP1Out);
  sender.add("tP2IN", tP2In);
  sender.add("tP2OUT", tP2Out);
  sender.add("prutok", lMin);
  sender.add("sPumpSolar/status", relayStatus==RELAY_ON ? 1 : 0);
  sender.add("tRoom", tRoom);
  sender.add("tBojler", tBojler);
  sender.add("tBojlerIN", tBojlerIn);
  sender.add("tBojlerOUT", tBojlerOut);
  sender.add("tControl", tControl);
  DEBUG_PRINTLN(F("Calling MQTT"));
  
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  digitalWrite(BUILTIN_LED, HIGH);
  return true;
}

bool sendStatisticMQTT(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Statistic"));

  SenderClass sender;
  sender.add("VersionSWSolar", VERSION);
  sender.add("Napeti",  ESP.getVcc());
  sender.add("HeartBeat", heartBeat++);
  if (heartBeat % 10 == 0) sender.add("RSSI", WiFi.RSSI());
  sender.add("tDiffON", tDiffON);
  sender.add("tDiffOFF", tDiffOFF);
  sender.add("controlSensorBojler", controlSensorBojler);
  
  DEBUG_PRINTLN(F("Calling MQTT"));
  
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
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
  for (byte i=0;i<numberOfDevices; i++) {
    float tempTemp=(float)TEMP_ERR;
    for (byte j=0;j<10;j++) { //try to read temperature ten times
      tempTemp = dsSensors.getTempC(tempDeviceAddresses[i]);
      if (tempTemp>=-55) {
        break;
      }
    }
    sensor[i] = tempTemp;
  }

  tP1In       = sensor[sensorOrder[0]]; //so0
  tP1Out      = sensor[sensorOrder[1]]; //so1
  tP2In       = sensor[sensorOrder[2]]; //so2
  tP2Out      = sensor[sensorOrder[3]]; //so3
  tBojlerIn   = sensor[sensorOrder[4]]; //so4
  tBojlerOut  = sensor[sensorOrder[5]]; //so5
  tRoom       = sensor[sensorOrder[6]]; //so6
  tBojler     = sensor[sensorOrder[7]]; //so7
/*
  tP1In       = sensor[7]; //so0
  tP1Out      = sensor[4]; //so1
  tP2In       = sensor[2]; //so2
  tP2Out      = sensor[1]; //so3
  tBojlerIn   = sensor[5]; //so4
  tBojlerOut  = sensor[3]; //so5
  tRoom       = sensor[6]; //so6
  tBojler     = sensor[0]; //so7
*/
  
  //controlSensorBojler==1 ? tControl = tBojler : tControl = tRoom;
  controlSensorBojler==1 ? tControl = tBojler : tControl = ROOMTEMPON;
  
  if (tP2Out>tMaxOut)       tMaxOut      = tP2Out;
  if (tP2In>tMaxIn)         tMaxIn       = tP2In;
  if (tBojler>tMaxBojler)   tMaxBojler   = tBojler;

  
  DEBUG_PRINT(F("P1 In:"));
  DEBUG_PRINTLN(tP1In);
  DEBUG_PRINT(F("P1 Out:"));
  DEBUG_PRINTLN(tP1Out);
  DEBUG_PRINT(F("P2 In:"));
  DEBUG_PRINTLN(tP2In);
  DEBUG_PRINT(F("P2 Out:"));
  DEBUG_PRINTLN(tP2Out);
  DEBUG_PRINT(F("Room:"));
  DEBUG_PRINTLN(tRoom);
  DEBUG_PRINT(F("Bojler:"));
  DEBUG_PRINTLN(tBojler);
  DEBUG_PRINT(F("Bojler In:"));
  DEBUG_PRINTLN(tBojlerIn);
  DEBUG_PRINT(F("Bojler Out:"));
  DEBUG_PRINTLN(tBojlerOut);
  DEBUG_PRINT(F("Control:"));
  DEBUG_PRINTLN(tControl);

  
  //obcas se vyskytne chyba a vsechna cidla prestanou merit
  //zkusim restartovat sbernici
  bool reset=false;
  for (byte i=0; i<numberOfDevices; i++) {
    //if (sensor[i]==0.0 || sensor[i]<-100.0) {
    if (sensor[i]<-100.0) {
      reset=true;
    }
  }
  if (reset) {
    dsInit();
  }
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
    if (relayStatus == RELAY_OFF && ((tP1Out - tControl) >= tDiffON || (tP2Out - tControl) >= tDiffON || (tP1In - tControl) >= tDiffON || (tP2In - tControl) >= tDiffON)) {
      lastOffOn = millis();
      relayStatus = RELAY_ON;
      changeRelay(relayStatus);
      sendRelayMQTT(1);
    //-----------------------------------zmena 1-0--------------------------------------------
    } else if (relayStatus == RELAY_ON && ((((tP2Out - tControl) < tDiffOFF)) && (millis() - DELAY_AFTER_ON >= lastOffOn))) { 
      DEBUG_PRINT(F("millis()="));
      DEBUG_PRINT(millis());
      DEBUG_PRINT(F(" delayAfterON="));
      DEBUG_PRINT(DELAY_AFTER_ON);
      DEBUG_PRINT(F(" lastOffOn="));
      DEBUG_PRINT(lastOffOn);
      DEBUG_PRINT(F(" tP2Out="));
      DEBUG_PRINT(tP2Out);
      DEBUG_PRINT(F("tControl="));
      DEBUG_PRINTLN(tControl);
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
      displayValue(TEMP5X,TEMP5Y, tControl, 3, 1);
      displayValue(TEMP6X,TEMP6Y, tBojlerIn, 3, 0);
      displayValue(TEMP7X,TEMP7Y, tBojlerOut, 3, 0);
      displayValue(TEMP8X,TEMP8Y, tBojler, 3, 0);
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
    controlSensorBojler==1 ? lcd.print(F("B")) : lcd.print(F("R"));
#ifdef time
    lcd.setCursor(SUNANGLEX,SUNANGLEY);
    lcd.print("Uhel:");
    lcd.print(90- sunAngle[month()-1]);
    lcd.write(byte(0));
#endif    
  } else if (displayType==DISPLAY_T_DIFF_ON) {
    displayInfoValue('TDiffON', tDiffON, 'C');
  } else if (displayType==DISPLAY_T_DIFF_OFF) { 
    displayInfoValue('TDiffOFF', tDiffOFF, 'C');
  } else if (displayType==DISPLAY_FLOW) {
    //displayInfoValue('Flow', lMin, 'l/min');
  } else if (displayType==DISPLAY_MAX_IO_TEMP) {
    lcd.setCursor(POZ0X,POZ0Y);
    lcd.print(F("Max IN:"));
    lcd.print(tMaxIn);
    lcd.print(F("     "));
    lcd.setCursor(0,1);
    lcd.print(F("Max OUT:"));
    lcd.print(tMaxOut);
    lcd.print(F("     "));
  } else if (displayType==DISPLAY_MAX_BOJLER) {
    displayInfoValue('Max bojler', tMaxBojler, 'C');
  } else if (displayType==DISPLAY_MAX_POWER_TODAY) { 
    //displayInfoValue('Max power today', maxPower, 'W');
  } else if (displayType==DISPLAY_CONTROL_SENSOR) {
    lcd.setCursor(POZ0X,POZ0Y);
    lcd.print(F("Control sensor"));
    lcd.setCursor(0,1);
    lcd.print(F(" ["));
    lcd.setCursor(0,2);
    if (controlSensorBojler==1) {
      lcd.print(tBojler);
      lcd.print(F("]   "));
      lcd.print(F("Bojler"));
    } else {
      lcd.print(tRoom);
      lcd.print(F("]   "));
      lcd.print(F("Room"));
    }
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

  // Loop through each device, print out address
  for (byte i=0;i<numberOfDevices; i++) {
      // Search the wire for address
    if (dsSensors.getAddress(tempDeviceAddress, i)) {
      memcpy(tempDeviceAddresses[i],tempDeviceAddress,8);
    }
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
/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  //IPAddress ntpServerIP; // NTP server's ip address
  IPAddress ntpServerIP = IPAddress(195, 113, 144, 201);

  while (EthernetUdp.parsePacket() > 0) ; // discard any previously received packets
  DEBUG_PRINTLN("Transmit NTP Request");
  // get a random server from the pool
  //WiFi.hostByName(ntpServerName, ntpServerIP);
  DEBUG_PRINT(ntpServerName);
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = EthernetUdp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      DEBUG_PRINTLN("Receive NTP Response");
      EthernetUdp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      DEBUG_PRINT("Seconds since Jan 1 1900 = " );
      DEBUG_PRINTLN(secsSince1900);

      // now convert NTP time into everyday time:
      DEBUG_PRINT("Unix time = ");
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // print Unix time:
      DEBUG_PRINTLN(epoch);
	  
      TimeChangeRule *tcr;
      time_t utc;
      utc = epoch;
      
      return CE.toLocal(utc, &tcr);
      //return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  DEBUG_PRINTLN("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  EthernetUdp.beginPacket(address, 123); //NTP requests are to port 123
  EthernetUdp.write(packetBuffer, NTP_PACKET_SIZE);
  EthernetUdp.endPacket();
}

void printSystemTime(){
  DEBUG_PRINT(day());
  DEBUG_PRINT(".");
  DEBUG_PRINT(month());
  DEBUG_PRINT(".");
  DEBUG_PRINT(year());
  DEBUG_PRINT(" ");
  print2digits(hour());
  DEBUG_PRINT(":");
  print2digits(minute());
  DEBUG_PRINT(":");
  print2digits(second());
}

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