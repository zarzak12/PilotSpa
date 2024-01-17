/*
 *    
 *    Main board: ESP32
 *  
 *    SPA display controller for Balboa system with VL801D
 *    
 */

#include <ArduinoHA.h>
#include "Balboa_GS_Interface.h"  // https://github.com/MagnusPer/Balboa-GS510SZ
#include "SPIFFS.h"
#include "time.h"
#include <ElegantOTA.h>
#include <EEPROM.h>
#include <Ticker.h>

// Adresse de début dans l'EEPROM pour stocker la configuration
#define EEPROM_CONFIG_ADDRESS 0

#define setClockPin 23  // CHANGE ME
#define setReadPin 22   // CHANGE ME
#define setWritePin 13  // CHANGE ME

struct ConfigData {
  char ssid[32];  // Remplacez la taille par la taille maximale attendue
  char password[64];  // Ajustez en fonction de vos besoins
  char mqttIp[16];  // Par exemple, "xxx.xxx.xxx.xxx"
  char mqttUsername[32];
  char mqttPassword[32];
  char adminUsername[32];
  char adminPassword[32];
};

ConfigData config;

//Constants
const char *wifi_hostname = "SPA_Balboa";
const int mqtt_port = 1883;                           // MQTT Broker PORT, default is 1883 but can be anything.
const char *softApSsid = "OpenSpa-config";            // SSID pour le mode SoftAP
const char *softApPassword = "openspa2801";           // Mot de passe pour le mode SoftAP

Ticker wifiReconnectTicker;
bool softAPMode = false;

const char *www_username = "admin";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600 * 1;
const int daylightOffset_sec = 3600 * 0;

//Globals
bool debug = true;  // If true activate debug values to write to serial port
String datetime;

const unsigned long ReportTimerMillis = 5000;  // Timer in milliseconds to report mqtt topics
unsigned long ReportTimerPrevMillis = 0;        // Store previous millis

byte mac[] = { 0x00, 0x10, 0xFA, 0x6E, 0x32, 0x4A };  // Leave this value, unless you own multiple hot tubs

//Initialize components
WiFiClient espClient;  // Setup WiFi client definition WiFi
HADevice device(mac, sizeof(mac));
HAMqtt mqtt(espClient, device, 30);
BalboaInterface Balboa(setClockPin, setReadPin, setWritePin);  // Setup Balboa interface
AsyncWebServer server(80);

const int led = 2;

HASensor display("Display");
HASensorNumber waterTemp("waterTemp", HANumber::PrecisionP1);
HASensorNumber paramTemp("paramTemp", HANumber::PrecisionP1);
HABinarySensor heater("Heater");
HABinarySensor pump1("Pump1");
HABinarySensor pump2("Pump2");
HABinarySensor blower("Blower");
HABinarySensor lights("Light");
HABinarySensor stdMode("StdMode");
HABinarySensor ecnMode("EcnMode");
HABinarySensor slpMode("SlpMode");
HABinarySensor iceMode("IceMode");
HASensor displayMode("DisplayMode");
HABinarySensor filtration("Filtration");
HABinarySensor filter1("Filter1");
HABinarySensor filter2("Filter2");

HABinarySensor button("Button");

HASensor ipAdd("IpAddress");
HABinarySensor wifiConn("WifiConn");
HASensor rssiWifi("RSSIWifi");

HAButton pump1Button("Pump1");
HAButton pump2Button("Pump2");
HAButton blowerButton("Blower");
HAButton lightsButton("Light");
HAButton tempUpButton("TempUp");
HAButton tempDownButton("TempDown");
HAButton modeButton("Mode");

HAHVAC hvac(
  "temp",
  HAHVAC::TargetTemperatureFeature);

String bufferDisplayData;
void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%1X", num);
  //Serial.print(hexCar);
  bufferDisplayData += hexCar;
}

void onTargetTemperatureCommand(HANumeric temperature, HAHVAC *sender) {
  float temperatureFloat = temperature.toFloat();

  Serial.print("Target temperature: ");
  Serial.println(temperatureFloat);

  Balboa.updateTemperature(temperatureFloat);

  sender->setTargetTemperature(temperature);  // report target temperature back to the HA panel
}

void onModeCommand(HAHVAC::Mode mode, HAHVAC *sender) {
  Serial.print("Mode: ");
  if (mode == HAHVAC::OffMode) {
    Serial.println("off");
  } else if (mode == HAHVAC::AutoMode) {
    Serial.println("auto");
  } else if (mode == HAHVAC::CoolMode) {
    Serial.println("cool");
  } else if (mode == HAHVAC::HeatMode) {
    Serial.println("heat");
  } else if (mode == HAHVAC::DryMode) {
    Serial.println("dry");
  } else if (mode == HAHVAC::FanOnlyMode) {
    Serial.println("fan only");
  }

  sender->setMode(mode);  // report mode back to the HA panel
}

/**************************************************************************/
/* Setup WiFi connection                                                  */
/**************************************************************************/

void writeConfigToEEPROM(const ConfigData& config) {
  EEPROM.begin(sizeof(config));
  EEPROM.put(EEPROM_CONFIG_ADDRESS, config);
  EEPROM.commit();
  EEPROM.end();
}

ConfigData readConfigFromEEPROM() {
  ConfigData config;
  EEPROM.begin(sizeof(ConfigData));
  EEPROM.get(EEPROM_CONFIG_ADDRESS, config);
  EEPROM.end();
  return config;
}

void configureSoftAP() {
  // Configuration du mode SoftAP
  
  Serial.println("\n[*] Creating AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(softApSsid, softApPassword);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());

  softAPMode = true;

  const char * user_softap;
  const char * password_softap;

  if (config.ssid[0] == '\0') {
    user_softap = www_username;
    password_softap = softApPassword;
  } else {
    // Des données sont présentes, tenter une connexion Wi-Fi
    user_softap = config.adminUsername;
    password_softap = config.adminPassword;
  }

  // Configurez le serveur web pour gérer la configuration WiFi
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    File config_html = SPIFFS.open("/config.html");
    if (!config_html) {
      Serial.println("Failed to open file for reading");
      return;
    }
    
    String config_html_brut;
    
    if (config_html.available()) {
      config_html_brut = config_html.readString();
    }
  
    config_html.close();
  
    config_html_brut.replace("$ssid", config.ssid); 
    config_html_brut.replace("$password", config.password);
    config_html_brut.replace("$ip", config.mqttIp);
    config_html_brut.replace("$mqttUsername", config.mqttUsername);
    config_html_brut.replace("$mqttPassword", config.mqttPassword);
    config_html_brut.replace("$adminUsername", config.adminUsername);
    config_html_brut.replace("$adminPassword", config.adminPassword);
  
    request->send(200, "text/html", config_html_brut);
  }).setAuthentication(user_softap, password_softap);

  // Gestion d'une requête POST pour la configuration
  server.on("/save-config", HTTP_POST, [](AsyncWebServerRequest *request){
    // Créer une structure ConfigData avec les données du formulaire
    ConfigData newConfig;

    memset(&newConfig, 0, sizeof(newConfig));  // Initialise toutes les zones mémoire à zéro
    writeConfigToEEPROM(newConfig);

    // Fonction pour extraire et convertir la valeur d'un paramètre
    auto extractParamValue = [request, &newConfig](const char *paramName, char *destination, size_t destinationSize) {
        if (request->hasParam(paramName, true)) {
            AsyncWebParameter *p = request->getParam(paramName, true);
            String value = p->value();
            Serial.println(String(paramName) + " received: " + value);
            // Convertir la valeur en const char* et copier dans la structure ConfigData
            if (value.length() < destinationSize) {
                value.toCharArray(destination, destinationSize);
            } else {
                Serial.println("Error: String too long for " + String(paramName));
            }
        } else {
            Serial.println("Error: " + String(paramName) + " not found in parameters");
        }
    };

    // Extraire et convertir les valeurs de chaque paramètre
    extractParamValue("ssid", newConfig.ssid, sizeof(newConfig.ssid));
    extractParamValue("password", newConfig.password, sizeof(newConfig.password));
    extractParamValue("ip", newConfig.mqttIp, sizeof(newConfig.mqttIp));
    extractParamValue("mqttUsername", newConfig.mqttUsername, sizeof(newConfig.mqttUsername));
    extractParamValue("mqttPassword", newConfig.mqttPassword, sizeof(newConfig.mqttPassword));
    extractParamValue("adminUsername", newConfig.adminUsername, sizeof(newConfig.adminUsername));
    extractParamValue("adminPassword", newConfig.adminPassword, sizeof(newConfig.adminPassword));

    // Appeler la méthode pour écrire dans l'EEPROM
    writeConfigToEEPROM(newConfig);

    // Redémarrez l'ESP32 pour appliquer les nouveaux paramètres
    ESP.restart();
  });

// Démarrage du serveur
server.begin();

}

bool connectToWiFi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);

  // Attendez la connexion pendant 10 secondes
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    if (debug){ 
      Serial.print("WiFi.status(): "); 
      Serial.print(WiFi.status()); 
      Serial.print("   WiFi retry: "); 
      Serial.println(attempts); } 
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    // La connexion WiFi a réussi
    return true;
  } else {
    // La connexion WiFi a échoué
    return false;
  }
}

void setup_wifi() {

  /*  WiFi status return values and meaning 
        WL_IDLE_STATUS      = 0,
        WL_NO_SSID_AVAIL    = 1,
        WL_SCAN_COMPLETED   = 2,
        WL_CONNECTED        = 3,
        WL_CONNECT_FAILED   = 4,
        WL_CONNECTION_LOST  = 5,
        WL_WRONG_PASSWORD   = 6,
        WL_DISCONNECTED     = 7 */

  if (debug) {
    Serial.print("WiFi.status(): ");
    Serial.println(WiFi.status());
  }

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(wifi_hostname);
  
  #ifdef ESP32
    WiFi.setSleep(false);
  #else
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
  #endif
  
  // Tentez de vous connecter au WiFi avec les informations stockées
  if (connectToWiFi(config.ssid, config.password)) {
    // La connexion WiFi a réussi
    if (debug) {
      Serial.print("WiFi connected: ");
      Serial.println(WiFi.localIP());
    }
  } else {
    // La connexion WiFi a échoué, passez en mode SoftAP
    configureSoftAP();
  }  

  // Démarrage du ticker pour la vérification périodique de la connexion Wi-Fi
  wifiReconnectTicker.attach(60, checkWiFiConnection);

}

/**************************************************************************/
/* Subscribe to MQTT topic                                                */
/**************************************************************************/

void onButtonPress(HAButton *sender) {

  // Handling incoming messages

  Serial.println("Commande HA activée!");
  Serial.println(sender->getName());

  String s_payload = sender->getName();

  if (s_payload == "Temp Up") {
    Balboa.writeDisplayData = true;
    Balboa.writeTempUp = true;
  } else if (s_payload == "Temp Down") {
    Balboa.writeDisplayData = true;
    Balboa.writeTempDown = true;
  } else if (s_payload == "Light") {
    Balboa.writeDisplayData = true;
    Balboa.writeLight = true;
  } else if (s_payload == "Circulation pump") {
    Balboa.writeDisplayData = true;
    Balboa.writePump1 = true;
  } else if (s_payload == "Pump2") {
    Balboa.writeDisplayData = true;
    Balboa.writePump2 = true;
  } else if (s_payload == "Blower") {
    Balboa.writeDisplayData = true;
    Balboa.writeBlower = true;
  } else if (s_payload == "Mode") {
    Balboa.writeDisplayData = true;
    Balboa.writeMode = true;
  } else if (s_payload == "Stop") {
    Balboa.stop();
  } else if (s_payload == "Reset") {
    ESP.restart();
  }
}

void setup_HA() {
  device.setName("Hottub");
  device.setSoftwareVersion("1.2");
  device.setManufacturer("Balboa");
  device.setModel("SF273");

  waterTemp.setUnitOfMeasurement("°C");
  waterTemp.setDeviceClass("temperature");
  waterTemp.setName("Water temperature");

  paramTemp.setUnitOfMeasurement("°C");
  paramTemp.setDeviceClass("temperature");
  paramTemp.setName("Param temperature");

  display.setName("Display");
  heater.setName("Heater");
  displayMode.setName("Display Mode");

  pump1.setName("Circulation pump");
  pump1Button.setName("Circulation pump");
  pump1Button.onCommand(onButtonPress);

  pump2.setName("Pump2");
  pump2Button.setName("Pump2");
  pump2Button.onCommand(onButtonPress);

  blower.setName("Blower");
  blowerButton.setName("Blower");
  blowerButton.onCommand(onButtonPress);

  lights.setName("Light");
  lightsButton.setName("Light");
  lightsButton.onCommand(onButtonPress);

  stdMode.setName("Standard Mode");
  ecnMode.setName("Eco Mode");
  slpMode.setName("Sleep Mode");
  iceMode.setName("Ice Mode");
  filtration.setName("Filtration");
  filter1.setName("Filter 1");
  filter2.setName("Filter 2");

  button.setName("Button spa");

  tempUpButton.setName("Temp Up");
  tempUpButton.onCommand(onButtonPress);

  tempDownButton.setName("Temp Down");
  tempDownButton.onCommand(onButtonPress);

  modeButton.setName("Mode");
  modeButton.onCommand(onButtonPress);

  // configure HVAC (optional)
  hvac.setName("Temp");
  hvac.setMinTemp(26);
  hvac.setMaxTemp(40);
  hvac.setTempStep(0.5);
  //hvac.onModeCommand(onModeCommand);
  hvac.onTargetTemperatureCommand(onTargetTemperatureCommand);

  ipAdd.setName("IP");
  ipAdd.setIcon("mdi:network-outline");
  wifiConn.setName("Status");
  wifiConn.setIcon("mdi:network");
  rssiWifi.setName("WiFi Signal");
  rssiWifi.setIcon("mdi:wifi");
  rssiWifi.setUnitOfMeasurement("dBm");

  mqtt.begin(config.mqttIp, config.mqttUsername, config.mqttPassword);
}

String index_html_display;
String index_html_brut;
File index_html;

void handleRoot(AsyncWebServerRequest *request, String page) {
  char tmpWater[6];
  dtostrf(Balboa.waterTemperature, 3, 1, tmpWater);
  String tmpWaterStr(tmpWater);

  char tmpParam[6];
  dtostrf(Balboa.setTemperature, 3, 1, tmpParam);
  String tmpParamStr(tmpParam);


  index_html = SPIFFS.open(page);
  if (!index_html) {
    Serial.println("Failed to open file for reading");
    return;
  }

  if (index_html.available()) {
    index_html_brut = index_html.readString();
  }

  index_html.close();

  index_html_brut.replace("$datetime", datetime);

  index_html_brut.replace("$tempParam", tmpParamStr);

  index_html_brut.replace("$tempReel", tmpWaterStr);

  if (Balboa.displayMode == " Std") {
    index_html_brut.replace("$mode", "Standard");
  } else if (Balboa.displayMode == " Ecn") {
    index_html_brut.replace("$mode", "Economie");
  } else if (Balboa.displayMode == " SLP") {
    index_html_brut.replace("$mode", "Sleep");
  } else if (Balboa.displayMode == " 1CE") {
    index_html_brut.replace("$mode", "Ice");
  } else {
    index_html_brut.replace("$mode", Balboa.displayMode);
  }

  if (Balboa.LCD_display.startsWith(" ")) {
    index_html_brut.replace("$lcd", Balboa.displayMode);
  } else {
    index_html_brut.replace("$lcd", Balboa.LCD_display.c_str());
  }

  if (Balboa.displayPump1) {
    index_html_brut.replace("$pump1", "Activée");
  } else {
    index_html_brut.replace("$pump1", "désactivée");
  }

  if (Balboa.displayPump2) {
    index_html_brut.replace("$pump2", "Activée");
  } else {
    index_html_brut.replace("$pump2", "désactivée");
  }

  if (Balboa.displayBlower) {
    index_html_brut.replace("$blower", "Activée");
  } else {
    index_html_brut.replace("$blower", "désactivée");
  }

  if (Balboa.displayLight) {
    index_html_brut.replace("$light", "Activée");
  } else {
    index_html_brut.replace("$light", "désactivée");
  }

  request->send(200, "text/html", index_html_brut);
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Page non trouvée");
}

// Fonction pour configurer le serveur web
void setupWebServer() {
  // Page racine
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    handleRoot(request,"/index.html");
  }).setAuthentication(config.adminPassword, config.adminPassword);

  // Page de pilotage
  server.on("/pilotage", HTTP_GET, [](AsyncWebServerRequest *request){
    handleRoot(request,"/pilotage.html");
  }).setAuthentication(config.adminPassword, config.adminPassword);

  // Fichier CSS
  server.on("/w3.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/w3.css", "text/css");
  });

  // Fichier CSS personnalisé
  server.on("/perso.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/perso.css", "text/css");
  });

  // Favicon
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/favicon.ico", "image/ico");
  });

  // Page non trouvée personnalisée
  server.onNotFound([](AsyncWebServerRequest *request){
    handleNotFound(request);
  });

  server.on("/parametrer-temperature", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("tempParametree", true)) {
        AsyncWebParameter* p = request->getParam("tempParametree", true);
        Serial.println("Data received: " + p->value());

        const char *tempParametree = p->value().c_str();
        float temperature = atof(tempParametree);

        Balboa.updateTemperature(temperature);
        request->send(200, "text/plain", "Température paramétrée avec succès !");
    } else {
        request->send(404, "text/plain", "Température paramétrée en erreur !");
    }
  });


  // Gérer la route POST pour contrôler les pompes
  server.on("/controler-pompe", HTTP_POST, [](AsyncWebServerRequest *request){

    if(request->hasParam("pompe", true)) {
      AsyncWebParameter* p = request->getParam("pompe", true);
      Serial.println("Data received: " + p->value());

      const char * strPompe = p->value().c_str();

      // Faites quelque chose avec la commande de contrôle de la pompe
      if (strPompe == "blower") {
        Balboa.writeDisplayData = true;
        Balboa.writeBlower = true;
      } else if (strPompe == "pump1") {
        Balboa.writeDisplayData = true;
        Balboa.writePump1 = true;
      } else if (strPompe == "pump2") {
        Balboa.writeDisplayData = true;
        Balboa.writePump2 = true;
      } else if (strPompe == "mode") {
        Balboa.writeDisplayData = true;
        Balboa.writeMode = true;
      } else if (strPompe == "light") {
        Balboa.writeDisplayData = true;
        Balboa.writeLight = true;
      }
  
      request->send(200, "text/plain", "Commande de pompe traitée avec succès !");
    
    } else {
      Serial.println("No data received");
      request->send(404, "text/plain", "Commande de pompe Erreur!");
    }

    
  });

  // Gestion du serveur NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

/**************************************************************************/
/* Setup                                                                  */
/**************************************************************************/

void setup() {

  if (debug) {
    Serial.begin(115200);
    Serial.println("Welcome to SPA - Balboa system");
  }

  config = readConfigFromEEPROM();
  setup_wifi();
  setup_HA();
  Serial.begin(115200);
  Balboa.begin();

  /* SPIFFS config */
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Configuration du serveur web
  setupWebServer();

  /************************NE PAS TOUCHER ***********************************/
  /*handling uploading firmware file */
  ElegantOTA.begin(&server,config.adminPassword, config.adminPassword);  // Start ElegantOTA
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  /**************************************************************************/

  server.begin();
  if (debug) { Serial.println("HTTP server started"); }

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
}

/**************************************************************************/
/* Main loop                                                              */
/**************************************************************************/

void loop() {

  Balboa.loop();
  mqtt.loop();
  ElegantOTA.loop();

  if (millis() - ReportTimerPrevMillis > ReportTimerMillis) {

    ReportTimerPrevMillis = millis();

    //MQTT Send
    display.setValue(Balboa.LCD_display.c_str());
    displayMode.setValue(Balboa.displayMode.c_str());
    waterTemp.setValue(Balboa.waterTemperature);
    paramTemp.setValue(Balboa.setTemperature);
    heater.setState(Balboa.displayHeater);
    pump1.setState(Balboa.displayPump1);
    pump2.setState(Balboa.displayPump2);
    //pump3.setState(Balboa.displayPump3);
    blower.setState(Balboa.displayBlower);
    lights.setState(Balboa.displayLight);
    filtration.setState(Balboa.displayFiltration);
    filter1.setState(Balboa.displayFilter1);
    filter2.setState(Balboa.displayFilter2);

    button.setState(Balboa.displayButton);

    //Serial.println(WiFi.localIP());
    //Serial.println(WiFi.RSSI());
    ipAdd.setValue(WiFi.localIP().toString().c_str());
    rssiWifi.setValue(String(WiFi.RSSI()).c_str());
    if(WiFi.status() != WL_CONNECTED){
      wifiConn.setState(false);
    }
    else
    {
      wifiConn.setState(true);
    }
    

    if (Balboa.displayMode == " Std") {
      stdMode.setState(true);
      ecnMode.setState(false);
      slpMode.setState(false);
      iceMode.setState(false);
    } else if (Balboa.displayMode == " Ecn") {
      stdMode.setState(false);
      ecnMode.setState(true);
      slpMode.setState(false);
      iceMode.setState(false);
    } else if (Balboa.displayMode == " SLP") {
      stdMode.setState(false);
      ecnMode.setState(false);
      slpMode.setState(true);
      iceMode.setState(false);
    } else if (Balboa.displayMode == " 1CE") {
      stdMode.setState(false);
      ecnMode.setState(false);
      slpMode.setState(false);
      iceMode.setState(true);
    } else {
      stdMode.setState(false);
      ecnMode.setState(false);
      slpMode.setState(false);
      iceMode.setState(false);
    }

    hvac.setCurrentTemperature(Balboa.waterTemperature);
    hvac.setCurrentTargetTemperature(Balboa.setTemperature);

    //Web Server send
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
    }
    //Serial.println(&timeinfo, "%d/%m/%Y %H:%M:%S");

    char str[20];
    strftime(str, sizeof str, "%d/%m/%Y %H:%M", &timeinfo);
    datetime = String(str);
    Serial.println(datetime);

    if (debug) {

      //testDisplayDebug();

      Serial.print("Température : ");
      Serial.println(Balboa.waterTemperature);
      Serial.print("Température param : ");
      Serial.println(Balboa.setTemperature);
      Serial.print("Etat Pompe brassage : ");
      Serial.println(Balboa.displayPump1);
      Serial.print("Etat Pompe 2 : ");
      Serial.println(Balboa.displayPump2);
      Serial.print("Etat Blower : ");
      Serial.println(Balboa.displayBlower);

      Serial.print("display button : ");
      Serial.println(Balboa.displayButton);

      if (Balboa.LCD_display.startsWith(" ")) {
        Serial.print("Display Mode : ");
        Serial.println(Balboa.displayMode);
      } else {
        Serial.print("Display temp : ");
        Serial.println(Balboa.LCD_display.c_str());
      }

      int i;

      Serial.print("displayDataBuffer : ");
      for (i = 0; i < sizeof(Balboa.displayDataBuffer); i++) {
        printHex(Balboa.displayDataBuffer[i]);
      }
      Serial.println(bufferDisplayData);
      bufferDisplayData = "";
    }
  }
}

void checkWiFiConnection() {
  if (!softAPMode && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Reconnecting...");
    WiFi.reconnect();
    // Attendez que la connexion soit établie...
  }
}
