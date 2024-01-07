// *****************************************************
// Firmware per Scent ex Machina di Integra Fragrances
// -----------------------------------------------------
// Processore: Espressif ESP32-WROOM-32
// *****************************************************

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <ArduinoJson.h>  // Gestisce il formato JSON
#include <Base64.h>
#include "esp_task_wdt.h"
#include <Preferences.h>
Preferences settings;

// PROCESSORE
String chipId;
const uint8_t *bleaddr_u8;  // Identificativo MAC del modulo Bluetooth (array di interi)

#define FWBUILD_DATE 20231031  // yyyymmvvv [y = year, m = month, v = version]
const short FWversion = 6;


/*  GEQO CONFIG ----------------------------------------------------------*/
String geqo_url = "iot.integra-fragrances.com";
String geqo_dbname = "integra";
String geqo_machine = "001";
String geqo_user = "user";
String geqo_pwd = "135792468";
String geqo_api = "integrasem";
String geqo_align = "60";
String geqo_zone = "Europe/Rome";
String iotName = "";


/*  WEB CONFIG -----------------------------------------------------------*/
const char *www_username = "admin";
const char *www_password = "1234";

/*  ETH CONFIG -----------------------------------------------------------*/
const bool EthEnable = true;
IPAddress EthIP(0, 0, 0, 0);
IPAddress EthGW(0, 0, 0, 0);
IPAddress EthSM(255, 255, 255, 0);
IPAddress EthDNS(8, 8, 8, 8);
byte mac[6];

/*  WIFI CONFIG ----------------------------------------------------------*/
const bool WifiEnable = true;
#define WifiTimeout 10000;
const char *WifiDefaultSSID = "SEM";
const char *WifiDefaultPassword = "12345678";

/* GPRS CONFIG -----------------------------------------------------------*/
const bool GPRSEnable = true;
String GPRS_PIN = "1503"; //1503 ""
String GPRS_APN = "TM"; // "cmw500.rohde-schwarz.com" ; //"TM";  //iliad "mobile.vodafone.it";
String GPRS_LOGIN = "";
String GPRS_PASSWORD = "";

/* LORA CONFIG -----------------------------------------------------------*/
const bool LORAEnable = false;


/* CORE MANAGER -----------------------------------------------------------*/
TaskHandle_t Task1;
TaskHandle_t Task2;

/* OTA MANAGER -----------------------------------------------------------*/
#define NO_OTA_NETWORK
#include <ArduinoOTA.h>
#include <Wire.h>

/* TIME NTP RTC       ----------------------------------------------------*/
#include <Time.h>
#include "zones.h"
#include <ESP32Time.h>
ESP32Time rtc;
#include "timestamp32bits.h"
timestamp32bits stamp = timestamp32bits();

char daysOfTheWeekEN[7][4] = {
  "Sun",
  "Mon",
  "Tue",
  "Wed",
  "Thu",
  "Fri",
  "Sat"
};
char daysOfTheWeekIT[7][4] = {
  "Dom",
  "Lun",
  "Mar",
  "Mer",
  "Gio",
  "Ven",
  "Sab"
};

/* BLE MANAGER -----------------------------------------------------------*/
//#include <BLEDevice.h>      // Controller Bluetooth
//#include <BLEServer.h>      // Server Bluetooth
//#include "esp_bt_device.h"
//#include <BLEUtils.h>       // Istruzioni aggiuntive Bluetooth


/* TIMER   ---------------------------------------------------------------*/
String jsonString;

unsigned int PREV_MILLIS_EV1 = 0;
unsigned int END_MILLIS_EV1 = 0;
unsigned int PREV_MILLIS_EV2 = 0;
unsigned int END_MILLIS_EV2 = 0;
unsigned int PREV_MILLIS_PMP = 0;
unsigned int PREV_MILLIS_BTN = 0;
unsigned int PREV_MILLIS_CONN = 0;
unsigned int PREV_MILLIS_SECOND = 0;

/* PIN   ---------------------------------------------------------------*/
#define PMPPWM 4   // PWM pompa
#define PMPRUN 15  // Comando pompa
#define FANPWM 26  // Comando ventola di diffusione
#define EVPIN1 13  // Elettrovalvola 1 (LED integrato)
#define EVPIN2 27  // Elettrovalvola 2 (LED integrato)
#define REDLED 25  // LED rosso
//#define EXPMSB 32  // 2^1 per espansione
//#define EXPLSB 33  // 2^0 per espansione
#define BUTTON 36  // Pulsante integrato
#define PMPVOL 35  // Tensione di uscita della pompa
#define PROTEC 39  // Assorbimento generale di corrente
#define ECSPIN 5   // Chip-Select pin per controller Ethernet

// PARAMETRI
#define WDT_TIMEOUT 550       // Timeout del watch-dog [s]
#define ConnectionInt 3600    // Intervallo di ricerca reti e connessione [s]
#define PAR_SerBaud 115200    // Velocità della comunicazione seriale. [300 - 2000000 baud]
#define PAR_T0Int 1000        // Specifica ogni quanto tempo chiamare la routine di interrupt sul Timer 0, dove vengono generati i clock di sistema. [microsecondi]
#define PAR_AIRes 10          // Risoluzione degli ingressi analogici. [8 - 12 bit. Se si modifica, correggere i parametri di conversione della lettura]
#define PAR_ADCAtt 0          // Attenuazione degli ingressi analogici. [0 = OFF, 1 = 2.5dB (1V = 2086), 2 = 6dB (1V = 2975), 3 = 11dB (1V = 3959)]
#define PAR_Spikes 1          // Numero di spikes consecutivi tollerabili sugli ingressi analogici. [n]
#define PAR_IFactor 0.01      // Fattore di conversione della lettura di corrente. [Corrente = LetturaAnalogica * QuestoValore]
#define PAR_VFactor 0.02      // Fattore di conversione della lettura di tensione. [Tensione = LetturaAnalogica * QuestoValore]
#define PAR_MaxVoltage 17.0   // Tensione oltre la quale la sovralimentazione della pompa è disattivata. [V]
#define PAR_FanLAG 3          // Ritardo accensione / spegnimento della ventola di diffusione. [1/10s]
#define PAR_SpikeLAG 500      // Per questo tempo, il limite di corrente verrà triplicato per consentire lo spunto alla pompa. [millisecondi]
#define PAR_EVStartTime 10    // Per questo tempo, viene inviato alla EV il duty al 100%, prima di passare a quello del EVDutyCicle. [1/10s]
#define PAR_EVRestartTime 10  // dopo questo tempo, viene reinviato alla EV il duty al 100%, per poi passare a quello del EVDutyCicle. [1s]
#define PAR_PMPCurrDelay 10   // dopo questo tempo inizia il monitoraggio della corrente della pompa [1/10s]

#define FANPWM_CH 2       // Canale 2 su Timer 1
#define PMPPWM_CH 3       // Canale 3 su Timer 1
#define EVPWM1_CH 4       // Canale 4 su Timer 2
#define EVPWM2_CH 5       // Canale 5 su Timer 2
#define PWM_FRQ_PM 32768  // Frequenza PWM della pompa (Hz)
#define PWM_FRQ_EV 512    // Frequenza PWM delle EV 1 e 2 (Hz)
#define PWM_RES 8         // Risoluzione PWM (bits)

float AirSpeedTrig = 0.0;
float AirSpeed     = 0.0;


int EV1_TON = 3000;
int EV2_TON = 3000;
int PMP_TON = 3000;
int EVDutyCicle = 20;    // EVDutyCicle in %
int FanDutyCicle = 100;  // FanDutyCicle in %
int PumpDutyCicle = 0;   // Duty-cycle per il PWM di sovralimentazione della pompa. [0 = 11.5V 10 = 12.5 V  20 = 13.6 V  30 = 15V su carico da 15ohm]
int PumpCurrent = 0;

bool EV1_ON = 0;
bool EV2_ON = 0;
bool O_Pompa = 0;  // Pompa compressore
bool buttonState = 1;
bool PumpState = 0;

/* RFID ------------------------------------------------------------------*/
/* MONITOR I2C SDA SCL  --------------------------------------------------*/
/* BATTERY PIN  ----------------------------------------------------------*/

/* BUZZER  ---------------------------------------------------------------*/
//#include "Tone32.h"
//#define BUZZER_PIN 33
//#define BUZZER_CHANNEL 0

/* WIFI LAN --------------------------------------------------------------*/
#include <esp_wifi.h>
#include <WiFi.h>
bool WifiUsable = false;

/* ETHERNET LAN ----------------------------------------------------------*/
#include <SPI.h>
#include <Ethernet.h>
bool EthUsable = false;

/* GPRS LAN --------------------------------------------------------------*/
//SoftwareSerial gprsSerial(10, 11); // RX, TX pins on Arduino
#define MODEM_TX 11
#define MODEM_RX 10
#define TINY_GSM_MODEM_SIM7600  // GSM module
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 1024
#endif
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200
#define TINY_GSM_DEBUG Serial
#define SerialGSM Serial2  // Modulo GSM
#include <TinyGsmClient.h>
TinyGsm modem(SerialGSM);
bool ModemUsable = false;
bool GPRSUsable = false;

/* LORA   ---------------------------------------------------------------
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
#define LORA_DIO1 35
#define LORA_DIO2 34
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             5        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
double txNumber;
int16_t rssi,rxSize;
bool lora_idle=true;
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
*/
bool LORAUsable = false;

/* COMMON TCP ------------------------------------------------------------*/
#include <ESPmDNS.h>
#include <DNSServer.h>
WiFiClient w_client;
EthernetClient e_client;
TinyGsmClient g_client(modem, 0);
DNSServer dnsServer;
String callURL(const String &Server, const String &url, const String &body);
String convertObjectToString(const void *data);

/* SERVER WEB ------------------------------------------------------------*/
#include <aWOT.h>
EthernetServer EthServer(80);
WiFiServer WifServer(80);
Application app;

/* HTTP WEB --------------------------------------------------------------*/
#include <ArduinoHttpClient.h>

/* TIME SYNC RTC      ----------------------------------------------------*/
uint8_t bin2bcd(uint8_t val) {
  return val + 6 * (val / 10);
}
uint8_t bcd2bin(uint8_t val) {
  return val - 6 * (val >> 4);
}
void syncTimeFromUTC(String TZString = "Europe/Rome", time_t utcTimestamp = 0) {
  Serial.println("TIME Syncing time ...");
  if (TZString.isEmpty()) return;

  unsetenv("TZ");
  tzset();

  if (utcTimestamp > 1689263334) {
    Serial.println("TIME Syncing time RTC INT...");
    rtc.setTime(utcTimestamp);

    Wire.beginTransmission(0x68);
    //il primo byte stabilisce il registro iniziale da scivere
    Wire.write((byte)0x00);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 1);
    uint8_t ch; //,d;
    ch = Wire.read();
    //d = Wire.read();
    if (ch & 0x80) {
      ch = 0x80;
    } else {
      ch = 0x00;
    }
    //specifico il tempo e la data
    Wire.beginTransmission(0x68);
    Wire.write(byte(0x00));
    Wire.write(bin2bcd(rtc.getSecond() | ch));    //1° byte SECONDI da 0x00 a 0x59
    Wire.write(bin2bcd(rtc.getMinute()));         //2° byte MINUTI da 0x00 a 0x59
    Wire.write(bin2bcd(rtc.getHour(true)));       //3° byte ORE da 0x00 a 0x23
    Wire.write(bin2bcd(0));                       //4° byte GIORNO della settimana da 0x01 a 0x07
    Wire.write(bin2bcd(rtc.getDay()));            //5° byte GIORNO del mese da 0x01 a 0x31
    Wire.write(bin2bcd(rtc.getMonth() + 1));      //6° byte MESE da 0x01 a 0x12
    Wire.write(bin2bcd((rtc.getYear() - 2000)));  //7° byte ANNO 0x00 a 0x99
    Wire.endTransmission();
    Serial.println("TIME Time updated in DS1307 in UTC!");
  }

  Serial.println("TIME Syncing time RTC EXT...");
  Wire.beginTransmission(0x68);
  Wire.write((byte)0x00);
  Wire.endTransmission();

  //richiedo 7 byte dal dispositivo con indirizzo 0x68
  Wire.requestFrom(0x68, 7);
  int sec = bcd2bin(Wire.read() & 0x7F);
  int min = bcd2bin(Wire.read());
  int hour = bcd2bin(Wire.read());
  int wday = bcd2bin(Wire.read());  //day of week
  int day = bcd2bin(Wire.read());
  int month = bcd2bin(Wire.read());
  int year = bcd2bin(Wire.read()) + 2000;
  rtc.setTime(sec, min, hour, day, month, year, 0);

  setenv("TZ", findTimeOffset(TZString), 1);
  tzset();
  Serial.println(rtc.getTime("RTC0: %A, %B %d %Y %H:%M:%S"));
  Serial.println("TIME Time synced from NTP!");
}

const char *findTimeOffset(const char *searchKey) {
  for (int i = 0; i < TimeZonesRows; i++) {
    if (strcmp(TimeZones[i][0], searchKey) == 0) {
      return TimeZones[i][1];
    }
  }
  return nullptr;
}
const char *findTimeOffset(const String &searchKey) {
  for (int i = 0; i < TimeZonesRows; i++) {
    if (searchKey.equals(TimeZones[i][0])) {
      return TimeZones[i][1];
    }
  }
  return nullptr;
}

/* LIBRARY  -------------------------------------------------------------*/
#include "AirQualityBox.h"
#include "Anemometer.h"
#include "LevelTank.h"

/* SCHEDULER -------------------------------------------------------------*/
void WeeklySchedulerFunction(String FuncName, unsigned long duration, unsigned long repeatevery);
#include "WeeklyScheduler.h"
WeeklyScheduler scheduler;

/* WEB SERVER FUNCTION ---------------------------------------------------*/
char redirectURL[30];

void redirect(Request &req, Response &res) {
  Serial.println("WEB redirect");
  //Serial.println(req);
  if (!res.statusSent() && (redirectURL != "")) {
    res.set("Location", redirectURL);
    res.sendStatus(302);
    //sprintf(redirectURL,  "");
  }
}
void popup(Request &req, Response &res) {
  Serial.println("WEB popup");
  handle_root(req, res);
}
void notFound(Request &req, Response &res) {
  Serial.println("WEB notFound");
  Serial.print("Method: ");
  Serial.println(req.method());
  Serial.print("Path: ");
  Serial.println(req.path());
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<h1>Not Found!</h1>");
  res.println("</body>");
  res.println("</html>");
}
void handle_chat(Request &req, Response &res) {
  Serial.println("WEB chat");
  //Serial.println(req);
  if (!res.statusSent() && (redirectURL != "")) {
    res.set("Location", redirectURL);
    res.sendStatus(302);
    //sprintf(redirectURL,  "");
  }
}

void handle_css(Request &req, Response &res) {
  Serial.println("WEB handle_css");
  res.set("Content-Type", "text/css");
  res.println("body { font-family: Arial, sans-serif; }");
  res.println("h1 { color: #333; text-align: center; }");
  res.println(".container { border: 1px solid #ccc; margin: auto;  margin-bottom: 20px; margin: 5px; }");
  res.println(".logo { width:50px; height:50px; margin: 20px;border:1px solid black;display: flex;align-items: center;justify-content: center; }");
  res.println(".btn-group  {  margin:auto; text-align: center; }");
  res.println(".button { background-color: #1c87c9; border: none; color: white; padding: 15px 5px; border-radius: 6px; text-align: center; text-decoration: none; display: inline-block; font-size: 20px; margin: 4px 2px; width: 100px; cursor: pointer;}");
  res.println(".status  { border: 1px solid #ccc; margin: auto;  margin-bottom: 20px; margin: 5px; }");
  res.println(".status b { font-weight: bold;}");
  res.println(".status p { margin: 5px 0; }");

  res.end();
}

void handle_manifest(Request &req, Response &res) {
  Serial.println("WEB handle_css");
  res.set("Content-Type", "text/json");
  res.println("{");
  res.println("\"name\": \"SEM\",");
  res.println("\"short_name\": \"SEM\",");
  res.println("\"start_url\": \"index.html\",");
  res.println("\"display\": \"standalone\",");
  res.println("\"background_color\": \"#fdfdfd\",");
  res.println("\"theme_color\": \"#db4938\",");
  res.println("\"orientation\": \"portrait-primary\",");
  res.println("\"icons\": [");
  res.println("{\"src\": \"/images/icons/icon-96x96.png\",\"type\": \"image/png\", \"sizes\": \"96x96\"},");
  res.println("{\"src\": \"/images/icons/icon-128x128.png\",\"type\": \"image/png\", \"sizes\": \"128x128\"},");
  res.println("]");
  res.println("}");
  res.end();
}

void handle_root(Request &req, Response &res) {
  Serial.println("WEB handle_root");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<link rel='manifest' href='manifest.json' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<div id='block_container'>");
  res.println("<div class='logo'>");
  res.println("<h1>IF</h1>");
  res.println("</div>");
    res.println("<div >");
    res.println("<h1>Welcome to SEM</h1>");
    res.println("</div>");
    res.println("<div >");
    res.println("<h1>" + iotName + "</h1>");
    res.println("</div>");
    res.println("<div >");
    res.println("<h2>" + chipId + "</h1>");
    res.println("</div>");
  res.println("</div>");

  res.println("<div class='status'>");
  res.println("<b>Current Ver:</b>");
  res.println(FWversion);
  res.println("<BR>");

  res.print("<p>Date:");
  res.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
  res.println("</p>");

  res.print("<p>GMT Zone:");
  res.println(geqo_zone);
  res.println("</p>");

  res.print("<p>GMT Rule:");
  res.println(findTimeOffset(geqo_zone));
  res.println("</p>");

  res.print("<p>Align Every:");
  res.println(geqo_align);
  res.println("</p>");

  res.println("<p>MAC:" + chipId + "</p>");

  res.println("<p>Ethernet:");
  if (EthUsable) {
    res.println((String) "OK IP:" + Ethernet.localIP().toString().c_str() + "</p>");
  } else {
    res.println("KO</p>");
  }
  res.println("<p>Wifi:");
  if (WifiUsable) {
    res.println((String) "OK IP:" + WiFi.localIP().toString().c_str() + " RSSI:" + WiFi.RSSI() + "</p>");
  } else {
    res.println("KO</p>");
  }

  res.println("<p>GPRS:");
  if (GPRSUsable) {
    res.println((String) "OK IP:" + modem.localIP().toString().c_str() + " RSSI:" + modem.getSignalQuality() + "</p>");
  } else {
    res.println("KO</p>");
  }
  res.println("</div>");


  res.println("<div class='container' >");

  res.println("<div class='btn-group'>");
  res.println("<a href='/ethconfig' class='button'>Ethernet</a>");
  res.println("<a href='/wificonfig' class='button'>WiFi</a>");
  res.println("<a href='/gprsconfig' class='button'>GPRS</a>");
  res.println("</div>");

  res.println("<div class='btn-group'>");
  res.println("<a href='/srvconfig' class='button'>SetUP</a>");
  res.println("<a href='/schedulerconfig' class='button'>Scheduler</a>");
  res.println("<a href='/sensors' class='button'>Sensors</a>");
  res.println("</div>");

  res.println("<div class='btn-group'>");
  res.println("<a href='/upload' class='button'>Firmware</a>");
  res.println("<a href='/reset' class='button'>Reset</a>");
  res.println("<a href='/reconnect' class='button'>Reconnect</a>");
  res.println("</div>");

  res.println("</div>");


  res.println("</body></html>");
  res.end();
}
void handle_reset(Request &req, Response &res) {
  Serial.println("WEB handle_reset");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  // Restart the device
  ESP.restart();

  res.println("</body></html>");
  res.end();
}

void handle_SrvConfig(Request &req, Response &res) {
  Serial.println("WEB handle_setup");
    settings.begin("geqo", false);

  char value[64];
  int32_t valueint;
  char name[64];

  setStringToArray(name, "geqo_url");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_url");
    Serial.println(value);
    settings.putString("geqo_url", value);
    geqo_url = String(settings.getString("geqo_url", "").c_str());
  }

  setStringToArray(name, "geqo_machine");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_machine");
    Serial.println(value);
    settings.putString("geqo_machine", value);
    geqo_machine = String(settings.getString("geqo_machine", "").c_str());
  }

  setStringToArray(name, "geqo_user");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_user");
    Serial.println(value);
    settings.putString("geqo_user", value);
    geqo_user = String(settings.getString("geqo_user", "").c_str());
  }

  setStringToArray(name, "geqo_pwd");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_pwd");
    Serial.println(value);
    settings.putString("geqo_pwd", value);
    geqo_pwd = String(settings.getString("geqo_pwd", "").c_str());
  }

  setStringToArray(name, "geqo_dbname");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_dbname");
    Serial.println(value);
    settings.putString("geqo_dbname", value);
    geqo_dbname = String(settings.getString("geqo_dbname", "").c_str());
  }

  setStringToArray(name, "geqo_align");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_align");
    Serial.println(value);
    settings.putString("geqo_align", value);
    geqo_align = String(settings.getString("geqo_align", "").c_str());
  }

  setStringToArray(name, "geqo_api");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_api");
    Serial.println(value);
    settings.putString("geqo_api", value);
    geqo_api = String(settings.getString("geqo_api", "").c_str());
  }

  setStringToArray(name, "geqo_zone");
  if (req.form(name, 64, value, 64)) {
    Serial.print("geqo_zone");
    Serial.println(value);
    settings.putString("geqo_zone", value);
    geqo_zone = String(settings.getString("geqo_zone", "").c_str());
  }

  setStringToArray(name, "PumpDutyCicle");
  if (req.form(name, 64, value, 64)) {
    Serial.print("PumpDutyCicle");
    Serial.println(value);
    settings.putInt("PumpDutyCicle", atoi(value));
    PumpDutyCicle = settings.getInt("PumpDutyCicle", 0);
  }

  setStringToArray(name, "FanDutyCicle");
  if (req.form(name, 64, value, 64)) {
    Serial.print("FanDutyCicle");
    Serial.println(value);
    settings.putInt("FanDutyCicle", atoi(value));
    FanDutyCicle = settings.getInt("FanDutyCicle", 100);
  }

  setStringToArray(name, "AirSpeedTrig");
  if (req.form(name, 64, value, 64)) {
    Serial.print("AirSpeedTrig");
    Serial.println(value);
    settings.putFloat("AirSpeedTrig", atof(value) );
    AirSpeedTrig = settings.getFloat("AirSpeedTrig", 0);
  }
  settings.end();


  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  settings.begin("geqo", false);

  res.println("<form action='/srvconfig' method='POST'>");
  res.println("<table id='setup' border='1' style='border: 1px solid black;' >");
  res.println("<tr>");
    res.println("<th>Var</th>");
    res.println("<th>Value</th>");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>geqo_url</td>");
    res.println("<td><input name='geqo_url' length=32 value='" + geqo_url + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>geqo_machine</td>");
    res.println("<td><input name='geqo_machine' length=32 value='" + geqo_machine + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>geqo_user</td>");
    res.println("<td><input name='geqo_user'  length=32  value='" + geqo_user + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>geqo_pwd</td>");
    res.println("<td><input name='geqo_pwd' length=32 value='" + geqo_pwd + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>geqo_dbname</td>");
    res.println("<td><input name='geqo_dbname' length=32 value='" + geqo_dbname + "'></td>\n");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>geqo_align</td>");
    res.print("<td><input name='geqo_align' length=32 value='");
    res.print(geqo_align);
    res.println("'></td>");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>geqo_api</td>");
    res.println("<td><input name='geqo_api' length=32 value='" + geqo_api + "'></td>");
  res.println("</tr>");

  res.println("<tr>");
  res.println("<td>geqo_zone</td>");

  res.println("<td><select id='geqo_zone' name='geqo_zone'>");
  for (int i = 0; i < TimeZonesRows; i++) {
    res.print("<option value='");
    res.print(TimeZones[i][0]);
    res.print("'");
    if (geqo_zone.equals(TimeZones[i][0])) res.print(" selected");
    res.print(">");
    res.print(TimeZones[i][0]);
    res.println("</option>");
  }
  res.println("</select></td>");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>PumpPower</td>");
    res.println("<td><select id='PumpDutyCicle' name='PumpDutyCicle'>");
      if (PumpDutyCicle ==  0 ) res.print("<option value= '0' selected>11.5V</option>"); else res.print("<option value= '0'>11.5V</option>");
      if (PumpDutyCicle == 10 ) res.print("<option value='10' selected>12.5V</option>"); else res.print("<option value='10'>12.5V</option>");
      if (PumpDutyCicle == 20 ) res.print("<option value='20' selected>13.6V</option>"); else res.print("<option value='20'>13.6V</option>");
      if (PumpDutyCicle == 30 ) res.print("<option value='30' selected>15.0V</option>"); else res.print("<option value='30'>15.0V</option>");
    res.println("</select></td>");
  res.println("</tr>");

  res.println("<tr>");
    res.println("<td>FanPower</td>");
    res.println((String)"<td><input name='FanDutyCicle' length=32 value='" + FanDutyCicle + "'></td>");
  res.println("</tr>");


  res.println("<tr>");
    res.println("<td>AirSpeedMin</td>");
    res.println((String)"<td><input name='AirSpeedTrig' length=32 value='" + AirSpeedTrig + "'></td>");
  res.println("</tr>");

  res.println("</table>");
  res.println("<BR>");
  res.println("<input class='button' type='submit'>");
  res.println("</form>");

  res.println("</html>");
  res.end();
  settings.end();
}


void handle_WiFiConfig(Request &req, Response &res) {
  Serial.println("WEB handleWiFiConfig");
  settings.begin("wifi_config", false);
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<h1>Wi-Fi Configuration</h1>");

  res.println("<h2>Available Networks:</h2>");
  res.println("<ul>");
  int numOfNetworks = WiFi.scanNetworks();
  for (int i = 0; i < numOfNetworks; i++) {
    res.print("<li>");
    res.print(WiFi.SSID(i));
    res.print(" ->(dB)");
    res.print(WiFi.RSSI(i));
    res.println("</li>");
  }
  res.println("</ul>");

  res.println("<h2>Configure Wi-Fi:</h2>");
  res.println("<form action='/wificonfigset' method='POST'>");
  res.println("SSID:<br><input type='text' name='ssid' value='" + String(settings.getString("ssid", "").c_str()) + "'><br>");
  res.println("Password:<br><input type='password' name='password' value='" + String(settings.getString("password", "").c_str()) + "'><br>");
  res.println("<input type='submit' class='button' value='Submit'>");
  res.println("</form>");

  settings.end();
  res.end();
}
void handle_WiFiConfigSet(Request &req, Response &res) {
  Serial.println("WEB handle_WiFiConfigSet");
  settings.begin("wifi_config", false);

  char value[64];
  char name[64];
  setStringToArray(name, "ssid");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("ssid", value);
  }

  setStringToArray(name, "password");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("password", value);
  }

  connectToInet();
  handle_WiFiConfig(req, res);
}


void handle_EthConfig(Request &req, Response &res) {
  Serial.println("WEB handleEthConfig");
  settings.begin("eth_config", false);
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<h1>Eth Configuration</h1>");

  res.println("<ul>");
  res.println("</ul>");

  res.println("<h2>Configure Eth:</h2>");
  res.println("<form action='/ethconfigset' method='POST'>");
  res.println("IP:<br><input type='text'  name='IP'  value='" + String(settings.getString("IP", "")) + "'><br>");
  res.println("SM:<br><input type='text'  name='SM'  value='" + String(settings.getString("SM", "")) + "'><br><br>");
  res.println("GW:<br><input type='text'  name='GW'  value='" + String(settings.getString("GW", "")) + "'><br><br>");
  res.println("DNS:<br><input type='text' name='DNS' value='" + String(settings.getString("DNS", "")) + "'><br><br>");
  res.println("<input type='submit' class='button' value='Submit'>");
  res.println("</form>");

  res.end();
}
void handle_EthConfigSet(Request &req, Response &res) {
  Serial.println("WEB handleEthConfigSet");
  settings.begin("eth_config", false);

  char value[64];
  char name[64];

  setStringToArray(name, "IP");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("IP", value);
  }

  setStringToArray(name, "GW");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("GW", value);
  }

  setStringToArray(name, "SM");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("SM", value);
  }

  setStringToArray(name, "DNS");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    settings.putString("DNS", value);
  }

  settings.end();

  connectToInet();
  handle_EthConfig(req, res);
}


void handle_GprsConfig(Request &req, Response &res) {
  Serial.println("WEB handle_GprsConfig");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  settings.begin("gprs_config", false);

  res.println("<h1>GPRS Configuration</h1>");

  res.println("<ul>");

  if (modem.isNetworkConnected()) {
    res.println("Network connected");
    res.println("<br>");
  }

  if (modem.isGprsConnected()) {
    res.println("GPRS status: connected");
    res.println("<br>");
  }

  String ccid = modem.getSimCCID();
  res.println("CCID:" + ccid);
  res.println("<br>");

  String imei = modem.getIMEI();
  res.println("IMEI:" + imei);
  res.println("<br>");

  String imsi = modem.getIMSI();
  res.println("IMSI:" + imsi);
  res.println("<br>");

  String cop = modem.getOperator();
  res.println("Operator:" + cop);
  res.println("<br>");

  IPAddress modemIP = modem.localIP();
  res.println((String) "Local IP:" + modemIP.toString().c_str());
  res.println("<br>");

  int csq = modem.getSignalQuality();
  res.println("Signal quality:" + csq);
  res.println("<br>");

  res.println("</ul>");

  res.println("<h2>Configure GPRS:</h2>");
  res.println("<form action='/gprsconfigset' method='POST'>");
  res.println("PIN:<br><input type='text'      name='PIN'      value='" + String(GPRS_PIN) + "'><br>");
  res.println("APN:<br><input type='text'      name='APN'      value='" + String(GPRS_APN) + "'><br><br>");
  res.println("LOGIN:<br><input type='text'    name='LOGIN'    value='" + String(GPRS_LOGIN) + "'><br><br>");
  res.println("PASSWORD:<br><input type='text' name='PASSWORD' value='" + String(GPRS_PASSWORD) + "'><br><br>");
  res.println("<input type='submit' class='button' value='Submit'>");
  res.println("</form>");

  res.end();
}
void handle_GprsConfigSet(Request &req, Response &res) {
  Serial.println("WEB handle_GprsConfigSet");
  settings.begin("gprs_config", false);
  res.set("Content-Type", "text/html");

  char value[64];
  char name[64];

  setStringToArray(name, "PIN");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    GPRS_PIN = value;
    settings.putString("GPRS_PIN", value);
  }

  setStringToArray(name, "APN");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    GPRS_APN = value;
    settings.putString("GPRS_APN", value);
  }

  setStringToArray(name, "LOGIN");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    GPRS_LOGIN = value;
    settings.putString("GPRS_LOGIN", value);
  }

  setStringToArray(name, "PASSWORD");
  if (req.form(name, 64, value, 64)) {
    Serial.println(value);
    GPRS_PASSWORD = value;
    settings.putString("GPRS_PASSWORD", value);
  }

  settings.end();

  connectToInet();
  handle_GprsConfig(req, res);
}

void handle_reconnect(Request &req, Response &res) {
  Serial.println("WEB handle_align");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  /* ACTIVATE */
  res.println("<br><h1>Connessione Dati</h1><br>");
  res.println((String) "connectToInet" + "<br>");
  connectToInet();

  res.println("<br><h1>Trasmissione Dati</h1><br>");
  String URLString = (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId + "&" + "ver=" +FWversion +"&" + "hb=1";
  res.println("<pre>" + URLString + "</pre>");

  esp_task_wdt_reset();  // Reset del watch-dog
  String response = callURL(geqo_url, URLString, "");

  res.println("<br><h1>Ricezione Dati</h1><br>");
  res.println((String) "<pre>" + response + "</pre>");

  res.println("<br><h1>Syncing date time</h1><br>");
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("TIME SYNC:");
  if (!response.isEmpty()) {
    Serial.println("TIME HTTP:" + response);
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
      Serial.println((String) "TIME Scheduler Failed to parse JSON: " + error.c_str());
      res.println((String) "<h1>TIME Scheduler Failed to parse JSON: " + error.c_str() + "</h1>");
    } else {
      //timing update align
      Serial.println("convert json to var");
      settings.begin("geqo", false);

      int uValue = doc["U"].as<int>();
      if (uValue > 3600)  uValue = 3600;
      if (uValue < 10)  uValue = 10;

      geqo_align = String(uValue);
      Serial.println(geqo_align);
      settings.putString("geqo_align", geqo_align);

      int dValue = doc["D"].as<int>();
      geqo_zone = doc["Z"].as<String>();
      Serial.println(geqo_zone);
      settings.putString("geqo_zone", geqo_zone);
      settings.end();

      syncTimeFromUTC(geqo_zone, dValue);
      
      iotName =doc["DD"].as<String>();
      Serial.println(iotName);
      settings.putString("iotname",iotName);

      PumpDutyCicle = doc["PP"].as<int>();
      settings.putInt("PumpDutyCicle", PumpDutyCicle);

      FanDutyCicle = doc["FP"].as<int>();
      settings.putInt("FanDutyCicle", FanDutyCicle);

      AirSpeedTrig = doc["AS"];
      settings.putFloat("AirSpeedTrig", AirSpeedTrig);

      //date time align
      syncTimeFromUTC(geqo_zone, dValue);
      res.println("<h1>Time Synced</h1>");
    }
  } else {
    Serial.println("TIME HTTP: NO RESPONSE");
    syncTimeFromUTC(geqo_zone, 0);
  }

  res.end();
}

/* UPDATE --------------------------------------------------------------*/
#include <Update.h>
void handle_upload(Request &req, Response &res) {
  Serial.println("WEB handle_upload");

  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("  <h1>");
  res.println("    Compiled: " __DATE__ " " __TIME__);
  res.println("  </h1>");
  res.println("  <form id='form'>");
  res.println("    <input id='file' type='file'>");
  res.println("    <input type='submit' value='Send' />");
  res.println("  </form>");
  res.println("</body>");
  res.println("<script>");
  res.println("  const form = document.getElementById('form');");
  res.println("  form.onsubmit = function(e) {");
  res.println("    e.preventDefault();");
  res.println("    const body = document.getElementById('file').files[0];");
  res.println("    fetch('/update', { method: 'POST', body }).then((response) => {");
  res.println("      if (!response.ok) {");
  res.println("        return alert('File upload failed');");
  res.println("      }");
  res.println("      alert('File upload succeeded');");
  res.println("    });");
  res.println("  }");
  res.println("</script>");
  res.println("</html>");
}
void handle_update(Request &req, Response &res) {
  Serial.println("WEB handle_update");
  int contentLength = req.left();
  Serial.println("Dim:" + contentLength);

  if (strcmp(req.get("Expect"), "100-continue") == 0) {
    res.status(100);
  }

  Serial.println("OTA Update...");
  if (!Update.begin(contentLength)) {
    res.status(500);
    return Update.printError(req);
  }

  Serial.println("OTA Reboot...");
  unsigned long start = millis();
  while (!req.available() && millis() - start <= 5000) {}

  if (!req.available()) {
    return res.sendStatus(408);
  }

  Serial.println("OTA write...");
  if (Update.writeStream(req) != contentLength) {
    res.status(400);
    return Update.printError(req);
  }

  Serial.println("OTA end...");
  if (!Update.end(true)) {
    res.status(500);
    return Update.printError(req);
  }

  res.sendStatus(204);
}


void handle_Sensors(Request &req, Response &res) {
  Serial.println("WEB handle_align");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<br><h1>Out:</h1><br>");
    res.println("<a href='sensorsrx'>Get Settings From Server</a>");
    res.println("<table id='output' border='1' style='width:100%; border: 1px solid black;' >");
    res.println("<tr><th>Var</th><th>Value</th><th>Definition</th></tr>");
    res.println((String) "<tr><td>Pump</td><td>" + O_Pompa + "<td>OutPump</td></td></tr>");
    res.println((String) "<tr><td>PumpPower</td><td>" + PumpDutyCicle + "</td><td>0 = 11.5V 10 = 12.5 V  20 = 13.6 V  30 = 15V</td></tr>");
    res.println((String) "<tr><td>FanDutyCicle</td><td>" + FanDutyCicle + "</td><td>0-100%</td></tr>");
    res.println((String) "<tr><td>AirSpeedTrig</td><td>" + AirSpeedTrig + "</td><td>0 = NoUse, x=Check min </td></tr>");
    res.println((String) "<tr><td>EV1</td><td>" + EV1_ON + "</td><td>Out1</td></tr>");
    res.println((String) "<tr><td>EV2</td><td>" + EV2_ON + "</td><td>Out2</td></tr>");
    res.println("</table>");

  res.println("<br><h1>In (sensors):</h1><br>");
    String jsonStringWeb;
    jsonStringWeb = "";
    jsonStringWeb.concat(jsonString);
    jsonStringWeb.remove(0,1);
    jsonStringWeb.remove(jsonStringWeb.length()-1);
    jsonStringWeb.replace(",", "</td></tr><tr><td>");
    jsonStringWeb.replace(":", "</td><td>");
    jsonStringWeb.replace("\"", "");

    res.println("<table id='sensori' border='1' style='width:100%; border: 1px solid black;' >");
    res.println("<tr><th>Var</th><th>Value</th></tr>");
    res.println("<tr><td>");
    res.println(jsonStringWeb);
    res.println("</td></tr>");
    res.println("</table>");

  res.println("<br><h1>TxOnline:</h1><br>");
    res.println("<br><h1>TxData</h1><br>");
    String URLString = (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId + "&" + "ver=" +FWversion ;
    res.println("<pre>" + URLString + "</pre>");
    String result = callURL(geqo_url, URLString, jsonString);
    res.println("<br><h1>Result</h1><br>");
    res.println((String) "<pre>" + result + "</pre>");

  res.end();
}
void handle_SensorsRx(Request &req, Response &res) {
  Serial.println("WEB handle_SensorsRx");
  SchedulerRx();
  //call parent
  handle_Sensors(req, res);
}


void handle_SchedulerConfig(Request &req, Response &res) {
  Serial.println("WEB handle_SchedulerConfig");
  res.set("Content-Type", "text/html");

  res.println("<html lang='en'>");
  res.println("<head>");
  res.println("<meta charset='UTF-8' />");
  res.println("<meta name='viewport' content='width=device-width, initial-scale=1.0' />");
  res.println("<meta http-equiv='X-UA-Compatible' content='ie=edge' />");
  res.println("<link rel='stylesheet' href='style.css' />");
  res.println("<title>SEM</title>");
  res.println("</head>");
  res.println("<body>");

  res.println("<h1>SCHEDULER</h1>");
  res.println("<div style='width:100%; border:1px solid; float:left; clear:none;'>");

  // Display current time from RTC
  res.println("<b>Current</b><BR>");
  res.print("WeekDay:");
  //DAFARE rtc.
  res.println(daysOfTheWeekEN[1]);
  res.print("Date:");
  res.println(rtc.getTime("RTC0: %A, %B %d %Y %H:%M:%S"));
  res.print("<BR>");
  res.println("<a href='schedulerrx'>Get Scheduler From Server</a>");

  res.println("<form action='schedulerconfigset' method='post'>");
  res.println("<table id='udc' border='1' style='width:100%; border: 1px solid black;' >");
  res.println("<tr>");
  res.println("<td>WDay</td>");
  res.println("<td>Start</td>");
  res.println("<td>End</td>");
  res.println("<td>Repeat</td>");
  res.println("<td>Durate</td>");
  res.println("<td>Out</td>");
  res.println("</tr>");


  for (byte i = 0; i < MAX_TASKS; i++) {
    res.println("<tr>");

    byte dayOfWeek = scheduler.tasks[i].dayOfWeek;
    byte startHour = scheduler.tasks[i].startHour;
    byte startMinute = scheduler.tasks[i].startMinute;
    byte endHour = scheduler.tasks[i].endHour;
    byte endMinute = scheduler.tasks[i].endMinute;
    unsigned long repeatEvery = scheduler.tasks[i].repeatEvery;
    unsigned long duration = scheduler.tasks[i].duration;
    String callbackName = scheduler.tasks[i].callbackName;

    /**  GIORNO **/
    res.println("<td>");
    res.print((String) "<select id='DW" + i + "' name='DW" + i + "'>");

    for (int d = 0; d < 7; d++) {
      res.print((String) "<option value='" + d + "'");
      if (dayOfWeek == d) res.print(" selected");
      res.println((String) ">" + daysOfTheWeekEN[d] + "</option>");
    }
    res.println("</select>");
    res.println("</td>");

    /**  INIZIO **/
    res.println((String) "<td><input type='time' id='SH" + i + "' name='SH" + i + "' value='" + padTime(startHour, startMinute) + "'></td>");

    /**  FINE **/
    res.print((String) "<td><input type='time' id='EH" + i + "' name='EH" + i + "' value='" + padTime(endHour, endMinute) + "'></td>");

    /**  RIPETIZIONE **/
    res.print((String) "<td><input type='number' id='RE" + i + "' name='RE" + i + "' min=0 max=65535 value='" + repeatEvery + "'></td>");

    /**  DURATA **/
    res.print((String) "<td><input type='number' id='DD" + i + "' name='DD" + i + "' min=0 max=65535 value='" + duration + "'></td>");

    /**  FUNZIONE **/
    res.println((String) "<td><select id='CB" + i + "' name='CB" + i + "'>");
    if (callbackName == "EV1") {
      res.println("<option value='EV1' selected>EV1</option>");
    } else {
      res.println("<option value='EV1' >EV1</option>");
    }
    if (callbackName == "EV2") {
      res.println("<option value='EV2' selected>EV2</option>");
    } else {
      res.println("<option value='EV2' >EV2</option>");
    }
    if (callbackName == "MON") {
      res.println("<option value='MON' selected>MON</option>");
    } else {
      res.println("<option value='MON' >MON</option>");
    }
    if (callbackName == "") {
      res.println("<option value='' selected>None</option>");
    } else {
      res.println("<option value='' >None</option>");
    }
    res.println("</select>");
    res.println("</td>");

    res.println("</tr>");
  }

  res.println("</tr>");
  res.println("</table>");

  res.println("<input type='submit'  class='button' value='Submit'>");
  res.println("</form>");


  res.println("</div>");
  res.println("</body></html>");

  res.end();
}

void handle_SchedulerConfigSet(Request &req, Response &res) {
  Serial.println("WEB handle_SchedulerConfigSet");
  char var[64];
  scheduler.deleteAllTasks();
  char dayOfWeek[64];
  char startHourMin[64];
  char endHourMin[64];
  char repeatEvery[64];
  char duration[64];
  char callbackName[64];
  char myString[3] = { '0' };
  char numberString[2] = { '0' };
  for (int i = 0; i < MAX_TASKS; i++) {

    itoa(i, numberString, 10);

    myString[0] = { 'D' };
    myString[1] = { 'W' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, dayOfWeek, 64);
    Serial.println(dayOfWeek);

    myString[0] = { 'S' };
    myString[1] = { 'H' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, startHourMin, 64);
    Serial.println(startHourMin);

    myString[0] = { 'E' };
    myString[1] = { 'H' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, endHourMin, 64);
    Serial.println(endHourMin);

    myString[0] = { 'R' };
    myString[1] = { 'E' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, repeatEvery, 64);
    Serial.println(repeatEvery);

    myString[0] = { 'D' };
    myString[1] = { 'D' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, duration, 64);
    Serial.println(duration);

    myString[0] = { 'C' };
    myString[1] = { 'B' };
    myString[2] = numberString[0];
    myString[3] = { '\0' };
    req.form(myString, 3, callbackName, 64);
    Serial.println(callbackName);

    scheduler.addTask(atoi(dayOfWeek), startHourMin, endHourMin, atoi(repeatEvery), atoi(duration), callbackName);
    Serial.println('INSERTED');
  }
  //call parent
  handle_SchedulerConfig(req, res);
}
void handle_SchedulerRx(Request &req, Response &res) {
  Serial.println("WEB handle_SchedulerRx");
  SchedulerRx();
  //call parent
  handle_SchedulerConfig(req, res);
}

/* FUNCTION --------------------------------------------------------------*/
String ipToString(IPAddress ip) {
  String s = "";
  for (int i = 0; i < 4; i++)
    s += i ? "." + String(ip[i]) : String(ip[i]);
  return s;
}
String mac2String(byte ar[]) {
  String s;
  for (byte i = 0; i < 6; ++i) {
    char buf[3];
    sprintf(buf, "%02X", ar[i]);  // J-M-L: slight modification, added the 0 in the format for padding
    s += buf;
    if (i < 5) s += ':';
  }
  return s;
}
void setStringToArray(char *arr, const char *str) {
  size_t size = sizeof(str);
  strncpy(arr, str, size - 1);
  arr[size - 1] = '\0';  // Add the null terminator manually
}
void setStringToArray(char outputArray[], const String &inputString) {
  int arraySize = inputString.length();
  for (int i = 0; i < inputString.length(); i++) {
    outputArray[i] = inputString.charAt(i);
  }
  outputArray[inputString.length()] = '\0';
}
void setStringToArray(char outputArray[], const String &inputString, int arraySize) {
  if (arraySize < inputString.length() + 1) {
    return;
  }
  for (int i = 0; i < inputString.length(); i++) {
    outputArray[i] = inputString.charAt(i);
  }
  outputArray[inputString.length()] = '\0';
}
String convertObjectToString(const void *data) {
  size_t dataSize = sizeof(data);
  const byte *rawData = reinterpret_cast<const byte *>(data);
  String result;
  for (size_t i = 0; i < dataSize; ++i) {
    if (i > 0) {
      result += "&";
    }
    result += String(rawData[i]);
  }
  return result;
}

/* LORA --------------------------------------------------------------------- */

/* WEB CLIENT FUNCTION --------------------------------------------------*/
void connectToInet() {

  /* CONNECTION WIFI ---------------------------------------------------------------*/
  Serial.println("WIFI");
  esp_task_wdt_reset();  // Reset del watch-dog
  if (WifiEnable) {
    settings.begin("wifi_config", false);
    if (settings.getString("ssid", "").isEmpty()) {
      WifiUsable = false;
      Serial.print("WiFi APMode: ");

      //captive portal inganno dns mettendo esp32 come server dns
      IPAddress apIP(192, 168, 123, 1);
      IPAddress netMsk(255, 255, 255, 0);
      WiFi.softAPConfig(apIP, apIP, netMsk);
      WiFi.mode(WIFI_AP);
      WiFi.softAP((String)"" + WifiDefaultSSID + "_" + chipId, WifiDefaultPassword,  1);
      Serial.print("Access Point IP address: ");
      Serial.println(WiFi.softAPIP());

      //dns relay to page
      IPAddress ip = WiFi.softAPIP();
      sprintf(redirectURL, "http://%d.%d.%d.%d/popup", ip[0], ip[1], ip[2], ip[3]);
      app.get("/popup", &popup);
      //app.use(&redirect);     //<-- alla accensione qlc url viene inoltrato qui, ma poi va disattivato .. c'è da scoprire come
      dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
      dnsServer.start(53, "*", ip);
    } 
    else {
      Serial.print("WiFi connecting: ");
      WiFi.begin(settings.getString("ssid", "").c_str(), settings.getString("password", "").c_str());
      unsigned long timeout = millis() + WifiTimeout;
      while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
        delay(500);
        Serial.print(".");
      }
      if (WiFi.status() == WL_CONNECTED) {
        WifiUsable = true;
        Serial.println("Connected");
      } else {
        Serial.println("NOT Connected");
      }
    }
    Serial.print("My WiFi address: ");
    Serial.println(WiFi.localIP());
  }

  /* CONNECTION ETHERNET -----------------------------------------------------------*/
  Serial.println("ETHERNET");
  esp_task_wdt_reset();  // Reset del watch-dog
  if (EthEnable) {
    Serial.println("Ethernet init");
    Ethernet.init(5);  // MKR ETH Shield
    switch (Ethernet.linkStatus()) {
      case Unknown:
        EthUsable = false;
        Serial.println("Ethernet Unknown");
        break;
      case LinkON:
        Serial.println("Ethernet ON");
        settings.begin("eth_config", false);
        if (settings.getString("IP", "").isEmpty()) {
          Ethernet.begin(mac);
        } 
        else {
          //byte mac[] =    { 0x90, 0xA2, 0xDA, 0x10, 0x38, 0x53 };
          EthIP.fromString(settings.getString("IP", ""));
          EthGW.fromString(settings.getString("GW", ""));
          EthDNS.fromString(settings.getString("DNS", ""));
          EthSM.fromString(settings.getString("SM", ""));
          Ethernet.begin(mac, EthIP, EthGW, EthDNS, EthSM);
        }
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
          EthUsable = false;
          Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        } 
        else if (Ethernet.linkStatus() == LinkOFF) {
          EthUsable = false;
          Serial.println("Ethernet cable is not connected.");
        } 
        else {
          EthUsable = true;
          Serial.print("My Eth IP: ");
          Serial.println(Ethernet.localIP());
          Serial.print("My Eth MAC: ");
          for (int i = 0; i < 6; i++) {
            Serial.print(mac[i], HEX);
            if (i < 5) Serial.print(':');
          }
          Serial.println();
        }

        break;
      case LinkOFF:
        EthUsable = false;
        Serial.println("Ethernet OFF");
        break;
    }
  }

  /* CONNECTION GPRS ---------------------------------------------------------------*/
  Serial.println("GPRS");
  esp_task_wdt_reset();  // Reset del watch-dog
  if (GPRSEnable) {
    //per evitare che perda un mare di tempo a syncronizzare il modem se è gia a posto salta il test
    char APN[20];
    GPRS_APN.toCharArray(APN, GPRS_APN.length() + 1);
    char PASSWORD[20];
    GPRS_PASSWORD.toCharArray(PASSWORD, GPRS_PASSWORD.length() + 1);
    char LOGIN[20];
    GPRS_LOGIN.toCharArray(LOGIN, GPRS_LOGIN.length() + 1);
    char PIN[5];
    GPRS_PIN.toCharArray(PIN, GPRS_PIN.length() + 1);

    Serial.println("GPRS Connection APN:" + (String)APN + " APN:" + (String)LOGIN + " GPRS_PASSWORD:" + (String)PASSWORD + " GPRS_PIN:" + (String)PIN);
    if (!GPRSUsable) {

      if (!ModemUsable) {
        Serial.println("serial begin");
        Serial.println("wait 10sec hope modem started...");
        TinyGsmAutoBaud(SerialGSM, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);

        Serial.println("modem init");
        if (!modem.init()) {
          //Serial.println("modem restart");
          //if (!modem.restart()) {
          Serial.println("Failed to restart modem, delaying 10s and retrying");
          //}
        }
        else{
            
          String ret;
          do {
            ret = modem.setNetworkMode(54);
            delay(500);
          } while (!ret);

          String name = modem.getModemName();
          Serial.println("modem Name:" + name);
          String modemInfo = modem.getModemInfo();
          Serial.println("modem Info:" + modemInfo);

          //GESTIONE PIN
          if (modem.getSimStatus() != 3) {
            Serial.println("modem PIN Unlock");
            modem.simUnlock(PIN);
          }

          ModemUsable = true;
        }
      }

      if (ModemUsable) {
        Serial.println("GPRS Network");
        unsigned long timeout = millis();
        while (modem.waitForNetwork() && (millis() - timeout < 10)) {
          Serial.print(".");
          timeout = millis();
        }
        if (modem.isNetworkConnected()) {
          Serial.println("modem Network connected");
        } else {
          Serial.println("modem Network NOT connected");
        }

        Serial.println("GPRS Connection APN");
        if (!modem.gprsConnect(APN, LOGIN, PASSWORD)) {
          Serial.println("gprs fail");
          GPRSUsable = false;
        } else {
          Serial.println("gprs OK");
          if (modem.isGprsConnected()) {
            Serial.println("gprs INET OK");
            GPRSUsable = true;
            /*
            modem.sendAT(GF("+CNSMOD?"));
            if (modem.waitResponse(GF(GSM_NL "+CNSMOD:")) != 1) { }
            int nmodec = modem.stream.readStringUntil(',').toInt() != 0;
            int nmode = modem.stream.readStringUntil('\n').toInt();
            modem.waitResponse();
            Serial.println("Network Mode:" + nmode);

            IPAddress local = modem.localIP();
            Serial.println("Local IP:" + local);

            int csq = modem.getSignalQuality();
            Serial.println("Signal quality:" + csq);
            */
          } else {
            Serial.println("gprs INET KO");
            GPRSUsable = false;
          }
        }
      }
    }
  }


}

String callURL(const String &Server = "api.geqo.it", const String &url = "/index.php?api=mytesttx", const String &body = "") {
  String response;
  bool NotTransmited = true;
  const char *host = Server.c_str(); // "iot.integra-fragrances.com" "api.geqo.it";
  int port = 80;               // The server port
  String encodedString = base64::encode(body);

  if (EthUsable && NotTransmited) {
    Serial.println("callURL ETH ");
    String urlComplete = url + "&chn=ETH" + "&sensors=" + encodedString;
    Serial.println("callURL:");
    Serial.println((String) "GET " + urlComplete + " HTTP/1.1");
    Serial.println((String) "Host: " + host);
    if (e_client.connect(host, port)) {
      e_client.println((String) "GET " + urlComplete + " HTTP/1.1");
      e_client.println((String) "Host: " + host);
      e_client.println((String) "Connection: close");
      e_client.println();
      delay(1000);
      Serial.println("callURL ETH read respond:");
      unsigned long timeout = millis();
      while (e_client.connected() && (millis() - timeout < 10000L)) {
        while (e_client.available()) {
          char c = e_client.read();
          Serial.print(c);
          response += c;
          timeout = millis();
        }
        NotTransmited = false;
      }
      if (e_client.connected()) {
        Serial.println("callURL ETH disconnecting from server.");
        e_client.stop();
      }
    } else {
      Serial.println("callURL ETH connection failed");
    }
  } 
  if (WifiUsable && NotTransmited) {
    Serial.println("callURL WIFI ");
    String urlComplete = url + "&chn=WIFI" + "&rssi=" +  WiFi.RSSI() + "&sensors=" + encodedString;
    Serial.println("callURL:");
    Serial.println((String) "GET " + urlComplete + " HTTP/1.1");
    Serial.println((String) "Host: " + host);
    if (w_client.connect(host, port)) {
      w_client.println((String) "GET " + urlComplete + " HTTP/1.1");
      w_client.println((String) "Host: " + host);
      w_client.println((String) "Connection: close");
      w_client.println();
      delay(1000);
      Serial.println("callURL WIFI read respond:");
      unsigned long timeout = millis();
      while (w_client.connected() && (millis() - timeout < 10000L)) {
        while (w_client.available()) {
          char c = w_client.read();
          Serial.print(c);
          response += c;
          timeout = millis();
        }
        NotTransmited = false;
      }

      if (w_client.connected()) {
        Serial.println("callURL WIFI disconnecting from server.");
        w_client.stop();
      }
    } else {
      Serial.println("callURL WIFI connection failed");
    }
  }
  if (GPRSUsable && NotTransmited) {
    Serial.println("callURL GPRS ");
    String urlComplete = url + "&chn=GPRS" "&rssi=" +  modem.getSignalQuality() + "&sensors=" + encodedString;
    Serial.println("callURL:");
    Serial.println((String) "GET " + urlComplete + " HTTP/1.1");
    Serial.println((String) "Host: " + host);
    if (g_client.connect(host, port)) {
      g_client.println((String) "GET " + urlComplete + " HTTP/1.1");
      g_client.println((String) "Host: " + host);
      g_client.println((String) "Connection: close");
      g_client.println();
      delay(1000);
      Serial.println("callURL GPRS read respond:");
      unsigned long timeout = millis();
      while (g_client.connected() && (millis() - timeout < 10000L)) {
        Serial.print('.');
        while (g_client.available()) {
          char c = g_client.read();
          Serial.print(c);
          response += c;
          timeout = millis();
        }
        NotTransmited = false;
      }
      if (g_client.connected()) {
        Serial.println("callURL GPRS disconnecting from server.");
        g_client.stop();
      }
    } 
    else {
      Serial.println("callURL GPRS ERRORE connection failed");
      ModemUsable = false;
      GPRSUsable = false;
      //RESET MODEM
      Serial.println("callURL GPRS RESET Modem");
      connectToInet();
      Serial.println("callURL RELOAD CALL");
      callURL(Server, url, body);
    }
  }
  //Serial.print("RAW received");
  //Serial.print(response);

  //remove header
  if (!response.isEmpty()) {
    int contentBodyIndex = response.lastIndexOf('\n');
    if (contentBodyIndex > 0) {
      return (response.substring(contentBodyIndex));
    }
  } else {
    Serial.print('callURL ERROR NO RESPONSE');
    return ("");
  }
}

void SchedulerRx() {
  Serial.println("Scheduled load tasks (JSON):");
  String jsonString = callURL(geqo_url, (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId + "&" + "ver=" +FWversion  );

  if (!jsonString.isEmpty()) {
    Serial.println("[SchedulerRxHTTP]:");
    Serial.println(jsonString);

    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
      Serial.print("Scheduler Failed to parse JSON: ");
      Serial.println(error.c_str());
      return;
    }

    //timing update align
    settings.begin("geqo", false);

    int uValue = doc["U"].as<int>();
    if (uValue > 3600)  uValue = 3600;
    if (uValue < 10)  uValue = 10;

    geqo_align = String(uValue);
    Serial.println(geqo_align);
    settings.putString("geqo_align", geqo_align);

    int dValue = doc["D"].as<int>();
    geqo_zone = doc["Z"].as<String>();
    Serial.println(geqo_zone);
    settings.putString("geqo_zone", geqo_zone);
    settings.end();

    syncTimeFromUTC(geqo_zone, dValue);
    
    iotName = doc["DD"].as<String>();
    Serial.println(iotName);
    settings.putString("iotname",iotName);

    PumpDutyCicle = doc["PP"].as<int>();
    settings.putInt("PumpDutyCicle", PumpDutyCicle);

    FanDutyCicle = doc["FP"].as<int>();
    settings.putInt("FanDutyCicle", FanDutyCicle);

    AirSpeedTrig = doc["AS"];
    settings.putFloat("AirSpeedTrig", AirSpeedTrig);

    JsonArray taskArray = doc["tasks"];
    scheduler.deleteAllTasks();
    for (const auto & taskObject: taskArray) {
      byte dayOfWeek = taskObject["D"];
      String start = taskObject["S"];
      String end = taskObject["E"];
      unsigned long repeatEvery = taskObject["R"];
      unsigned long duration = taskObject["T"];
      String callbackName = taskObject["N"];

      // Create a new task
      scheduler.addTask(dayOfWeek, start, end, repeatEvery, duration, callbackName);
    }
  } 
  else {
    Serial.println("Scheduler HTTP: NO RESPONSE");
  }
}

void SchedulerTx() {
  String jsonString = scheduler.toJSON();
  if (!jsonString.isEmpty()) {
    String response = callURL(geqo_url, (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId + "&" + "ver=" +FWversion + "&" + "scheduler=" + jsonString);
    Serial.println("[SchedulerTxHTTP]:" + response);
  }
}

/* FUNCTION WEB -------------------------------------------------------------*/
void taskMon() {

  Serial.println("Task ARIA start.");
  Serial.print("Preparazione Dati");
  Serial.println(jsonString);

  Serial.println("<br><h1>Trasmissione Dati</h1><br>");
  String URLString = (String) "/index.php?" + "api=" + geqo_api + "&" + "mac=" + chipId + "&" + "ver=" +FWversion ;
  Serial.println("<pre>" + URLString + "</pre>");

  String result = callURL(geqo_url, URLString, jsonString);

  Serial.print("Ricezione Dati");
  Serial.println(result);
  Serial.println("Task ARIA executed.");
}

/* SCHEDULER FUNCTION ----------------------------------------------------*/
void WeeklySchedulerFunction(String FuncName, unsigned long duration, unsigned long repeatevery) {
  
    byte currentDayOfWeek = rtc.getDayofWeek();
    byte currentHour = rtc.getHour(true);
    byte currentMinute = rtc.getMinute();
    //byte currentSecond = rtc.getSecond();
    Serial.print('day:');
    Serial.print(currentDayOfWeek);
    Serial.print('time:');
    Serial.print(currentHour);
    Serial.print(':');
    Serial.println(currentMinute);

  if (FuncName == "EV1" && (AirSpeed >= AirSpeedTrig)) {
    Serial.println("EV1 function");
    O_Pompa = 1;
    PMP_TON = duration * 1000;
    EV1_ON = 1;
    EV1_TON = (repeatevery + 10) * 1000;
    END_MILLIS_EV1 = millis();
  } 
  else if (FuncName == "EV2" && (AirSpeed >= AirSpeedTrig)) {
    Serial.println("EV2 function");
    O_Pompa = 1;
    PMP_TON = duration * 1000;
    EV2_ON = 1;
    EV2_TON = (repeatevery + 10) * 1000;
    END_MILLIS_EV2 = millis();
  } 
  else if (FuncName == "MON") {
    Serial.println("MON function");
    taskMon();
  } 
  else {
    // Handle unrecognized function name
    Serial.println("Error: >" + FuncName + "< Unknown function");
  }
}

/* UPDATE FIRMWARE -----------------------------------------------------------------*/
bool checkupdate(const String &Server = "api.geqo.it") {
  const char *host = Server.c_str(); // "iot.integra-fragrances.com" "api.geqo.it";  
  int port = 80;
  char buff[32];
  sprintf(buff, "/fota/sem%d.bin", FWversion + 1);
  Serial.print("Firmware file ");
  Serial.println(buff);

  if (EthUsable) {
    Serial.println("Firmware ETH ");
    HttpClient http_client(e_client, host, port);
    http_client.get(buff);
    int statusCode = http_client.responseStatusCode();
    Serial.print("Firmware status code: ");
    Serial.println(statusCode);
    if (statusCode != 200) {
      http_client.stop();
      return false;
    }
        
    long length = http_client.contentLength();
    if (length == HttpClient::kNoContentLengthHeader) {
      http_client.stop();
      Serial.println("Firmware Error Content-length header");
      return false;
    }
    Serial.print("Firmware update file size ");
    Serial.println(length);

    if (!InternalStorage.open(length)) {
      http_client.stop();
      Serial.println("Firmware Error not enough space");
      return false;
    }
    byte b;
    while (length > 0) {
      if (!http_client.readBytes(&b, 1))
        break;
      InternalStorage.write(b);
      length--;
    }
    InternalStorage.close();
    http_client.stop();
    if (length > 0) {
      Serial.println("Firmware Error Timeout");
      return false;
    }
    Serial.println("Firmware UPDATED");
    Serial.flush();
    InternalStorage.apply(); 
  }
  else if (WifiUsable) {
    Serial.println("FIRMWARE WIFI ");
    HttpClient http_client(w_client, host, port);
    http_client.get(buff);
    int statusCode = http_client.responseStatusCode();
    Serial.print("Firmware status code: ");
    Serial.println(statusCode);
    if (statusCode != 200) {
      http_client.stop();
      return false;
    }
        
    long length = http_client.contentLength();
    if (length == HttpClient::kNoContentLengthHeader) {
      http_client.stop();
      Serial.println("Firmware Error Content-length header");
      return false;
    }
    Serial.print("Firmware update file size ");
    Serial.println(length);

    if (!InternalStorage.open(length)) {
      http_client.stop();
      Serial.println("Firmware Error not enough space");
      return false;
    }
    byte b;
    while (length > 0) {
      if (!http_client.readBytes(&b, 1))
        break;
      InternalStorage.write(b);
      length--;
    }
    InternalStorage.close();
    http_client.stop();
    if (length > 0) {
      Serial.println("Firmware Error Timeout");
      return false;
    }

    Serial.println("Firmware UPDATED");
    Serial.flush();
    InternalStorage.apply(); 
  } 
  else if (GPRSUsable) {
    Serial.println("FIRMWARE GPRS ");
    HttpClient http_client(g_client, host, port);
    http_client.get(buff);
    int statusCode = http_client.responseStatusCode();
    Serial.print("Firmware status code: ");
    Serial.println(statusCode);
    if (statusCode != 200) {
      http_client.stop();
      return false;
    }
        
    long length = http_client.contentLength();
    if (length == HttpClient::kNoContentLengthHeader) {
      http_client.stop();
      Serial.println("Firmware Error Content-length header");
      return false;
    }
    Serial.print("Firmware update file size ");
    Serial.println(length);

    if (!InternalStorage.open(length)) {
      http_client.stop();
      Serial.println("Firmware Error not enough space");
      return false;
    }
    byte b;
    while (length > 0) {
      if (!http_client.readBytes(&b, 1))
        break;
      InternalStorage.write(b);
      length--;
    }
    InternalStorage.close();
    http_client.stop();
    if (length > 0) {
      Serial.println("Firmware Error Timeout");
      return false;
    }
    Serial.println("Firmware UPDATED");
    Serial.flush();
    InternalStorage.apply(); 
  } 
  else{
    return false;
  }
  

  return true;
}

/* SETUP -----------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  delay(10000);
  Wire.begin();
  print_wakeup_reason();
  analogSetPinAttenuation(32, ADC_11db);

  /* INIZIALIZZAZIONE DEL WATCH-DOG-------------------------------------------*/
  esp_task_wdt_init(WDT_TIMEOUT, true);  // Abilita il riavvio su fermata prolungata
  esp_task_wdt_add(NULL);                // Thread da aggiungere alla finestra del watch-dog [NULL = loop principale]

  Serial.println("Hello!");
  uint64_t EfuseMac = ESP.getEfuseMac();
  chipId = mac2String((byte *)&EfuseMac);
  Serial.print("ESP32ChipID");
  Serial.println(chipId);
  
  //get MAC ADDRESS to Ethernet
  uint8_t macRaw[6];
  esp_read_mac(macRaw, ESP_MAC_WIFI_STA);
  for (int i = 0; i < 6; i++) {
    mac[i] = macRaw[i];
  }

  // Inizializzazione pin
  pinMode(PMPRUN, OUTPUT);
  pinMode(PMPPWM, OUTPUT);
  pinMode(FANPWM, OUTPUT);
  pinMode(EVPIN1, OUTPUT);
  pinMode(EVPIN2, OUTPUT);
  pinMode(LEVEL1PIN, OUTPUT);
  //pinMode(EXPLSB, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(BUTTON, INPUT);

  #ifdef AQBPIN
    pinMode(AQBPIN, INPUT);
  #endif
  #ifdef ANEPIN
    pinMode(ANEPIN, INPUT);
  #endif
  #ifdef LEVEL1PIN
    pinMode(LEVEL1PIN, INPUT);
  #endif
  #ifdef LEVEL2
    pinMode(LEVEL2, INPUT);
  #endif

  // Inizializzazione canali PWM
  ledcSetup(FANPWM_CH, PWM_FRQ_EV, PWM_RES);
  ledcSetup(PMPPWM_CH, PWM_FRQ_PM, PWM_RES);
  ledcSetup(EVPWM1_CH, PWM_FRQ_EV, PWM_RES);
  ledcSetup(EVPWM2_CH, PWM_FRQ_EV, PWM_RES);

  // Assegnazione PIN -> CANALE PWM
  ledcAttachPin(FANPWM, FANPWM_CH);
  ledcAttachPin(PMPPWM, PMPPWM_CH);
  ledcAttachPin(EVPIN1, EVPWM1_CH);
  ledcAttachPin(EVPIN2, EVPWM2_CH);

  // Settaggio output tutti a zero
  digitalWrite(PMPRUN, LOW);
  ledcWrite(FANPWM_CH, 0);  // Ventola di diffusione
  ledcWrite(PMPPWM_CH, 0);  // Pompa
  ledcWrite(EVPWM1_CH, 0);  // EV 1
  ledcWrite(EVPWM2_CH, 0);  // EV 2


  /* EPROM GEQO ---------------------------------------------------------------------*/
  Serial.println("EPROM");
  settings.begin("geqo", false);
  if (settings.getString("geqo_url", "") == "") {
    settings.putString("geqo_url", geqo_url);
    settings.putString("geqo_machine", geqo_machine);
    settings.putString("geqo_user", geqo_user);
    settings.putString("geqo_pwd", geqo_pwd);
    settings.putString("geqo_dbname", geqo_dbname);
    settings.putString("geqo_align", geqo_align);
    settings.putString("geqo_api", geqo_api);
    settings.putString("geqo_zone", geqo_zone);
    settings.putString("iotname", iotName);
    settings.putInt("PumpDutyCicle", PumpDutyCicle);
    settings.putInt("FanDutyCicle", FanDutyCicle);
    settings.putFloat("AirSpeedTrig", AirSpeedTrig);
  }
  geqo_url = settings.getString("geqo_url", "");
  geqo_machine = settings.getString("geqo_machine", "");
  geqo_user = settings.getString("geqo_user", "");
  geqo_pwd = settings.getString("geqo_pwd", "");
  geqo_dbname = settings.getString("geqo_dbname", "");
  geqo_align = settings.getString("geqo_align", "");
  geqo_api = settings.getString("geqo_api", "");
  geqo_zone = settings.getString("geqo_zone", "");
  iotName = settings.getString("iotname", "");
  PumpDutyCicle = settings.getInt("PumpDutyCicle", 0);
  FanDutyCicle = settings.getInt("FanDutyCicle", 100);
  AirSpeedTrig = settings.getFloat("AirSpeedTrig", 0.0);
  settings.end();

  settings.begin("gprs_config", false);
  if (settings.getString("GPRS_APN", "") == "") {
    settings.putString("GPRS_PIN", GPRS_PIN);
    settings.putString("GPRS_APN", GPRS_APN);
    settings.putString("GPRS_LOGIN", GPRS_LOGIN);
    settings.putString("GPRS_PASSWORD", GPRS_PASSWORD);
  }
  GPRS_PIN = settings.getString("GPRS_PIN", "");
  GPRS_APN = settings.getString("GPRS_APN", "");
  GPRS_LOGIN = settings.getString("GPRS_LOGIN", "");
  GPRS_PASSWORD = settings.getString("GPRS_PASSWORD", "");
  settings.end();

  /* MONITOR  ---------------------------------------------------------------------*/
  

  /* SCHEDULER LOOP ----------------------------------------------------------------*/
  Serial.println("Scheduler:");
  scheduler.eprom = true;  //SAVE in EPROM SCHEDULER
  scheduler.begin();
  //scheduler.addTask(0, 8, 0, 19, 30, 30, 5, "PMP");
  //scheduler.addTask(1, 8, 0, 18, 30, 30, 5, "PMP");
  //scheduler.addTask(2, 8, 0, 17, 30, 30, 5, "PMP");

  /* CONNECT ONLINE  ---------------------------------------------------------------*/
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("connectToInet:");
  connectToInet();


  /* TIME SYNC RTC -------------------------------------------------------------------*/
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("TIME SYNC:");
  String response = callURL(geqo_url, "/index.php?api=" + geqo_api + "&" + "mac=" + chipId + "&" + "ver=" +FWversion + "&" + "hb=1");
  if (!response.isEmpty()) {
    Serial.println("TIME HTTP:" + response);
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
      Serial.println((String) "TIME Scheduler Failed to parse JSON: " + error.c_str());
      //date time align from ext rtc
      syncTimeFromUTC(geqo_zone, 0);
    } 
    else {
      //timing update align
      Serial.println("convert json to var");
      settings.begin("geqo", false);

      int uValue = doc["U"].as<int>();
      if (uValue > 3600)  uValue = 3600;
      if (uValue < 10)  uValue = 10;

      geqo_align = String(uValue);
      Serial.println(geqo_align);
      settings.putString("geqo_align", geqo_align);

      int dValue = doc["D"].as<int>();
      geqo_zone = doc["Z"].as<String>();
      Serial.println(geqo_zone);
      settings.putString("geqo_zone", geqo_zone);

      geqo_machine = doc["DD"].as<String>();
      Serial.println(geqo_machine);
      settings.putString("geqo_machine",geqo_machine);

      PumpDutyCicle = doc["PP"].as<int>();
      settings.putInt("PumpDutyCicle", PumpDutyCicle);

      FanDutyCicle = doc["FP"].as<int>();
      settings.putInt("FanDutyCicle", FanDutyCicle);

      AirSpeedTrig = doc["AS"];
      settings.putFloat("AirSpeedTrig", AirSpeedTrig);

      settings.end();

      syncTimeFromUTC(geqo_zone, dValue);
    }
  } else {
    Serial.println("TIME HTTP: NO RESPONSE");
    syncTimeFromUTC(geqo_zone, 0);
  }


  /* REMOTE FIRMWARE UPDATE  -------------------------------------------------*/
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("FIRMWARE:");
  bool responseUpd = checkupdate(geqo_url);
 

  /* RETRIVE GEQO SCHEDULER ONLINE -------------------------------------------------*/
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("SCHEDULER:");
  SchedulerRx();

  /* HTTP  SERVER ------------------------------------------------------------------*/
  esp_task_wdt_reset();  // Reset del watch-dog
  Serial.println("WEB SERVER");
  app.use("/", &handle_root);
  app.use("/style.css", &handle_css);
  app.use("/manifest.json", &handle_manifest);


  app.use("/wificonfig", &handle_WiFiConfig);
  app.post("/wificonfigset", &handle_WiFiConfigSet);

  app.use("/ethconfig", &handle_EthConfig);
  app.post("/ethconfigset", &handle_EthConfigSet);

  app.use("/gprsconfig", &handle_GprsConfig);
  app.post("/gprsconfigset", &handle_GprsConfigSet);

  app.use("/srvconfig", &handle_SrvConfig);

  app.use("/schedulerconfig", &handle_SchedulerConfig);
  app.post("/schedulerconfigset", &handle_SchedulerConfigSet);
  app.use("/schedulerrx", &handle_SchedulerRx);

  app.use("/sensors", &handle_Sensors);
  app.use("/sensorsrx", &handle_SensorsRx);
  

  app.use("/reconnect", &handle_reconnect);
  app.use("/reset", &handle_reset);
  app.use("/upload", &handle_upload);
  app.use("/update", &handle_update);
  app.use("/chat", &handle_chat);
  app.notFound(&notFound);
  

  WifServer.begin();
  EthServer.begin();

  xTaskCreatePinnedToCore(
                          loop2,     /* Task function. */
                          "loop2",   /* name of task. */
                          10000,       /* Stack size of task */
                          NULL,        /* parameter of the task */
                          1,           /* priority of the task */
                          &Task2,      /* Task handle to keep track of created task */
                          0);          /* pin task to core 1 */

  /* END LOOP ----------------------------------------------------------------*/
  PREV_MILLIS_CONN = millis();
  Serial.println("OK ALL started");



}

/* FUNCTION -----------------------------------------------------------------*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  :
    {
      Serial.println("Wakeup caused by external signal using RTC_IO");
      delay(2);
    } break;
    case 2  :
    {
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      delay(2);
    } break;
    case 3  :
    {
      Serial.println("Wakeup caused by timer");
      delay(2);
    } break;
    case 4  :
    {
      Serial.println("Wakeup caused by touchpad");
      delay(2);
    } break;
    case 5  :
    {
      Serial.println("Wakeup caused by ULP program");
      delay(2);
    } break;
    default :
    {
      Serial.println("Wakeup was not caused by deep sleep");
      delay(2);
    } break;
  }
}

/* LOOP MAIN  ------------------------------------------------------------*/
void loop2 (void * pvParameters ) { 
  DynamicJsonDocument doc(2048);
  while(true) {
    #ifdef AQBPIN
      AirQuality(doc);
    #endif

    #ifdef ANEPIN
      Anemometer(doc);
    #endif

    #ifdef LEVEL1PIN
      LevelTank(doc);
    #endif

    jsonString = "";
    serializeJson(doc, jsonString);
    delay(1000);
  }
}

void loop() {

  /* CONNECT ONLINE SCHEDULER UPDATE -------------------------------------*/
  if (geqo_align.toInt() > 3600) {
    geqo_align ="3600";
  }
  if (geqo_align.toInt() <10) {
    geqo_align ="10";
  }
  if (geqo_align.toInt() > 0) {
    if (millis() - PREV_MILLIS_CONN > (geqo_align.toInt() * 1000)) {
      PREV_MILLIS_CONN = millis();
      SchedulerRx();
    }
  }

  /* GESTIONE WIFI CHIAMATE SITO WEB */
  if (WifiEnable){
    WiFiClient WifiClient = WifServer.available();
    if (WifiClient.connected()) {
      app.process(&WifiClient);
      WifiClient.stop();
    }
  }
  
  /* GESTIONE ETH CHIAMATE SITO WEB */
  if (EthEnable){
    if (!EthUsable) {
      if (Ethernet.linkStatus() == LinkON) {
        Serial.println("Ethernet Reconnected");
        connectToInet();
      }
    }
    else{
      if (Ethernet.linkStatus() == LinkOFF) {
        EthUsable = false;
        Serial.println("Ethernet Disconnected");
      }
      else{
        EthernetClient EthClient = EthServer.available();
        if (EthClient.connected()) {
          app.process(&EthClient);
          EthClient.stop();
        }
      }
    }
  }

  /* ETH STATUS FORSE INUTILE ?
  switch (Ethernet.maintain()) {
    case 1:
      //renewed fail
      Serial.println("ETH renewed: renewed fail");
      break;
    case 2:
      //renewed success
      Serial.println("Renewed success");
      //print your local IP address:
      Serial.print("ETH renewed IP address: ");
      Serial.println(Ethernet.localIP());
      break;
    case 3:
      //rebind fail
      Serial.println("ETH renewed Erro rebind fail");
      break;
    case 4:
      //rebind success
      Serial.println("ETH renewed Rebind success");
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;
    default:
      //nothing happened
      break;
  }
  */

  dnsServer.processNextRequest();

  //GESTIONE SCHEDULER
  scheduler.run();

  // EVENTI OGNI SECONDO
  if ((millis() - PREV_MILLIS_SECOND) > 1000) {
    PREV_MILLIS_SECOND = millis();
  }

  //Gestione del pulsante con eventi
  if ((digitalRead(BUTTON) == 0) && (buttonState == 1)) {
    PREV_MILLIS_BTN = millis();
    Serial.println("Pulsante premuto");
    //Per testare il pulsante attivo tutte le uscite per 6 secondi
    //EV1_ON = 1;
    //EV2_ON = 1;
    //O_Pompa = 1;
    buttonState = 0;
  }

  if ((digitalRead(BUTTON) == 1) && (buttonState == 0)) {
    if ((millis() - PREV_MILLIS_BTN) > 5000) {
      //Reset WIFI se premuto pulsante per più di 5 secondi, DA FARE
      /*Serial.println("Erase settings and restart ...");
      delay(1000);
      ConnectionWifi();*/
      ESP.restart();
    }
    Serial.println("Pulsante rilasciato");
    buttonState = 1;
    PREV_MILLIS_BTN = millis();
  }

  //cose da fare mentre il pulsante è premuto
  if (buttonState == 0) {
  }

  /* MONITOR */
  
  /* BATTERY */

   /* LORA */
  // settaggio degli output in base a EV1_ON, EV2_ON, PMP_ON
  if (EV1_ON && (ledcRead(EVPWM1_CH) == 0)) {
    ledcWrite(EVPWM1_CH, 255);
    PREV_MILLIS_EV1 = millis();
    END_MILLIS_EV1 = millis();
  }
  if (EV1_ON && ((millis() - PREV_MILLIS_EV1) > (PAR_EVStartTime * 100))) {
    ledcWrite(EVPWM1_CH, map(EVDutyCicle, 0, 100, 0, 255));
  }
  if (EV1_ON && ((millis() - PREV_MILLIS_EV1) > (PAR_EVRestartTime * 1000))) {
    ledcWrite(EVPWM1_CH, 255);
    PREV_MILLIS_EV1 = millis();
  }
  if (EV1_ON == 0 || (EV1_ON && (millis() - END_MILLIS_EV1 > EV1_TON))) {
    ledcWrite(EVPWM1_CH, 0);
    EV1_ON = 0;
  }

  if (EV2_ON && (ledcRead(EVPWM2_CH) == 0)) {
    ledcWrite(EVPWM2_CH, 255);
    PREV_MILLIS_EV2 = millis();
    END_MILLIS_EV2 = millis();
  }
  if (EV2_ON && ((millis() - PREV_MILLIS_EV2) > (PAR_EVStartTime * 100))) {
    ledcWrite(EVPWM2_CH, map(EVDutyCicle, 0, 100, 0, 255));
  }
  if (EV2_ON && ((millis() - PREV_MILLIS_EV2) > (PAR_EVRestartTime * 1000))) {
    ledcWrite(EVPWM2_CH, 255);
    PREV_MILLIS_EV2 = millis();
  }
  if (EV2_ON == 0 || (EV2_ON && ((millis() - END_MILLIS_EV2) > EV2_TON))) {
    ledcWrite(EVPWM2_CH, 0);
    EV2_ON = 0;
  }

  if (O_Pompa && (PumpState == 0)) {
    digitalWrite(PMPRUN, HIGH);
    ledcWrite(PMPPWM_CH, map(PumpDutyCicle, 0, 100, 0, 255));
    PumpState = 1;
    PREV_MILLIS_PMP = millis();
  }

  if (PumpState && ((millis() - PREV_MILLIS_PMP) > (PAR_PMPCurrDelay * 100))) {
    analogSetPinAttenuation(PROTEC, ADC_0db);
    PumpCurrent = analogRead(PROTEC);
    //decidere cosa fare con il valore letto
  }

  if (O_Pompa && ((millis() - PREV_MILLIS_PMP) > (PAR_FanLAG * 100))) {
    ledcWrite(FANPWM_CH, map(FanDutyCicle, 0, 100, 0, 255));
  }

  if (O_Pompa == 0 || (O_Pompa && ((millis() - PREV_MILLIS_PMP) > PMP_TON))) {
    ledcWrite(PMPPWM_CH, 0);
    digitalWrite(PMPRUN, LOW);
    O_Pompa = 0;
    PumpState = 0;
    //delay(PAR_FanLAG * 100);   //è brutto ma potrebebe andare se rimaniamo sotto al secondo con PAR_FanLAG
    ledcWrite(FANPWM_CH, 0);
  }

  esp_task_wdt_reset();  // Reset del watch-dog
}
