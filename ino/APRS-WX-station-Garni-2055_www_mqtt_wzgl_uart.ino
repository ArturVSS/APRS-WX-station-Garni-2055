///////////////////////////////////////////////////////////////////////////////////////////////////
// BresserWeatherSensorBasicAPRS.ino
// 
// Integracja: Bresser 7-in-1 + ESP8266 + BME280 + CC1101 -> APRS (TCP) + WebUI + MQTT + UART
// Biblioteka: BresserWeatherSensorReceiver v0.37.0
// Lepszego programu już nie wymyślę - SP3WRO
//
// MAPA PINÓW ESP8266 (Wemos D1 Mini):
//   CC1101 SCK    → GPIO14 (D5)
//   CC1101 MISO   → GPIO12 (D6)
//   CC1101 MOSI   → GPIO13 (D7)
//   CC1101 CS     → GPIO15 (D8)  PIN_RECEIVER_CS
//   CC1101 GDO0   → GPIO4  (D2)  PIN_RECEIVER_IRQ
//   CC1101 GDO2   → GPIO5  (D1)  PIN_RECEIVER_GPIO
//   BME280 SDA    → GPIO0  (D3)
//   BME280 SCL    → GPIO3  (RX)
//   UART WX TX    → GPIO2  (D4)  Serial1 9600 baud  co 5 sekund
//   Debug Serial  → GPIO1  (TX)  Serial  115200 baud
//
// FORMAT UART:
//   WX,HH:MM,DD-MM-YYYY,temp,hum,wdir,wspd,wgst,rain,uv,lux,rssi,bat,btemp,bhum,bpress,qnh
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <time.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LittleFS.h>

#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"

// ==========================================================================
#define wxSerial Serial1
#define UART_INTERVAL_MS 5000UL

// ==========================================================================
// --- KONFIGURACJA APRS I WIFI
String WIFI_SSID     = "Xiaomi2G";
String WIFI_PASS     = "dupablada";
String APRS_CALLSIGN = "SQ3MKP-13";
String APRS_PASSCODE = "22702";
double SITE_LAT      = 52.3659;
double SITE_LON      = 16.9489;
float  SITE_ALT      = 85.0;
int    UTC_OFFSET    = 1;       // przesunięcie względem UTC w godzinach, np. +1 = CET, +2 = CEST
// ==========================================================================

// ==========================================================================
// --- KONFIGURACJA MQTT
String   MQTT_HOST    = "broker.hivemq.com";
uint16_t MQTT_PORT    = 1883;
String   MQTT_USER    = "";
String   MQTT_PASS    = "";
String   MQTT_TOPIC   = "wx/SQ3MKP-13";
bool     MQTT_ENABLED = false;
// ==========================================================================

static const char* APRS_HOST = "rotate.aprs2.net";
static const uint16_t APRS_PORT = 14580;
unsigned long REPORT_INTERVAL_MS = 15UL * 60UL * 1000UL;

WeatherSensor ws;
Adafruit_BME280 bme;
bool bme_available = false;

ESP8266WebServer server(80);
WiFiClient       espClient;
PubSubClient     mqttClient(espClient);

float wind_speed_sum = 0.0;
int   wind_sample_count = 0;
float wind_gust_max_period = 0.0;

unsigned long last_uart_send = 0;

struct WeatherData {
  float   temp_c = NAN;
  uint8_t humidity = 0;
  float   wind_dir = NAN;
  float   wind_speed = NAN;
  float   wind_gust = NAN;
  float   rain_total_mm = NAN;
  float   uv_index = NAN;
  float   light_klx = NAN;
  int     radio_rssi = -100;
  bool    battery_ok = true;
  bool    valid_data = false;
  float   bme_temp = NAN;
  float   bme_humidity = NAN;
  float   bme_pressure = NAN;
  float   bme_qnh = NAN;
  unsigned long last_update = 0;
} current_wx;

struct RainHistory { float total_mm; bool valid; };
RainHistory rain_buffer[4];
uint8_t rain_idx = 0;
unsigned long last_report_time = 0;

// --- PRZELICZENIE QNH ----------------------------------------------------
float calc_qnh(float pressure_hpa, float temp_c, float altitude_m) {
  if (isnan(pressure_hpa) || isnan(temp_c) || altitude_m <= 0.0) return pressure_hpa;
  float t_kelvin = temp_c + 273.15;
  return pressure_hpa * pow(
    1.0 - (0.0065 * altitude_m) / (t_kelvin + 0.0065 * altitude_m), -5.257);
}

// --- CZAS LOKALNY z uwzględnieniem UTC_OFFSET ----------------------------
// Zwraca time_t przesunięty o UTC_OFFSET godzin
time_t local_time() {
  return time(nullptr) + (time_t)(UTC_OFFSET * 3600);
}

String get_time_hhmm() {
  time_t t = local_time();
  if (t < 100000) return "NTP";
  struct tm* tm_info = gmtime(&t);   // gmtime bo już ręcznie dodaliśmy offset
  char buf[10];
  snprintf(buf, sizeof(buf), "%02d:%02d", tm_info->tm_hour, tm_info->tm_min);
  return String(buf);
}

String get_date_ddmmyyyy() {
  time_t t = local_time();
  if (t < 100000) return "NTP";
  struct tm* tm_info = gmtime(&t);
  char buf[14];
  snprintf(buf, sizeof(buf), "%02d-%02d-%04d",
           tm_info->tm_mday, tm_info->tm_mon + 1, tm_info->tm_year + 1900);
  return String(buf);
}

// Czytelny label strefy czasowej, np. "UTC+1" / "UTC-5" / "UTC"
String get_tz_label() {
  if (UTC_OFFSET == 0) return "UTC";
  if (UTC_OFFSET > 0)  return "UTC+" + String(UTC_OFFSET);
  return "UTC" + String(UTC_OFFSET);   // String(-5) = "-5"
}

// --- UART WX -------------------------------------------------------------
// Format: WX,HH:MM,DD-MM-YYYY,temp,hum,wdir,wspd,wgst,rain,uv,lux,rssi,bat,btemp,bhum,bpress,qnh
String buildUartString() {
  String s = "WX,";
  s += get_time_hhmm();                                                                 s += ",";
  s += get_date_ddmmyyyy();                                                             s += ",";
  s += !isnan(current_wx.temp_c)        ? String(current_wx.temp_c, 1)        : "NAN"; s += ",";
  s += current_wx.humidity > 0          ? String(current_wx.humidity)          : "NAN"; s += ",";
  s += !isnan(current_wx.wind_dir)      ? String(current_wx.wind_dir, 1)       : "NAN"; s += ",";
  s += !isnan(current_wx.wind_speed)    ? String(current_wx.wind_speed, 1)     : "NAN"; s += ",";
  s += !isnan(current_wx.wind_gust)     ? String(current_wx.wind_gust, 1)      : "NAN"; s += ",";
  s += !isnan(current_wx.rain_total_mm) ? String(current_wx.rain_total_mm, 1)  : "NAN"; s += ",";
  s += !isnan(current_wx.uv_index)      ? String(current_wx.uv_index, 1)       : "NAN"; s += ",";
  s += !isnan(current_wx.light_klx)     ? String(current_wx.light_klx, 1)      : "NAN"; s += ",";
  s += String(current_wx.radio_rssi);                                                    s += ",";
  s += current_wx.battery_ok            ? "1"                                   : "0";  s += ",";
  s += !isnan(current_wx.bme_temp)      ? String(current_wx.bme_temp, 1)       : "NAN"; s += ",";
  s += !isnan(current_wx.bme_humidity)  ? String(current_wx.bme_humidity, 1)   : "NAN"; s += ",";
  s += !isnan(current_wx.bme_pressure)  ? String(current_wx.bme_pressure, 1)   : "NAN"; s += ",";
  s += !isnan(current_wx.bme_qnh)       ? String(current_wx.bme_qnh, 1)        : "NAN";
  return s;
}

void sendUartWX() {
  wxSerial.println(buildUartString());
}

// --- KONWERSJE -----------------------------------------------------------
int c_to_f(float c) { return (int)lround(c * 1.8 + 32); }
int ms_to_mph(float ms) { return (int)lround(ms * 2.23694); }
int mm_to_hin(float mm) { return (int)lround(mm * 3.93701); }

String format_lat(double lat) {
  char b[20]; char h = (lat >= 0) ? 'N' : 'S'; lat = fabs(lat);
  int d = (int)lat; double m = (lat - d) * 60.0;
  snprintf(b, sizeof(b), "%02d%05.2f%c", d, m, h);
  return String(b);
}

String format_lon(double lon) {
  char b[20]; char h = (lon >= 0) ? 'E' : 'W'; lon = fabs(lon);
  int d = (int)lon; double m = (lon - d) * 60.0;
  snprintf(b, sizeof(b), "%03d%05.2f%c", d, m, h);
  return String(b);
}

String p3(int v) {
  char b[5]; snprintf(b, sizeof(b), "%03d", (v<0?0:(v>999?999:v))); return String(b);
}

// APRS timestamp zawsze UTC (bez offsetu!)
String get_timestamp_utc() {
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char buff[10];
  snprintf(buff, sizeof(buff), "%02d%02d%02dz", t->tm_mday, t->tm_hour, t->tm_min);
  return String(buff);
}

// --- ZAPIS/ODCZYT KONFIGURACJI -------------------------------------------
void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (f) {
    f.println(WIFI_SSID);
    f.println(WIFI_PASS);
    f.println(APRS_CALLSIGN);
    f.println(APRS_PASSCODE);
    f.println(SITE_LAT, 6);
    f.println(SITE_LON, 6);
    f.println(REPORT_INTERVAL_MS / 60000);
    f.println(SITE_ALT, 1);
    f.println(MQTT_HOST);
    f.println(MQTT_PORT);
    f.println(MQTT_USER);
    f.println(MQTT_PASS);
    f.println(MQTT_TOPIC);
    f.println(MQTT_ENABLED ? "1" : "0");
    f.println(UTC_OFFSET);           // <-- nowe pole
    f.close();
    Serial.println(F("[CFG] Zapisano"));
  } else {
    Serial.println(F("[CFG] Blad zapisu!"));
  }
}

void loadConfig() {
  if (LittleFS.exists("/config.txt")) {
    File f = LittleFS.open("/config.txt", "r");
    if (f) {
      WIFI_SSID     = f.readStringUntil('\n'); WIFI_SSID.trim();
      WIFI_PASS     = f.readStringUntil('\n'); WIFI_PASS.trim();
      APRS_CALLSIGN = f.readStringUntil('\n'); APRS_CALLSIGN.trim();
      APRS_PASSCODE = f.readStringUntil('\n'); APRS_PASSCODE.trim();
      SITE_LAT      = f.readStringUntil('\n').toDouble();
      SITE_LON      = f.readStringUntil('\n').toDouble();
      REPORT_INTERVAL_MS = f.readStringUntil('\n').toInt() * 60000UL;
      SITE_ALT      = f.readStringUntil('\n').toFloat();
      MQTT_HOST     = f.readStringUntil('\n'); MQTT_HOST.trim();
      MQTT_PORT     = f.readStringUntil('\n').toInt();
      MQTT_USER     = f.readStringUntil('\n'); MQTT_USER.trim();
      MQTT_PASS     = f.readStringUntil('\n'); MQTT_PASS.trim();
      MQTT_TOPIC    = f.readStringUntil('\n'); MQTT_TOPIC.trim();
      String en     = f.readStringUntil('\n'); en.trim();
      MQTT_ENABLED  = (en == "1");
      String ofs    = f.readStringUntil('\n'); ofs.trim();
      if (ofs.length() > 0) UTC_OFFSET = ofs.toInt();
      f.close();
      Serial.println(F("[CFG] Wczytano"));
    }
  } else {
    MQTT_TOPIC = "wx/" + APRS_CALLSIGN;
  }
}

// --- MQTT ----------------------------------------------------------------
void mqttReconnect() {
  if (!MQTT_ENABLED) return;
  if (!WiFi.isConnected()) return;
  if (mqttClient.connected()) return;
  Serial.print(F("[MQTT] Laczenie... "));
  String clientId = "WX-" + APRS_CALLSIGN + "-" + String(ESP.getChipId(), HEX);
  bool ok = MQTT_USER.length() > 0
    ? mqttClient.connect(clientId.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str())
    : mqttClient.connect(clientId.c_str());
  Serial.println(ok ? F("OK") : F("FAIL"));
  if (!ok) { Serial.print(F("rc=")); Serial.println(mqttClient.state()); }
}

void mqttPublishWX() {
  if (!MQTT_ENABLED) return;
  if (!mqttClient.connected()) mqttReconnect();
  if (!mqttClient.connected()) return;

  String p = "{";
  p += "\"call\":\""  + APRS_CALLSIGN + "\",";
  p += "\"time\":\""  + get_time_hhmm() + "\",";
  p += "\"date\":\""  + get_date_ddmmyyyy() + "\",";
  p += "\"tz\":\""    + get_tz_label() + "\",";
  p += "\"lat\":"     + String(SITE_LAT, 6) + ",";
  p += "\"lon\":"     + String(SITE_LON, 6) + ",";
  p += "\"alt\":"     + String(SITE_ALT, 1) + ",";
  p += "\"temp\":"    + (isnan(current_wx.temp_c)        ? String("null") : String(current_wx.temp_c, 1))        + ",";
  p += "\"hum\":"     + (current_wx.humidity > 0         ? String(current_wx.humidity) : String("null"))         + ",";
  p += "\"wdir\":"    + (isnan(current_wx.wind_dir)      ? String("null") : String(current_wx.wind_dir, 1))      + ",";
  p += "\"wspd\":"    + (isnan(current_wx.wind_speed)    ? String("null") : String(current_wx.wind_speed, 1))    + ",";
  p += "\"wgst\":"    + (isnan(current_wx.wind_gust)     ? String("null") : String(current_wx.wind_gust, 1))     + ",";
  p += "\"rain\":"    + (isnan(current_wx.rain_total_mm) ? String("null") : String(current_wx.rain_total_mm, 1)) + ",";
  p += "\"uv\":"      + (isnan(current_wx.uv_index)      ? String("null") : String(current_wx.uv_index, 1))      + ",";
  p += "\"light\":"   + (isnan(current_wx.light_klx)     ? String("null") : String(current_wx.light_klx, 1))     + ",";
  p += "\"rssi\":"    + String(current_wx.radio_rssi) + ",";
  p += "\"bat\":"     + String(current_wx.battery_ok ? "true" : "false") + ",";
  p += "\"btemp\":"   + (isnan(current_wx.bme_temp)      ? String("null") : String(current_wx.bme_temp, 1))      + ",";
  p += "\"bhum\":"    + (isnan(current_wx.bme_humidity)  ? String("null") : String(current_wx.bme_humidity, 1))  + ",";
  p += "\"bpress\":"  + (isnan(current_wx.bme_pressure)  ? String("null") : String(current_wx.bme_pressure, 1))  + ",";
  p += "\"qnh\":"     + (isnan(current_wx.bme_qnh)       ? String("null") : String(current_wx.bme_qnh, 1))       + ",";
  p += "\"unix\":"    + String(time(nullptr)) + ",";
  p += "\"uptime\":"  + String(millis() / 1000);
  p += "}";

  bool ok = mqttClient.publish(MQTT_TOPIC.c_str(), p.c_str(), true);
  Serial.print(F("[MQTT] ")); Serial.print(MQTT_TOPIC);
  Serial.println(ok ? F(" -> OK") : F(" -> FAIL"));
}

// --- APRS ----------------------------------------------------------------
void send_aprs() {
  Serial.println(F("\n[APRS] Wysylanie..."));
  float avg_w = (wind_sample_count > 0) ? (wind_speed_sum / wind_sample_count) : 0.0;

  int r1h = 0;
  if (!isnan(current_wx.rain_total_mm)) rain_buffer[rain_idx] = {current_wx.rain_total_mm, true};
  uint8_t oid = (rain_idx + 1) % 4;
  if (rain_buffer[rain_idx].valid && rain_buffer[oid].valid) {
    float d = rain_buffer[rain_idx].total_mm - rain_buffer[oid].total_mm;
    r1h = mm_to_hin(d < 0 ? 0 : d);
  }
  rain_idx = (rain_idx + 1) % 4;

  int baro = 0;
  if (bme_available) {
    float p = bme.readPressure();
    if (!isnan(p) && p > 80000.0) {
      current_wx.bme_pressure = p / 100.0;
      baro = (int)(p / 10.0);
    }
  }

  int lum_wm2 = 0;
  if (!isnan(current_wx.light_klx)) lum_wm2 = (int)(current_wx.light_klx * 7.9);

  // APRS zawsze UTC!
  String body = "@" + get_timestamp_utc() + format_lat(SITE_LAT) + "/" + format_lon(SITE_LON) + "_";
  int wd = isnan(current_wx.wind_dir) ? 0 : (int)current_wx.wind_dir;
  body += p3(wd) + "/" + p3(ms_to_mph(avg_w));
  body += "g" + p3(ms_to_mph(wind_gust_max_period));
  body += "t" + p3(c_to_f(current_wx.temp_c));
  if (r1h > 0) body += "r" + p3(r1h);
  if (current_wx.humidity > 0) {
    char hb[5]; snprintf(hb, sizeof(hb), "h%02d", (int)current_wx.humidity); body += hb;
  }
  if (baro > 0) {
    char bb[10]; snprintf(bb, sizeof(bb), "b%05d", baro); body += bb;
  }
  if (lum_wm2 > 0) {
    lum_wm2 = min(lum_wm2, 999);
    char lb[6]; snprintf(lb, sizeof(lb), "L%03d", lum_wm2); body += lb;
  }

  String comment = " GarniWX";
  comment += " Sig:" + String(current_wx.radio_rssi) + "dBm";
  if (!isnan(current_wx.uv_index)) comment += " UV:" + String(current_wx.uv_index, 1);
  comment += current_wx.battery_ok ? " Bat:OK" : " Bat:LOW";

  String packet = APRS_CALLSIGN + ">APRS,TCPIP*:" + body + comment;

  WiFiClient cl;
  if (cl.connect(APRS_HOST, APRS_PORT)) {
    cl.printf("user %s pass %s vers RodosBME 1.9\n",
              APRS_CALLSIGN.c_str(), APRS_PASSCODE.c_str());
    delay(200);
    cl.println(packet);
    delay(500);
    cl.stop();
    Serial.println("-> " + packet);
    wind_speed_sum = 0; wind_sample_count = 0; wind_gust_max_period = 0;
  } else {
    Serial.println(F("[APRS] Blad polaczenia!"));
  }
}

// --- WWW -----------------------------------------------------------------
void handleRoot() {
  String html = F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Stacja Pogodowa APRS</title>"
    "<style>"
    "body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#000;color:#fff}"
    ".container{display:flex;gap:20px;max-width:1400px;margin:0 auto;flex-wrap:wrap}"
    ".column{flex:1;min-width:300px;background:#1a1a1a;padding:20px;border-radius:8px;"
    "box-shadow:0 2px 8px rgba(255,255,255,0.1);border:1px solid #333}"
    "h1{color:#00aaff;margin-top:0;font-size:24px;text-shadow:0 0 10px rgba(0,170,255,0.5)}"
    "h2{color:#00aaff;border-bottom:2px solid #00aaff;padding-bottom:8px;font-size:18px}"
    ".data-row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #333}"
    ".label{font-weight:bold;color:#aaa}"
    ".value{color:#fff}"
    ".value-highlight{color:#00ffcc;font-weight:bold;text-shadow:0 0 6px rgba(0,255,200,0.5)}"
    ".status-ok{color:#0f0;font-weight:bold;text-shadow:0 0 5px rgba(0,255,0,0.5)}"
    ".status-warn{color:#ff0;font-weight:bold;text-shadow:0 0 5px rgba(255,255,0,0.5)}"
    "input,select{width:100%;padding:8px;margin:5px 0 15px 0;border:1px solid #444;"
    "border-radius:4px;box-sizing:border-box;background:#222;color:#fff}"
    "label{font-weight:bold;color:#aaa;display:block}"
    "button{background:#00aaff;color:#000;padding:12px 24px;border:none;border-radius:4px;"
    "cursor:pointer;font-size:16px;width:100%;font-weight:bold}"
    "button:hover{background:#0088cc;box-shadow:0 0 15px rgba(0,170,255,0.6)}"
    ".info{background:#2a2a2a;padding:10px;border-radius:4px;margin:10px 0;"
    "font-size:14px;border:1px solid #444}"
    "a{color:#00aaff;text-decoration:none;font-weight:bold}"
    "a:hover{color:#0088cc;text-decoration:underline}"
    ".aprs-link{background:#2a2a2a;padding:12px;border-radius:4px;margin:0 0 15px 0;"
    "text-align:center;border:1px solid #00aaff}"
    ".section-title{margin-top:20px}"
    ".uart-box{background:#111;border:1px solid #00aaff;border-radius:4px;padding:10px;"
    "font-family:monospace;font-size:12px;color:#00ffcc;margin-top:10px;word-break:break-all}"
    ".pin-table{width:100%;border-collapse:collapse;font-size:13px;margin-top:8px}"
    ".pin-table td{padding:4px 8px;border:1px solid #333}"
    ".pin-table tr:nth-child(even){background:#222}"
    ".pin-ok{color:#0f0}"
    ".tz-badge{display:inline-block;background:#003355;border:1px solid #00aaff;"
    "border-radius:4px;padding:2px 8px;font-size:13px;color:#00aaff;margin-left:8px}"
    "</style>"
    "<script>"
    "setInterval(function(){fetch('/data').then(r=>r.json()).then(d=>{"
    "document.getElementById('temp').innerText=d.temp;"
    "document.getElementById('hum').innerText=d.hum;"
    "document.getElementById('wdir').innerText=d.wdir;"
    "document.getElementById('wspd').innerText=d.wspd;"
    "document.getElementById('wgst').innerText=d.wgst;"
    "document.getElementById('rain').innerText=d.rain;"
    "document.getElementById('uv').innerText=d.uv;"
    "document.getElementById('lux').innerText=d.lux;"
    "document.getElementById('rssi').innerText=d.rssi;"
    "document.getElementById('bat').innerText=d.bat;"
    "document.getElementById('bat').className=d.bat=='OK'?'value status-ok':'value status-warn';"
    "document.getElementById('btemp').innerText=d.btemp;"
    "document.getElementById('bhum').innerText=d.bhum;"
    "document.getElementById('bpress').innerText=d.bpress;"
    "document.getElementById('bqnh').innerText=d.bqnh;"
    "document.getElementById('wxtime').innerText=d.wxtime;"
    "document.getElementById('wxdate').innerText=d.wxdate;"
    "document.getElementById('wxtzbadge').innerText=d.tz;"
    "document.getElementById('upd').innerText=d.upd;"
    "document.getElementById('uart_preview').innerText=d.uart;"
    "});},2000);"
    "</script>"
    "</head><body>"
    "<h1>🌤️ Stacja Pogodowa APRS - ");
  html += APRS_CALLSIGN;
  html += F("</h1><div class='container'><div class='column'>");

  html += F("<div class='aprs-link'>📍 <a href='https://aprs.fi/");
  html += APRS_CALLSIGN;
  html += F("' target='_blank'>Zobacz stację na APRS.fi</a></div>");

  // Czas lokalny z badgem strefy
  html += F("<div class='data-row'><span class='label'>⏰ Czas lokalny:</span>"
            "<span class='value-highlight' id='wxtime'>");
  html += get_time_hhmm();
  html += F("</span><span class='tz-badge' id='wxtzbadge'>");
  html += get_tz_label();
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>📅 Data:</span>"
            "<span class='value' id='wxdate'>");
  html += get_date_ddmmyyyy();
  html += F("</span></div>");

  html += F("<h2>📡 Dane z Radia (Bresser 7-in-1)</h2>");

  html += F("<div class='data-row'><span class='label'>Temperatura:</span>"
            "<span class='value' id='temp'>");
  html += !isnan(current_wx.temp_c) ? String(current_wx.temp_c, 1) + " °C" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Wilgotność:</span>"
            "<span class='value' id='hum'>");
  html += current_wx.humidity > 0 ? String(current_wx.humidity) + " %" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Kierunek wiatru:</span>"
            "<span class='value' id='wdir'>");
  html += !isnan(current_wx.wind_dir) ? String((int)current_wx.wind_dir) + " °" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Prędkość wiatru:</span>"
            "<span class='value' id='wspd'>");
  html += !isnan(current_wx.wind_speed) ? String(current_wx.wind_speed, 1) + " m/s" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Porywy wiatru:</span>"
            "<span class='value' id='wgst'>");
  html += !isnan(current_wx.wind_gust) ? String(current_wx.wind_gust, 1) + " m/s" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Opady (suma):</span>"
            "<span class='value' id='rain'>");
  html += !isnan(current_wx.rain_total_mm) ? String(current_wx.rain_total_mm, 1) + " mm" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Indeks UV:</span>"
            "<span class='value' id='uv'>");
  html += !isnan(current_wx.uv_index) ? String(current_wx.uv_index, 1) : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Światło:</span>"
            "<span class='value' id='lux'>");
  html += !isnan(current_wx.light_klx) ? String(current_wx.light_klx, 1) + " klx" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Siła sygnału CC1101:</span>"
            "<span class='value' id='rssi'>");
  html += String(current_wx.radio_rssi) + " dBm";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Bateria stacji:</span>"
            "<span class='value' id='bat' class='");
  html += current_wx.battery_ok ? F("status-ok'>OK") : F("status-warn'>LOW");
  html += F("</span></div>");

  html += F("<h2 class='section-title'>🌡️ Dane z BME280 (GPIO0/GPIO3)</h2>");

  html += F("<div class='data-row'><span class='label'>Temperatura:</span>"
            "<span class='value' id='btemp'>");
  html += !isnan(current_wx.bme_temp) ? String(current_wx.bme_temp, 1) + " °C" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Wilgotność:</span>"
            "<span class='value' id='bhum'>");
  html += !isnan(current_wx.bme_humidity) ? String(current_wx.bme_humidity, 1) + " %" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Ciśnienie absolutne:</span>"
            "<span class='value' id='bpress'>");
  html += !isnan(current_wx.bme_pressure) ? String(current_wx.bme_pressure, 1) + " hPa" : "---";
  html += F("</span></div>");

  html += F("<div class='data-row'><span class='label'>Ciśnienie QNH (MSL):</span>"
            "<span class='value-highlight' id='bqnh'>");
  html += !isnan(current_wx.bme_qnh) ? String(current_wx.bme_qnh, 1) + " hPa" : "---";
  html += F("</span></div>");

  html += F("<div class='info'>Ostatnia aktualizacja: <span id='upd'>");
  html += current_wx.last_update > 0
    ? String((millis() - current_wx.last_update) / 1000) + " s temu" : "brak danych";
  html += F("</span></div>");

  html += F("<h2 class='section-title'>🔌 UART TX (GPIO2/D4, 9600, co 5s)</h2>"
            "<div class='uart-box' id='uart_preview'>oczekiwanie na dane...</div>"
            "</div>");

  // ---- PRAWA KOLUMNA ----
  html += F("<div class='column'><h2>⚙️ Konfiguracja Stacji</h2>"
    "<form action='/save' method='POST'>"
    "<label>Nazwa WiFi (SSID):</label><input name='ssid' value='");
  html += WIFI_SSID;
  html += F("'><label>Hasło WiFi:</label><input name='pass' type='password' value='");
  html += WIFI_PASS;
  html += F("'><label>Znak wywoławczy APRS:</label><input name='call' value='");
  html += APRS_CALLSIGN;
  html += F("'><label>Passcode APRS:</label><input name='passcode' value='");
  html += APRS_PASSCODE;
  html += F("'><label>Szerokość geograficzna:</label>"
    "<input name='lat' type='number' step='0.0001' value='");
  html += String(SITE_LAT, 6);
  html += F("'><label>Długość geograficzna:</label>"
    "<input name='lon' type='number' step='0.0001' value='");
  html += String(SITE_LON, 6);
  html += F("'><label>Wysokość stacji n.p.m. [m]:</label>"
    "<input name='alt' type='number' step='0.1' value='");
  html += String(SITE_ALT, 1);
  html += F("'><label>Interwał raportów APRS (min):</label>"
    "<input name='interval' type='number' min='1' value='");
  html += String(REPORT_INTERVAL_MS / 60000);

  // ---- STREFA CZASOWA ----
  html += F("'><h2 class='section-title'>🕐 Strefa czasowa (UART)</h2>"
    "<label>Przesunięcie względem UTC (godziny, -12 do +14):</label>"
    "<select name='utc_offset'>");
  for (int i = -12; i <= 14; i++) {
    html += "<option value='" + String(i) + "'";
    if (i == UTC_OFFSET) html += " selected";
    if (i == 0)          html += ">UTC";
    else if (i > 0)      html += ">UTC+" + String(i);
    else                 html += ">UTC" + String(i);
    html += "</option>";
  }
  html += F("</select>"
    "<div class='info' style='margin-top:-10px;margin-bottom:10px;font-size:13px'>"
    "⚠️ Czas APRS zawsze wysyłany jako UTC (standard APRS).<br>"
    "Strefa czasowa dotyczy tylko ramek UART i wyświetlania na stronie.</div>");

  // ---- MQTT ----
  html += F("<h2 class='section-title'>📡 MQTT 3.1.1</h2>"
    "<label>MQTT włączony:</label><select name='mqtt_en'>");
  html += MQTT_ENABLED
    ? F("<option value='1' selected>Tak</option><option value='0'>Nie</option>")
    : F("<option value='1'>Tak</option><option value='0' selected>Nie</option>");
  html += F("</select>"
    "<label>MQTT Host:</label><input name='mqtt_host' value='");
  html += MQTT_HOST;
  html += F("'><label>MQTT Port:</label>"
    "<input name='mqtt_port' type='number' min='1' max='65535' value='");
  html += String(MQTT_PORT);
  html += F("'><label>MQTT User:</label><input name='mqtt_user' value='");
  html += MQTT_USER;
  html += F("'><label>MQTT Hasło:</label>"
    "<input name='mqtt_pass' type='password' value='");
  html += MQTT_PASS;
  html += F("'><label>MQTT Topic (JSON):</label><input name='mqtt_topic' value='");
  html += MQTT_TOPIC;
  html += F("'><button type='submit'>💾 Zapisz Ustawienia</button></form>");

  html += F("<div class='info' style='margin-top:15px'>"
    "IP: "); html += WiFi.localIP().toString();
  html += F("<br>APRS: "); html += APRS_HOST;
  html += F("<br>MQTT: "); html += MQTT_ENABLED ? "WLACZONY" : "WYLACZONY";
  html += F("<br>Strefa: "); html += get_tz_label();
  html += F("<br>Alt: "); html += String(SITE_ALT, 0); html += F(" m n.p.m.");
  html += F("</div>");

  html += F("<div class='info'><b>📌 Mapa pinów:</b>"
    "<table class='pin-table'>"
    "<tr><td>CC1101 SCK</td> <td class='pin-ok'>GPIO14 D5</td></tr>"
    "<tr><td>CC1101 MISO</td><td class='pin-ok'>GPIO12 D6</td></tr>"
    "<tr><td>CC1101 MOSI</td><td class='pin-ok'>GPIO13 D7</td></tr>"
    "<tr><td>CC1101 CS</td>  <td class='pin-ok'>GPIO15 D8</td></tr>"
    "<tr><td>CC1101 GDO0</td><td class='pin-ok'>GPIO4  D2 ← IRQ</td></tr>"
    "<tr><td>CC1101 GDO2</td><td class='pin-ok'>GPIO5  D1</td></tr>"
    "<tr><td>BME280 SDA</td> <td class='pin-ok'>GPIO0  D3</td></tr>"
    "<tr><td>BME280 SCL</td> <td class='pin-ok'>GPIO3  RX</td></tr>"
    "<tr><td>UART WX TX</td> <td class='pin-ok'>GPIO2  D4  9600</td></tr>"
    "<tr><td>Debug TX</td>   <td class='pin-ok'>GPIO1  TX  115200</td></tr>"
    "</table></div>"
    "</div></div></body></html>");

  server.send(200, "text/html", html);
}

void handleData() {
  String uart_str = buildUartString();
  String json = "{";
  json += "\"wxtime\":\""  + get_time_hhmm() + "\",";
  json += "\"wxdate\":\""  + get_date_ddmmyyyy() + "\",";
  json += "\"tz\":\""      + get_tz_label() + "\",";
  json += "\"temp\":\""   + (!isnan(current_wx.temp_c)       ? String(current_wx.temp_c, 1) + " °C"        : "---") + "\",";
  json += "\"hum\":\""    + (current_wx.humidity > 0          ? String(current_wx.humidity) + " %"          : "---") + "\",";
  json += "\"wdir\":\""   + (!isnan(current_wx.wind_dir)      ? String((int)current_wx.wind_dir) + " °"     : "---") + "\",";
  json += "\"wspd\":\""   + (!isnan(current_wx.wind_speed)    ? String(current_wx.wind_speed, 1) + " m/s"   : "---") + "\",";
  json += "\"wgst\":\""   + (!isnan(current_wx.wind_gust)     ? String(current_wx.wind_gust, 1) + " m/s"    : "---") + "\",";
  json += "\"rain\":\""   + (!isnan(current_wx.rain_total_mm) ? String(current_wx.rain_total_mm, 1) + " mm" : "---") + "\",";
  json += "\"uv\":\""     + (!isnan(current_wx.uv_index)      ? String(current_wx.uv_index, 1)               : "---") + "\",";
  json += "\"lux\":\""    + (!isnan(current_wx.light_klx)     ? String(current_wx.light_klx, 1) + " klx"    : "---") + "\",";
  json += "\"rssi\":\""   + String(current_wx.radio_rssi) + " dBm\",";
  json += "\"bat\":\""    + String(current_wx.battery_ok ? "OK" : "LOW") + "\",";
  json += "\"btemp\":\""  + (!isnan(current_wx.bme_temp)      ? String(current_wx.bme_temp, 1) + " °C"      : "---") + "\",";
  json += "\"bhum\":\""   + (!isnan(current_wx.bme_humidity)  ? String(current_wx.bme_humidity, 1) + " %"   : "---") + "\",";
  json += "\"bpress\":\"" + (!isnan(current_wx.bme_pressure)  ? String(current_wx.bme_pressure, 1) + " hPa" : "---") + "\",";
  json += "\"bqnh\":\""   + (!isnan(current_wx.bme_qnh)       ? String(current_wx.bme_qnh, 1) + " hPa"      : "---") + "\",";
  json += "\"upd\":\""    + (current_wx.last_update > 0
    ? String((millis() - current_wx.last_update) / 1000) + " s temu" : "brak") + "\",";
  json += "\"uart\":\"" + uart_str + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSave() {
  if (server.hasArg("ssid"))       WIFI_SSID     = server.arg("ssid");
  if (server.hasArg("pass"))       WIFI_PASS     = server.arg("pass");
  if (server.hasArg("call"))       APRS_CALLSIGN = server.arg("call");
  if (server.hasArg("passcode"))   APRS_PASSCODE = server.arg("passcode");
  if (server.hasArg("lat"))        SITE_LAT      = server.arg("lat").toDouble();
  if (server.hasArg("lon"))        SITE_LON      = server.arg("lon").toDouble();
  if (server.hasArg("alt"))        SITE_ALT      = server.arg("alt").toFloat();
  if (server.hasArg("interval"))   REPORT_INTERVAL_MS = server.arg("interval").toInt() * 60000UL;
  if (server.hasArg("utc_offset")) UTC_OFFSET    = server.arg("utc_offset").toInt();
  if (server.hasArg("mqtt_en"))    MQTT_ENABLED  = (server.arg("mqtt_en") == "1");
  if (server.hasArg("mqtt_host"))  MQTT_HOST     = server.arg("mqtt_host");
  if (server.hasArg("mqtt_port"))  MQTT_PORT     = server.arg("mqtt_port").toInt();
  if (server.hasArg("mqtt_user"))  MQTT_USER     = server.arg("mqtt_user");
  if (server.hasArg("mqtt_pass"))  MQTT_PASS     = server.arg("mqtt_pass");
  if (server.hasArg("mqtt_topic")) MQTT_TOPIC    = server.arg("mqtt_topic");

  saveConfig();
  mqttClient.setServer(MQTT_HOST.c_str(), MQTT_PORT);

  Serial.print(F("[CFG] UTC_OFFSET = ")); Serial.println(UTC_OFFSET);

  server.send(200, "text/html",
    F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta http-equiv='refresh' content='3;url=/'>"
      "<style>body{font-family:Arial;text-align:center;padding:50px;background:#000;color:#fff}"
      ".msg{background:#1a1a1a;padding:30px;border-radius:8px;display:inline-block;"
      "box-shadow:0 2px 8px rgba(0,170,255,0.3);border:1px solid #00aaff}"
      "h1{color:#0f0}</style></head><body><div class='msg'>"
      "<h1>✅ Zapisano!</h1><p>Przekierowanie za 3 sekundy...</p>"
      "</div></body></html>"));
}

// =========================================================================
void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println(F("\n\n=== START RodosWX_2 ==="));

  LittleFS.begin();
  loadConfig();

  Serial.print(F("[CFG] UTC_OFFSET = ")); Serial.println(UTC_OFFSET);

  Wire.begin(0, 3);
  Wire.setClock(100000);

  if (bme.begin(0x76)) {
    Serial.println(F("[BME] OK 0x76")); bme_available = true;
  } else if (bme.begin(0x77)) {
    Serial.println(F("[BME] OK 0x77")); bme_available = true;
  } else {
    Serial.println(F("[BME] NIE WYKRYTO!"));
  }

  ws.begin();
  Serial.println(F("[CC1101] OK"));

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());
  Serial.print(F("[WiFi] Laczenie"));
  int wt = 0;
  while (WiFi.status() != WL_CONNECTED && wt < 40) {
    delay(500); Serial.print("."); wt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F(" OK"));
    Serial.print(F("[WiFi] IP: ")); Serial.println(WiFi.localIP());
    configTime(0, 0, "pool.ntp.org");    // NTP zawsze UTC, offset robimy ręcznie
    Serial.print(F("[NTP] Synchronizacja"));
    while (time(nullptr) < 100000) { delay(500); Serial.print("."); }
    Serial.println(F(" OK"));
    Serial.print(F("[NTP] Czas UTC: ")); Serial.println(time(nullptr));
  } else {
    Serial.println(F(" TIMEOUT!"));
  }

  mqttClient.setServer(MQTT_HOST.c_str(), MQTT_PORT);
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
  Serial.println(F("[HTTP] OK"));

  for (int i = 0; i < 4; i++) rain_buffer[i].valid = false;

  last_uart_send = millis();
  sendUartWX();

  Serial.println(F("=== SETUP OK ===\n"));
}

// =========================================================================
void loop() {
  server.handleClient();

  if (MQTT_ENABLED) {
    if (!mqttClient.connected()) {
      static unsigned long lastTry = 0;
      if (millis() - lastTry > 5000) { lastTry = millis(); mqttReconnect(); }
    } else {
      mqttClient.loop();
    }
  }

  if (bme_available) {
    float raw_p             = bme.readPressure() / 100.0;
    current_wx.bme_temp     = bme.readTemperature();
    current_wx.bme_humidity = bme.readHumidity();
    current_wx.bme_pressure = raw_p;
    float t_ref = !isnan(current_wx.temp_c) ? current_wx.temp_c : current_wx.bme_temp;
    current_wx.bme_qnh = calc_qnh(raw_p, t_ref, SITE_ALT);
  }

  ws.clearSlots();
  if (ws.getMessage() == DECODE_OK && ws.sensor[0].valid) {
    current_wx.last_update = millis();
    Serial.print(F("[RADIO] OK | RSSI: ")); Serial.println(ws.sensor[0].rssi);

    if (ws.sensor[0].w.temp_ok)     current_wx.temp_c        = ws.sensor[0].w.temp_c;
    if (ws.sensor[0].w.humidity_ok) current_wx.humidity      = ws.sensor[0].w.humidity;
    if (ws.sensor[0].w.rain_ok)     current_wx.rain_total_mm = ws.sensor[0].w.rain_mm;

    #if defined BRESSER_6_IN_1 || defined BRESSER_7_IN_1
      if (ws.sensor[0].w.uv_ok)    current_wx.uv_index  = ws.sensor[0].w.uv;
    #endif
    #ifdef BRESSER_7_IN_1
      if (ws.sensor[0].w.light_ok) current_wx.light_klx = ws.sensor[0].w.light_klx;
    #endif

    current_wx.radio_rssi = ws.sensor[0].rssi;
    current_wx.battery_ok = ws.sensor[0].battery_ok;

    if (ws.sensor[0].w.wind_ok) {
      current_wx.wind_dir   = ws.sensor[0].w.wind_direction_deg;
      current_wx.wind_speed = ws.sensor[0].w.wind_avg_meter_sec;
      current_wx.wind_gust  = ws.sensor[0].w.wind_gust_meter_sec;
      wind_speed_sum       += current_wx.wind_speed;
      wind_sample_count++;
      if (current_wx.wind_gust > wind_gust_max_period)
        wind_gust_max_period = current_wx.wind_gust;
    }
    current_wx.valid_data = true;
  }

  // UART TX co 5 sekund
  if (millis() - last_uart_send >= UART_INTERVAL_MS) {
    last_uart_send = millis();
    sendUartWX();
    Serial.print(F("[UART] ")); Serial.println(buildUartString());
  }

  // APRS + MQTT
  if (millis() - last_report_time >= REPORT_INTERVAL_MS) {
    if (current_wx.valid_data && WiFi.status() == WL_CONNECTED) {
      send_aprs();
      mqttPublishWX();
    }
    last_report_time = millis();
  }

  delay(50);
}
