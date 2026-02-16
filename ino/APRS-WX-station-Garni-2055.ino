///////////////////////////////////////////////////////////////////////////////////////////////////
// BresserWeatherSensorBasicAPRS.ino
// 
// Integracja: Bresser 7-in-1 + ESP8266 + BME280 + CC1101 -> APRS (TCP) + WebUI
// Biblioteka: BresserWeatherSensorReceiver v0.37.0 (podstawa)
//  
//
// Interfejs WWW by SP3VSS 2k26
// Wpisz w "KONFIGURACJA APRS I WIFI" swoje dane, potem możesz je zmień poprzez stronę WWW
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <time.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> 
#include <LittleFS.h> 

#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"

// ==========================================================================
// --- KONFIGURACJA APRS I WIFI (DOMYŚLNE - edytowalne przez WebUI) --------
String WIFI_SSID     = "SSID";
String WIFI_PASS     = "password";
String APRS_CALLSIGN = "AA0AAA-13";
String APRS_PASSCODE = "12345";
double SITE_LAT      = 01.2345;
double SITE_LON      = 01.2345;
// ==========================================================================

static const char* APRS_HOST = "rotate.aprs2.net";
static const uint16_t APRS_PORT = 14580;
unsigned long REPORT_INTERVAL_MS = 15UL * 60UL * 1000UL;

WeatherSensor ws;
Adafruit_BME280 bme;
bool bme_available = false;

ESP8266WebServer server(80);

float wind_speed_sum = 0.0;
int wind_sample_count = 0;
float wind_gust_max_period = 0.0;

struct WeatherData {
  float temp_c = NAN;
  uint8_t humidity = 0;
  float wind_dir = NAN;
  float wind_speed = NAN;
  float wind_gust = NAN;
  float rain_total_mm = NAN;
  float uv_index = NAN;
  float light_klx = NAN; 
  int radio_rssi = -100;  
  bool battery_ok = true; 
  bool valid_data = false;
  
  // Dane z BME280
  float bme_temp = NAN;
  float bme_humidity = NAN;
  float bme_pressure = NAN;
  
  unsigned long last_update = 0;
} current_wx;

struct RainHistory { float total_mm; bool valid; };
RainHistory rain_buffer[4]; 
uint8_t rain_idx = 0;
unsigned long last_report_time = 0;

// --- KONWERSJE I CZAS ---
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

String p3(int v) { char b[5]; snprintf(b, sizeof(b), "%03d", (v<0?0:(v>999?999:v))); return String(b); }

String get_timestamp() {
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char buff[10];
  snprintf(buff, sizeof(buff), "%02d%02d%02dz", t->tm_mday, t->tm_hour, t->tm_min);
  return String(buff);
}

// --- ZAPIS/ODCZYT KONFIGURACJI ---
void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (f) {
    f.println(WIFI_SSID);
    f.println(WIFI_PASS);
    f.println(APRS_CALLSIGN);
    f.println(APRS_PASSCODE);
    f.println(SITE_LAT, 6);
    f.println(SITE_LON, 6);
    f.println(REPORT_INTERVAL_MS / 60000); // min
    f.close();
    Serial.println(F("[CFG] Zapisano"));
  }
}

void loadConfig() {
  if (LittleFS.exists("/config.txt")) {
    File f = LittleFS.open("/config.txt", "r");
    if (f) {
      WIFI_SSID = f.readStringUntil('\n'); WIFI_SSID.trim();
      WIFI_PASS = f.readStringUntil('\n'); WIFI_PASS.trim();
      APRS_CALLSIGN = f.readStringUntil('\n'); APRS_CALLSIGN.trim();
      APRS_PASSCODE = f.readStringUntil('\n'); APRS_PASSCODE.trim();
      SITE_LAT = f.readStringUntil('\n').toDouble();
      SITE_LON = f.readStringUntil('\n').toDouble();
      REPORT_INTERVAL_MS = f.readStringUntil('\n').toInt() * 60000UL;
      f.close();
      Serial.println(F("[CFG] Wczytano"));
    }
  }
}

void send_aprs() {
  Serial.println(F("\n[APRS] Wysylanie raportu RodosWX_2..."));
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
      baro = (int)(p / 10.0);
      current_wx.bme_pressure = p / 100.0; // hPa
    }
  }

  int lum_wm2 = 0;
  if (!isnan(current_wx.light_klx)) {
      lum_wm2 = (int)(current_wx.light_klx * 7.9);
  }

  String ts = get_timestamp();
  String body = "@" + ts + format_lat(SITE_LAT) + "/" + format_lon(SITE_LON) + "_";
  
  int wd = (int)current_wx.wind_dir;
  body += p3(wd <= 0 ? 0 : wd) + "/" + p3(ms_to_mph(avg_w));
  body += "g" + p3(ms_to_mph(wind_gust_max_period));
  body += "t" + p3(c_to_f(current_wx.temp_c));
  if (r1h > 0) body += "r" + p3(r1h);
  if (current_wx.humidity > 0) { char hb[5]; snprintf(hb, sizeof(hb), "h%02d", (int)current_wx.humidity); body += hb; }
  if (baro > 0) { char bb[10]; snprintf(bb, sizeof(bb), "b%05d", baro); body += bb; }
  
  if (lum_wm2 > 0) {
      if (lum_wm2 > 999) lum_wm2 = 999;
      char lb[6]; snprintf(lb, sizeof(lb), "L%03d", lum_wm2);
      body += lb;
  }

  String comment = " GarniWX ";
  comment += " Sig:" + String(current_wx.radio_rssi) + "dBm";
  if (!isnan(current_wx.uv_index)) comment += " UV:" + String(current_wx.uv_index, 1);
  comment += current_wx.battery_ok ? " Bat:OK" : " Bat:LOW";

  String packet = APRS_CALLSIGN + ">APRS,TCPIP*:" + body + comment;

  WiFiClient cl;
  if (cl.connect(APRS_HOST, APRS_PORT)) {
    cl.printf("user %s pass %s vers RodosBME 1.7\n", APRS_CALLSIGN.c_str(), APRS_PASSCODE.c_str());
    delay(200);
    cl.println(packet);
    delay(500);
    cl.stop();
    Serial.println("-> " + packet);
    wind_speed_sum = 0; wind_sample_count = 0; wind_gust_max_period = 0;
  }
}

// --- INTERFEJS WEBOWY ---
void handleRoot() {
  String html = F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Stacja Pogodowa APRS</title>"
    "<style>"
    "body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#000;color:#fff}"
    ".container{display:flex;gap:20px;max-width:1400px;margin:0 auto;flex-wrap:wrap}"
    ".column{flex:1;min-width:300px;background:#1a1a1a;padding:20px;border-radius:8px;box-shadow:0 2px 8px rgba(255,255,255,0.1);border:1px solid #333}"
    "h1{color:#00aaff;margin-top:0;font-size:24px;text-shadow:0 0 10px rgba(0,170,255,0.5)}"
    "h2{color:#00aaff;border-bottom:2px solid #00aaff;padding-bottom:8px;font-size:18px}"
    ".data-row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #333}"
    ".label{font-weight:bold;color:#aaa}"
    ".value{color:#fff}"
    ".status-ok{color:#0f0;font-weight:bold;text-shadow:0 0 5px rgba(0,255,0,0.5)}"
    ".status-warn{color:#ff0;font-weight:bold;text-shadow:0 0 5px rgba(255,255,0,0.5)}"
    ".status-error{color:#f00;font-weight:bold;text-shadow:0 0 5px rgba(255,0,0,0.5)}"
    "input,select{width:100%;padding:8px;margin:5px 0 15px 0;border:1px solid #444;border-radius:4px;box-sizing:border-box;background:#222;color:#fff}"
    "label{font-weight:bold;color:#aaa;display:block}"
    "button{background:#00aaff;color:#000;padding:12px 24px;border:none;border-radius:4px;cursor:pointer;font-size:16px;width:100%;font-weight:bold}"
    "button:hover{background:#0088cc;box-shadow:0 0 15px rgba(0,170,255,0.6)}"
    ".info{background:#2a2a2a;padding:10px;border-radius:4px;margin:10px 0;font-size:14px;border:1px solid #444}"
    "a{color:#00aaff;text-decoration:none;font-weight:bold}"
    "a:hover{color:#0088cc;text-decoration:underline}"
    ".aprs-link{background:#2a2a2a;padding:12px;border-radius:4px;margin:0 0 15px 0;text-align:center;border:1px solid #00aaff}"
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
    "document.getElementById('upd').innerText=d.upd;"
    "});},2000);"
    "</script>"
    "</head><body>"
    "<h1>🌤️ Stacja Pogodowa APRS - ");
  html += APRS_CALLSIGN;
  html += F("</h1><div class='container'>"
    "<div class='column'>");
  
  // LINK DO APRS.FI
  html += F("<div class='aprs-link'>📍 <a href='https://aprs.fi/");
  html += APRS_CALLSIGN;
  html += F("' target='_blank'>Zobacz stację na APRS.fi</a></div>");
  
  html += F("<h2>📡 Dane z Radia (Bresser 7-in-1)</h2>");
  
  html += F("<div class='data-row'><span class='label'>Temperatura:</span><span class='value' id='temp'>");
  html += !isnan(current_wx.temp_c) ? String(current_wx.temp_c, 1) + " °C" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Wilgotność:</span><span class='value' id='hum'>");
  html += current_wx.humidity > 0 ? String(current_wx.humidity) + " %" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Kierunek wiatru:</span><span class='value' id='wdir'>");
  html += !isnan(current_wx.wind_dir) ? String((int)current_wx.wind_dir) + " °" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Prędkość wiatru:</span><span class='value' id='wspd'>");
  html += !isnan(current_wx.wind_speed) ? String(current_wx.wind_speed, 1) + " m/s" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Porywy wiatru:</span><span class='value' id='wgst'>");
  html += !isnan(current_wx.wind_gust) ? String(current_wx.wind_gust, 1) + " m/s" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Opady (suma):</span><span class='value' id='rain'>");
  html += !isnan(current_wx.rain_total_mm) ? String(current_wx.rain_total_mm, 1) + " mm" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Indeks UV:</span><span class='value' id='uv'>");
  html += !isnan(current_wx.uv_index) ? String(current_wx.uv_index, 1) : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Światło:</span><span class='value' id='lux'>");
  html += !isnan(current_wx.light_klx) ? String(current_wx.light_klx, 1) + " klx" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Siła sygnału:</span><span class='value' id='rssi'>");
  html += String(current_wx.radio_rssi) + " dBm";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Bateria:</span><span class='value' id='bat' class='");
  html += current_wx.battery_ok ? F("status-ok'>OK") : F("status-warn'>LOW");
  html += F("</span></div>");
  
  html += F("<h2>🌡️ Dane z BME280 (lokalne)</h2>");
  
  html += F("<div class='data-row'><span class='label'>Temperatura:</span><span class='value' id='btemp'>");
  html += !isnan(current_wx.bme_temp) ? String(current_wx.bme_temp, 1) + " °C" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Wilgotność:</span><span class='value' id='bhum'>");
  html += !isnan(current_wx.bme_humidity) ? String(current_wx.bme_humidity, 1) + " %" : "---";
  html += F("</span></div>");
  
  html += F("<div class='data-row'><span class='label'>Ciśnienie:</span><span class='value' id='bpress'>");
  html += !isnan(current_wx.bme_pressure) ? String(current_wx.bme_pressure, 1) + " hPa" : "---";
  html += F("</span></div>");
  
  html += F("<div class='info'>Ostatnia aktualizacja: <span id='upd'>");
  html += current_wx.last_update > 0 ? String((millis() - current_wx.last_update) / 1000) + " s temu" : "brak";
  html += F("</span></div></div>");
  
  // PRAWA KOLUMNA - USTAWIENIA
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
  html += F("'><label>Szerokość geograficzna:</label><input name='lat' type='number' step='0.0001' value='");
  html += String(SITE_LAT, 6);
  html += F("'><label>Długość geograficzna:</label><input name='lon' type='number' step='0.0001' value='");
  html += String(SITE_LON, 6);
  html += F("'><label>Interwał raportów (min):</label><input name='interval' type='number' min='1' value='");
  html += String(REPORT_INTERVAL_MS / 60000);
  html += F("'><button type='submit'>💾 Zapisz Ustawienia</button></form>"
    "<div class='info' style='margin-top:20px'>IP stacji: ");
  html += WiFi.localIP().toString();
  html += F("<br>Serwer APRS: ");
  html += APRS_HOST;
  html += F("</div></div></div></body></html>");
  
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"temp\":\"" + (!isnan(current_wx.temp_c) ? String(current_wx.temp_c, 1) + " °C" : "---") + "\",";
  json += "\"hum\":\"" + (current_wx.humidity > 0 ? String(current_wx.humidity) + " %" : "---") + "\",";
  json += "\"wdir\":\"" + (!isnan(current_wx.wind_dir) ? String((int)current_wx.wind_dir) + " °" : "---") + "\",";
  json += "\"wspd\":\"" + (!isnan(current_wx.wind_speed) ? String(current_wx.wind_speed, 1) + " m/s" : "---") + "\",";
  json += "\"wgst\":\"" + (!isnan(current_wx.wind_gust) ? String(current_wx.wind_gust, 1) + " m/s" : "---") + "\",";
  json += "\"rain\":\"" + (!isnan(current_wx.rain_total_mm) ? String(current_wx.rain_total_mm, 1) + " mm" : "---") + "\",";
  json += "\"uv\":\"" + (!isnan(current_wx.uv_index) ? String(current_wx.uv_index, 1) : "---") + "\",";
  json += "\"lux\":\"" + (!isnan(current_wx.light_klx) ? String(current_wx.light_klx, 1) + " klx" : "---") + "\",";
  json += "\"rssi\":\"" + String(current_wx.radio_rssi) + " dBm\",";
  json += "\"bat\":\"" + String(current_wx.battery_ok ? "OK" : "LOW") + "\",";
  json += "\"btemp\":\"" + (!isnan(current_wx.bme_temp) ? String(current_wx.bme_temp, 1) + " °C" : "---") + "\",";
  json += "\"bhum\":\"" + (!isnan(current_wx.bme_humidity) ? String(current_wx.bme_humidity, 1) + " %" : "---") + "\",";
  json += "\"bpress\":\"" + (!isnan(current_wx.bme_pressure) ? String(current_wx.bme_pressure, 1) + " hPa" : "---") + "\",";
  json += "\"upd\":\"" + (current_wx.last_update > 0 ? String((millis() - current_wx.last_update) / 1000) + " s temu" : "brak") + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSave() {
  if (server.hasArg("ssid")) WIFI_SSID = server.arg("ssid");
  if (server.hasArg("pass")) WIFI_PASS = server.arg("pass");
  if (server.hasArg("call")) APRS_CALLSIGN = server.arg("call");
  if (server.hasArg("passcode")) APRS_PASSCODE = server.arg("passcode");
  if (server.hasArg("lat")) SITE_LAT = server.arg("lat").toDouble();
  if (server.hasArg("lon")) SITE_LON = server.arg("lon").toDouble();
  if (server.hasArg("interval")) REPORT_INTERVAL_MS = server.arg("interval").toInt() * 60000UL;
  
  saveConfig();
  
  String html = F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<meta http-equiv='refresh' content='3;url=/'>"
    "<style>body{font-family:Arial;text-align:center;padding:50px;background:#000;color:#fff}"
    ".msg{background:#1a1a1a;padding:30px;border-radius:8px;display:inline-block;box-shadow:0 2px 8px rgba(0,170,255,0.3);border:1px solid #00aaff}"
    "h1{color:#0f0;text-shadow:0 0 10px rgba(0,255,0,0.5)}</style></head><body><div class='msg'>"
    "<h1>✅ Zapisano!</h1><p>Przekierowanie za 3 sekundy...</p>"
    "</div></body></html>");
  server.send(200, "text/html", html);
}

void setup() {
  delay(3000); 
  Serial.begin(115200);
  Serial.println(F("\n\n--- START RodosWX_2 Web ---"));

  LittleFS.begin();
  loadConfig();

  Wire.begin(2, 5); 
  Wire.setClock(100000);
  
  if (bme.begin(0x76)) {
      Serial.println(F("BME280: OK"));
      bme_available = true;
  } else {
      Serial.println(F("BME280: NIE WYKRYTO"));
  }

  ws.begin();

  Serial.println(F("Laczenie WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());
  
  int wifi_timeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_timeout < 40) { 
      delay(500); Serial.print("."); wifi_timeout++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
      Serial.println(F(" WiFi OK"));
      Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
      configTime(0, 0, "pool.ntp.org");
      while (time(nullptr) < 100000) { delay(500); Serial.print("T"); }
      Serial.println(F(" Time OK"));
  }
  
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
  Serial.println(F("Serwer HTTP uruchomiony"));
  
  for(int i=0; i<4; i++) rain_buffer[i].valid = false;
  Serial.println(F("--- SETUP ZAKONCZONY ---"));
}

void loop() {
  server.handleClient();
  
  // Odczyt BME280
  if (bme_available) {
    current_wx.bme_temp = bme.readTemperature();
    current_wx.bme_humidity = bme.readHumidity();
    current_wx.bme_pressure = bme.readPressure() / 100.0;
  }
  
  ws.clearSlots();
  if (ws.getMessage() == DECODE_OK && ws.sensor[0].valid) {
      Serial.println(F("[RADIO] Odebrano dane"));
      current_wx.last_update = millis();
      
      if (ws.sensor[0].w.temp_ok) current_wx.temp_c = ws.sensor[0].w.temp_c;
      if (ws.sensor[0].w.humidity_ok) current_wx.humidity = ws.sensor[0].w.humidity;
      if (ws.sensor[0].w.rain_ok) current_wx.rain_total_mm = ws.sensor[0].w.rain_mm;
      
      #if defined BRESSER_6_IN_1 || defined BRESSER_7_IN_1
        if (ws.sensor[0].w.uv_ok) current_wx.uv_index = ws.sensor[0].w.uv;
      #endif
      #ifdef BRESSER_7_IN_1
        if (ws.sensor[0].w.light_ok) current_wx.light_klx = ws.sensor[0].w.light_klx;
      #endif

      current_wx.radio_rssi = ws.sensor[0].rssi;
      current_wx.battery_ok = ws.sensor[0].battery_ok;

      if (ws.sensor[0].w.wind_ok) {
          current_wx.wind_dir = ws.sensor[0].w.wind_direction_deg;
          current_wx.wind_speed = ws.sensor[0].w.wind_avg_meter_sec;
          current_wx.wind_gust = ws.sensor[0].w.wind_gust_meter_sec;
          
          wind_speed_sum += current_wx.wind_speed; 
          wind_sample_count++;
          if (current_wx.wind_gust > wind_gust_max_period) wind_gust_max_period = current_wx.wind_gust;
          
          Serial.printf("Wind: %.1f m/s | Gust: %.1f | RSSI: %d dBm | Lux: %.1f\n", 
                        current_wx.wind_speed, current_wx.wind_gust, current_wx.radio_rssi, current_wx.light_klx);
      }
      current_wx.valid_data = true;
  }

  if (millis() - last_report_time >= REPORT_INTERVAL_MS) {
      if (current_wx.valid_data && WiFi.status() == WL_CONNECTED) {
          send_aprs();
      }
      last_report_time = millis();
  }
  delay(50);
}
