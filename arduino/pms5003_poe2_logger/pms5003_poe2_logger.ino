/*
  ESP32-POE2 + PMS5003 + microSD + NTP + web server + Wi-Fi AP
  ============================================================

  Board:  Olimex ESP32-POE2 (ESP32-WROVER-E, 4 MB flash, PSRAM)

  In Arduino IDE / arduino-cli select a board with PSRAM enabled
  (e.g. "ESP32 Wrover Module") - POE2 uses a WROVER module.
  Note: the Arduino core 3.3.11 has no board named "esp32-poe2".

  Web server (port 80) - reachable over Ethernet and over the Wi-Fi AP:
    /               HTML: graph, current values, file list
    /admin          administration: network settings and manual time
    /api/status     JSON: time, networks, records, current values, OTA
    /api/data       JSON: records for the graph (?f=file or ?from=&to=)
    /api/files      JSON: files on the card
    /download?f=    download a file
    /delete?f=      delete a file (the active one is protected)

  Networks:
    - Ethernet (LAN8720) is the primary link; DHCP or static IP
    - Wi-Fi AP is available when enabled in /admin (SSID, password,
      channel); clients get an address from 192.168.4.0/24
    - web server and OTA work on both networks

  Time:
    - NTP when the network is available ("NTP" in the CSV)
    - manually set time from /admin when NTP is not available ("MANUAL")
    - otherwise millis() ("MILLIS")

  Settings are stored in NVS (Preferences) and survive a restart.
  Changing network settings restarts the device.

  Data is written to a separate CSV file per creation date and time:
      /pms_YYYYMMDD_HHMMSS.csv
  The file rotates automatically at midnight (no board restart needed).

  OTA: the board can be flashed over the network (ArduinoOTA), no USB:
      arduino-cli upload -p <IP> --fqbn esp32:esp32:esp32wrover \
          --protocol network <path_to_sketch>

  ------------------------------------------------------------------
  PINS (verified against Olimex documentation for ESP32-POE2)
  ------------------------------------------------------------------
  Ethernet - LAN8720: PHY_ADDR=0, MDC=GPIO23, MDIO=GPIO18, POWER=GPIO12,
  CLK=GPIO0 (CLK_OUT) - REQUIRED for POE2, GPIO16/17 are used by PSRAM.

  microSD (onboard slot, 1-bit SDMMC): CLK=GPIO14, CMD=GPIO15, D0=GPIO2.

  PMS5003: VCC=5V (EXT1 pin 5), GND=GND, TX=GPIO33 -> ESP32 RX,
  RX=GPIO13 <- ESP32 TX (optional), SET/RESET not connected.

  NOTE: POE2 has no galvanic isolation between PoE power and USB.
  Disconnect the Ethernet cable while programming over USB if the board
  is powered over PoE.
*/

// ---------------------------------------------------------------------------
// Ethernet definitions MUST come before #include <ETH.h>
// ---------------------------------------------------------------------------
#ifndef ETH_PHY_TYPE
#define ETH_PHY_TYPE  ETH_PHY_LAN8720
#define ETH_PHY_ADDR  0
#define ETH_PHY_MDC   23
#define ETH_PHY_MDIO  18
#define ETH_PHY_POWER 12
#define ETH_CLK_MODE  ETH_CLOCK_GPIO0_OUT
#endif

#include <SD_MMC.h>
#include <ETH.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Update.h>
#include <sys/time.h>
#include <time.h>

// ============================ CONFIGURATION ============================

// ---- Serial output ----
// 0 = completely off (everything is on the web page)
// 1 = on, 115200 baud (diagnostics)
#define SERIAL_DEBUG 0

// ---- PMS5003 (UART2) ----
#define PMS_RX_PIN   33     // ESP32 RX  <- PMS5003 TX
#define PMS_TX_PIN   13     // ESP32 TX  -> PMS5003 RX (optional)
#define PMS_BAUD     9600

// ---- Time zone ----
// The "timestamp" CSV column is always a Unix epoch in UTC; the time zone is
// used only for the readable file name and the time shown on the page.
#define TZ_INFO      "CET-1CEST,M3.5.0,M10.5.0/3"   // Europe/Zagreb
#define CSV_PREFIX   "/pms_"

// ---- Network and time ----
#define NTP_SERVER              "pool.ntp.org"
#define ETH_TIMEOUT_MS          10000UL
#define NET_TOTAL_BUDGET_MS     20000UL
#define NTP_TIMEOUT_MS          10000UL
#define NTP_EPOCH_MIN           1000000000UL
#define PMS_SILENCE_WARN_MS     5000UL

// ---- Web server and graph ----
#define WEB_PORT                80
#define RING_MAX                500     // records kept in RAM for a single file
#define HIST_MAX                3000    // max records for a multi-file history query

// ---- OTA (network update) ----
#define OTA_ENABLE      1
#define OTA_HOSTNAME    "esp32-poe2-pms"
#define OTA_PASSWORD    ""

// ---- Default network settings (editable on /admin) ----
#define DEF_ETH_IP      "192.168.3.50"
#define DEF_ETH_MASK    "255.255.255.0"
#define DEF_ETH_GW      "192.168.3.1"
#define DEF_ETH_DNS     "8.8.8.8"
#define DEF_AP_SSID     "esp32-poe2-pms"
#define DEF_AP_PASSWORD "pms5003pms"    // "" or <8 chars = open network
#define DEF_AP_CHANNEL  6
#define NVS_NAMESPACE   "pms5003"

// ---------------------------- SERIAL OUTPUT ----------------------------
#if SERIAL_DEBUG
  #define SLOG Serial
#else
  class NullPrint : public Print {
   public:
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t *, size_t n) override { return n; }
  };
  static NullPrint nullPrint;
  #define SLOG nullPrint
#endif

// ========================= DEVICE SETTINGS =========================

struct DeviceSettings {
  bool    ethDhcp;
  char    ethIp[16];
  char    ethMask[16];
  char    ethGw[16];
  char    ethDns[16];
  bool    apEnabled;
  char    apSsid[33];
  char    apPassword[65];
  uint8_t apChannel;
  char    ntpServer[64];
  char    theme[16];
};

static DeviceSettings settings;
static Preferences    prefs;

// ======================= PMS5003 DATA STRUCTURE =======================

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard;
  uint16_t pm25_standard;
  uint16_t pm100_standard;
  uint16_t pm10_env;      // atmospheric values - these are logged
  uint16_t pm25_env;
  uint16_t pm100_env;
  uint16_t particles_03um;
  uint16_t particles_05um;
  uint16_t particles_10um;
  uint16_t particles_25um;
  uint16_t particles_50um;
  uint16_t particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

static pms5003data pms;

// ============================ RUNTIME STATE ============================

static fs::FS     *sdFs         = nullptr;
static bool        sdReady      = false;
static char        csvPath[48]  = "";

static bool        ethActive    = false;      // Ethernet link + IP
static bool        ntpSynced    = false;
static bool        manualTime   = false;      // time set on /admin
static const char *netMode      = "offline";

static uint32_t    bootEpoch       = 0;       // 0 = unknown
static uint32_t    restartScheduled = 0;      // 0 = not scheduled

static uint16_t    lastWrittenPm25  = 0xFFFF; // last WRITTEN value (for compare)
static uint16_t    lastWrittenPm100 = 0xFFFF;
static bool        firstRecord      = true;

static uint32_t    recordCount = 0;
static bool        haveSample  = false;
static uint32_t    lastTs      = 0;
static uint16_t    lastPm1     = 0;
static uint16_t    lastPm25    = 0;
static uint16_t    lastPm100   = 0;

static uint32_t    lastPmsMs    = 0;
static uint32_t    lastWarnMs   = 0;
static uint32_t    netStartMs   = 0;

static uint32_t    framesOk  = 0;   // valid 32-byte frames
static uint32_t    framesBad = 0;   // frames with bad header/checksum

// ---- Daily file rotation ----
static int         rotDay    = -1;
static int         rotMonth  = -1;
static int         rotYear   = -1;
static uint32_t    lastRotCheck = 0;

static WebServer   server(WEB_PORT);

// ---- In-RAM ring of records (single file view) ----
struct LogRow {
  uint32_t ts;
  uint16_t pm1;
  uint16_t pm25;
  uint16_t pm100;
};

static LogRow  ring[RING_MAX];
static int     ringCount = 0;
static int     ringHead  = 0;
static bool    ringValid = false;
static String  ringFile  = "";

// ============================== TIME ==============================

static const char *timeSourceName() {
  if (ntpSynced)  return "NTP";
  if (manualTime) return "MANUAL";
  return "MILLIS";
}

static bool epochValid() {
  return (ntpSynced || manualTime) && time(nullptr) > (time_t)NTP_EPOCH_MIN;
}

static uint32_t currentTimestamp() {
  if (epochValid()) {
    return (uint32_t)time(nullptr);
  }
  return millis();
}

static void formatEpochTs(uint32_t e, char *buf, size_t n) {
  if (e == 0) {
    snprintf(buf, n, "unknown");
    return;
  }
  struct tm tmInfo;
  time_t t = (time_t)e;
  localtime_r(&t, &tmInfo);
  snprintf(buf, n, "%04d-%02d-%02d %02d:%02d:%02d",
           tmInfo.tm_year + 1900, tmInfo.tm_mon + 1, tmInfo.tm_mday,
           tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec);
}

static void formatNow(char *buf, size_t n) {
  if (epochValid()) {
    formatEpochTs((uint32_t)time(nullptr), buf, n);
  } else {
    snprintf(buf, n, "uptime %lu s", (unsigned long)(millis() / 1000UL));
  }
}

// Manual time (used when NTP is not available). Time is not kept across a
// restart because the board has no battery-backed RTC.
static void setManualTime(time_t epoch) {
  struct timeval tv;
  tv.tv_sec  = epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  manualTime = true;
  if (bootEpoch == 0) {
    bootEpoch = (uint32_t)epoch - (millis() / 1000UL);
  }
  SLOG.print("[TIME] Manually set to ");
  SLOG.println((uint32_t)epoch);
}

// ===================== SETTINGS: LOAD / SAVE =====================

static void loadSettings() {
  settings.ethDhcp = true;
  strlcpy(settings.ethIp,       DEF_ETH_IP,       sizeof(settings.ethIp));
  strlcpy(settings.ethMask,     DEF_ETH_MASK,     sizeof(settings.ethMask));
  strlcpy(settings.ethGw,       DEF_ETH_GW,       sizeof(settings.ethGw));
  strlcpy(settings.ethDns,      DEF_ETH_DNS,      sizeof(settings.ethDns));
  settings.apEnabled = true;
  strlcpy(settings.apSsid,      DEF_AP_SSID,      sizeof(settings.apSsid));
  strlcpy(settings.apPassword,  DEF_AP_PASSWORD,  sizeof(settings.apPassword));
  settings.apChannel = DEF_AP_CHANNEL;
  strlcpy(settings.ntpServer,   NTP_SERVER,       sizeof(settings.ntpServer));
  strlcpy(settings.theme,       "light",          sizeof(settings.theme));

  if (!prefs.begin(NVS_NAMESPACE, true)) {
    return;                                   // no stored settings
  }
  settings.ethDhcp   = prefs.getBool("ethdhcp", settings.ethDhcp);
  settings.apEnabled = prefs.getBool("ap",      settings.apEnabled);
  settings.apChannel = prefs.getUChar("apkanal", settings.apChannel);

  String s;
  s = prefs.getString("ethip",    settings.ethIp);       strlcpy(settings.ethIp,      s.c_str(), sizeof(settings.ethIp));
  s = prefs.getString("ethmaska", settings.ethMask);     strlcpy(settings.ethMask,    s.c_str(), sizeof(settings.ethMask));
  s = prefs.getString("ethgw",    settings.ethGw);       strlcpy(settings.ethGw,      s.c_str(), sizeof(settings.ethGw));
  s = prefs.getString("ethdns",   settings.ethDns);      strlcpy(settings.ethDns,     s.c_str(), sizeof(settings.ethDns));
  s = prefs.getString("apssid",   settings.apSsid);      strlcpy(settings.apSsid,     s.c_str(), sizeof(settings.apSsid));
  s = prefs.getString("aploz",    settings.apPassword);  strlcpy(settings.apPassword, s.c_str(), sizeof(settings.apPassword));
  s = prefs.getString("ntp",      settings.ntpServer);   strlcpy(settings.ntpServer,  s.c_str(), sizeof(settings.ntpServer));
  s = prefs.getString("theme",    settings.theme);      strlcpy(settings.theme,      s.c_str(), sizeof(settings.theme));
  prefs.end();

  if (settings.apChannel < 1 || settings.apChannel > 13) {
    settings.apChannel = DEF_AP_CHANNEL;
  }
}

static void saveSettings() {
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    SLOG.println("[NVS] Cannot open for writing");
    return;
  }
  prefs.putBool("ethdhcp",  settings.ethDhcp);
  prefs.putBool("ap",       settings.apEnabled);
  prefs.putUChar("apkanal", settings.apChannel);
  prefs.putString("ethip",    settings.ethIp);
  prefs.putString("ethmaska", settings.ethMask);
  prefs.putString("ethgw",    settings.ethGw);
  prefs.putString("ethdns",   settings.ethDns);
  prefs.putString("apssid",   settings.apSsid);
  prefs.putString("aploz",    settings.apPassword);
  prefs.putString("ntp",      settings.ntpServer);
  prefs.putString("theme",    settings.theme);
  prefs.end();
}

// ============================== RECORD RING ==============================

static void ringReset(const String &file) {
  ringCount = 0;
  ringHead  = 0;
  ringFile  = file;
  ringValid = true;
}

static void ringPush(uint32_t ts, uint16_t a, uint16_t b, uint16_t c) {
  int idx;
  if (ringCount < RING_MAX) {
    idx = (ringHead + ringCount) % RING_MAX;
    ringCount++;
  } else {
    idx = ringHead;
    ringHead = (ringHead + 1) % RING_MAX;
  }
  ring[idx].ts    = ts;
  ring[idx].pm1   = a;
  ring[idx].pm25  = b;
  ring[idx].pm100 = c;
}

// Loads a CSV file into the ring (last RING_MAX records). If the ring already
// holds that file, nothing is read again.
static bool loadIntoRing(const String &file) {
  if (ringValid && ringFile == file) {
    return true;
  }
  if (sdFs == nullptr) {
    return false;
  }
  File f = sdFs->open(file, FILE_READ);
  if (!f || f.isDirectory()) {
    return false;
  }

  ringReset(file);
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 8) continue;
    if (line[0] < '0' || line[0] > '9') continue;   // skip header

    unsigned long ts = 0;
    unsigned a = 0, b = 0, c = 0;
    char source[16] = {0};
    if (sscanf(line.c_str(), "%lu,%15[^,],%u,%u,%u", &ts, source, &a, &b, &c) == 5) {
      ringPush((uint32_t)ts, (uint16_t)a, (uint16_t)b, (uint16_t)c);
    }
  }
  f.close();
  return true;
}

// ====================== HISTORY QUERY (MULTIPLE FILES) ======================
//
// For a requested time range this reads every CSV file that covers it,
// filters records by time and returns them sorted. If there are more records
// than HIST_MAX the buffer is halved and every other record is kept from then
// on (even decimation) so the graph always covers the whole range.

static LogRow  *histBuf    = nullptr;   // allocated in PSRAM when possible
static int      histCount  = 0;
static int      histDecim  = 1;
static int      histCnt2   = 0;
static uint32_t histTotal  = 0;         // records in range (before decimation)
static String   histFiles  = "";

static bool histEnsure() {
  if (histBuf == nullptr) {
    histBuf = (LogRow *)ps_malloc(sizeof(LogRow) * HIST_MAX);
    if (histBuf == nullptr) {
      histBuf = (LogRow *)malloc(sizeof(LogRow) * HIST_MAX);
    }
  }
  return histBuf != nullptr;
}

static void histReset() {
  histCount = 0;
  histDecim = 1;
  histCnt2  = 0;
  histTotal = 0;
  histFiles = "";
}

static void histPush(const LogRow &r) {
  histTotal++;
  if (histBuf == nullptr) {
    return;
  }
  if (histCount >= HIST_MAX) {
    int w = 0;
    for (int i = 0; i < histCount; i += 2) {
      histBuf[w++] = histBuf[i];
    }
    histCount = w;
    histDecim *= 2;
    histCnt2 = 0;
  }
  if (histCnt2 == 0) {
    histBuf[histCount++] = r;
  }
  histCnt2 = (histCnt2 + 1) % histDecim;
}

static int cmpLogRow(const void *a, const void *b) {
  uint32_t ta = ((const LogRow *)a)->ts;
  uint32_t tb = ((const LogRow *)b)->ts;
  if (ta < tb) return -1;
  if (ta > tb) return 1;
  return 0;
}

// YYYYMMDD from an epoch (local time); 0 when the epoch is not valid
static int dateKey(uint32_t epoch) {
  if (epoch < NTP_EPOCH_MIN) {
    return 0;
  }
  struct tm t;
  time_t tt = (time_t)epoch;
  localtime_r(&tt, &t);
  return (t.tm_year + 1900) * 10000 + (t.tm_mon + 1) * 100 + t.tm_mday;
}

// YYYYMMDD from a /pms_YYYYMMDD_HHMMSS.csv name; 0 when the name has no date
static int fileNameDateKey(const String &name) {
  int p = name.indexOf(CSV_PREFIX);
  if (p < 0) {
    return 0;
  }
  int s = p + (int)strlen(CSV_PREFIX);
  if ((int)name.length() < s + 8) {
    return 0;
  }
  for (int i = 0; i < 8; i++) {
    char ch = name[s + i];
    if (ch < '0' || ch > '9') {
      return 0;
    }
  }
  return name.substring(s, s + 8).toInt();
}

static bool loadHistoryRange(uint32_t from, uint32_t till) {
  if (sdFs == nullptr || !histEnsure()) {
    return false;
  }
  histReset();

  int keyFrom = 0;
  int keyTill = 0;
  if (from >= NTP_EPOCH_MIN) {
    keyFrom = dateKey(from > 172800UL ? (from - 172800UL) : 0);
    uint32_t upper = (till == 0) ? (epochValid() ? (uint32_t)time(nullptr) : from) : till;
    keyTill = dateKey(upper + 86400UL);
  }

  String candidates[64];
  int    candidateCount = 0;

  File root = sdFs->open("/");
  if (!root || !root.isDirectory()) {
    return false;
  }
  File f = root.openNextFile();
  while (f && candidateCount < 64) {
    if (!f.isDirectory()) {
      String name = String(f.name());
      if (!name.startsWith("/")) {
        name = "/" + name;
      }
      if (name.indexOf(".csv") > 0 && name.indexOf(CSV_PREFIX) >= 0) {
        bool include;
        if (from >= NTP_EPOCH_MIN) {
          int fk = fileNameDateKey(name);
          include = (fk != 0) && (fk >= keyFrom) && (fk <= keyTill);
        } else {
          include = (fileNameDateKey(name) == 0);   // MILLIS files
        }
        if (include) {
          candidates[candidateCount++] = name;
        }
      }
    }
    f = root.openNextFile();
  }
  root.close();

  for (int i = 0; i < candidateCount; i++) {
    uint32_t before = histTotal;
    File g = sdFs->open(candidates[i], FILE_READ);
    if (!g) {
      continue;
    }
    while (g.available()) {
      String line = g.readStringUntil('\n');
      line.trim();
      if (line.length() < 8) continue;
      if (line[0] < '0' || line[0] > '9') continue;

      unsigned long ts = 0;
      unsigned a = 0, b = 0, c = 0;
      char source[16] = {0};
      if (sscanf(line.c_str(), "%lu,%15[^,],%u,%u,%u", &ts, source, &a, &b, &c) == 5) {
        if ((from == 0 || ts >= from) && (till == 0 || ts <= till)) {
          LogRow r;
          r.ts    = (uint32_t)ts;
          r.pm1   = (uint16_t)a;
          r.pm25  = (uint16_t)b;
          r.pm100 = (uint16_t)c;
          histPush(r);
        }
      }
    }
    g.close();

    if (histTotal > before && histFiles.length() < 120) {
      if (histFiles.length() > 0) {
        histFiles += ",";
      }
      histFiles += candidates[i];
    }
  }

  if (histCount > 1) {
    qsort(histBuf, histCount, sizeof(LogRow), cmpLogRow);
  }
  return true;
}

// ============================== SD CARD ==============================

static bool mountSD() {
  if (SD_MMC.begin("/sdcard", true)) {   // 1-bit: CLK=14, CMD=15, D0=2
    sdFs = &SD_MMC;
    return true;
  }
  return false;
}

static void buildFileName() {
  time_t now = time(nullptr);

  if (epochValid()) {
    struct tm tmInfo;
    localtime_r(&now, &tmInfo);
    snprintf(csvPath, sizeof(csvPath),
             CSV_PREFIX "%04d%02d%02d_%02d%02d%02d.csv",
             tmInfo.tm_year + 1900, tmInfo.tm_mon + 1, tmInfo.tm_mday,
             tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec);
  } else {
    snprintf(csvPath, sizeof(csvPath),
             CSV_PREFIX "millis_%010lu.csv", (unsigned long)millis());
  }
}

static bool prepareCsv() {
  if (csvPath[0] == '\0') {
    buildFileName();
  }

  if (sdFs->exists(csvPath)) {
    SLOG.print("[SD] File already exists, continuing with: ");
    SLOG.println(csvPath);
    return true;
  }

  File f = sdFs->open(csvPath, FILE_WRITE);
  if (!f) {
    SLOG.print("[SD] ERROR: cannot create ");
    SLOG.println(csvPath);
    return false;
  }
  f.println("timestamp,time_source,pm1_0,pm2_5,pm10");
  f.close();
  SLOG.print("[SD] Created file with header: ");
  SLOG.println(csvPath);
  return true;
}

// ---- Daily rotation ----

static void rememberRotationDate() {
  if (!epochValid()) {
    return;
  }
  struct tm t;
  time_t now = time(nullptr);
  localtime_r(&now, &t);
  rotDay   = t.tm_mday;
  rotMonth = t.tm_mon;
  rotYear  = t.tm_year;
}

// Opens a new file named after the current date and time. There is no open
// handle to close because the file is opened per write.
static void rotateFile() {
  struct tm t;
  time_t now = time(nullptr);
  localtime_r(&now, &t);

  buildFileName();
  recordCount      = 0;
  firstRecord      = true;
  lastWrittenPm25  = 0xFFFF;     // first change in the new file is written
  lastWrittenPm100 = 0xFFFF;
  ringValid        = false;      // the graph will switch to the new file

  if (sdReady) {
    prepareCsv();
  }

  rotDay   = t.tm_mday;
  rotMonth = t.tm_mon;
  rotYear  = t.tm_year;

  SLOG.print("[SD] Daily rotation, new file: ");
  SLOG.println(csvPath);
}

static void checkRotation() {
  if (!epochValid() || !sdReady || rotDay < 0) {
    return;
  }
  struct tm t;
  time_t now = time(nullptr);
  localtime_r(&now, &t);
  if (t.tm_mday != rotDay || t.tm_mon != rotMonth || t.tm_year != rotYear) {
    rotateFile();
  }
}

static bool writeRow(uint32_t ts, const char *source,
                     uint16_t pm1, uint16_t pm25, uint16_t pm100) {
  if (sdFs == nullptr) {
    return false;
  }
  File f = sdFs->open(csvPath, FILE_APPEND);
  if (!f) {
    SLOG.print("[SD] ERROR: cannot open ");
    SLOG.print(csvPath);
    SLOG.println(" for writing");
    return false;
  }
  f.print(ts);      f.print(',');
  f.print(source);  f.print(',');
  f.print(pm1);     f.print(',');
  f.print(pm25);    f.print(',');
  f.println(pm100);
  f.close();
  recordCount++;

  if (ringValid && ringFile == String(csvPath)) {
    ringPush(ts, pm1, pm25, pm100);
  }
  return true;
}

// ============================== PMS5003 ==============================

// Reads one 32-byte frame. The loop re-syncs on the 0x42 0x4D header, so a
// loss of sync repairs itself - otherwise every following frame would stay
// misaligned and data would stop arriving.
static bool readPmsData(Stream *s) {
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];

  while (s->available() >= 32) {
    if (s->peek() != 0x42) {      // look for the first header byte
      s->read();
      continue;
    }

    s->readBytes(buffer, 32);

    if (buffer[1] != 0x4D) {      // false header, keep looking
      framesBad++;
      continue;
    }

    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++) {
      sum += buffer[i];
    }
    uint16_t receivedChecksum = ((uint16_t)buffer[30] << 8) | buffer[31];
    if (sum != receivedChecksum) {
      framesBad++;
      continue;
    }

    pms.framelen        = ((uint16_t)buffer[2]  << 8) | buffer[3];
    pms.pm10_standard   = ((uint16_t)buffer[4]  << 8) | buffer[5];
    pms.pm25_standard   = ((uint16_t)buffer[6]  << 8) | buffer[7];
    pms.pm100_standard  = ((uint16_t)buffer[8]  << 8) | buffer[9];
    pms.pm10_env        = ((uint16_t)buffer[10] << 8) | buffer[11];
    pms.pm25_env        = ((uint16_t)buffer[12] << 8) | buffer[13];
    pms.pm100_env       = ((uint16_t)buffer[14] << 8) | buffer[15];
    pms.particles_03um  = ((uint16_t)buffer[16] << 8) | buffer[17];
    pms.particles_05um  = ((uint16_t)buffer[18] << 8) | buffer[19];
    pms.particles_10um  = ((uint16_t)buffer[20] << 8) | buffer[21];
    pms.particles_25um  = ((uint16_t)buffer[22] << 8) | buffer[23];
    pms.particles_50um  = ((uint16_t)buffer[24] << 8) | buffer[25];
    pms.particles_100um = ((uint16_t)buffer[26] << 8) | buffer[27];
    pms.unused          = ((uint16_t)buffer[28] << 8) | buffer[29];
    pms.checksum        = receivedChecksum;

    framesOk++;
    return true;
  }

  return false;
}

// ============================== NETWORK ==============================

static bool waitEthernet() {
  // A static IP must be configured before begin().
  if (!settings.ethDhcp) {
    IPAddress ip, gw, mask, dns;
    ip.fromString(settings.ethIp);
    gw.fromString(settings.ethGw);
    mask.fromString(settings.ethMask);
    dns.fromString(settings.ethDns);
    bool okCfg = ETH.config(ip, gw, mask, dns);
    SLOG.print("[NET] ETH.config(static): ");
    SLOG.println(okCfg ? "OK" : "FAILED");
  }

  bool ok = ETH.begin(ETH_PHY_LAN8720, ETH_PHY_ADDR, ETH_PHY_MDC,
                      ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLOCK_GPIO0_OUT);
  SLOG.print("[NET] ETH.begin():        ");
  SLOG.println(ok ? "OK" : "FAILED");
  if (!ok) {
    return false;
  }

  uint32_t start = millis();
  while ((millis() - start) < ETH_TIMEOUT_MS &&
         (millis() - netStartMs) < NET_TOTAL_BUDGET_MS) {
    if (ETH.linkUp() && ETH.localIP() != IPAddress(0, 0, 0, 0)) {
      return true;
    }
    delay(100);
  }
  return false;
}

static void startAP() {
  if (!settings.apEnabled) {
    SLOG.println("[AP] Disabled in settings");
    return;
  }

  WiFi.mode(WIFI_AP);
  bool ok;
  if (strlen(settings.apPassword) >= 8) {
    ok = WiFi.softAP(settings.apSsid, settings.apPassword, settings.apChannel);
  } else {
    ok = WiFi.softAP(settings.apSsid, NULL, settings.apChannel);   // open network
  }

  SLOG.print("[AP] ");
  SLOG.print(settings.apSsid);
  SLOG.print(" channel ");
  SLOG.print(settings.apChannel);
  SLOG.print(" -> ");
  SLOG.print(WiFi.softAPIP());
  SLOG.println(ok ? "" : " (ERROR)");
}

static bool syncNtp() {
  SLOG.print("[NTP] configTime(");
  SLOG.print(settings.ntpServer);
  SLOG.println(")");
  configTime(0, 0, settings.ntpServer);

  uint32_t start = millis();
  while (true) {
    if (time(nullptr) > (time_t)NTP_EPOCH_MIN) {
      return true;
    }
    uint32_t now = millis();
    if ((now - start) >= NTP_TIMEOUT_MS) {
      return false;
    }
    if ((now - netStartMs) >= NET_TOTAL_BUDGET_MS) {
      return false;
    }
    delay(200);
  }
}

// ============================== SAMPLE HANDLING ==============================

static void handleSample() {
  // Atmospheric values = frame bytes 10-15
  uint16_t pm1   = pms.pm10_env;
  uint16_t pm25  = pms.pm25_env;
  uint16_t pm100 = pms.pm100_env;

  haveSample = true;
  uint32_t ts       = currentTimestamp();
  const char *src   = timeSourceName();
  lastTs    = ts;
  lastPm1   = pm1;
  lastPm25  = pm25;
  lastPm100 = pm100;

  bool changed = (pm25 != lastWrittenPm25) || (pm100 != lastWrittenPm100);

  if (changed) {
    if (!sdReady) {
      sdReady = prepareCsv();
    }
    if (sdReady && writeRow(ts, src, pm1, pm25, pm100)) {
      lastWrittenPm25  = pm25;
      lastWrittenPm100 = pm100;
      if (firstRecord) {
        firstRecord = false;
        SLOG.print("[SD] First recorded entry: ");
        SLOG.print(src);
        SLOG.print(" ");
        SLOG.println(ts);
      }
    }
  }
}

// ============================== MAIN PAGE ==============================

static const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-POE2 &middot; PMS5003</title>
<style>
:root{
  --bg:#f5f7fa; --card:#ffffff; --text:#111827; --muted:#6b7280; --line:#e5e7eb;
  --line-soft:#f1f2f4; --hover:#fafbfc;
  --accent:#2563eb; --accent-dark:#1d4ed8; --accent-soft:#eef4ff;
  --ok:#15803d; --warn:#b45309; --err:#b91c1c;
  --pm1:#16a34a; --pm25:#ea580c; --pm10:#dc2626;
  --r:14px; --sh:0 1px 2px rgba(16,24,40,.05),0 10px 28px rgba(16,24,40,.06);
}
body.theme-dark{
  --bg:#0f1319; --card:#171d26; --text:#e7ebf2; --muted:#94a3b8; --line:#252d3a;
  --line-soft:#1f2733; --hover:#1d2430;
  --accent:#4f8cff; --accent-dark:#3a74e6; --accent-soft:#1b2534;
  --pm1:#22c55e; --pm25:#f97316; --pm10:#ef4444;
  --sh:0 1px 2px rgba(0,0,0,.5),0 12px 30px rgba(0,0,0,.45);
}
body.theme-contrast{
  --bg:#000000; --card:#000000; --text:#ffffff; --muted:#ffd400; --line:#ffffff;
  --line-soft:#3a3a3a; --hover:#141414;
  --accent:#00e5ff; --accent-dark:#00b8cc; --accent-soft:#00323a;
  --pm1:#00ff66; --pm25:#ffcc00; --pm10:#ff3b30;
  --sh:none;
}
body.theme-contrast .card,body.theme-contrast .val{border-width:2px}
*{box-sizing:border-box}
body{margin:0;padding:18px;background:var(--bg);color:var(--text);
  font:14px/1.55 system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;-webkit-font-smoothing:antialiased}
h1{display:flex;align-items:center;flex-wrap:wrap;gap:8px 14px;font-size:20px;font-weight:650;margin:0}
.card{background:var(--card);border:1px solid var(--line);border-radius:var(--r);box-shadow:var(--sh);padding:16px;margin-bottom:14px}
.row{display:flex;flex-wrap:wrap;gap:8px 24px;align-items:baseline}
.kv{font-size:13px;color:var(--muted)}
.kv b{color:var(--text);font-weight:600}
.tag{display:inline-block;padding:2px 10px;border-radius:999px;background:var(--accent-soft);color:var(--accent-dark);font-size:12px;font-weight:600}
.ctrl{display:flex;flex-wrap:wrap;gap:10px 14px;align-items:center;margin:4px 0 10px}
select,input[type=datetime-local]{font:inherit;font-size:13px;padding:7px 10px;border:1px solid var(--line);border-radius:10px;background:var(--card);color:var(--text)}
select:focus,input:focus{outline:2px solid var(--accent-soft);border-color:var(--accent)}
button{font:inherit;font-size:13px;font-weight:600;padding:8px 14px;border:1px solid var(--line);border-radius:10px;background:var(--card);color:var(--text);cursor:pointer;transition:.15s}
button:hover{background:var(--hover);border-color:var(--accent)}
button:active{transform:translateY(1px)}
#apply{background:var(--accent);border-color:var(--accent);color:#fff}
#apply:hover{background:var(--accent-dark);border-color:var(--accent-dark)}
a{color:var(--accent);text-decoration:none;font-weight:500}
a:hover{text-decoration:underline}
.del{color:var(--err)}
.help{font-size:13px;font-weight:500;color:var(--muted)}
.help:hover{color:var(--accent)}
.muted{color:var(--muted);font-size:12px}
.stale{opacity:.45}
canvas{width:100%;height:320px;display:block;border:1px solid var(--line);border-radius:12px;background:var(--card)}
.vals{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:12px;margin-top:8px}
.val{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:10px 12px;border-top:3px solid var(--line)}
.val .muted{font-size:11.5px;text-transform:uppercase;letter-spacing:.05em}
.big{font-size:30px;font-weight:700;line-height:1.15;font-variant-numeric:tabular-nums}
#cvals .val:nth-child(1){border-top-color:var(--pm1)}
#cvals .val:nth-child(2){border-top-color:var(--pm25)}
#cvals .val:nth-child(3){border-top-color:var(--pm10)}
table{border-collapse:separate;border-spacing:0;width:100%;font-size:13.5px}
th{text-align:left;font-size:11.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);font-weight:600;padding:8px 10px;border-bottom:1px solid var(--line)}
td{padding:9px 10px;border-bottom:1px solid var(--line-soft);vertical-align:middle}
tbody tr:hover td{background:var(--hover)}
tbody tr:last-child td{border-bottom:none}
.modal{display:none;position:fixed;inset:0;background:rgba(15,23,42,.5);z-index:10;padding:18px;overflow:auto;-webkit-backdrop-filter:blur(2px);backdrop-filter:blur(2px)}
.modal.open{display:block}
.modalbox{background:var(--card);border:1px solid var(--line);border-radius:16px;max-width:820px;margin:0 auto;overflow:hidden;box-shadow:0 24px 60px rgba(0,0,0,.35)}
.modalhead{display:flex;justify-content:space-between;align-items:center;gap:12px;padding:14px 18px;border-bottom:1px solid var(--line);background:var(--card)}
.modalbody{padding:8px 18px 20px;font-size:13.5px;line-height:1.55}
.modalbody h3{font-size:14px;margin:18px 0 6px}
.modalbody table{margin-bottom:10px;font-size:13px}
.modalbody td,.modalbody th{padding:5px 8px}
.modalbody ul{margin:6px 0 10px 18px;padding:0}
.modalbody li{margin-bottom:4px}
@media(max-width:640px){body{padding:10px}.card{padding:13px;border-radius:12px}h1{font-size:17px}.big{font-size:24px}canvas{height:250px}.ctrl{gap:8px}}
</style>
</head>
<body>

<div class="card">
  <h1>ESP32-POE2 &middot; PMS5003
    <a href="#" id="help" class="help">[ help / reference values ]</a>
    <a href="/admin" class="help">[ administration ]</a>
  </h1>
  <div class="row">
    <div class="kv">Board time: <b id="clock">-</b> <span class="tag" id="tsrc">-</span></div>
    <div class="kv">Started: <b id="boot">-</b></div>
    <div class="kv">File: <b id="file">-</b></div>
    <div class="kv">Records: <b id="recs">0</b></div>
    <div class="kv">OTA: <b id="ota">-</b></div>
  </div>
  <div class="row" style="margin-top:6px">
    <div class="kv">Wired network: <b id="ethinfo">-</b></div>
    <div class="kv">Wireless (AP): <b id="apinfo">-</b></div>
  </div>
</div>

<div class="card">
  <div class="kv" style="margin-bottom:8px">Graph of PM1.0 / PM2.5 / PM10 changes (refreshes on every new record)</div>
  <div class="ctrl">
    <label class="kv">Range:
      <select id="range">
        <option value="0">all</option>
        <option value="300" selected>last 5 min</option>
        <option value="1800">last 30 min</option>
        <option value="3600">last 1 h</option>
        <option value="21600">last 6 h</option>
        <option value="86400">last 24 h</option>
      </select>
    </label>
    <label class="kv">From: <input type="datetime-local" id="from" step="1"></label>
    <label class="kv">To: <input type="datetime-local" id="till" step="1"></label>
    <button id="apply">Apply range</button>
    <button id="clear">All</button>
    <button id="png">Download PNG</button>
  </div>
  <div class="muted" id="rangeinfo" style="margin-bottom:6px">-</div>
  <canvas id="chart"></canvas>
  <div class="muted" id="chartinfo">-</div>
</div>

<div class="card">
  <div class="muted">Current values (updated every 5 s)</div>
  <div class="kv" id="cage" style="margin:6px 0">sensor: -</div>
  <div class="vals" id="cvals">
    <div class="val"><div class="muted">PM1.0</div><div class="big" id="c1">-</div></div>
    <div class="val"><div class="muted">PM2.5</div><div class="big" id="c25">-</div></div>
    <div class="val"><div class="muted">PM10</div><div class="big" id="c10">-</div></div>
  </div>
  <div class="muted" id="cupd">-</div>
</div>

<div class="card">
  <div class="kv" style="margin-bottom:6px">Files on the card</div>
  <table>
    <thead><tr><th>File</th><th>Size</th><th>Actions</th></tr></thead>
    <tbody id="fbody"></tbody>
  </table>
  <div class="ctrl" style="margin-top:10px">
    <button id="prevpage">Previous</button>
    <span class="muted" id="filepage">-</span>
    <button id="nextpage">Next</button>
  </div>
</div>

<div id="modal" class="modal">
  <div class="modalbox">
    <div class="modalhead">
      <b>Reference PM2.5 / PM10 values</b>
      <a href="#" id="close">&times; close</a>
    </div>
    <div class="modalbody">

      <h3>Baseline (reference)</h3>
      <table>
        <tr><th>Situation</th><th>PM2.5 (&micro;g/m&sup3;)</th></tr>
        <tr><td>Clean indoor air</td><td>5 - 15</td></tr>
        <tr><td>Urban outdoor background</td><td>10 - 30</td></tr>
        <tr><td>Polluted city / smog</td><td>50 - 150</td></tr>
        <tr><td>Cooking without an extractor hood</td><td>50 - 300</td></tr>
      </table>

      <h3>Cigarette smoke</h3>
      <table>
        <tr><th>Position</th><th>PM2.5 (&micro;g/m&sup3;)</th></tr>
        <tr><td>Right next to the smoke (within 1 m)</td><td>300 - 1500+</td></tr>
        <tr><td>Room average, one cigarette, closed room</td><td>20 - 100</td></tr>
        <tr><td>Room average, several cigarettes, smoky</td><td>150 - 500</td></tr>
        <tr><td>Small closed space, continuous smoking</td><td>300 - 1000</td></tr>
      </table>
      <p style="margin:4px 0 0">Rises within seconds, falls over 5 - 30 min depending on ventilation.</p>

      <h3>Fire smoke</h3>
      <table>
        <tr><th>Situation</th><th>PM2.5 (&micro;g/m&sup3;)</th></tr>
        <tr><td>Match, blown-out candle, incense</td><td>100 - 1000 (short peak)</td></tr>
        <tr><td>Fireplace / wood stove in the room</td><td>50 - 500</td></tr>
        <tr><td>Wildfire smoke outdoors, visible smoke</td><td>200 - 1000+</td></tr>
        <tr><td>Dense smoke, close range</td><td>1000 - 10000+</td></tr>
        <tr><td>Developed fire in a room</td><td>tens of thousands (sensor saturated)</td></tr>
      </table>

      <h3>PMS5003 sensor limits</h3>
      <ul>
        <li>Effective range: <b>0 - 500 &micro;g/m&sup3;</b>, maximum around
            <b>1000 &micro;g/m&sup3;</b>.</li>
        <li>Above that readings are unreliable and saturate - typically the value
            &quot;sticks&quot; to a constant (e.g. ~1000) while the smoke lasts.</li>
        <li>Cigarette smoke and moderate fire smoke are measured well; dense
            smoke indoors is seen only as a maximum.</li>
      </ul>

      <h3>Telling smoke types apart from the log</h3>
      <ul>
        <li><b>PM2.5 / PM10 ratio:</b> ~0.9 - 1.0 = fine aerosol (cigarette, fresh
            smoke, exhaust); ~0.5 - 0.8 = coarse fraction present (dust, ash,
            smoke close to the source).</li>
        <li><b>Rise time:</b> cigarette and match jump within seconds, dust rises
            gradually.</li>
        <li><b>Decay time:</b> cigarette smoke in a ventilated room halves in
            1 - 5 min; dust settles more slowly.</li>
      </ul>

      <h3>Humidity note</h3>
      <p style="margin:4px 0">The PMS5003 has no heated inlet, so above roughly 80 %
        relative humidity (fog, rain, early morning outdoors) it counts droplets as
        particles and shows falsely high values. For outdoor installations this is
        the most common source of &quot;phantom smoke&quot; in the data.</p>

      <h3>Practical chain test</h3>
      <p style="margin:4px 0">Light and blow out a match ~30 cm from the sensor - the
        log should show a peak within a few seconds and a gradual fall over
        1 - 3 minutes.</p>

    </div>
  </div>
</div>

<script>
var selFile = "";
var lastCount = -1;
var rows = [];
var srcLabel = "NTP";
var rangeSec = 300;                 // default range: last 5 minutes
var fromEpoch = 0;
var tillEpoch = 0;
var mode = "file";                  // "file" = single file, "range" = history range
var boardEpoch = 0;                 // current board time
var currentFile = "";               // active file on the board
var rangeFrom = 0;
var rangeTill = 0;
var lastRangeLoad = 0;
var rangeInitialised = false;
var currentTheme = "light";
var allFiles = [];
var filePage = 1;
var filesPerPage = 15;

function q(id){ return document.getElementById(id); }
function getJSON(u){ return fetch(u, {cache:"no-store"}).then(function(r){ return r.json(); }); }

function fmtShort(ts){
  if (hasEpochTime() && ts > 1000000000) {
    var d = new Date(ts*1000);
    return ("0"+d.getHours()).slice(-2)+":"+("0"+d.getMinutes()).slice(-2)+":"+("0"+d.getSeconds()).slice(-2);
  }
  return (ts/1000).toFixed(0)+"s";
}

function hasEpochTime(){ return (srcLabel === "NTP" || srcLabel === "MANUAL"); }

function fmtFull(ts){
  if (hasEpochTime() && ts > 1000000000) {
    var d = new Date(ts*1000);
    return ("0"+d.getDate()).slice(-2)+"."+("0"+(d.getMonth()+1)).slice(-2)+"."+d.getFullYear()+
           " "+("0"+d.getHours()).slice(-2)+":"+("0"+d.getMinutes()).slice(-2)+":"+("0"+d.getSeconds()).slice(-2);
  }
  return (ts/1000).toFixed(0)+" s";
}

function fmtAxis(ts, span){
  if (hasEpochTime() && ts > 1000000000) {
    var d = new Date(ts*1000);
    if (span > 6*3600) {
      return ("0"+d.getDate()).slice(-2)+"."+("0"+(d.getMonth()+1)).slice(-2)+" "+("0"+d.getHours()).slice(-2)+":"+("0"+d.getMinutes()).slice(-2);
    }
    return ("0"+d.getHours()).slice(-2)+":"+("0"+d.getMinutes()).slice(-2)+":"+("0"+d.getSeconds()).slice(-2);
  }
  return (ts/1000).toFixed(0)+"s";
}

function rangeLabel(){
  if (rows.length > 1 && hasEpochTime()) {
    return fmtFull(rows[0][0]) + "  -  " + fmtFull(rows[rows.length-1][0]);
  }
  if (mode === "range" && rangeFrom) {
    return fmtFull(rangeFrom) + "  -  " + (rangeTill ? fmtFull(rangeTill) : "now");
  }
  if (fromEpoch || tillEpoch) {
    return fmtFull(fromEpoch) + "  -  " + (tillEpoch ? fmtFull(tillEpoch) : "end");
  }
  var el = q("range");
  return el.options[el.selectedIndex].text;
}

/* Filter: the custom from-to range first (epoch only), then the quick range */
function applyRange(){
  var d = rows.slice();
  if (hasEpochTime() && (fromEpoch || tillEpoch)) {
    return d.filter(function(r){
      return (!fromEpoch || r[0] >= fromEpoch) && (!tillEpoch || r[0] <= tillEpoch);
    });
  }
  if (!rangeSec) return d;
  var unit = hasEpochTime() ? 1 : 1000;
  var last = rows[rows.length-1][0];
  var limit = last - rangeSec * unit;
  return d.filter(function(r){ return r[0] >= limit; });
}

function pollStatus(){
  getJSON("/api/status").then(function(s){
    q("clock").textContent = s.time;
    q("tsrc").textContent  = s.source;
    q("recs").textContent  = s.records;
    q("ota").textContent   = s.ota ? s.ota : "-";
    q("boot").textContent  = s.boot;
    if (s.theme && s.theme !== currentTheme) {
      currentTheme = s.theme;
      document.body.className = "theme-" + currentTheme;
      draw();
    }
    q("ethinfo").textContent = s.ethActive
        ? (s.ethIp + " / " + s.ethMask + "  gw " + s.ethGw + (s.ethDhcp ? "  (DHCP)" : "  (static)"))
        : "not active";
    q("apinfo").textContent = s.apEnabled
        ? (s.apSsid + "  " + s.apIp + "  channel " + s.apChannel + "  clients: " + s.apClients)
        : "disabled";
    renderSensorState(s);
    if (s.epoch > 1000000000) { boardEpoch = s.epoch; }
    currentFile = s.file;
    if (selFile === "") { selFile = s.file; q("file").textContent = selFile; }

    if (!rangeInitialised && rangeSec > 0 && boardEpoch > 0) {
      rangeInitialised = true;
      mode = "range"; rangeFrom = boardEpoch - rangeSec; rangeTill = 0; rangeSec = 0;
      loadRange();
    }

    if (s.records !== lastCount) {
      lastCount = s.records;
      if (mode === "range") {
        if (rangeTill === 0 && boardEpoch > 0 && (Date.now()/1000 - lastRangeLoad) > 5) { loadRange(); }
      } else if (selFile === s.file) {
        loadData();
      }
    }
  }).catch(function(){});
}

function renderSensorState(s){
  var el = q("cage");
  var vals = q("cvals");
  var age = (typeof s.ageSec === "number") ? s.ageSec : -1;
  var frames = "  |  frames: " + s.framesOk + " (rejected " + s.framesBad + ")";

  if (!s.current || s.current.ts === 0) {
    el.textContent = "sensor: no data yet" + frames;
    el.style.color = "#b91c1c";
    vals.className = "vals stale";
  } else if (age >= 0 && age < 5) {
    el.textContent = "sensor: OK (last sample " + age + " s ago)" + frames;
    el.style.color = "#15803d";
    vals.className = "vals";
  } else if (age >= 0 && age < 15) {
    el.textContent = "sensor: delayed (last sample " + age + " s ago)" + frames;
    el.style.color = "#b45309";
    vals.className = "vals stale";
  } else {
    el.textContent = "sensor: NO DATA for " + age + " s" + frames;
    el.style.color = "#b91c1c";
    vals.className = "vals stale";
  }
}

function pollCurrent(){
  getJSON("/api/status").then(function(s){
    if (s.current) {
      q("c1").textContent  = s.current.pm1;
      q("c25").textContent = s.current.pm25;
      q("c10").textContent = s.current.pm100;
    }
    q("cupd").textContent = "read at: " + s.time + " (every 5 s)";
  }).catch(function(){});
}

function loadRange(){
  var qs = "/api/data?from=" + rangeFrom + "&to=" + (rangeTill || 0);
  getJSON(qs).then(function(d){
    rows = d.rows || [];
    srcLabel = d.source || "NTP";
    lastRangeLoad = Date.now()/1000;
    var nf = d.files ? d.files.split(",").length : 0;
    var info = "history: " + nf + (nf === 1 ? " file" : " files");
    if (d.files) { info += " (" + d.files + ")"; }
    info += "  |  records: " + rows.length;
    if (d.decim > 1) { info += " of " + d.total + " (decimated x" + d.decim + ")"; }
    info += "  |  time source: " + srcLabel;
    q("chartinfo").textContent = info;
    q("file").textContent = "history";
    draw();
  }).catch(function(){});
}

function loadData(){
  getJSON("/api/data?f=" + encodeURIComponent(selFile)).then(function(d){
    rows = d.rows || [];
    srcLabel = d.source || "NTP";
    q("chartinfo").textContent = d.file + "  |  total records: " + rows.length +
                                 "  |  time source: " + srcLabel;
    draw();
  }).catch(function(){});
}

function loadFiles(){
  getJSON("/api/files").then(function(d){
    allFiles = (d.files || []).slice().sort(function(a, b){
      if (a.name < b.name) return 1;
      if (a.name > b.name) return -1;
      return 0;
    });
    renderFiles();
  }).catch(function(){});
}

/* Shows up to filesPerPage files (newest first); the rest are on later pages */
function renderFiles(){
  var tb = q("fbody");
  tb.innerHTML = "";
  var pages = Math.max(1, Math.ceil(allFiles.length / filesPerPage));
  if (filePage > pages) filePage = pages;
  if (filePage < 1) filePage = 1;
  var start = (filePage - 1) * filesPerPage;
  allFiles.slice(start, start + filesPerPage).forEach(function(f){
    var tr = document.createElement("tr");

    var td1 = document.createElement("td");
    td1.textContent = f.name;
    tr.appendChild(td1);

    var td2 = document.createElement("td");
    td2.textContent = f.size + " B";
    tr.appendChild(td2);

    var td3 = document.createElement("td");

    var a1 = document.createElement("a");
    a1.textContent = "graph";
    a1.href = "#";
    a1.onclick = function(e){
      e.preventDefault();
      selFile = f.name;
      mode = "file";
      fromEpoch = 0; tillEpoch = 0; rangeSec = 0; rangeFrom = 0; rangeTill = 0;
      q("from").value = ""; q("till").value = ""; q("range").value = "0";
      lastCount = -1;
      q("file").textContent = selFile;
      loadData();
    };
    td3.appendChild(a1);
    td3.appendChild(document.createTextNode(" | "));

    var a2 = document.createElement("a");
    a2.textContent = "download";
    a2.href = "/download?f=" + encodeURIComponent(f.name);
    td3.appendChild(a2);

    if (f.active) {
      td3.appendChild(document.createTextNode(" | (active)"));
    } else {
      td3.appendChild(document.createTextNode(" | "));
      var a3 = document.createElement("a");
      a3.textContent = "delete";
      a3.className = "del";
      a3.href = "/delete?f=" + encodeURIComponent(f.name);
      a3.onclick = function(){ return confirm("Delete " + f.name + " ?"); };
      td3.appendChild(a3);
    }

    tr.appendChild(td3);
    tb.appendChild(tr);
  });
  q("filepage").textContent = "Page " + filePage + " / " + pages + "   (" + allFiles.length + " files)";
  q("prevpage").disabled = (filePage <= 1);
  q("nextpage").disabled = (filePage >= pages);
}

/* Colour palette per theme (also used for the exported PNG) */
function palette(){
  if (currentTheme === "dark") {
    return {bg:"#171d26", grid:"#2a3342", axis:"#5b6a80", label:"#94a3b8", text:"#e7ebf2",
            pm1:"#22c55e", pm25:"#f97316", pm10:"#ef4444"};
  }
  if (currentTheme === "contrast") {
    return {bg:"#000000", grid:"#3a3a3a", axis:"#ffffff", label:"#ffd400", text:"#ffffff",
            pm1:"#00ff66", pm25:"#ffcc00", pm10:"#ff3b30"};
  }
  return {bg:"#ffffff", grid:"#e6e8eb", axis:"#aaa", label:"#666", text:"#333",
          pm1:"#16a34a", pm25:"#ea580c", pm10:"#dc2626"};
}

/* Draw the chart into the given context */
function drawChart(g, L, T, pw, ph, data, fs, lw){
  g.font = fs + "px Arial";
  var pal = palette();

  if (!data.length) {
    g.fillStyle = pal.label;
    g.fillText("No records in the selected range.", L, T + fs + 4);
    return;
  }

  var xmin = data[0][0], xmax = data[data.length-1][0];
  if (xmax <= xmin) xmax = xmin + 1;
  var ymax = 10, i, v, t, x, y;
  data.forEach(function(r){ ymax = Math.max(ymax, r[1], r[2], r[3]); });
  ymax = Math.ceil(ymax * 1.15);

  function X(tt){ return L + (tt - xmin) * pw / (xmax - xmin); }
  function Y(vv){ return T + ph - vv * ph / ymax; }

  g.strokeStyle = pal.grid;
  g.fillStyle = pal.label;
  g.lineWidth = 1;
  for (i = 0; i <= 4; i++) {
    v = ymax * i / 4; y = Y(v);
    g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
    g.textAlign = "right"; g.fillText(v.toFixed(0), L - 6, y + fs*0.35);
  }
  for (i = 0; i <= 4; i++) {
    t = xmin + (xmax - xmin) * i / 4; x = X(t);
    g.beginPath(); g.moveTo(x, T); g.lineTo(x, T + ph); g.stroke();
    g.textAlign = "center"; g.fillText(fmtAxis(Math.round(t), xmax - xmin), x, T + ph + fs + 4);
  }

  g.strokeStyle = pal.axis;
  g.beginPath(); g.moveTo(L, T); g.lineTo(L, T + ph); g.lineTo(L + pw, T + ph); g.stroke();

  /* Y axis title */
  g.save();
  g.translate(fs * 0.95, T + ph / 2);
  g.rotate(-Math.PI / 2);
  g.textAlign = "center";
  g.fillStyle = pal.label;
  g.fillText("\u00b5g/m\u00b3", 0, 0);
  g.restore();

  var series = [
    {idx:1, col:pal.pm1,  name:"PM1.0"},
    {idx:2, col:pal.pm25, name:"PM2.5"},
    {idx:3, col:pal.pm10, name:"PM10"}
  ];

  series.forEach(function(s){
    g.strokeStyle = s.col;
    g.lineWidth = lw;
    g.beginPath();
    data.forEach(function(r, k){
      var px = X(r[0]), py = Y(r[s.idx]);
      if (k === 0) g.moveTo(px, py); else g.lineTo(px, py);
    });
    g.stroke();
    g.fillStyle = s.col;
    data.forEach(function(r){
      g.beginPath();
      g.arc(X(r[0]), Y(r[s.idx]), lw + 0.6, 0, 6.2832);
      g.fill();
    });
  });

  var lx = L;
  series.forEach(function(s){
    g.fillStyle = s.col; g.fillRect(lx, T - fs*1.4, fs*0.9, fs*0.9);
    g.fillStyle = pal.text; g.textAlign = "left";
    g.fillText(s.name, lx + fs*1.2, T - fs*0.5);
    lx += fs * 6.4;
  });
}

function draw(){
  var cv = q("chart");
  var w = cv.clientWidth || 640;
  var h = cv.clientHeight || 320;
  var dpr = window.devicePixelRatio || 1;
  cv.width = Math.round(w*dpr);
  cv.height = Math.round(h*dpr);

  var g = cv.getContext("2d");
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);

  var d = applyRange();
  var info = "showing " + d.length + " of " + rows.length + " records";
  if (d.length > 1) { info += "  |  range: " + fmtFull(d[0][0]) + "  -  " + fmtFull(d[d.length-1][0]); }
  if (mode !== "range") { info += "  |  selection: " + rangeLabel(); }
  q("rangeinfo").textContent = info;

  drawChart(g, 52, 26, w-52-10, h-26-28, d, 12, 2);
}

function exportPNG(){
  var d = applyRange();
  var w = 1200, h = 620;
  var cv = document.createElement("canvas");
  cv.width = w; cv.height = h;
  var g = cv.getContext("2d");

  var pal = palette();
  g.fillStyle = pal.bg; g.fillRect(0, 0, w, h);
  g.fillStyle = pal.text; g.font = "bold 20px Arial"; g.textAlign = "left";
  g.fillText("PMS5003 - " + (mode === "range" ? "history" : selFile), 24, 34);
  g.font = "14px Arial"; g.fillStyle = pal.label;
  g.fillText("Range: " + rangeLabel() + "  |  records: " + d.length +
             "  |  time source: " + srcLabel +
             "  |  exported: " + new Date().toLocaleString(), 24, 58);

  drawChart(g, 70, 100, w - 70 - 40, h - 100 - 60, d, 14, 2.4);

  var a = document.createElement("a");
  var ts = new Date().toISOString().replace(/[:T]/g, "-").slice(0, 19);
  a.download = "pms_chart_" + ts + ".png";
  a.href = cv.toDataURL("image/png");
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

function applyCustomRange(){
  var from = q("from").value;
  var till = q("till").value;
  var fromEp = from ? Math.floor(new Date(from).getTime()/1000) : 0;
  var tillEp = till ? Math.floor(new Date(till).getTime()/1000) : 0;

  if (!fromEp && !tillEp) { clearRanges(); return; }

  /* history query on the board across all files in the range */
  mode = "range";
  rangeFrom = fromEp;
  rangeTill = tillEp;
  fromEpoch = 0;
  tillEpoch = 0;
  rangeSec = 0;
  q("range").value = "0";
  loadRange();
}

function clearRanges(){
  mode = "file";
  fromEpoch = 0;
  tillEpoch = 0;
  rangeSec = 0;
  rangeFrom = 0;
  rangeTill = 0;
  q("from").value = "";
  q("till").value = "";
  q("range").value = "0";
  try { localStorage.setItem("pms_range", "0"); } catch(e) {}
  if (currentFile) { selFile = currentFile; }
  lastCount = -1;
  loadData();
}

window.addEventListener("load", function(){
  /* Remember the selected range in the browser (localStorage) */
  try {
    var sp = localStorage.getItem("pms_range");
    if (sp !== null) {
      q("range").value = sp;
      rangeSec = parseInt(sp, 10) || 0;
    } else {
      q("range").value = "300";
      rangeSec = 300;
    }
  } catch (e) {}

  q("range").onchange = function(){
    rangeSec = parseInt(this.value, 10) || 0;
    fromEpoch = 0; tillEpoch = 0;
    q("from").value = ""; q("till").value = "";
    try { localStorage.setItem("pms_range", this.value); } catch (e) {}

    if (rangeSec > 0 && boardEpoch > 0) {
      /* pull data from every file covering the selected range */
      mode = "range";
      rangeFrom = boardEpoch - rangeSec;
      rangeTill = 0;
      rangeSec = 0;
      loadRange();
    } else {
      mode = "file";
      if (currentFile) { selFile = currentFile; }
      lastCount = -1;
      loadData();
    }
  };
  q("apply").onclick = applyCustomRange;
  q("clear").onclick = clearRanges;
  q("png").onclick = exportPNG;
  q("prevpage").onclick = function(){ filePage--; renderFiles(); };
  q("nextpage").onclick = function(){ filePage++; renderFiles(); };

  /* Modal with reference values */
  q("help").onclick = function(e){ e.preventDefault(); q("modal").className = "modal open"; };
  q("close").onclick = function(e){ e.preventDefault(); q("modal").className = "modal"; };
  q("modal").onclick = function(e){ if (e.target === this) q("modal").className = "modal"; };
  document.addEventListener("keydown", function(e){
    if (e.key === "Escape") q("modal").className = "modal";
  });

  pollStatus();
  pollCurrent();
  loadFiles();
  setInterval(pollStatus, 2000);    // graph on change + networks + time
  setInterval(pollCurrent, 5000);   // current values
  setInterval(loadFiles, 15000);    // file list
  window.addEventListener("resize", draw);
});
</script>
</body>
</html>
)HTML";

// ============================== ADMIN PAGE ==============================

static const char ADMIN_STYLE[] PROGMEM = R"CSS(
:root{--bg:#f5f7fa;--card:#fff;--text:#111827;--muted:#6b7280;--line:#e5e7eb;--accent:#2563eb;--accent-dark:#1d4ed8;--r:14px}
*{box-sizing:border-box}
body{margin:0;padding:18px;background:var(--bg);color:var(--text);font:14px/1.55 system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif}
.card{background:var(--card);border:1px solid var(--line);border-radius:var(--r);max-width:880px;margin:0 auto 14px;box-shadow:0 1px 2px rgba(16,24,40,.05),0 10px 28px rgba(16,24,40,.06);padding:18px}
h1{font-size:19px;font-weight:650;margin:0 0 6px;display:flex;flex-wrap:wrap;gap:8px 14px;align-items:center}
h3{font-size:14.5px;font-weight:650;margin:20px 0 8px;padding-bottom:6px;border-bottom:1px solid var(--line)}
table{border-collapse:separate;border-spacing:0;font-size:13.5px;width:100%}
td{padding:6px 8px 6px 0;vertical-align:middle}
td:first-child{color:var(--muted);width:130px}
input,select{font:inherit;font-size:13px;padding:7px 10px;border:1px solid var(--line);border-radius:10px;background:#fff;color:var(--text);width:100%;max-width:360px}
input[type=radio],input[type=checkbox]{width:auto;margin-right:6px;vertical-align:middle}
input:focus{outline:2px solid #eef4ff;border-color:var(--accent)}
button{font:inherit;font-size:13px;font-weight:600;padding:9px 16px;border:1px solid var(--accent);border-radius:10px;background:var(--accent);color:#fff;cursor:pointer;transition:.15s}
button:hover{background:var(--accent-dark);border-color:var(--accent-dark)}
a{color:var(--accent);text-decoration:none;font-weight:500}
a:hover{text-decoration:underline}
hr{border:none;border-top:1px solid var(--line);margin:22px 0}
.muted{color:var(--muted);font-size:12px}
p{margin:8px 0}
@media(max-width:640px){body{padding:10px}.card{padding:13px}td:first-child{width:auto}}
)CSS";

static void handleAdminGet() {
  String h = F("<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">");
  h += F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  h += F("<title>Administration - ESP32-POE2</title><style>");
  h += FPSTR(ADMIN_STYLE);
  h += F("</style></head><body class=\"theme-");
  h += String(settings.theme);
  h += F("\"><div class=\"card\">");
  h += F("<h1>Device administration <a href=\"/\">&larr; back to graph</a></h1>");

  // --- form 1: network settings ---
  h += F("<form method=\"POST\" action=\"/admin\">");
  h += F("<input type=\"hidden\" name=\"action\" value=\"network\">");
  h += F("<h3>Wired network (Ethernet)</h3>");
  h += F("<p><label><input type=\"radio\" name=\"ethdhcp\" value=\"1\"");
  if (settings.ethDhcp) h += F(" checked");
  h += F("> DHCP &nbsp; </label><label><input type=\"radio\" name=\"ethdhcp\" value=\"0\"");
  if (!settings.ethDhcp) h += F(" checked");
  h += F("> Static IP</label></p><table>");
  h += F("<tr><td>IP address</td><td><input name=\"ethip\" value=\"");
  h += String(settings.ethIp);
  h += F("\"></td></tr><tr><td>Netmask</td><td><input name=\"ethmaska\" value=\"");
  h += String(settings.ethMask);
  h += F("\"></td></tr><tr><td>Gateway</td><td><input name=\"ethgw\" value=\"");
  h += String(settings.ethGw);
  h += F("\"></td></tr><tr><td>DNS</td><td><input name=\"ethdns\" value=\"");
  h += String(settings.ethDns);
  h += F("\"></td></tr></table>");

  h += F("<h3>Wireless network (Wi-Fi AP)</h3>");
  h += F("<p><label><input type=\"checkbox\" name=\"ap\" value=\"1\"");
  if (settings.apEnabled) h += F(" checked");
  h += F("> Enabled</label></p><table>");
  h += F("<tr><td>SSID</td><td><input name=\"apssid\" value=\"");
  h += String(settings.apSsid);
  h += F("\"></td></tr><tr><td>Password</td><td><input name=\"aploz\" value=\"");
  h += String(settings.apPassword);
  h += F("\"> <span class=\"muted\">empty or &lt;8 chars = open network</span></td></tr>");
  h += F("<tr><td>Channel</td><td><input name=\"apkanal\" type=\"number\" min=\"1\" max=\"13\" value=\"");
  h += String(settings.apChannel);
  h += F("\"></td></tr></table>");

  h += F("<h3>NTP</h3><table><tr><td>Server</td><td><input name=\"ntp\" value=\"");
  h += String(settings.ntpServer);
  h += F("\"></td></tr></table>");

  h += F("<p class=\"muted\">Saving network settings restarts the device.</p>");
  h += F("<p><button type=\"submit\">Save network and restart</button></p>");
  h += F("</form>");

  // --- display / theme ---
  h += F("<hr><h3>Display</h3>");
  h += F("<form method=\"POST\" action=\"/admin\">");
  h += F("<input type=\"hidden\" name=\"action\" value=\"display\">");
  h += F("<p>Theme: <select name=\"theme\">");
  h += F("<option value=\"light\"");
  if (strcmp(settings.theme, "light") == 0) h += F(" selected");
  h += F(">Light</option><option value=\"dark\"");
  if (strcmp(settings.theme, "dark") == 0) h += F(" selected");
  h += F(">Dark</option><option value=\"contrast\"");
  if (strcmp(settings.theme, "contrast") == 0) h += F(" selected");
  h += F(">High contrast</option></select> ");
  h += F("<button type=\"submit\">Save theme</button></p></form>");

  // --- form 2: manual time ---
  // --- firmware update ---
  h += F("<h3>Firmware update</h3>");
  h += F("<p class=\"muted\">Upload a compiled .bin file. Do not power off the device during the update.</p>");
  h += F("<form method=\"POST\" action=\"/update\" enctype=\"multipart/form-data\">");
  h += F("<p><input type=\"file\" name=\"firmware\" accept=\".bin\"> <button type=\"submit\">Upload firmware</button></p>");
  h += F("</form>");
  h += F("<p class=\"muted\">ArduinoOTA hostname: ");
  h += String(OTA_HOSTNAME);
  h += F(", port 3232.</p>");

  h += F("<hr><h3>Manual time setting</h3>");
  h += F("<p class=\"muted\">Used when NTP is not available. The time is not kept across a restart.</p>");
  h += F("<form method=\"POST\" action=\"/admin\">");
  h += F("<input type=\"hidden\" name=\"action\" value=\"time\">");
  h += F("<p><input type=\"datetime-local\" name=\"manualtime\" step=\"1\"> ");
  h += F("<button type=\"submit\">Set time</button></p></form>");

  h += F("<p class=\"muted\">Current state: ");
  h += String(netMode);
  h += F(" | time source: ");
  h += String(timeSourceName());
  h += F(" | SD: ");
  h += (sdReady ? F("OK") : F("not available"));
  h += F(" | file: ");
  h += String(csvPath);
  h += F("</p>");

  h += F("</div></body></html>");

  server.send(200, "text/html; charset=utf-8", h);
}

static void handleAdminPost() {
  String action = server.arg("action");
  String message;

  // Manual time
  if (server.hasArg("manualtime") && server.arg("manualtime").length() >= 16) {
    int y = 0, mo = 0, d = 0, hh = 0, mi = 0, ss = 0;
    int parsed = sscanf(server.arg("manualtime").c_str(), "%d-%d-%dT%d:%d:%d",
                        &y, &mo, &d, &hh, &mi, &ss);
    if (parsed >= 5) {
      struct tm t = {};
      t.tm_year  = y - 1900;
      t.tm_mon   = mo - 1;
      t.tm_mday  = d;
      t.tm_hour  = hh;
      t.tm_min   = mi;
      t.tm_sec   = ss;
      t.tm_isdst = -1;
      time_t e = mktime(&t);
      if (e > (time_t)NTP_EPOCH_MIN) {
        setenv("TZ", TZ_INFO, 1);
        tzset();
        setManualTime(e);
        message = F("The time has been set.");
      } else {
        message = F("Invalid date/time.");
      }
    } else {
      message = F("Invalid time format.");
    }
  }

  // Display theme
  if (action == "display") {
    String th = server.arg("theme");
    if (th == "light" || th == "dark" || th == "contrast") {
      strlcpy(settings.theme, th.c_str(), sizeof(settings.theme));
      saveSettings();
      message = F("Theme saved.");
    } else {
      message = F("Invalid theme.");
    }
  }

  // Network settings
  if (action == "network") {
    bool changed = false;

    bool newDhcp = (server.arg("ethdhcp") == "1");
    if (newDhcp != settings.ethDhcp) { settings.ethDhcp = newDhcp; changed = true; }
    if (server.hasArg("ethip") && server.arg("ethip") != settings.ethIp) {
      strlcpy(settings.ethIp, server.arg("ethip").c_str(), sizeof(settings.ethIp)); changed = true;
    }
    if (server.hasArg("ethmaska") && server.arg("ethmaska") != settings.ethMask) {
      strlcpy(settings.ethMask, server.arg("ethmaska").c_str(), sizeof(settings.ethMask)); changed = true;
    }
    if (server.hasArg("ethgw") && server.arg("ethgw") != settings.ethGw) {
      strlcpy(settings.ethGw, server.arg("ethgw").c_str(), sizeof(settings.ethGw)); changed = true;
    }
    if (server.hasArg("ethdns") && server.arg("ethdns") != settings.ethDns) {
      strlcpy(settings.ethDns, server.arg("ethdns").c_str(), sizeof(settings.ethDns)); changed = true;
    }

    bool newAp = server.hasArg("ap");
    if (newAp != settings.apEnabled) { settings.apEnabled = newAp; changed = true; }
    if (server.hasArg("apssid") && server.arg("apssid").length() > 0 &&
        server.arg("apssid") != settings.apSsid) {
      strlcpy(settings.apSsid, server.arg("apssid").c_str(), sizeof(settings.apSsid)); changed = true;
    }
    if (server.hasArg("aploz") && server.arg("aploz") != settings.apPassword) {
      strlcpy(settings.apPassword, server.arg("aploz").c_str(), sizeof(settings.apPassword)); changed = true;
    }
    if (server.hasArg("apkanal")) {
      int channel = server.arg("apkanal").toInt();
      if (channel >= 1 && channel <= 13 && (uint8_t)channel != settings.apChannel) {
        settings.apChannel = (uint8_t)channel; changed = true;
      }
    }
    if (server.hasArg("ntp") && server.arg("ntp").length() > 0 &&
        server.arg("ntp") != settings.ntpServer) {
      strlcpy(settings.ntpServer, server.arg("ntp").c_str(), sizeof(settings.ntpServer)); changed = true;
    }

    saveSettings();
    if (changed) {
      message = F("Network settings saved. The device is restarting...");
      restartScheduled = millis();
      if (restartScheduled == 0) restartScheduled = 1;
    } else if (message.length() == 0) {
      message = F("No changes in the network settings.");
    }
  }

  String h = F("<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">");
  h += F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  h += F("<title>Administration</title><style>");
  h += FPSTR(ADMIN_STYLE);
  h += F("</style></head><body class=\"theme-");
  h += String(settings.theme);
  h += F("\"><div class=\"card\"><h1>");
  h += message;
  h += F("</h1><p><a href=\"/\">back to graph</a> &nbsp;|&nbsp; <a href=\"/admin\">administration</a></p>");
  h += F("</div></body></html>");
  server.send(200, "text/html; charset=utf-8", h);
}

// ============================== WEB HANDLERS ==============================

static void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");
  server.sendContent_P(PAGE_HTML);
  server.sendContent("");
}

static void handleStatus() {
  char now[32];
  char boot[24];
  formatNow(now, sizeof(now));
  formatEpochTs(bootEpoch, boot, sizeof(boot));

  uint32_t epoch = epochValid() ? (uint32_t)time(nullptr) : 0;

  String j = "{";
  j += "\"records\":";    j += String(recordCount);
  j += ",\"file\":\"";    j += String(csvPath);             j += "\"";
  j += ",\"net\":\"";     j += String(netMode);             j += "\"";
  j += ",\"source\":\"";  j += String(timeSourceName());    j += "\"";
  j += ",\"epoch\":";     j += String(epoch);
  j += ",\"time\":\"";    j += String(now);                 j += "\"";
  j += ",\"boot\":\"";    j += String(boot);                j += "\"";
  j += ",\"uptime\":";    j += String((uint32_t)(millis() / 1000UL));
  j += ",\"ageSec\":";    j += String((uint32_t)((millis() - lastPmsMs) / 1000UL));
  j += ",\"framesOk\":";  j += String(framesOk);
  j += ",\"framesBad\":"; j += String(framesBad);

  // Wired network
  j += ",\"ethActive\":"; j += (ethActive ? "true" : "false");
  j += ",\"ethDhcp\":";   j += (settings.ethDhcp ? "true" : "false");
  j += ",\"ethIp\":\"";   j += (ethActive ? ETH.localIP().toString()     : String("-")); j += "\"";
  j += ",\"ethMask\":\""; j += (ethActive ? ETH.subnetMask().toString()  : String("-")); j += "\"";
  j += ",\"ethGw\":\"";   j += (ethActive ? ETH.gatewayIP().toString()   : String("-")); j += "\"";
  j += ",\"ethMac\":\"";  j += ETH.macAddress(); j += "\"";

  // Wireless network (AP)
  j += ",\"apEnabled\":";  j += (settings.apEnabled ? "true" : "false");
  j += ",\"apSsid\":\"";   j += String(settings.apSsid); j += "\"";
  j += ",\"apIp\":\"";     j += (settings.apEnabled ? WiFi.softAPIP().toString() : String("-")); j += "\"";
  j += ",\"apChannel\":";  j += String(settings.apChannel);
  j += ",\"apClients\":";  j += String((uint32_t)(settings.apEnabled ? WiFi.softAPgetStationNum() : 0));

#if OTA_ENABLE
  j += ",\"ota\":\"";      j += String(OTA_HOSTNAME);  j += "\"";
#else
  j += ",\"ota\":\"disabled\"";
#endif
  j += ",\"theme\":\"";   j += String(settings.theme); j += "\"";

  j += ",\"current\":{";
  if (haveSample) {
    j += "\"ts\":";      j += String(lastTs);
    j += ",\"pm1\":";    j += String(lastPm1);
    j += ",\"pm25\":";   j += String(lastPm25);
    j += ",\"pm100\":";  j += String(lastPm100);
  } else {
    j += "\"ts\":0,\"pm1\":\"-\",\"pm25\":\"-\",\"pm100\":\"-\"";
  }
  j += "}}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

static void handleData() {
  String fromArg = server.arg("from");
  String tillArg = server.arg("to");

  // History query: data from every file covering the range
  if (fromArg.length() > 0 || tillArg.length() > 0) {
    uint32_t from = fromArg.length() ? (uint32_t)strtoul(fromArg.c_str(), nullptr, 10) : 0;
    uint32_t till = tillArg.length() ? (uint32_t)strtoul(tillArg.c_str(), nullptr, 10) : 0;
    if (from > 0 && till > 0 && till < from) {
      uint32_t t = from; from = till; till = t;
    }

    if (sdFs == nullptr || !loadHistoryRange(from, till)) {
      server.send(500, "application/json", "{\"error\":\"cannot read the card\"}");
      return;
    }

    server.sendHeader("Cache-Control", "no-store");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");

    char buf[240];
    snprintf(buf, sizeof(buf),
             "{\"mode\":\"range\",\"from\":%lu,\"to\":%lu,\"source\":\"%s\","
             "\"count\":%d,\"total\":%lu,\"decim\":%d,\"files\":\"%s\",\"rows\":[",
             (unsigned long)from, (unsigned long)till, timeSourceName(),
             histCount, (unsigned long)histTotal, histDecim, histFiles.c_str());
    server.sendContent(buf);

    char row[96];
    size_t pos = 0;
    for (int i = 0; i < histCount; i++) {
      snprintf(row, sizeof(row), "%s[%lu,%u,%u,%u]",
               (i == 0 ? "" : ","),
               (unsigned long)histBuf[i].ts, histBuf[i].pm1, histBuf[i].pm25, histBuf[i].pm100);
      size_t lr = strlen(row);
      if (pos + lr > sizeof(buf) - 1) {
        buf[pos] = 0;
        server.sendContent(buf);
        pos = 0;
      }
      memcpy(buf + pos, row, lr);
      pos += lr;
    }
    if (pos > 0) {
      buf[pos] = 0;
      server.sendContent(buf);
    }
    server.sendContent("]}");
    server.sendContent("");
    return;
  }

  String file = server.arg("f");
  if (file.length() == 0) {
    file = String(csvPath);
  }
  if (!file.startsWith("/")) {
    file = "/" + file;
  }

  if (sdFs == nullptr || file.length() < 3) {
    server.send(400, "application/json", "{\"error\":\"invalid file\"}");
    return;
  }
  if (!loadIntoRing(file)) {
    server.send(404, "application/json", "{\"error\":\"file could not be opened\"}");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  char buf[160];
  snprintf(buf, sizeof(buf), "{\"file\":\"%s\",\"source\":\"%s\",\"count\":%d,\"rows\":[",
           file.c_str(), timeSourceName(), ringCount);
  server.sendContent(buf);

  for (int i = 0; i < ringCount; i++) {
    LogRow r = ring[(ringHead + i) % RING_MAX];
    snprintf(buf, sizeof(buf), "%s[%lu,%u,%u,%u]",
             (i == 0 ? "" : ","),
             (unsigned long)r.ts, r.pm1, r.pm25, r.pm100);
    server.sendContent(buf);
  }
  server.sendContent("]}");
  server.sendContent("");
}

static void handleFiles() {
  if (sdFs == nullptr) {
    server.send(500, "application/json", "{\"files\":[]}");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("{\"files\":[");

  File root = sdFs->open("/");
  if (root && root.isDirectory()) {
    bool first = true;
    File f = root.openNextFile();
    while (f) {
      if (!f.isDirectory()) {
        String name = String(f.name());
        if (!name.startsWith("/")) {
          name = "/" + name;
        }
        char buf[160];
        snprintf(buf, sizeof(buf), "%s{\"name\":\"%s\",\"size\":%lu,\"active\":%s}",
                 (first ? "" : ","), name.c_str(), (unsigned long)f.size(),
                 (name == String(csvPath)) ? "true" : "false");
        server.sendContent(buf);
        first = false;
      }
      f = root.openNextFile();
    }
    root.close();
  }

  server.sendContent("]}");
  server.sendContent("");
}

static void handleDownload() {
  String file = server.arg("f");
  if (file.length() == 0 || !file.startsWith("/")) {
    file = "/" + file;
  }
  if (sdFs == nullptr || file.length() < 3) {
    server.send(400, "text/plain", "Invalid file.");
    return;
  }

  File f = sdFs->open(file, FILE_READ);
  if (!f || f.isDirectory()) {
    server.send(404, "text/plain", "File does not exist.");
    return;
  }

  String downloadName = file.substring(file.lastIndexOf('/') + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + downloadName + "\"");
  server.streamFile(f, "text/csv");
  f.close();
}

static void handleDelete() {
  String file = server.arg("f");
  if (file.length() == 0 || !file.startsWith("/")) {
    file = "/" + file;
  }
  if (sdFs == nullptr || file.length() < 3) {
    server.send(400, "text/plain", "Invalid file.");
    return;
  }
  if (file == String(csvPath)) {
    server.send(403, "text/plain",
                "The active file cannot be deleted. Restart the board with a new "
                "file or delete it on a computer.");
    return;
  }
  if (!sdFs->exists(file)) {
    server.send(404, "text/plain", "File does not exist.");
    return;
  }
  if (!sdFs->remove(file)) {
    server.send(500, "text/plain", "Delete failed.");
    return;
  }

  if (ringValid && ringFile == file) {
    ringValid = false;
  }
  SLOG.print("[WEB] Deleted file: ");
  SLOG.println(file);

  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "File deleted.");
}

static void handleFavicon() {
  server.send(204, "text/plain", "");
}


// HTTP firmware update (independent of ArduinoOTA, works from a browser)
static void handleUpdateDone() {
  if (Update.hasError()) {
    server.send(500, "text/plain", String("Update failed: ") + Update.errorString());
  } else {
    server.send(200, "text/html",
                "<meta http-equiv=\"refresh\" content=\"12;url=/\">"
                "<h3>Update OK, the device is restarting...</h3>");
    delay(300);
    ESP.restart();
  }
}

static void handleUpdateUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    SLOG.print("[OTA] HTTP update: ");
    SLOG.println(up.filename);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      SLOG.print("[OTA] begin failed: ");
      SLOG.println(Update.errorString());
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      SLOG.print("[OTA] write failed: ");
      SLOG.println(Update.errorString());
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      SLOG.print("[OTA] end failed: ");
      SLOG.println(Update.errorString());
    }
  }
}

static void startWebServer() {
  server.on("/", handleRoot);
  server.on("/admin", HTTP_GET,  handleAdminGet);
  server.on("/admin", HTTP_POST, handleAdminPost);
  server.on("/api/status", handleStatus);
  server.on("/api/data", handleData);
  server.on("/api/files", handleFiles);
  server.on("/download", handleDownload);
  server.on("/delete", handleDelete);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.on("/favicon.ico", handleFavicon);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Unknown path.");
  });
  server.begin();
  SLOG.print("[WEB] Server started (Ethernet: ");
  SLOG.print(ETH.localIP());
  SLOG.print(", AP: ");
  SLOG.print(settings.apEnabled ? WiFi.softAPIP() : IPAddress(0,0,0,0));
  SLOG.println(")");
}

// ============================== SETUP ==============================

void setup() {
#if SERIAL_DEBUG
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32-POE2 / PMS5003 / microSD / NTP / WEB / AP ===");
  Serial.print("Chip: ");
  Serial.println(ESP.getChipModel());
#endif

  loadSettings();

  // ---------- 1. microSD ----------
  sdReady = mountSD();
  SLOG.print("[SD] SD_MMC.begin():     ");
  SLOG.println(sdReady ? "OK" : "FAILED");

  // ---------- 2. PMS5003 ----------
  Serial2.begin(PMS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  SLOG.print("[PMS] Serial2 @ ");
  SLOG.print(PMS_BAUD);
  SLOG.print(" 8N1  RX=GPIO");
  SLOG.print(PMS_RX_PIN);
  SLOG.print("  TX=GPIO");
  SLOG.println(PMS_TX_PIN);

  // ---------- 3. Ethernet ----------
  netStartMs = millis();
  if (waitEthernet()) {
    ethActive = true;
    netMode   = "Ethernet";
    SLOG.print("[NET] Ethernet ACTIVE, IP=");
    SLOG.print(ETH.localIP());
    SLOG.print(" (");
    SLOG.print(settings.ethDhcp ? "DHCP" : "static");
    SLOG.println(")");
  } else {
    ethActive = false;
    netMode   = "offline";
    SLOG.println("[NET] Ethernet is not available");
  }

  // ---------- 4. Wi-Fi AP ----------

  // Make Ethernet the default interface so OTA/UDP sockets bind to it
  if (ethActive) {
    ETH.setDefault();
  }
  startAP();

  // ---------- 5. NTP ----------
  if (ethActive) {
    ntpSynced = syncNtp();
    if (ntpSynced) {
      setenv("TZ", TZ_INFO, 1);
      tzset();
      bootEpoch = (uint32_t)time(nullptr) - (millis() / 1000UL);
      SLOG.print("[NTP] OK, epoch=");
      SLOG.println((uint32_t)time(nullptr));
    } else {
      SLOG.println("[NTP] FAILED -> MILLIS (time can be set on /admin)");
    }
  } else {
    SLOG.println("[NTP] Skipped (no network)");
  }

  // ---------- 6. CSV file ----------
  if (sdReady) {
    sdReady = prepareCsv();
  }
  rememberRotationDate();

  // ---------- 7. Web server (Ethernet or AP) ----------
  if (ethActive || settings.apEnabled) {
    startWebServer();
  } else {
    SLOG.println("[WEB] Server not started (no network)");
  }

  // ---------- 8. OTA ----------
#if OTA_ENABLE
  if (ethActive || settings.apEnabled) {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (strlen(OTA_PASSWORD) > 0) {
      ArduinoOTA.setPassword(OTA_PASSWORD);
    }
    ArduinoOTA.begin();
    SLOG.print("[OTA] Ready: ");
    SLOG.println(OTA_HOSTNAME);
  }
#endif

  lastPmsMs      = millis();
  lastWarnMs     = millis();
  lastRotCheck   = millis();
}

// ============================== LOOP ==============================

void loop() {
  if (ethActive || settings.apEnabled) {
    server.handleClient();
#if OTA_ENABLE
    ArduinoOTA.handle();
#endif
  }

  // Scheduled restart after saving network settings
  if (restartScheduled != 0 && (millis() - restartScheduled) > 2500UL) {
    SLOG.println("[SYS] Restarting...");
    delay(50);
    ESP.restart();
  }

  // Daily file rotation (checked once per second)
  uint32_t nowMs = millis();
  if ((nowMs - lastRotCheck) >= 1000UL) {
    lastRotCheck = nowMs;
    checkRotation();
  }

  // No delay(): handle a sample as soon as a full 32-byte frame is available.
  if (Serial2.available() >= 32) {
    if (readPmsData(&Serial2)) {
      lastPmsMs = millis();
      handleSample();
    }
  }

  // Serial warning when the sensor stops sending data
  uint32_t now = millis();
  if ((now - lastPmsMs) >= PMS_SILENCE_WARN_MS &&
      (now - lastWarnMs) >= PMS_SILENCE_WARN_MS) {
    lastWarnMs = now;
    SLOG.print("[WARNING] PMS5003 has sent no data for ");
    SLOG.print((now - lastPmsMs) / 1000);
    SLOG.println(" s");
  }
}
