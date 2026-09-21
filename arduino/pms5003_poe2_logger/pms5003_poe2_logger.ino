/*
  ESP32-POE2 + PMS5003 + microSD + NTP + web server + Wi-Fi AP
  ============================================================

  Ploca:  Olimex ESP32-POE2 (ESP32-WROVER-E, 4 MB flash, PSRAM)

  U Arduino IDE / arduino-cli odaberi plocu koja ukljucuje PSRAM
  (npr. "ESP32 Wrover Module") - POE2 ima WROVER modul.
  Napomena: u Arduino jezgri 3.3.11 ne postoji ploca "esp32-poe2".

  Web server (port 80) - dostupan preko Ethernet-a i preko Wi-Fi AP-a:
    /               HTML: graf, trenutne vrijednosti, popis datoteka
    /admin          administracija: mrezne postavke i rucno vrijeme
    /api/status     JSON: vrijeme, mreze, zapisi, trenutne vrijednosti, OTA
    /api/data?f=    JSON: zapisi (timestamp, pm1, pm25, pm100) za graf
    /api/files      JSON: popis datoteka na kartici
    /download?f=    skidanje datoteke
    /delete?f=      brisanje datoteke (aktivna se ne moze obrisati)

  Mreze:
    - Ethernet (LAN8720) je primarni put; moze biti DHCP ili staticni IP
    - Wi-Fi AP je uvijek dostupan ako je ukljucen u /admin (SSID, lozinka,
      kanal); klijenti dobivaju adresu iz 192.168.4.0/24
    - web server i OTA rade na obje mreze

  Vrijeme:
    - NTP ako je mreza dostupna ("NTP" u CSV-u)
    - rucno postavljeno vrijeme preko /admin ako NTP nije dostupan ("RUCNO")
    - inace millis() ("MILLIS")

  Postavke se pamte u NVS (Preferences) i prezivljavaju restart.
  Promjena mreznih postavki restarta uredjaj.

  Zapis ide u zasebnu CSV datoteku po datumu i vremenu kreiranja:
      /pms_YYYYMMDD_HHMMSS.csv
  Datoteka se rotira automatski u ponoc (bez restarta ploce).

  OTA: ploca se moze flashirati preko mreze (ArduinoOTA), bez USB kabela:
      arduino-cli upload -p <IP> --fqbn esp32:esp32:esp32wrover \
          --protocol network <putanja_do_skice>

  ------------------------------------------------------------------
  PINOVI (provjereno prema Olimex dokumentaciji za ESP32-POE2)
  ------------------------------------------------------------------
  Ethernet - LAN8720: PHY_ADDR=0, MDC=GPIO23, MDIO=GPIO18, POWER=GPIO12,
  CLK=GPIO0 (CLK_OUT) - OBAVEZNO za POE2, GPIO16/17 su PSRAM.

  microSD (ugradjeni utor, 1-bit SDMMC): CLK=GPIO14, CMD=GPIO15, D0=GPIO2.

  PMS5003: VCC=5V (EXT1 pin 5), GND=GND, TX=GPIO33 -> ESP32 RX,
  RX=GPIO13 <- ESP32 TX (opcionalno), SET/RESET nisu spojeni.

  NAPOMENA: POE2 nema galvansku izolaciju izmedju PoE napajanja i USB-a.
  Tijekom programiranja preko USB-a iskljuci Ethernet kabel ako se ploca
  napaja preko PoE-a.
*/

// ---------------------------------------------------------------------------
// Ethernet definicije MORAJU biti prije #include <ETH.h>
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
#include <sys/time.h>
#include <time.h>

// ============================ KONFIGURACIJA ============================

// ---- Serijski ispis ----
// 0 = potpuno iskljucen (sve informacije su na web stranici)
// 1 = ukljucen, 115200 baud (dijagnostika)
#define SERIAL_DEBUG 0

// ---- PMS5003 (UART2) ----
#define PMS_RX_PIN   33     // ESP32 RX  <- PMS5003 TX
#define PMS_TX_PIN   13     // ESP32 TX  -> PMS5003 RX (opcionalno)
#define PMS_BAUD     9600

// ---- Vremenska zona ----
// CSV stupac "timestamp" ostaje Unix epoch u UTC-u; zona se koristi za
// citljivo ime datoteke i prikaz vremena na stranici.
#define TZ_INFO      "CET-1CEST,M3.5.0,M10.5.0/3"   // Europe/Zagreb
#define CSV_PREFIX   "/pms_"

// ---- Mreza i vrijeme ----
#define NTP_SERVER              "pool.ntp.org"
#define ETH_TIMEOUT_MS          10000UL
#define NET_TOTAL_BUDGET_MS     20000UL
#define NTP_TIMEOUT_MS          10000UL
#define NTP_EPOCH_MIN           1000000000UL
#define PMS_SILENCE_WARN_MS     5000UL

// ---- Web server i graf ----
#define WEB_PORT                80
#define LOG_RING_MAX            500     // koliko zapisa se drzi u RAM-u za graf

// ---- OTA (update preko mreze) ----
#define OTA_ENABLE      1
#define OTA_HOSTNAME    "esp32-poe2-pms"
#define OTA_PASSWORD    ""

// ---- Zadane mrezne postavke (mijenjaju se na /admin) ----
#define ZAD_ETH_IP      "192.168.3.50"
#define ZAD_ETH_MASKA   "255.255.255.0"
#define ZAD_ETH_GW      "192.168.3.1"
#define ZAD_ETH_DNS     "8.8.8.8"
#define ZAD_AP_SSID     "esp32-poe2-pms"
#define ZAD_AP_LOZINKA  "pms5003pms"    // "" ili <8 znakova = otvorena mreza
#define ZAD_AP_KANAL    6
#define NVS_NAMESPACE   "pms5003"

// ---------------------------- SERIJSKI ISPIS ----------------------------
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

// ======================= STRUKTURA PODATAKA PMS =======================

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard;
  uint16_t pm25_standard;
  uint16_t pm100_standard;
  uint16_t pm10_env;      // atmosferske vrijednosti - ove se zapisuju
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

// ===================== POSTAVKE UREDJAJA (NVS) =====================

struct PostavkeUredjaja {
  bool    ethDhcp;
  char    ethIp[16];
  char    ethMaska[16];
  char    ethGw[16];
  char    ethDns[16];
  bool    apUkljucen;
  char    apSsid[33];
  char    apLozinka[65];
  uint8_t apKanal;
  char    ntpServer[64];
};

static PostavkeUredjaja postavke;
static Preferences      prefs;

// ============================ STANJE RADA ============================

static fs::FS     *sdFs          = nullptr;
static bool        sdSpreman     = false;
static char        csvPath[48]   = "";

static bool        mrezaAktivna  = false;      // Ethernet link + IP
static bool        ntpSinkroniziran = false;
static bool        rucnoVrijeme  = false;      // vrijeme postavljeno na /admin
static const char *nacinMreze    = "offline";

static uint32_t    bootEpoch     = 0;          // 0 = nepoznato
static uint32_t    restartZakazan = 0;         // 0 = nije zakazan

static uint16_t    zadnjiZapisPM25  = 0xFFFF;  // zadnje ZAPISANO (za usporedbu)
static uint16_t    zadnjiZapisPM100 = 0xFFFF;
static bool        prviZapis        = true;

static uint32_t    zapisaUkupno  = 0;
static bool        imaUzorak     = false;
static uint32_t    zadnjiTs      = 0;
static uint16_t    zadnjiPm1     = 0;
static uint16_t    zadnjiPm25    = 0;
static uint16_t    zadnjiPm100   = 0;

static uint32_t    zadnjiPMS        = 0;
static uint32_t    zadnjeUpozorenje = 0;
static uint32_t    mrezaStart       = 0;

static uint32_t    okviraPrihvaceno = 0;   // valjani 32-bajtni okviri
static uint32_t    okviraOdbaceno   = 0;   // okviri s losim headerom/checksumom

// ---- Dnevna rotacija datoteke ----
static int         rotDan     = -1;
static int         rotMjesec  = -1;
static int         rotGodina  = -1;
static uint32_t    zadnjaRotProvjera = 0;

static WebServer   server(WEB_PORT);

// ---- Prsten zapisa u RAM-u (za graf) ----
struct LogRow {
  uint32_t ts;
  uint16_t pm1;
  uint16_t pm25;
  uint16_t pm100;
};

static LogRow  logRing[LOG_RING_MAX];
static int     logRingCount = 0;
static int     logRingHead  = 0;
static bool    logRingValid = false;
static String  logRingFile  = "";

// ============================== VRIJEME ==============================

static const char *izvorVremena() {
  if (ntpSinkroniziran) return "NTP";
  if (rucnoVrijeme)     return "RUCNO";
  return "MILLIS";
}

static bool epochVrijedi() {
  return (ntpSinkroniziran || rucnoVrijeme) && time(nullptr) > (time_t)NTP_EPOCH_MIN;
}

static uint32_t trenutnoVrijeme() {
  if (epochVrijedi()) {
    return (uint32_t)time(nullptr);
  }
  return millis();
}

static void formatEpoch(uint32_t e, char *buf, size_t n) {
  if (e == 0) {
    snprintf(buf, n, "nepoznato");
    return;
  }
  struct tm tmInfo;
  time_t t = (time_t)e;
  localtime_r(&t, &tmInfo);
  snprintf(buf, n, "%04d-%02d-%02d %02d:%02d:%02d",
           tmInfo.tm_year + 1900, tmInfo.tm_mon + 1, tmInfo.tm_mday,
           tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec);
}

static void formatVrijeme(char *buf, size_t n) {
  if (epochVrijedi()) {
    formatEpoch((uint32_t)time(nullptr), buf, n);
  } else {
    snprintf(buf, n, "uptime %lu s", (unsigned long)(millis() / 1000UL));
  }
}

// Rucno postavljanje vremena (kad NTP nije dostupan). Vrijeme se ne pamti
// kroz restart jer ploca nema RTC s baterijom.
static void postaviRucnoVrijeme(time_t epoch) {
  struct timeval tv;
  tv.tv_sec  = epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  rucnoVrijeme = true;
  if (bootEpoch == 0) {
    bootEpoch = (uint32_t)epoch - (millis() / 1000UL);
  }
  SLOG.print("[VRIJEME] Rucno postavljeno: ");
  SLOG.println((uint32_t)epoch);
}

// ===================== POSTAVKE: UCI/SPREMI =====================

static void ucitajPostavke() {
  postavke.ethDhcp = true;
  strlcpy(postavke.ethIp,     ZAD_ETH_IP,     sizeof(postavke.ethIp));
  strlcpy(postavke.ethMaska,  ZAD_ETH_MASKA,  sizeof(postavke.ethMaska));
  strlcpy(postavke.ethGw,     ZAD_ETH_GW,     sizeof(postavke.ethGw));
  strlcpy(postavke.ethDns,    ZAD_ETH_DNS,    sizeof(postavke.ethDns));
  postavke.apUkljucen = true;
  strlcpy(postavke.apSsid,    ZAD_AP_SSID,    sizeof(postavke.apSsid));
  strlcpy(postavke.apLozinka, ZAD_AP_LOZINKA, sizeof(postavke.apLozinka));
  postavke.apKanal = ZAD_AP_KANAL;
  strlcpy(postavke.ntpServer, NTP_SERVER,     sizeof(postavke.ntpServer));

  if (!prefs.begin(NVS_NAMESPACE, true)) {
    return;                                   // nema spremljenih postavki
  }
  postavke.ethDhcp    = prefs.getBool("ethdhcp", postavke.ethDhcp);
  postavke.apUkljucen = prefs.getBool("ap",      postavke.apUkljucen);
  postavke.apKanal    = prefs.getUChar("apkanal", postavke.apKanal);

  String s;
  s = prefs.getString("ethip",    postavke.ethIp);     strlcpy(postavke.ethIp,     s.c_str(), sizeof(postavke.ethIp));
  s = prefs.getString("ethmaska", postavke.ethMaska);  strlcpy(postavke.ethMaska,  s.c_str(), sizeof(postavke.ethMaska));
  s = prefs.getString("ethgw",    postavke.ethGw);     strlcpy(postavke.ethGw,     s.c_str(), sizeof(postavke.ethGw));
  s = prefs.getString("ethdns",   postavke.ethDns);    strlcpy(postavke.ethDns,    s.c_str(), sizeof(postavke.ethDns));
  s = prefs.getString("apssid",   postavke.apSsid);    strlcpy(postavke.apSsid,    s.c_str(), sizeof(postavke.apSsid));
  s = prefs.getString("aploz",    postavke.apLozinka); strlcpy(postavke.apLozinka, s.c_str(), sizeof(postavke.apLozinka));
  s = prefs.getString("ntp",      postavke.ntpServer); strlcpy(postavke.ntpServer, s.c_str(), sizeof(postavke.ntpServer));
  prefs.end();

  if (postavke.apKanal < 1 || postavke.apKanal > 13) {
    postavke.apKanal = ZAD_AP_KANAL;
  }
}

static void spremiPostavke() {
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    SLOG.println("[NVS] Ne mogu otvoriti za pisanje");
    return;
  }
  prefs.putBool("ethdhcp",  postavke.ethDhcp);
  prefs.putBool("ap",       postavke.apUkljucen);
  prefs.putUChar("apkanal", postavke.apKanal);
  prefs.putString("ethip",    postavke.ethIp);
  prefs.putString("ethmaska", postavke.ethMaska);
  prefs.putString("ethgw",    postavke.ethGw);
  prefs.putString("ethdns",   postavke.ethDns);
  prefs.putString("apssid",   postavke.apSsid);
  prefs.putString("aploz",    postavke.apLozinka);
  prefs.putString("ntp",      postavke.ntpServer);
  prefs.end();
}

// ============================== PRSTEN ZAPISA ==============================

static void ringReset(const String &datoteka) {
  logRingCount = 0;
  logRingHead  = 0;
  logRingFile  = datoteka;
  logRingValid = true;
}

static void ringPush(uint32_t ts, uint16_t a, uint16_t b, uint16_t c) {
  int idx;
  if (logRingCount < LOG_RING_MAX) {
    idx = (logRingHead + logRingCount) % LOG_RING_MAX;
    logRingCount++;
  } else {
    idx = logRingHead;
    logRingHead = (logRingHead + 1) % LOG_RING_MAX;
  }
  logRing[idx].ts    = ts;
  logRing[idx].pm1   = a;
  logRing[idx].pm25  = b;
  logRing[idx].pm100 = c;
}

static bool ucitajUPrsten(const String &datoteka) {
  if (logRingValid && logRingFile == datoteka) {
    return true;
  }
  if (sdFs == nullptr) {
    return false;
  }
  File f = sdFs->open(datoteka, FILE_READ);
  if (!f || f.isDirectory()) {
    return false;
  }

  ringReset(datoteka);
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 8) continue;
    if (line[0] < '0' || line[0] > '9') continue;   // preskoci header

    unsigned long ts = 0;
    unsigned a = 0, b = 0, c = 0;
    char izvor[16] = {0};
    if (sscanf(line.c_str(), "%lu,%15[^,],%u,%u,%u", &ts, izvor, &a, &b, &c) == 5) {
      ringPush((uint32_t)ts, (uint16_t)a, (uint16_t)b, (uint16_t)c);
    }
  }
  f.close();
  return true;
}

// ============================== SD KARTICA ==============================

static bool montirajSD() {
  if (SD_MMC.begin("/sdcard", true)) {   // 1-bit: CLK=14, CMD=15, D0=2
    sdFs = &SD_MMC;
    return true;
  }
  return false;
}

static void sastaviImeDatoteke() {
  time_t sada = time(nullptr);

  if (epochVrijedi()) {
    struct tm tmInfo;
    localtime_r(&sada, &tmInfo);
    snprintf(csvPath, sizeof(csvPath),
             CSV_PREFIX "%04d%02d%02d_%02d%02d%02d.csv",
             tmInfo.tm_year + 1900, tmInfo.tm_mon + 1, tmInfo.tm_mday,
             tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec);
  } else {
    snprintf(csvPath, sizeof(csvPath),
             CSV_PREFIX "millis_%010lu.csv", (unsigned long)millis());
  }
}

static bool pripremiCSV() {
  if (csvPath[0] == '\0') {
    sastaviImeDatoteke();
  }

  if (sdFs->exists(csvPath)) {
    SLOG.print("[SD] Datoteka vec postoji, nastavljam u: ");
    SLOG.println(csvPath);
    return true;
  }

  File f = sdFs->open(csvPath, FILE_WRITE);
  if (!f) {
    SLOG.print("[SD] GRESKA: ne mogu kreirati ");
    SLOG.println(csvPath);
    return false;
  }
  f.println("timestamp,time_source,pm1_0,pm2_5,pm10");
  f.close();
  SLOG.print("[SD] Kreirana datoteka s headerom: ");
  SLOG.println(csvPath);
  return true;
}

// ---- Dnevna rotacija ----

static void zapisiDatumRotacije() {
  if (!epochVrijedi()) {
    return;
  }
  struct tm t;
  time_t sada = time(nullptr);
  localtime_r(&sada, &t);
  rotDan    = t.tm_mday;
  rotMjesec = t.tm_mon;
  rotGodina = t.tm_year;
}

// Otvori novu datoteku s imenom po trenutnom datumu i vremenu.
// Datoteka se ionako otvara po zapisu, pa nema otvorenog handlea.
static void rotirajDatoteku() {
  struct tm t;
  time_t sada = time(nullptr);
  localtime_r(&sada, &t);

  sastaviImeDatoteke();
  zapisaUkupno     = 0;
  prviZapis        = true;
  zadnjiZapisPM25  = 0xFFFF;     // prva promjena u novoj datoteci se pise
  zadnjiZapisPM100 = 0xFFFF;
  logRingValid     = false;      // graf ce prijeci na novu datoteku

  if (sdSpreman) {
    pripremiCSV();
  }

  rotDan    = t.tm_mday;
  rotMjesec = t.tm_mon;
  rotGodina = t.tm_year;

  SLOG.print("[SD] Dnevna rotacija, nova datoteka: ");
  SLOG.println(csvPath);
}

static void provjeriRotaciju() {
  if (!epochVrijedi() || !sdSpreman || rotDan < 0) {
    return;
  }
  struct tm t;
  time_t sada = time(nullptr);
  localtime_r(&sada, &t);
  if (t.tm_mday != rotDan || t.tm_mon != rotMjesec || t.tm_year != rotGodina) {
    rotirajDatoteku();
  }
}

static bool zapisiRedak(uint32_t ts, const char *izvor,
                        uint16_t pm1, uint16_t pm25, uint16_t pm100) {
  if (sdFs == nullptr) {
    return false;
  }
  File f = sdFs->open(csvPath, FILE_APPEND);
  if (!f) {
    SLOG.print("[SD] GRESKA: ne mogu otvoriti ");
    SLOG.print(csvPath);
    SLOG.println(" za pisanje");
    return false;
  }
  f.print(ts);     f.print(',');
  f.print(izvor);  f.print(',');
  f.print(pm1);    f.print(',');
  f.print(pm25);   f.print(',');
  f.println(pm100);
  f.close();
  zapisaUkupno++;

  if (logRingValid && logRingFile == String(csvPath)) {
    ringPush(ts, pm1, pm25, pm100);
  }
  return true;
}

// ============================== PMS5003 ==============================

// Cita jedan 32-bajtni okvir. Petlja se poravnava na header 0x42 0x4D, pa
// se gubitak sinkronizacije sam ispravlja - bez toga bi svaki sljedeci
// okvir trajno bio "pomaknut" i podaci bi prestali dolaziti.
static bool readPMSdata(Stream *s) {
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];

  while (s->available() >= 32) {
    if (s->peek() != 0x42) {      // trazi prvi bajt headera
      s->read();
      continue;
    }

    s->readBytes(buffer, 32);

    if (buffer[1] != 0x4D) {      // lazni header, nastavi traziti
      okviraOdbaceno++;
      continue;
    }

    uint16_t zbroj = 0;
    for (uint8_t i = 0; i < 30; i++) {
      zbroj += buffer[i];
    }
    uint16_t primljeniChecksum = ((uint16_t)buffer[30] << 8) | buffer[31];
    if (zbroj != primljeniChecksum) {
      okviraOdbaceno++;
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
    pms.checksum        = primljeniChecksum;

    okviraPrihvaceno++;
    return true;
  }

  return false;
}

// ============================== MREZA ==============================

static bool cekajEthernet() {
  // Staticni IP (ako je tako postavljeno) mora se zadati prije begin().
  if (!postavke.ethDhcp) {
    IPAddress ip, gw, maska, dns;
    ip.fromString(postavke.ethIp);
    gw.fromString(postavke.ethGw);
    maska.fromString(postavke.ethMaska);
    dns.fromString(postavke.ethDns);
    bool okCfg = ETH.config(ip, gw, maska, dns);
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
         (millis() - mrezaStart) < NET_TOTAL_BUDGET_MS) {
    if (ETH.linkUp() && ETH.localIP() != IPAddress(0, 0, 0, 0)) {
      return true;
    }
    delay(100);
  }
  return false;
}

static void pokreniAP() {
  if (!postavke.apUkljucen) {
    SLOG.println("[AP] Iskljucen u postavkama");
    return;
  }

  WiFi.mode(WIFI_AP);
  bool ok;
  if (strlen(postavke.apLozinka) >= 8) {
    ok = WiFi.softAP(postavke.apSsid, postavke.apLozinka, postavke.apKanal);
  } else {
    ok = WiFi.softAP(postavke.apSsid, NULL, postavke.apKanal);   // otvorena mreza
  }

  SLOG.print("[AP] ");
  SLOG.print(postavke.apSsid);
  SLOG.print(" kanal ");
  SLOG.print(postavke.apKanal);
  SLOG.print(" -> ");
  SLOG.print(WiFi.softAPIP());
  SLOG.println(ok ? "" : " (GRESKA)");
}

static bool sinkronizirajNTP() {
  SLOG.print("[NTP] configTime(");
  SLOG.print(postavke.ntpServer);
  SLOG.println(")");
  configTime(0, 0, postavke.ntpServer);

  uint32_t start = millis();
  while (true) {
    if (time(nullptr) > (time_t)NTP_EPOCH_MIN) {
      return true;
    }
    uint32_t sada = millis();
    if ((sada - start) >= NTP_TIMEOUT_MS) {
      return false;
    }
    if ((sada - mrezaStart) >= NET_TOTAL_BUDGET_MS) {
      return false;
    }
    delay(200);
  }
}

// ============================== OBRADA UZORKA ==============================

static void obradiUzorak() {
  // Atmosferske vrijednosti = bajtovi 10-15 okvira
  uint16_t pm1   = pms.pm10_env;
  uint16_t pm25  = pms.pm25_env;
  uint16_t pm100 = pms.pm100_env;

  imaUzorak = true;
  uint32_t ts     = trenutnoVrijeme();
  const char *izv = izvorVremena();
  zadnjiTs    = ts;
  zadnjiPm1   = pm1;
  zadnjiPm25  = pm25;
  zadnjiPm100 = pm100;

  bool promjena = (pm25 != zadnjiZapisPM25) || (pm100 != zadnjiZapisPM100);

  if (promjena) {
    if (!sdSpreman) {
      sdSpreman = pripremiCSV();
    }
    if (sdSpreman && zapisiRedak(ts, izv, pm1, pm25, pm100)) {
      zadnjiZapisPM25  = pm25;
      zadnjiZapisPM100 = pm100;
      if (prviZapis) {
        prviZapis = false;
        SLOG.print("[SD] Prva zabiljezena stavka: ");
        SLOG.print(izv);
        SLOG.print(" ");
        SLOG.println(ts);
      }
    }
  }
}

// ============================== GLAVNA STRANICA ==============================

static const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="hr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-POE2 &middot; PMS5003</title>
<style>
body{font-family:Arial,Helvetica,sans-serif;margin:16px;background:#f4f5f7;color:#222}
h1{font-size:19px;margin:0 0 10px}
.card{background:#fff;border-radius:8px;padding:14px;margin-bottom:14px;box-shadow:0 1px 3px rgba(0,0,0,.12)}
.row{display:flex;flex-wrap:wrap;gap:18px;align-items:baseline}
.kv{font-size:13px;color:#555}
.big{font-size:28px;font-weight:700;line-height:1.1}
.vals{display:flex;gap:30px;flex-wrap:wrap;margin-top:4px}
.val{min-width:96px}
canvas{width:100%;height:320px;display:block}
table{border-collapse:collapse;width:100%;font-size:14px}
th,td{border-bottom:1px solid #e3e5e8;padding:6px 8px;text-align:left}
a{color:#0b63ce;text-decoration:none;cursor:pointer}
a:hover{text-decoration:underline}
.del{color:#b3261e}
.muted{color:#777;font-size:12px}
.stale{opacity:.45}
.tag{padding:2px 8px;border-radius:10px;background:#e8f0fe;font-size:12px;color:#174ea6}
select,button,input{font-size:13px;padding:4px 8px;border:1px solid #c9ccd1;border-radius:6px;background:#fff;color:#222}
button{cursor:pointer}
button:hover{background:#f0f3f7}
.ctrl{display:flex;flex-wrap:wrap;gap:14px;align-items:center;margin-bottom:8px}
.help{font-size:13px;font-weight:400;margin-left:8px}
.modal{display:none;position:fixed;left:0;top:0;right:0;bottom:0;background:rgba(0,0,0,.45);z-index:10;padding:18px;overflow:auto}
.modal.open{display:block}
.modalbox{background:#fff;border-radius:8px;max-width:780px;margin:0 auto;box-shadow:0 6px 24px rgba(0,0,0,.3)}
.modalhead{display:flex;justify-content:space-between;align-items:center;gap:12px;padding:12px 16px;border-bottom:1px solid #e3e5e8}
.modalbody{padding:6px 16px 18px;font-size:13px;line-height:1.5}
.modalbody h3{font-size:14px;margin:16px 0 6px}
.modalbody table{margin-bottom:10px;font-size:13px}
.modalbody td,.modalbody th{padding:4px 8px}
.modalbody ul{margin:6px 0 10px 18px;padding:0}
.modalbody li{margin-bottom:4px}
</style>
</head>
<body>

<div class="card">
  <h1>ESP32-POE2 &middot; PMS5003
    <a href="#" id="help" class="help">[ pomoc / referentne vrijednosti ]</a>
    <a href="/admin" class="help">[ administracija ]</a>
  </h1>
  <div class="row">
    <div class="kv">Vrijeme na ploci: <b id="clock">-</b> <span class="tag" id="tsrc">-</span></div>
    <div class="kv">Pokrenut: <b id="boot">-</b></div>
    <div class="kv">Datoteka: <b id="file">-</b></div>
    <div class="kv">Zapisa: <b id="recs">0</b></div>
    <div class="kv">OTA: <b id="ota">-</b></div>
  </div>
  <div class="row" style="margin-top:6px">
    <div class="kv">Zicana mreza: <b id="ethinfo">-</b></div>
    <div class="kv">Bezicna mreza (AP): <b id="apinfo">-</b></div>
  </div>
</div>

<div class="card">
  <div class="kv" style="margin-bottom:8px">Graf promjena PM1.0 / PM2.5 / PM10 (osvjezava se na svaki novi zapis)</div>
  <div class="ctrl">
    <label class="kv">Raspon:
      <select id="range">
        <option value="0">sve</option>
        <option value="300">zadnjih 5 min</option>
        <option value="1800">zadnjih 30 min</option>
        <option value="3600">zadnji 1 h</option>
        <option value="21600">zadnjih 6 h</option>
        <option value="86400">zadnjih 24 h</option>
      </select>
    </label>
    <label class="kv">Od: <input type="datetime-local" id="od" step="1"></label>
    <label class="kv">Do: <input type="datetime-local" id="do" step="1"></label>
    <button id="primijeni">Primijeni raspon</button>
    <button id="ocisti">Sve</button>
    <button id="png">Preuzmi PNG</button>
  </div>
  <div class="muted" id="rangeinfo" style="margin-bottom:6px">-</div>
  <canvas id="chart"></canvas>
  <div class="muted" id="chartinfo">-</div>
</div>

<div class="card">
  <div class="muted">Trenutne vrijednosti (osvjezava se svakih 5 s)</div>
  <div class="kv" id="cage" style="margin:6px 0">senzor: -</div>
  <div class="vals" id="cvals">
    <div class="val"><div class="muted">PM1.0</div><div class="big" id="c1">-</div></div>
    <div class="val"><div class="muted">PM2.5</div><div class="big" id="c25">-</div></div>
    <div class="val"><div class="muted">PM10</div><div class="big" id="c10">-</div></div>
  </div>
  <div class="muted" id="cupd">-</div>
</div>

<div class="card">
  <div class="kv" style="margin-bottom:6px">Datoteke na kartici</div>
  <table>
    <thead><tr><th>Datoteka</th><th>Velicina</th><th>Radnje</th></tr></thead>
    <tbody id="fbody"></tbody>
  </table>
</div>

<div id="modal" class="modal">
  <div class="modalbox">
    <div class="modalhead">
      <b>Referentne vrijednosti PM2.5 / PM10</b>
      <a href="#" id="close">&times; zatvori</a>
    </div>
    <div class="modalbody">

      <h3>Osnovna razina (referenca)</h3>
      <table>
        <tr><th>Situacija</th><th>PM2.5 (&micro;g/m&sup3;)</th></tr>
        <tr><td>Cista unutrasnjost</td><td>5 - 15</td></tr>
        <tr><td>Gradska vanjska pozadina</td><td>10 - 30</td></tr>
        <tr><td>Zagadjen grad / smog</td><td>50 - 150</td></tr>
        <tr><td>Kuhanje bez nape</td><td>50 - 300</td></tr>
      </table>

      <h3>Cigareta</h3>
      <table>
        <tr><th>Pozicija</th><th>PM2.5 (&micro;g/m&sup3;)</th></tr>
        <tr><td>Neposredno uz dim (do 1 m)</td><td>300 - 1500+</td></tr>
        <tr><td>Prosjek sobe, jedna cigareta, zatvoreno</td><td>20 - 100</td></tr>
        <tr><td>Prosjek sobe, vise cigareta, zadimljeno</td><td>150 - 500</td></tr>
        <tr><td>Mali zatvoreni prostor, stalno pusenje</td><td>300 - 1000</td></tr>
      </table>
      <p style="margin:4px 0 0">Porast u sekundama, pad kroz 5 - 30 min ovisno o ventilaciji.</p>

      <h3>Dim od pozara</h3>
      <table>
        <tr><th>Situacija</th><th>PM2.5 (&micro;g/m&sup3;)</th></tr>
        <tr><td>Sibica, ugasena svijeca, tamjan</td><td>100 - 1000 (kratki pik)</td></tr>
        <tr><td>Kamina / pec na drva u sobi</td><td>50 - 500</td></tr>
        <tr><td>Dim sumskog pozara, vani, vidljiv dim</td><td>200 - 1000+</td></tr>
        <tr><td>Gusti dim, neposredna blizina</td><td>1000 - 10000+</td></tr>
        <tr><td>Razvijeni pozar u prostoriji</td><td>desetci tisuca (senzor zasicen)</td></tr>
      </table>

      <h3>Ogranicenje senzora PMS5003</h3>
      <ul>
        <li>Efektivno podrucje: <b>0 - 500 &micro;g/m&sup3;</b>, maksimum oko
            <b>1000 &micro;g/m&sup3;</b>.</li>
        <li>Iznad toga brojke su nepouzdane i zasicuju se - tipicno vrijednost
            &quot;zalijepi&quot; na konstantu (npr. stalno ~1000) dok dim traje.</li>
        <li>Cigareta i umjereni dim se mjere dobro; gusti dim u zatvorenom
            senzor vidi samo kao maksimum.</li>
      </ul>

      <h3>Kako razlikovati vrstu dima iz loga</h3>
      <ul>
        <li><b>Omjer PM2.5 / PM10:</b> ~0,9 - 1,0 = fini aerosol (cigareta, svjezi
            dim, ispuh); ~0,5 - 0,8 = ima krupne frakcije (prasina, pepeo, dim
            blizu izvora).</li>
        <li><b>Brzina porasta:</b> cigareta i sibica skacu u sekundama, prasina
            raste postupno.</li>
        <li><b>Brzina pada:</b> dim cigarete u ventiliranoj sobi padne na pola u
            1 - 5 min; prasina se talozi sporije.</li>
      </ul>

      <h3>Napomena o vlazi</h3>
      <p style="margin:4px 0">PMS5003 nema grijani ulaz, pa pri relativnoj vlazi iznad
        ~80 % (magla, kisa, rano jutro vani) broji kapljice kao cestice i pokazuje
        lazno visoke vrijednosti. Za vanjsku ugradnju to je najcesci izvor
        &quot;laznog dima&quot; u podacima.</p>

      <h3>Prakticni test lanca</h3>
      <p style="margin:4px 0">Upali i ugasi sibicu na ~30 cm od senzora - u logu se
        ocekuje pik u roku nekoliko sekundi i postupan pad kroz 1 - 3 minute.</p>

    </div>
  </div>
</div>

<script>
var selFile = "";
var lastCount = -1;
var rows = [];
var srcLabel = "NTP";
var rangeSec = 0;
var odEpoch = 0;
var doEpoch = 0;

function q(id){ return document.getElementById(id); }
function getJSON(u){ return fetch(u, {cache:"no-store"}).then(function(r){ return r.json(); }); }

function fmtShort(ts){
  if (srcLabel === "NTP" || srcLabel === "RUCNO") {
    if (ts > 1000000000) {
      var d = new Date(ts*1000);
      return ("0"+d.getHours()).slice(-2)+":"+("0"+d.getMinutes()).slice(-2)+":"+("0"+d.getSeconds()).slice(-2);
    }
  }
  return (ts/1000).toFixed(0)+"s";
}

function imaEpochVrijeme(){ return (srcLabel === "NTP" || srcLabel === "RUCNO"); }

function rangeLabel(){
  if (odEpoch || doEpoch) {
    var a = odEpoch ? new Date(odEpoch*1000).toLocaleString() : "pocetak";
    var b = doEpoch ? new Date(doEpoch*1000).toLocaleString() : "kraj";
    return a + " - " + b;
  }
  var el = q("range");
  return el.options[el.selectedIndex].text;
}

/* Filtar: prvo vlastiti od-do raspon (samo kad vrijeme ima epoch), inace brzi raspon */
function applyRange(){
  var d = rows.slice();
  if (imaEpochVrijeme() && (odEpoch || doEpoch)) {
    return d.filter(function(r){
      return (!odEpoch || r[0] >= odEpoch) && (!doEpoch || r[0] <= doEpoch);
    });
  }
  if (!rangeSec) return d;
  var jedinica = imaEpochVrijeme() ? 1 : 1000;
  var zadnji = rows[rows.length-1][0];
  var prag = zadnji - rangeSec * jedinica;
  return d.filter(function(r){ return r[0] >= prag; });
}

function pollStatus(){
  getJSON("/api/status").then(function(s){
    q("clock").textContent = s.time;
    q("tsrc").textContent  = s.source;
    q("recs").textContent  = s.records;
    q("ota").textContent   = s.ota ? s.ota : "-";
    q("boot").textContent  = s.boot;
    q("ethinfo").textContent = s.ethAktivna
        ? (s.ethIp + " / " + s.ethMaska + "  gw " + s.ethGw + (s.ethDhcp ? "  (DHCP)" : "  (static)"))
        : "nije aktivna";
    q("apinfo").textContent = s.apUkljucen
        ? (s.apSsid + "  " + s.apIp + "  kanal " + s.apKanal + "  klijenata: " + s.apKlijenti)
        : "iskljucena";
    prikaziStanjeSenzora(s);
    if (selFile === "") { selFile = s.file; q("file").textContent = selFile; }
    if (s.records !== lastCount) {
      lastCount = s.records;
      if (selFile === s.file) { loadData(); }
    }
  }).catch(function(){});
}

function prikaziStanjeSenzora(s){
  var el = q("cage");
  var vals = q("cvals");
  var age = (typeof s.ageSec === "number") ? s.ageSec : -1;
  var okviri = "  |  okvira: " + s.framesOk + " (odbaceno " + s.framesBad + ")";

  if (!s.current || s.current.ts === 0) {
    el.textContent = "senzor: jos nema podataka" + okviri;
    el.style.color = "#b3261e";
    vals.className = "vals stale";
  } else if (age >= 0 && age < 5) {
    el.textContent = "senzor: OK (zadnji uzorak prije " + age + " s)" + okviri;
    el.style.color = "#1b5e20";
    vals.className = "vals";
  } else if (age >= 0 && age < 15) {
    el.textContent = "senzor: kasni (zadnji uzorak prije " + age + " s)" + okviri;
    el.style.color = "#ef6c00";
    vals.className = "vals stale";
  } else {
    el.textContent = "senzor: NEMA PODATAKA vec " + age + " s" + okviri;
    el.style.color = "#b3261e";
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
    q("cupd").textContent = "osvjezeno: " + s.time + " (svakih 5 s)";
  }).catch(function(){});
}

function loadData(){
  getJSON("/api/data?f=" + encodeURIComponent(selFile)).then(function(d){
    rows = d.rows || [];
    srcLabel = d.source || "NTP";
    q("chartinfo").textContent = d.file + "  |  ukupno zapisa: " + rows.length +
                                 "  |  izvor vremena: " + srcLabel;
    draw();
  }).catch(function(){});
}

function loadFiles(){
  getJSON("/api/files").then(function(d){
    var tb = q("fbody");
    tb.innerHTML = "";
    (d.files || []).forEach(function(f){
      var tr = document.createElement("tr");

      var td1 = document.createElement("td");
      td1.textContent = f.name;
      tr.appendChild(td1);

      var td2 = document.createElement("td");
      td2.textContent = f.size + " B";
      tr.appendChild(td2);

      var td3 = document.createElement("td");

      var a1 = document.createElement("a");
      a1.textContent = "graf";
      a1.href = "#";
      a1.onclick = function(e){
        e.preventDefault();
        selFile = f.name;
        lastCount = -1;
        q("file").textContent = selFile;
        loadData();
      };
      td3.appendChild(a1);
      td3.appendChild(document.createTextNode(" | "));

      var a2 = document.createElement("a");
      a2.textContent = "preuzmi";
      a2.href = "/download?f=" + encodeURIComponent(f.name);
      td3.appendChild(a2);

      if (f.active) {
        td3.appendChild(document.createTextNode(" | (aktivna)"));
      } else {
        td3.appendChild(document.createTextNode(" | "));
        var a3 = document.createElement("a");
        a3.textContent = "obrisi";
        a3.className = "del";
        a3.href = "/delete?f=" + encodeURIComponent(f.name);
        a3.onclick = function(){ return confirm("Obrisati " + f.name + " ?"); };
        td3.appendChild(a3);
      }

      tr.appendChild(td3);
      tb.appendChild(tr);
    });
  }).catch(function(){});
}

/* Nacrtaj graf u zadani kontekst */
function drawChart(g, L, T, pw, ph, data, fs, lw){
  g.font = fs + "px Arial";

  if (!data.length) {
    g.fillStyle = "#888";
    g.fillText("Nema zapisa u odabranom rasponu.", L, T + fs + 4);
    return;
  }

  var xmin = data[0][0], xmax = data[data.length-1][0];
  if (xmax <= xmin) xmax = xmin + 1;
  var ymax = 10, i, v, t, x, y;
  data.forEach(function(r){ ymax = Math.max(ymax, r[1], r[2], r[3]); });
  ymax = Math.ceil(ymax * 1.15);

  function X(tt){ return L + (tt - xmin) * pw / (xmax - xmin); }
  function Y(vv){ return T + ph - vv * ph / ymax; }

  g.strokeStyle = "#e6e8eb";
  g.fillStyle = "#666";
  g.lineWidth = 1;
  for (i = 0; i <= 4; i++) {
    v = ymax * i / 4; y = Y(v);
    g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
    g.textAlign = "right"; g.fillText(v.toFixed(0), L - 6, y + fs*0.35);
  }
  for (i = 0; i <= 4; i++) {
    t = xmin + (xmax - xmin) * i / 4; x = X(t);
    g.beginPath(); g.moveTo(x, T); g.lineTo(x, T + ph); g.stroke();
    g.textAlign = "center"; g.fillText(fmtShort(Math.round(t)), x, T + ph + fs + 4);
  }

  g.strokeStyle = "#aaa";
  g.beginPath(); g.moveTo(L, T); g.lineTo(L, T + ph); g.lineTo(L + pw, T + ph); g.stroke();

  /* Naslov Y osi */
  g.save();
  g.translate(fs * 0.95, T + ph / 2);
  g.rotate(-Math.PI / 2);
  g.textAlign = "center";
  g.fillStyle = "#666";
  g.fillText("\u00b5g/m\u00b3", 0, 0);
  g.restore();

  var series = [
    {idx:1, col:"#2e7d32", name:"PM1.0"},
    {idx:2, col:"#ef6c00", name:"PM2.5"},
    {idx:3, col:"#c62828", name:"PM10"}
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
    g.fillStyle = "#333"; g.textAlign = "left";
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
  var info = "prikazano " + d.length + " od " + rows.length + " zapisa";
  if (d.length > 1) { info += "  |  raspon: " + fmtShort(d[0][0]) + " - " + fmtShort(d[d.length-1][0]); }
  info += "  |  odabir: " + rangeLabel();
  if (!imaEpochVrijeme() && (odEpoch || doEpoch)) { info += "  (vlastiti raspon radi samo za NTP/RUCNO zapise)"; }
  q("rangeinfo").textContent = info;

  drawChart(g, 52, 26, w-52-10, h-26-28, d, 12, 2);
}

function exportPNG(){
  var d = applyRange();
  var w = 1200, h = 620;
  var cv = document.createElement("canvas");
  cv.width = w; cv.height = h;
  var g = cv.getContext("2d");

  g.fillStyle = "#ffffff"; g.fillRect(0, 0, w, h);
  g.fillStyle = "#222"; g.font = "bold 20px Arial"; g.textAlign = "left";
  g.fillText("PMS5003 - " + selFile, 24, 34);
  g.font = "14px Arial"; g.fillStyle = "#555";
  g.fillText("Raspon: " + rangeLabel() + "  |  zapisa: " + d.length +
             "  |  izvor vremena: " + srcLabel +
             "  |  izvoz: " + new Date().toLocaleString(), 24, 58);

  drawChart(g, 70, 100, w - 70 - 40, h - 100 - 60, d, 14, 2.4);

  var a = document.createElement("a");
  var ts = new Date().toISOString().replace(/[:T]/g, "-").slice(0, 19);
  a.download = "pms_graf_" + ts + ".png";
  a.href = cv.toDataURL("image/png");
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

function primijeniVlastitiRaspon(){
  var od = q("od").value;
  var dov = q("do").value;
  odEpoch = od ? Math.floor(new Date(od).getTime()/1000) : 0;
  doEpoch = dov ? Math.floor(new Date(dov).getTime()/1000) : 0;
  draw();
}

function ocistiRaspone(){
  odEpoch = 0;
  doEpoch = 0;
  rangeSec = 0;
  q("od").value = "";
  q("do").value = "";
  q("range").value = "0";
  try { localStorage.setItem("pms_range", "0"); } catch(e) {}
  draw();
}

window.addEventListener("load", function(){
  /* Zapamti odabrani raspon u pregledniku (localStorage) */
  try {
    var sp = localStorage.getItem("pms_range");
    if (sp !== null) {
      q("range").value = sp;
      rangeSec = parseInt(sp, 10) || 0;
    }
  } catch (e) {}

  q("range").onchange = function(){
    rangeSec = parseInt(this.value, 10) || 0;
    odEpoch = 0; doEpoch = 0;
    q("od").value = ""; q("do").value = "";
    try { localStorage.setItem("pms_range", this.value); } catch (e) {}
    draw();
  };
  q("primijeni").onclick = primijeniVlastitiRaspon;
  q("ocisti").onclick = ocistiRaspone;
  q("png").onclick = exportPNG;

  /* Modal s referentnim vrijednostima */
  q("help").onclick = function(e){ e.preventDefault(); q("modal").className = "modal open"; };
  q("close").onclick = function(e){ e.preventDefault(); q("modal").className = "modal"; };
  q("modal").onclick = function(e){ if (e.target === this) q("modal").className = "modal"; };
  document.addEventListener("keydown", function(e){
    if (e.key === "Escape") q("modal").className = "modal";
  });

  pollStatus();
  pollCurrent();
  loadFiles();
  setInterval(pollStatus, 2000);    // graf na promjenu + mreze + vrijeme
  setInterval(pollCurrent, 5000);   // trenutne vrijednosti
  setInterval(loadFiles, 15000);    // popis datoteka
  window.addEventListener("resize", draw);
});
</script>
</body>
</html>
)HTML";

// ============================== ADMIN STRANICA ==============================

static const char ADMIN_STYLE[] PROGMEM = R"CSS(
body{font-family:Arial,Helvetica,sans-serif;margin:16px;background:#f4f5f7;color:#222}
.card{background:#fff;border-radius:8px;padding:14px;margin-bottom:14px;box-shadow:0 1px 3px rgba(0,0,0,.12)}
h1{font-size:19px;margin:0 0 10px}
h3{font-size:15px;margin:18px 0 6px}
table{border-collapse:collapse;font-size:14px}
td{padding:4px 8px}
input{font-size:13px;padding:4px 8px;border:1px solid #c9ccd1;border-radius:6px}
button{font-size:13px;padding:6px 12px;border:1px solid #c9ccd1;border-radius:6px;background:#fff;cursor:pointer}
button:hover{background:#f0f3f7}
.muted{color:#777;font-size:12px}
a{color:#0b63ce;text-decoration:none}
)CSS";

static void handleAdminGet() {
  String h = F("<!DOCTYPE html><html lang=\"hr\"><head><meta charset=\"utf-8\">");
  h += F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  h += F("<title>Administracija - ESP32-POE2</title><style>");
  h += FPSTR(ADMIN_STYLE);
  h += F("</style></head><body><div class=\"card\">");
  h += F("<h1>Administracija uredjaja <a href=\"/\">&larr; natrag na graf</a></h1>");

  // --- forma 1: mrezne postavke ---
  h += F("<form method=\"POST\" action=\"/admin\">");
  h += F("<input type=\"hidden\" name=\"akcija\" value=\"mreza\">");
  h += F("<h3>Zicana mreza (Ethernet)</h3>");
  h += F("<p><label><input type=\"radio\" name=\"ethdhcp\" value=\"1\"");
  if (postavke.ethDhcp) h += F(" checked");
  h += F("> DHCP &nbsp; </label><label><input type=\"radio\" name=\"ethdhcp\" value=\"0\"");
  if (!postavke.ethDhcp) h += F(" checked");
  h += F("> Staticna adresa</label></p><table>");
  h += F("<tr><td>IP adresa</td><td><input name=\"ethip\" value=\"");
  h += String(postavke.ethIp);
  h += F("\"></td></tr><tr><td>Maska</td><td><input name=\"ethmaska\" value=\"");
  h += String(postavke.ethMaska);
  h += F("\"></td></tr><tr><td>Gateway</td><td><input name=\"ethgw\" value=\"");
  h += String(postavke.ethGw);
  h += F("\"></td></tr><tr><td>DNS</td><td><input name=\"ethdns\" value=\"");
  h += String(postavke.ethDns);
  h += F("\"></td></tr></table>");

  h += F("<h3>Bezicna mreza (Wi-Fi AP)</h3>");
  h += F("<p><label><input type=\"checkbox\" name=\"ap\" value=\"1\"");
  if (postavke.apUkljucen) h += F(" checked");
  h += F("> Ukljucen</label></p><table>");
  h += F("<tr><td>SSID</td><td><input name=\"apssid\" value=\"");
  h += String(postavke.apSsid);
  h += F("\"></td></tr><tr><td>Lozinka</td><td><input name=\"aploz\" value=\"");
  h += String(postavke.apLozinka);
  h += F("\"> <span class=\"muted\">prazno ili &lt;8 znakova = otvorena mreza</span></td></tr>");
  h += F("<tr><td>Kanal</td><td><input name=\"apkanal\" type=\"number\" min=\"1\" max=\"13\" value=\"");
  h += String(postavke.apKanal);
  h += F("\"></td></tr></table>");

  h += F("<h3>NTP</h3><table><tr><td>Server</td><td><input name=\"ntp\" value=\"");
  h += String(postavke.ntpServer);
  h += F("\"></td></tr></table>");

  h += F("<p class=\"muted\">Spremanje mreznih postavki restarta uredjaj.</p>");
  h += F("<p><button type=\"submit\">Spremi mrezu i restartaj</button></p>");
  h += F("</form>");

  // --- forma 2: rucno vrijeme ---
  h += F("<hr><h3>Rucno postavljanje vremena</h3>");
  h += F("<p class=\"muted\">Koristi se kad NTP nije dostupan. Vrijeme se ne pamti kroz restart.</p>");
  h += F("<form method=\"POST\" action=\"/admin\">");
  h += F("<input type=\"hidden\" name=\"akcija\" value=\"vrijeme\">");
  h += F("<p><input type=\"datetime-local\" name=\"rucnovrijeme\" step=\"1\"> ");
  h += F("<button type=\"submit\">Postavi vrijeme</button></p></form>");

  h += F("<p class=\"muted\">Trenutno stanje: ");
  h += String(nacinMreze);
  h += F(" | izvor vremena: ");
  h += String(izvorVremena());
  h += F(" | SD: ");
  h += (sdSpreman ? F("OK") : F("nije dostupna"));
  h += F(" | datoteka: ");
  h += String(csvPath);
  h += F("</p>");

  h += F("</div></body></html>");

  server.send(200, "text/html; charset=utf-8", h);
}

static void handleAdminPost() {
  String akcija = server.arg("akcija");
  String poruka;

  // Rucno vrijeme
  if (server.hasArg("rucnovrijeme") && server.arg("rucnovrijeme").length() >= 16) {
    int g = 0, mj = 0, d = 0, sat = 0, min_ = 0, sek = 0;
    int broj = sscanf(server.arg("rucnovrijeme").c_str(), "%d-%d-%dT%d:%d:%d",
                      &g, &mj, &d, &sat, &min_, &sek);
    if (broj >= 5) {
      struct tm t = {};
      t.tm_year  = g - 1900;
      t.tm_mon   = mj - 1;
      t.tm_mday  = d;
      t.tm_hour  = sat;
      t.tm_min   = min_;
      t.tm_sec   = sek;
      t.tm_isdst = -1;
      time_t e = mktime(&t);
      if (e > (time_t)NTP_EPOCH_MIN) {
        setenv("TZ", TZ_INFO, 1);
        tzset();
        postaviRucnoVrijeme(e);
        poruka = F("Vrijeme je postavljeno.");
      } else {
        poruka = F("Neispravan datum/vrijeme.");
      }
    } else {
      poruka = F("Neispravan format vremena.");
    }
  }

  // Mrezne postavke
  if (akcija == "mreza") {
    bool promjena = false;

    bool noviDhcp = (server.arg("ethdhcp") == "1");
    if (noviDhcp != postavke.ethDhcp) { postavke.ethDhcp = noviDhcp; promjena = true; }
    if (server.hasArg("ethip") && server.arg("ethip") != postavke.ethIp) {
      strlcpy(postavke.ethIp, server.arg("ethip").c_str(), sizeof(postavke.ethIp)); promjena = true;
    }
    if (server.hasArg("ethmaska") && server.arg("ethmaska") != postavke.ethMaska) {
      strlcpy(postavke.ethMaska, server.arg("ethmaska").c_str(), sizeof(postavke.ethMaska)); promjena = true;
    }
    if (server.hasArg("ethgw") && server.arg("ethgw") != postavke.ethGw) {
      strlcpy(postavke.ethGw, server.arg("ethgw").c_str(), sizeof(postavke.ethGw)); promjena = true;
    }
    if (server.hasArg("ethdns") && server.arg("ethdns") != postavke.ethDns) {
      strlcpy(postavke.ethDns, server.arg("ethdns").c_str(), sizeof(postavke.ethDns)); promjena = true;
    }

    bool noviAp = server.hasArg("ap");
    if (noviAp != postavke.apUkljucen) { postavke.apUkljucen = noviAp; promjena = true; }
    if (server.hasArg("apssid") && server.arg("apssid").length() > 0 &&
        server.arg("apssid") != postavke.apSsid) {
      strlcpy(postavke.apSsid, server.arg("apssid").c_str(), sizeof(postavke.apSsid)); promjena = true;
    }
    if (server.hasArg("aploz") && server.arg("aploz") != postavke.apLozinka) {
      strlcpy(postavke.apLozinka, server.arg("aploz").c_str(), sizeof(postavke.apLozinka)); promjena = true;
    }
    if (server.hasArg("apkanal")) {
      int kanal = server.arg("apkanal").toInt();
      if (kanal >= 1 && kanal <= 13 && (uint8_t)kanal != postavke.apKanal) {
        postavke.apKanal = (uint8_t)kanal; promjena = true;
      }
    }
    if (server.hasArg("ntp") && server.arg("ntp").length() > 0 &&
        server.arg("ntp") != postavke.ntpServer) {
      strlcpy(postavke.ntpServer, server.arg("ntp").c_str(), sizeof(postavke.ntpServer)); promjena = true;
    }

    spremiPostavke();
    if (promjena) {
      poruka = F("Mrezne postavke su spremljene. Uredjaj se restarta...");
      restartZakazan = millis();
      if (restartZakazan == 0) restartZakazan = 1;
    } else if (poruka.length() == 0) {
      poruka = F("Nema promjena u mreznim postavkama.");
    }
  }

  String h = F("<!DOCTYPE html><html lang=\"hr\"><head><meta charset=\"utf-8\">");
  h += F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  h += F("<title>Administracija</title><style>");
  h += FPSTR(ADMIN_STYLE);
  h += F("</style></head><body><div class=\"card\"><h1>");
  h += poruka;
  h += F("</h1><p><a href=\"/\">natrag na graf</a> &nbsp;|&nbsp; <a href=\"/admin\">administracija</a></p>");
  h += F("</div></body></html>");
  server.send(200, "text/html; charset=utf-8", h);
}

// ============================== WEB RUKOVATELJI ==============================

static void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");
  server.sendContent_P(PAGE_HTML);
  server.sendContent("");
}

static void handleStatus() {
  char vrijeme[32];
  char bootStr[24];
  formatVrijeme(vrijeme, sizeof(vrijeme));
  formatEpoch(bootEpoch, bootStr, sizeof(bootStr));

  uint32_t epoch = epochVrijedi() ? (uint32_t)time(nullptr) : 0;

  String j = "{";
  j += "\"records\":";   j += String(zapisaUkupno);
  j += ",\"file\":\"";   j += String(csvPath);        j += "\"";
  j += ",\"net\":\"";    j += String(nacinMreze);     j += "\"";
  j += ",\"source\":\""; j += String(izvorVremena()); j += "\"";
  j += ",\"epoch\":";    j += String(epoch);
  j += ",\"time\":\"";   j += String(vrijeme);        j += "\"";
  j += ",\"boot\":\"";   j += String(bootStr);        j += "\"";
  j += ",\"uptime\":";   j += String((uint32_t)(millis() / 1000UL));
  j += ",\"ageSec\":";   j += String((uint32_t)((millis() - zadnjiPMS) / 1000UL));
  j += ",\"framesOk\":"; j += String(okviraPrihvaceno);
  j += ",\"framesBad\":";j += String(okviraOdbaceno);

  // Zicana mreza
  j += ",\"ethAktivna\":"; j += (mrezaAktivna ? "true" : "false");
  j += ",\"ethDhcp\":";    j += (postavke.ethDhcp ? "true" : "false");
  j += ",\"ethIp\":\"";    j += (mrezaAktivna ? ETH.localIP().toString()    : String("-")); j += "\"";
  j += ",\"ethMaska\":\""; j += (mrezaAktivna ? ETH.subnetMask().toString() : String("-")); j += "\"";
  j += ",\"ethGw\":\"";    j += (mrezaAktivna ? ETH.gatewayIP().toString()  : String("-")); j += "\"";
  j += ",\"ethMac\":\"";   j += ETH.macAddress(); j += "\"";

  // Bezicna mreza (AP)
  j += ",\"apUkljucen\":"; j += (postavke.apUkljucen ? "true" : "false");
  j += ",\"apSsid\":\"";   j += String(postavke.apSsid); j += "\"";
  j += ",\"apIp\":\"";     j += (postavke.apUkljucen ? WiFi.softAPIP().toString() : String("-")); j += "\"";
  j += ",\"apKanal\":";    j += String(postavke.apKanal);
  j += ",\"apKlijenti\":"; j += String((uint32_t)(postavke.apUkljucen ? WiFi.softAPgetStationNum() : 0));

#if OTA_ENABLE
  j += ",\"ota\":\"";     j += String(OTA_HOSTNAME);  j += "\"";
#else
  j += ",\"ota\":\"iskljucen\"";
#endif

  j += ",\"current\":{";
  if (imaUzorak) {
    j += "\"ts\":";      j += String(zadnjiTs);
    j += ",\"pm1\":";    j += String(zadnjiPm1);
    j += ",\"pm25\":";   j += String(zadnjiPm25);
    j += ",\"pm100\":";  j += String(zadnjiPm100);
  } else {
    j += "\"ts\":0,\"pm1\":\"-\",\"pm25\":\"-\",\"pm100\":\"-\"";
  }
  j += "}}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

static void handleData() {
  String datoteka = server.arg("f");
  if (datoteka.length() == 0) {
    datoteka = String(csvPath);
  }
  if (!datoteka.startsWith("/")) {
    datoteka = "/" + datoteka;
  }

  if (sdFs == nullptr || datoteka.length() < 3) {
    server.send(400, "application/json", "{\"error\":\"neispravna datoteka\"}");
    return;
  }
  if (!ucitajUPrsten(datoteka)) {
    server.send(404, "application/json", "{\"error\":\"datoteka nije otvorena\"}");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  char buf[160];
  snprintf(buf, sizeof(buf), "{\"file\":\"%s\",\"source\":\"%s\",\"count\":%d,\"rows\":[",
           datoteka.c_str(), izvorVremena(), logRingCount);
  server.sendContent(buf);

  for (int i = 0; i < logRingCount; i++) {
    LogRow r = logRing[(logRingHead + i) % LOG_RING_MAX];
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
    bool prvi = true;
    File f = root.openNextFile();
    while (f) {
      if (!f.isDirectory()) {
        String ime = String(f.name());
        if (!ime.startsWith("/")) {
          ime = "/" + ime;
        }
        char buf[160];
        snprintf(buf, sizeof(buf), "%s{\"name\":\"%s\",\"size\":%lu,\"active\":%s}",
                 (prvi ? "" : ","), ime.c_str(), (unsigned long)f.size(),
                 (ime == String(csvPath)) ? "true" : "false");
        server.sendContent(buf);
        prvi = false;
      }
      f = root.openNextFile();
    }
    root.close();
  }

  server.sendContent("]}");
  server.sendContent("");
}

static void handleDownload() {
  String datoteka = server.arg("f");
  if (datoteka.length() == 0 || !datoteka.startsWith("/")) {
    datoteka = "/" + datoteka;
  }
  if (sdFs == nullptr || datoteka.length() < 3) {
    server.send(400, "text/plain", "Neispravna datoteka.");
    return;
  }

  File f = sdFs->open(datoteka, FILE_READ);
  if (!f || f.isDirectory()) {
    server.send(404, "text/plain", "Datoteka ne postoji.");
    return;
  }

  String imeZaPrenos = datoteka.substring(datoteka.lastIndexOf('/') + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + imeZaPrenos + "\"");
  server.streamFile(f, "text/csv");
  f.close();
}

static void handleDelete() {
  String datoteka = server.arg("f");
  if (datoteka.length() == 0 || !datoteka.startsWith("/")) {
    datoteka = "/" + datoteka;
  }
  if (sdFs == nullptr || datoteka.length() < 3) {
    server.send(400, "text/plain", "Neispravna datoteka.");
    return;
  }
  if (datoteka == String(csvPath)) {
    server.send(403, "text/plain",
                "Aktivna datoteka se ne moze obrisati. Pokreni plocu s novom "
                "datotekom ili je obrisi s kartice na racunalu.");
    return;
  }
  if (!sdFs->exists(datoteka)) {
    server.send(404, "text/plain", "Datoteka ne postoji.");
    return;
  }
  if (!sdFs->remove(datoteka)) {
    server.send(500, "text/plain", "Brisanje nije uspjelo.");
    return;
  }

  if (logRingValid && logRingFile == datoteka) {
    logRingValid = false;
  }
  SLOG.print("[WEB] Obrisana datoteka: ");
  SLOG.println(datoteka);

  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "Datoteka obrisana.");
}

static void handleFavicon() {
  server.send(204, "text/plain", "");
}

static void pokreniWebServer() {
  server.on("/", handleRoot);
  server.on("/admin", HTTP_GET,  handleAdminGet);
  server.on("/admin", HTTP_POST, handleAdminPost);
  server.on("/api/status", handleStatus);
  server.on("/api/data", handleData);
  server.on("/api/files", handleFiles);
  server.on("/download", handleDownload);
  server.on("/delete", handleDelete);
  server.on("/favicon.ico", handleFavicon);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Nepoznata putanja.");
  });
  server.begin();
  SLOG.print("[WEB] Server pokrenut (Ethernet: ");
  SLOG.print(ETH.localIP());
  SLOG.print(", AP: ");
  SLOG.print(postavke.apUkljucen ? WiFi.softAPIP() : IPAddress(0,0,0,0));
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

  ucitajPostavke();

  // ---------- 1. microSD ----------
  sdSpreman = montirajSD();
  SLOG.print("[SD] SD_MMC.begin():     ");
  SLOG.println(sdSpreman ? "OK" : "FAILED");

  // ---------- 2. PMS5003 ----------
  Serial2.begin(PMS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  SLOG.print("[PMS] Serial2 @ ");
  SLOG.print(PMS_BAUD);
  SLOG.print(" 8N1  RX=GPIO");
  SLOG.print(PMS_RX_PIN);
  SLOG.print("  TX=GPIO");
  SLOG.println(PMS_TX_PIN);

  // ---------- 3. Ethernet ----------
  mrezaStart = millis();
  if (cekajEthernet()) {
    mrezaAktivna = true;
    nacinMreze   = "Ethernet";
    SLOG.print("[NET] Ethernet AKTIVAN, IP=");
    SLOG.print(ETH.localIP());
    SLOG.print(" (");
    SLOG.print(postavke.ethDhcp ? "DHCP" : "static");
    SLOG.println(")");
  } else {
    mrezaAktivna = false;
    nacinMreze   = "offline";
    SLOG.println("[NET] Ethernet nije dostupan");
  }

  // ---------- 4. Wi-Fi AP ----------
  pokreniAP();

  // ---------- 5. NTP ----------
  if (mrezaAktivna) {
    ntpSinkroniziran = sinkronizirajNTP();
    if (ntpSinkroniziran) {
      setenv("TZ", TZ_INFO, 1);
      tzset();
      bootEpoch = (uint32_t)time(nullptr) - (millis() / 1000UL);
      SLOG.print("[NTP] OK, epoch=");
      SLOG.println((uint32_t)time(nullptr));
    } else {
      SLOG.println("[NTP] NIJE uspjela -> MILLIS (vrijeme se moze postaviti na /admin)");
    }
  } else {
    SLOG.println("[NTP] Preskocen (nema mreze)");
  }

  // ---------- 6. CSV datoteka ----------
  if (sdSpreman) {
    sdSpreman = pripremiCSV();
  }
  zapisiDatumRotacije();

  // ---------- 7. Web server (Ethernet ili AP) ----------
  if (mrezaAktivna || postavke.apUkljucen) {
    pokreniWebServer();
  } else {
    SLOG.println("[WEB] Server nije pokrenut (nema mreze)");
  }

  // ---------- 8. OTA ----------
#if OTA_ENABLE
  if (mrezaAktivna || postavke.apUkljucen) {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (strlen(OTA_PASSWORD) > 0) {
      ArduinoOTA.setPassword(OTA_PASSWORD);
    }
    ArduinoOTA.begin();
    SLOG.print("[OTA] Spreman: ");
    SLOG.println(OTA_HOSTNAME);
  }
#endif

  zadnjiPMS          = millis();
  zadnjeUpozorenje   = millis();
  zadnjaRotProvjera  = millis();
}

// ============================== LOOP ==============================

void loop() {
  if (mrezaAktivna || postavke.apUkljucen) {
    server.handleClient();
#if OTA_ENABLE
    ArduinoOTA.handle();
#endif
  }

  // Zakazani restart nakon spremanja mreznih postavki
  if (restartZakazan != 0 && (millis() - restartZakazan) > 2500UL) {
    SLOG.println("[SYS] Restart...");
    delay(50);
    ESP.restart();
  }

  // Dnevna rotacija datoteke (provjera jednom u sekundi)
  uint32_t sadaLoop = millis();
  if ((sadaLoop - zadnjaRotProvjera) >= 1000UL) {
    zadnjaRotProvjera = sadaLoop;
    provjeriRotaciju();
  }

  // Bez delay(): obrada ide odmah kad je pun 32-bajtni okvir dostupan.
  if (Serial2.available() >= 32) {
    if (readPMSdata(&Serial2)) {
      zadnjiPMS = millis();
      obradiUzorak();
    }
  }

  // Upozorenje u serijskom logu ako senzor ne salje podatke
  uint32_t sada = millis();
  if ((sada - zadnjiPMS) >= PMS_SILENCE_WARN_MS &&
      (sada - zadnjeUpozorenje) >= PMS_SILENCE_WARN_MS) {
    zadnjeUpozorenje = sada;
    SLOG.print("[UPOZORENJE] PMS5003 ne salje podatke vec ");
    SLOG.print((sada - zadnjiPMS) / 1000);
    SLOG.println(" s");
  }
}
