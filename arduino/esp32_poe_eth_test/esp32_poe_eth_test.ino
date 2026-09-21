/*
  Ethernet test: LAN8720 + DHCP + status na serijskom portu

  Ploca:  Olimex ESP32-POE2 (i ESP32-POE)
  FQBN:   esp32:esp32:esp32wrover (PSRAM ukljucen)

  Ponasanje:
    - Blinka na GPIO4 svakih 500 ms
    - Podize Ethernet (LAN8720, addr 0, MDC=23, MDIO=18, POWER=12)
    - DHCP: ispise IP, masku, gateway i DNS kad ih dobije
    - Svakih 5 s ispise status; odmah ispise svaku promjenu linka ili IP-a

  Vazno za POE2: Ethernet clock je GPIO0 (CLK_OUT) jer su GPIO16/17 zauzeti
  PSRAM-om. Na ESP32-POE (WROOM) clock je GPIO17. Zato se ovdje vrijednosti
  zadaju eksplicitno, ne preko makroa varijante.

  Napomena: ploca nema galvansku izolaciju od PoE napajanja - pri
  programiranju preko USB-a iskljuci Ethernet kabel ako je prisutan PoE.
*/

#include <ETH.h>

#define ETH_CLK_GPIO0   1      // 1 = POE2 (PSRAM, clock GPIO0), 0 = POE1 (clock GPIO17)

#define LED_PIN          4
#define LED_ACTIVE_HIGH  1
#define SERIAL_BAUD      115200
#define BLINK_MS         500
#define STATUS_MS        5000

static bool      zadnjiLink = false;
static IPAddress zadnjaIP(0, 0, 0, 0);
static uint32_t  brojPromjena = 0;
static uint32_t  zadnjaPromjena = 0;
static uint32_t  zadnjiStatus = 0;
static bool      ledStanje = false;

static void ispisiStatus(const char *dogadjaj) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.print(dogadjaj);
  Serial.print(" | link=");
  Serial.print(ETH.linkUp() ? "UP" : "DOWN");
  if (ETH.linkUp()) {
    Serial.print(" ");
    Serial.print(ETH.linkSpeed());
    Serial.print("Mb/s ");
    Serial.print(ETH.fullDuplex() ? "full" : "half");
  }
  Serial.print(" | ip=");
  Serial.print(ETH.localIP());
  Serial.print(" | heap=");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" B");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ACTIVE_HIGH ? LOW : HIGH);

  Serial.println();
  Serial.println("=== Olimex ESP32-POE Ethernet test ===");
  Serial.print("Chip:      "); Serial.println(ESP.getChipModel());
  Serial.print("PHY:       LAN8720  addr=0");
  Serial.println();
  Serial.println("MDC:       GPIO23");
  Serial.println("MDIO:      GPIO18");
  Serial.println("PHY power: GPIO12");
#if ETH_CLK_GPIO0
  Serial.println("ETH clock: GPIO0  (POE2 / PSRAM)");
#else
  Serial.println("ETH clock: GPIO17 (POE1 / WROOM)");
#endif
  Serial.println("-----------------------------------------");

#if ETH_CLK_GPIO0
  bool ok = ETH.begin(ETH_PHY_LAN8720, 0, 23, 18, 12, ETH_CLOCK_GPIO0_OUT);
#else
  bool ok = ETH.begin(ETH_PHY_LAN8720, 0, 23, 18, 12, ETH_CLOCK_GPIO17_OUT);
#endif
  Serial.print("ETH.begin(): ");
  Serial.println(ok ? "OK" : "FAILED");

  if (ok) {
    ETH.setHostname("esp32-poe-eth-test");
    Serial.print("MAC:         ");
    Serial.println(ETH.macAddress());
    Serial.println("Cekam link i DHCP...");
  }
  Serial.println();
}

void loop() {
  uint32_t sada = millis();

  if (sada - zadnjaPromjena >= BLINK_MS) {
    zadnjaPromjena = sada;
    ledStanje = !ledStanje;
    digitalWrite(LED_PIN, LED_ACTIVE_HIGH ? ledStanje : !ledStanje);
    brojPromjena++;
    Serial.print("[");
    Serial.print(sada);
    Serial.print(" ms] blink #");
    Serial.print(brojPromjena);
    Serial.print("  LED=");
    Serial.print(ledStanje ? "ON " : "OFF");
    Serial.print("  uptime=");
    Serial.print(sada / 1000UL);
    Serial.println(" s");
  }

  bool link = ETH.linkUp();
  if (link != zadnjiLink) {
    zadnjiLink = link;
    ispisiStatus(link ? "LINK UP" : "LINK DOWN");
  }

  IPAddress ip = ETH.localIP();
  if (ip != zadnjaIP) {
    zadnjaIP = ip;
    if (ETH.localIP() != IPAddress(0, 0, 0, 0)) {
      ispisiStatus("IP PROMJENA");
      Serial.print("           maska:   "); Serial.println(ETH.subnetMask());
      Serial.print("           gateway: "); Serial.println(ETH.gatewayIP());
      Serial.print("           dns:     "); Serial.println(ETH.dnsIP(0));
    }
  }

  if (sada - zadnjiStatus >= STATUS_MS) {
    zadnjiStatus = sada;
    ispisiStatus("status");
  }
}
