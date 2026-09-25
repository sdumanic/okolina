/*
  Ethernet test: LAN8720 + DHCP + status on the serial port

  Board:  Olimex ESP32-POE2 (and ESP32-POE)
  FQBN:   esp32:esp32:esp32wrover (PSRAM enabled)

  Behaviour:
    - Blinks on GPIO4 every 500 ms
    - Brings up Ethernet (LAN8720, addr 0, MDC=23, MDIO=18, POWER=12)
    - DHCP: prints IP, netmask, gateway and DNS once obtained
    - Prints status every 5 s; prints every link or IP change immediately

  Important for POE2: the Ethernet clock is GPIO0 (CLK_OUT) because GPIO16/17
  are used by PSRAM. On the ESP32-POE (WROOM) the clock is GPIO17. Therefore
  the values are set explicitly here instead of relying on variant macros.

  Note: the board has no galvanic isolation from PoE power - disconnect the
  Ethernet cable while programming over USB if PoE is present.
*/

#include <ETH.h>

#define ETH_CLK_GPIO0   1      // 1 = POE2 (PSRAM, clock GPIO0), 0 = POE1 (clock GPIO17)

#define LED_PIN          4
#define LED_ACTIVE_HIGH  1
#define SERIAL_BAUD      115200
#define BLINK_MS         500
#define STATUS_MS        5000

static bool      lastLink = false;
static IPAddress lastIp(0, 0, 0, 0);
static uint32_t  blinkCount = 0;
static uint32_t  lastBlink = 0;
static uint32_t  lastStatus = 0;
static bool      ledState = false;

static void printStatus(const char *event) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.print(event);
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
  Serial.println("PHY:       LAN8720  addr=0");
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
    Serial.println("Waiting for link and DHCP...");
  }
  Serial.println();
}

void loop() {
  uint32_t now = millis();

  if (now - lastBlink >= BLINK_MS) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, LED_ACTIVE_HIGH ? ledState : !ledState);
    blinkCount++;
    Serial.print("[");
    Serial.print(now);
    Serial.print(" ms] blink #");
    Serial.print(blinkCount);
    Serial.print("  LED=");
    Serial.print(ledState ? "ON " : "OFF");
    Serial.print("  uptime=");
    Serial.print(now / 1000UL);
    Serial.println(" s");
  }

  bool link = ETH.linkUp();
  if (link != lastLink) {
    lastLink = link;
    printStatus(link ? "LINK UP" : "LINK DOWN");
  }

  IPAddress ip = ETH.localIP();
  if (ip != lastIp) {
    lastIp = ip;
    if (ETH.localIP() != IPAddress(0, 0, 0, 0)) {
      printStatus("IP CHANGE");
      Serial.print("           mask:    "); Serial.println(ETH.subnetMask());
      Serial.print("           gateway: "); Serial.println(ETH.gatewayIP());
      Serial.print("           dns:     "); Serial.println(ETH.dnsIP(0));
    }
  }

  if (now - lastStatus >= STATUS_MS) {
    lastStatus = now;
    printStatus("status");
  }
}
