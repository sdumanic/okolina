/*
  Test skica: blink + ispis na serijski port

  Ploca:  Olimex ESP32-POE2 (i ESP32-POE)
  FQBN:   esp32:esp32:esp32wrover (PSRAM ukljucen)

  Ponasanje:
    - Blinka na LED_PIN svakih BLINK_MS milisekundi
    - Pri svakoj promjeni ispise stanje i dijagnostiku na serijski port
      (115200 baud, 8N1)

  Napomena o LED pinu:
    Arduino jezgra ne definira LED_BUILTIN za ove ploce, a POE2 nema
    korisnicku LEDicu na ploci. GPIO4 je pin koji Olimex koristi u svom
    sluzbenom blink primjeru i izveden je na EXT konektoru.
    Za vidljiv blink spoji vanjski LED + serijski otpornik (npr. 330R)
    izmedju GPIO4 i GND.

  Rezervirani pinovi (ne dirati):
    GPIO12 PHY power, GPIO0 ETH clock, GPIO18 ETH MDIO, GPIO23 ETH MDC,
    GPIO14/15/2 microSD, GPIO16/17 PSRAM, GPIO1/3 USB-serial
*/

#define LED_PIN          4
#define LED_ACTIVE_HIGH  1
#define SERIAL_BAUD      115200
#define BLINK_MS         500

static uint32_t brojPromjena = 0;
static uint32_t zadnjaPromjena = 0;
static bool     ledStanje = false;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ACTIVE_HIGH ? LOW : HIGH);

  Serial.println();
  Serial.println("=== Olimex ESP32-POE test ===");
  Serial.print("Chip:         "); Serial.println(ESP.getChipModel());
  Serial.print("Cores:        "); Serial.println(ESP.getChipCores());
  Serial.print("CPU freq:     "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz");
  Serial.print("Flash size:   "); Serial.print(ESP.getFlashChipSize() / (1024UL * 1024UL)); Serial.println(" MB");
  Serial.print("PSRAM:        ");
  if (psramFound()) {
    Serial.print(ESP.getPsramSize() / 1024);
    Serial.println(" KB");
  } else {
    Serial.println("nije aktiviran");
  }
  Serial.print("Free heap:    "); Serial.print(ESP.getFreeHeap()); Serial.println(" B");
  Serial.print("SDK:          "); Serial.println(ESP.getSdkVersion());
  Serial.print("LED pin:      GPIO"); Serial.println(LED_PIN);
  Serial.print("Blink period: "); Serial.print(BLINK_MS); Serial.println(" ms");
  Serial.println("---------------------------------");
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
    Serial.print(" ms] promjena #");
    Serial.print(brojPromjena);
    Serial.print("  LED=");
    Serial.print(ledStanje ? "ON " : "OFF");
    Serial.print("  heap=");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" B  uptime=");
    Serial.print(sada / 1000UL);
    Serial.println(" s");
  }
}
