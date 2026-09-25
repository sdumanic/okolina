/*
  Test sketch: blink + serial output

  Board:  Olimex ESP32-POE2 (and ESP32-POE)
  FQBN:   esp32:esp32:esp32wrover (PSRAM enabled)

  Behaviour:
    - Blinks on LED_PIN every BLINK_MS milliseconds
    - Prints state and diagnostics to the serial port on every change
      (115200 baud, 8N1)

  Note about the LED pin:
    The Arduino core does not define LED_BUILTIN for these boards and the POE2
    has no user LED on the board. GPIO4 is the pin Olimex uses in its official
    blink example and it is available on the EXT connector.
    For a visible blink connect an external LED + series resistor (e.g. 330R)
    between GPIO4 and GND.

  Reserved pins (do not use):
    GPIO12 PHY power, GPIO0 ETH clock, GPIO18 ETH MDIO, GPIO23 ETH MDC,
    GPIO14/15/2 microSD, GPIO16/17 PSRAM, GPIO1/3 USB serial
*/

#define LED_PIN          4
#define LED_ACTIVE_HIGH  1
#define SERIAL_BAUD      115200
#define BLINK_MS         500

static uint32_t changeCount = 0;
static uint32_t lastChange = 0;
static bool     ledState = false;

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
    Serial.println("not enabled");
  }
  Serial.print("Free heap:    "); Serial.print(ESP.getFreeHeap()); Serial.println(" B");
  Serial.print("SDK:          "); Serial.println(ESP.getSdkVersion());
  Serial.print("LED pin:      GPIO"); Serial.println(LED_PIN);
  Serial.print("Blink period: "); Serial.print(BLINK_MS); Serial.println(" ms");
  Serial.println("---------------------------------");
}

void loop() {
  uint32_t now = millis();

  if (now - lastChange >= BLINK_MS) {
    lastChange = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, LED_ACTIVE_HIGH ? ledState : !ledState);
    changeCount++;

    Serial.print("[");
    Serial.print(now);
    Serial.print(" ms] change #");
    Serial.print(changeCount);
    Serial.print("  LED=");
    Serial.print(ledState ? "ON " : "OFF");
    Serial.print("  heap=");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" B  uptime=");
    Serial.print(now / 1000UL);
    Serial.println(" s");
  }
}
