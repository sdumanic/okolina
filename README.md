# ESP32-POE2 — PMS5003 logger s web suceljem i administracijom

Mjerenje krutih cestica (PM1.0 / PM2.5 / PM10) senzorom **PMS5003** na ploci
**Olimex ESP32-POE2**: zapis na ugrađenu microSD karticu, NTP vrijeme,
web stranica s grafom, Wi-Fi pristupna tocka i administracija uredjaja.

## Hardver

| Sklop | Detalji |
|---|---|
| Ploca | Olimex ESP32-POE2 (ESP32-WROVER-E, 4 MB flash, 8 MB PSRAM) |
| Ethernet | LAN8720: PHY addr 0, MDC=GPIO23, MDIO=GPIO18, POWER=GPIO12, CLK=GPIO0 |
| microSD | ugradjeni utor, 1-bit SDMMC: CLK=GPIO14, CMD=GPIO15, D0=GPIO2 |
| PMS5003 | VCC=5V, GND, TX=GPIO33 (RX ploce), RX=GPIO13 (TX ploce, opcionalno) |
| Wi-Fi | softAP (pristupna tocka), zadano `esp32-poe2-pms` |

Pinovi koje ne koristiti: GPIO16/17 (PSRAM), GPIO18/23 (Ethernet), GPIO12
(PHY power), GPIO0 (ETH clock), GPIO14/15/2 (SD), GPIO1/3 (USB serijski),
GPIO34-39 (samo ulaz).

> **Napajanje:** POE2 nema galvansku izolaciju izmedju PoE napajanja i USB-a.
> Pri programiranju preko USB-a iskljuci Ethernet kabel ako se ploca napaja
> preko PoE-a.

## Funkcije

- PMS5003 se cita preko `Serial2` (9600 8N1), 32-bajtni okvir s headerom
  `0x42 0x4D` i checksumom; parser se sam poravnava na header i broji
  prihvacene/odbacene okvire.
- Zapis u CSV **samo kad se PM2.5 ili PM10 promijeni**.
- Datoteka po datumu i vremenu kreiranja: `/pms_YYYYMMDD_HHMMSS.csv`, uz
  **automatsku rotaciju u ponoc** bez restarta ploce.
- Vrijeme: **NTP** ako je mreza dostupna, **RUCNO** ako je vrijeme postavljeno
  u administraciji, inace **MILLIS** (uvijek zapisano u stupcu `time_source`).
- Ethernet (DHCP ili staticni IP) + **Wi-Fi pristupna tocka** (SSID, lozinka,
  kanal); web i OTA rade na obje mreze.
- Web stranica: graf s filtrima raspona (5 min … 24 h + vlastiti `od–do`),
  izvoz grafa u PNG, trenutne vrijednosti, indikator svjezine senzora,
  popis datoteka s preuzimanjem i brisanjem, pomoc s referentnim
  vrijednostima PM i zaglavlje s mreznim postavkama i vremenom pokretanja.
- **Administracija** na `/admin`: mrezne postavke (Ethernet, AP, NTP) i rucno
  postavljanje vremena. Postavke se pamte u NVS (`Preferences`), promjena
  mreze restarta uredjaj.
- **OTA** update preko mreze (ArduinoOTA, hostname `esp32-poe2-pms`).
- Serijski ispis je iskljucen (`SERIAL_DEBUG 0`).

## CSV format

```
timestamp,time_source,pm1_0,pm2_5,pm10
1789733325,NTP,3,7,7
```

`timestamp` je Unix epoch (UTC) kad vrijeme ima epoch (NTP ili RUCNO),
inace `millis()`.

## Web krajnje tocke

| Putanja | Namjena |
|---|---|
| `/` | HTML stranica s grafom |
| `/admin` | administracija (GET prikaz, POST spremanje) |
| `/api/status` | JSON: vrijeme, mreze, zapisi, trenutne vrijednosti, OTA |
| `/api/data?f=` | JSON zapisi za graf |
| `/api/files` | popis datoteka na kartici |
| `/download?f=` | skidanje datoteke |
| `/delete?f=` | brisanje datoteke (aktivna je zasticena) |

## Build i upload

U Arduino jezgri 3.3.11 ne postoji ploca `esp32-poe2`, pa se koristi WROVER
cilj s ukljucenim PSRAM-om:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32wrover arduino/pms5003_poe2_logger
arduino-cli upload  --fqbn esp32:esp32:esp32wrover -p COM4 arduino/pms5003_poe2_logger
```

OTA (bez USB kabela):

```sh
arduino-cli upload --fqbn esp32:esp32:esp32wrover --protocol network \
    -p <IP_ploce> arduino/pms5003_poe2_logger
```

## Skice u repozitoriju

| Skica | Namjena |
|---|---|
| `arduino/pms5003_poe2_logger` | glavna skica: PMS5003 + SD + NTP + web + AP + OTA |
| `arduino/esp32_poe_test` | najosnovniji test: blink + serijski ispis |
| `arduino/esp32_poe_eth_test` | test Etherneta (LAN8720, DHCP) |

## Zadane vrijednosti

| Postavka | Zadano |
|---|---|
| Ethernet | DHCP |
| Wi-Fi AP | ukljucen, `esp32-poe2-pms`, WPA2 `pms5003pms`, kanal 6 |
| NTP server | `pool.ntp.org` |
| Vremenska zona | `CET-1CEST,M3.5.0,M10.5.0/3` (samo za ime datoteke i prikaz) |
