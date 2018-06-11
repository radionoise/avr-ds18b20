# DS18B20 library for AVR microcontroller

### Overview
This library was tested on ATmega 16 @ 8 MHz.

### Features
  * Search ROM functionality.
  * Match ROM and Skip ROM functions.
  * Configuration (resolution, TH and TL thresholds) read/write/load from EEPROM/save to EEPROM functionality.

### Example
```
Ds18b20Port port = {&DDRA, &PORTA, &PINA, PIN0};
Ds18b20Port *portPtr = &port;

uint8_t deviceCount = 10;
uint64_t *roms = malloc(deviceCount * sizeof(uint64_t));
ds18b20SearchRom(roms, deviceCount, portPtr);

for (int count = 0; count < deviceCount; count++) {
    if (roms[count] != DS18B20_NOT_FOUND_ROM && roms[count] != DS18B20_ROM_CRC_ERROR) {
        float temperature = ds18b20ReadTemperatureMatchRom(roms[count], portPtr);
        if (temperature != DS18B20_TEMPERATURE_CRC_ERROR) {
            // do something
        }
    }
}

free(roms);
```