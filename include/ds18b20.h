/*
 * DS18B20 temperature sensor library for AVR.
 */

#ifndef AVR_DS18B20_DS18B20_H
#define AVR_DS18B20_DS18B20_H

#include <avr/io.h>
#include <stdbool.h>

#define DS18B20_NOT_FOUND_ROM 0
#define DS18B20_ROM_CRC_ERROR (0xffffffffffffffff)
#define DS18B20_TEMPERATURE_CRC_ERROR 999

typedef struct Ds18b20Port {
    volatile uint8_t * avrDdrPort;
    volatile uint8_t * avrPort;
    volatile uint8_t * avrPinPort;
    uint8_t avrPin;
} Ds18b20Port;

typedef enum Ds18b20PowerMode {
    DS18B20_PARASITE = 0,
    DS18B20_EXTERNAL = 1
} Ds18b20PowerMode;

typedef enum Ds18b20Resolution {
    DS18B20_NINE = 9,
    DS18B20_TEN = 10,
    DS18B20_ELEVEN = 11,
    DS18B20_TWELVE = 12
} Ds18b20Resolution;

typedef struct Ds18b20Scratchpad {
    Ds18b20Resolution resolution;
    int8_t highTempThreshold;
    int8_t lowTempThreshold;
} Ds18b20Scratchpad;

/*
 * Sends reset pulse and receives presence pulse from connected sensors.
 * Returns true if one or more devices are present on a 1-Wire bus.
 * Otherwise returns false.
 */
bool ds18b20IsPresent(Ds18b20Port *port);

/*
 * Reads unique 64-bit ROM code. This function can only be used if there is a single
 * device on a 1-Wire bus. If more than one device is present, data collision will occur when all devices
 * will try to transmit at the same time.
 */
uint64_t ds18b20ReadRom(Ds18b20Port *port);

/*
 * Searches devices on 1-Wire bus and fills provided buffer with their ROM codes.
 */
void ds18b20SearchRom(uint64_t *buffer, uint8_t bufferSize, Ds18b20Port *port);

/*
 * Searches devices which triggered the alarm on 1-Wire bus and fills provided buffer with their ROM codes.
 */
void ds18b20AlarmSearch(uint64_t *buffer, uint8_t bufferSize, Ds18b20Port *port);

/*
 * Starts temperature conversion on device.
 */
void ds18b20ConvertTMatchRom(uint64_t rom, Ds18b20Port *port);

/*
 * Starts temperature conversion on device.
 * This function can only be used when one device is present on 1-Wire bus.
 */
void ds18b20ConvertTSkipRom(Ds18b20Port *port);

/*
 * Reads temperature from device.
 */
float ds18b20ReadTemperatureMatchRom(uint64_t rom, Ds18b20Port *port);

/*
 * Starts temperature conversion on device.
 * This function can only be used when one device is present on 1-Wire bus.
 */
float ds18b20ReadTemperatureSkipRom(Ds18b20Port *port);

/*
 * Reads device configuration such as resolution, TH and TL alarm thresholds.
 */
Ds18b20Scratchpad *ds18b20ReadScratchpadMatchRom(uint64_t rom, Ds18b20Port *port);

/*
 * Reads device configuration such as resolution, TH and TL alarm thresholds.
 * This function can only be used when one device is present on 1-Wire bus.
 */
Ds18b20Scratchpad *ds18b20ReadScratchpadSkipRom(Ds18b20Port *port);

/*
 * Writes device configuration such as resolution, TH and TL alarm thresholds.
 */
void ds18b20WriteScratchpadMatchRom(Ds18b20Scratchpad *configuration, uint64_t rom, Ds18b20Port *port);

/*
 * Writes device configuration such as resolution, TH and TL alarm thresholds.
 * This function can only be used when one device is present on 1-Wire bus.
 */
void ds18b20WriteScratchpadSkipRom(Ds18b20Scratchpad *configuration, Ds18b20Port *port);

/*
 * Copies device configuration such as resolution, TH and TL alarm thresholds
 * to non-volatile EEPROM.
 */
void ds18b20CopyScratchpadMatchRom(uint64_t rom, Ds18b20Port *port);

/*
 * Copies device configuration such as resolution, TH and TL alarm thresholds
 * to non-volatile EEPROM.
 * This function can only be used when one device is present on 1-Wire bus.
 */
void ds18b20CopyScratchpadSkipRom(Ds18b20Port *port);

/*
 * Loads device configuration such as resolution, TH and TL alarm thresholds
 * from non-volatile EEPROM.
 */
void ds18b20RecallE2MatchRom(uint64_t *rom, Ds18b20Port *port);

/*
 * Loads device configuration such as resolution, TH and TL alarm thresholds
 * from non-volatile EEPROM.
 * This function can only be used when one device is present on 1-Wire bus.
 */
void ds18b20RecallE2SkipRom(Ds18b20Port *port);

/*
 * Reads device power supply mode.
 */
Ds18b20PowerMode ds18b20ReadPowerSupplyMatchRom(uint64_t rom, Ds18b20Port *port);

/*
 * Reads device power supply mode.
 * This function can only be used when one device is present on 1-Wire bus.
 */
Ds18b20PowerMode ds18b20ReadPowerSupplySkipRom(Ds18b20Port *port);

#endif //AVR_DS18B20_DS18B20_H