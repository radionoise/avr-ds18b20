#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/crc16.h>
#include <stdlib.h>
#include "ds18b20.h"

#define sbi(port,bit) (port) |= (1 << (bit))
#define cbi(port,bit) (port) &= ~(1 << (bit))
#define sbi64Bit(port,bit) (port) |= ((uint64_t) 1 << (bit))
#define cbi64Bit(port,bit) (port) &= ~((uint64_t) 1 << (bit))

#define READ_SCRATCHPAD_ATTEMPTS 10

#define READ_ROM_COMMAND 0x33
#define SEARCH_ROM_COMMAND 0xF0
#define ALARM_SEARCH_COMMAND 0xEC
#define MATCH_ROM_COMMAND 0x55
#define SKIP_ROM_COMMAND 0xCC

#define CONVERT_T_COMMAND 0x44
#define READ_SCRATCHPAD_COMMAND 0xBE
#define WRITE_SCRATCHPAD_COMMAND 0x4E
#define COPY_SCRATCHPAD 0x48
#define RECALL_E2 0xB8
#define READ_POWER_SUPPLY 0xB4

#define INITIALIZED (-1)
#define CONFLICT_SELECTED_0_PATH 0
#define CONFLICT_SELECTED_1_PATH 1

typedef struct Ds18b20ScratchpadFull {
    uint64_t data;
    uint8_t crc;
} Ds18b20ScratchpadFull;

void ds18b20SetInput(Ds18b20Port *port) {
    cbi(*port->avrDdrPort, port->avrPin);
}

void ds18b20SetOutput(Ds18b20Port *port) {
    sbi(*port->avrDdrPort, port->avrPin);
}

void ds18b20SetBit(Ds18b20Port *port) {
    sbi(*port->avrPort, port->avrPin);
}

void ds18b20ClearBit(Ds18b20Port *port) {
    cbi(*port->avrPort, port->avrPin);
}

uint8_t ds18b20ReadBit(Ds18b20Port *port) {
    uint8_t bits = *port->avrPinPort & (1 << port->avrPin);
    return bits >> port->avrPin;
}

bool ds18b20Reset(Ds18b20Port *port) {
    ds18b20SetOutput(port);
    ds18b20SetBit(port);
    _delay_ms(1);
    cli();
    ds18b20ClearBit(port);
    _delay_us(480);
    ds18b20SetInput(port);
    _delay_us(60);
    uint8_t bit0 = ds18b20ReadBit(port);
    _delay_us(200);
    uint8_t bit1 = ds18b20ReadBit(port);
    _delay_us(220);
    sei();

    return (!bit0 && bit1);
}

bool ds18b20IsPresent(Ds18b20Port *port) {
    return ds18b20Reset(port);
}

void ds18b20SendBitData(uint64_t bit, uint8_t index, Ds18b20Port *port) {
    cli();
    if ((bit >> index) & 1) {
        ds18b20ClearBit(port);
        ds18b20SetOutput(port);
        _delay_us(15);
        ds18b20SetBit(port);
        _delay_us(46);
    } else {
        ds18b20ClearBit(port);
        ds18b20SetOutput(port);
        _delay_us(120);
        ds18b20SetBit(port);
        _delay_us(1);
    }
    sei();
}

void ds18b20SendRom(uint64_t rom, Ds18b20Port *port) {
    for (uint8_t index = 0; index < 64; index++) {
        ds18b20SendBitData(rom, index, port);
    }
}

void ds18b20SendCommand(uint8_t command, Ds18b20Port *port) {
    for (uint8_t index = 0; index < 8; index++) {
        ds18b20SendBitData(command, index, port);
    }
}

uint8_t ds18b20ReadBitData(Ds18b20Port *port) {
    cli();
    ds18b20ClearBit(port);
    ds18b20SetOutput(port);
    _delay_us(15);
    ds18b20SetInput(port);
    uint8_t bit = ds18b20ReadBit(port);
    _delay_us(46);
    sei();

    return bit;
}

bool ds18b20CheckRomCrc(uint64_t data) {
    uint8_t calculated = 0;

    for (int index = 0; index < 8; index++) {
        calculated = _crc_ibutton_update(calculated, data >> (8 * index));
    }

    return calculated == 0;
}

uint64_t ds18b20ReadRom(Ds18b20Port *port) {
    ds18b20Reset(port);
    ds18b20SendCommand(READ_ROM_COMMAND, port);
    uint64_t data = 0;

    for (int i = 0; i < 64; i++) {
        uint8_t d = ds18b20ReadBitData(port);
        data = data | ((uint64_t) d << i);
    }

    if (ds18b20CheckRomCrc(data)) {
        return data;
    } else {
        return (uint64_t) DS18B20_ROM_CRC_ERROR;
    }
}

int ds18b20GetMaxIndexOf(int8_t value, int8_t *array, uint8_t arraySize) {
    int maxIndex = -1;
    for (int index = 0; index < arraySize; index++) {
        if (array[index] == value && index > maxIndex) {
            maxIndex = index;
        }
    }

    return maxIndex;
}

void ds18b20ConflictChoose0Path(uint64_t *rom, int8_t *conflictingBits, uint8_t index, Ds18b20Port *port) {
    ds18b20SendBitData(0, 0, port);
    cbi64Bit(*rom, index);
    conflictingBits[index] = CONFLICT_SELECTED_0_PATH;
}

void ds18b20ConflictChoose1Path(uint64_t *rom, int8_t *conflictingBits, uint8_t index, Ds18b20Port *port) {
    ds18b20SendBitData(1, 0, port);
    sbi64Bit(*rom, index);
    conflictingBits[index] = CONFLICT_SELECTED_1_PATH;
}

void ds18b20Search(uint8_t command, uint64_t *buffer, uint8_t bufferSize, Ds18b20Port *port) {
    uint8_t numberOfBits = 64;
    int8_t conflictingBits[numberOfBits];
    for (int index = 0; index < numberOfBits; index++) {
        conflictingBits[index] = INITIALIZED;
    }

    for (uint8_t deviceCount = 0; deviceCount < bufferSize; deviceCount++) {
        ds18b20Reset(port);
        ds18b20SendCommand(command, port);

        uint64_t rom = 0;
        for (uint8_t index = 0; index < numberOfBits; index++) {
            uint8_t bits = 0;
            bits |= ds18b20ReadBitData(port);
            bits |= (ds18b20ReadBitData(port) << 1);

            switch (bits) {
                case 0:
                {
                    int8_t conflictingBit = conflictingBits[index];
                    if (conflictingBit == CONFLICT_SELECTED_0_PATH || conflictingBit == CONFLICT_SELECTED_1_PATH) {
                        int8_t max0PathIndex = ds18b20GetMaxIndexOf(CONFLICT_SELECTED_0_PATH, conflictingBits, numberOfBits);

                        if (conflictingBit == CONFLICT_SELECTED_1_PATH || max0PathIndex == index) {
                            if (max0PathIndex < index) {
                                ds18b20ConflictChoose0Path(&rom, conflictingBits, index, port);
                            } else {
                                ds18b20ConflictChoose1Path(&rom, conflictingBits, index, port);
                            }
                        } else {
                            ds18b20ConflictChoose0Path(&rom, conflictingBits, index, port);
                        }
                    } else {
                        ds18b20ConflictChoose0Path(&rom, conflictingBits, index, port);
                    }

                    break;
                }
                case 1:
                    ds18b20SendBitData(1, 0, port);
                    sbi64Bit(rom, index);
                    break;
                case 2:
                    ds18b20SendBitData(0, 0, port);
                    cbi64Bit(rom, index);
                    break;
                case 3:
                    buffer[deviceCount] = DS18B20_NOT_FOUND_ROM;
            }
        }

        if (ds18b20CheckRomCrc(rom)) {
            buffer[deviceCount] = rom;
        } else {
            buffer[deviceCount] = DS18B20_ROM_CRC_ERROR;
        }
    }
}

void ds18b20SearchRom(uint64_t *buffer, uint8_t bufferSize, Ds18b20Port *port) {
    ds18b20Search(SEARCH_ROM_COMMAND, buffer, bufferSize, port);
}

void ds18b20AlarmSearch(uint64_t *buffer, uint8_t bufferSize, Ds18b20Port *port) {
    ds18b20Search(ALARM_SEARCH_COMMAND, buffer, bufferSize, port);
}

bool ds18b20CheckScratchpadCrc(uint64_t data, uint8_t crc) {
    uint8_t calculatedCrc = 0;

    for (int index = 0; index < 8; index++) {
        calculatedCrc = _crc_ibutton_update(calculatedCrc, data >> (8 * index));
    }

    calculatedCrc = _crc_ibutton_update(calculatedCrc, crc);

    return calculatedCrc == 0;
}

void ds18b20MatchOrSkipRom(uint64_t *rom, Ds18b20Port *port) {
    if (rom == NULL) {
        ds18b20SendCommand(SKIP_ROM_COMMAND, port);
    } else {
        ds18b20SendCommand(MATCH_ROM_COMMAND, port);
        ds18b20SendRom(*rom, port);
    }
}

Ds18b20ScratchpadFull* ds18b20ReadScratchpadFull(uint64_t *rom, Ds18b20Port *port) {
    Ds18b20ScratchpadFull *scratchpad = malloc(sizeof(Ds18b20ScratchpadFull));

    ds18b20Reset(port);
    ds18b20MatchOrSkipRom(rom, port);
    ds18b20SendCommand(READ_SCRATCHPAD_COMMAND, port);

    uint64_t data = 0;
    for (int i = 0; i < 64; i++) {
        data = data | ((uint64_t) ds18b20ReadBitData(port) << i);
    }

    uint8_t crc = 0;
    for (int i = 0; i < 8; i++) {
        crc = crc | (ds18b20ReadBitData(port) << i);
    }

    ds18b20Reset(port);

    scratchpad->data = data;
    scratchpad->crc = crc;

    return scratchpad;
}

int8_t ds18b20Parse8BitTemperature(uint8_t temperature) {
    int8_t result;
    if (temperature >> 7 != 0) {
        result = ((~temperature) + 1) * -1;
    } else {
        result = temperature;
    }

    return result;
}

float ds18b20Parse16BitTemperature(uint16_t temperature) {
    float result;
    if (temperature >> 11 != 0) {
        temperature = (~temperature) + 1;
        result = (float) temperature / 16 * -1;
    } else {
        result = (float) temperature / 16;
    }

    return result;
}

void ds18b20ConvertT(uint64_t *rom, Ds18b20Port *port) {
    ds18b20Reset(port);
    ds18b20MatchOrSkipRom(rom, port);
    ds18b20SendCommand(CONVERT_T_COMMAND, port);
    while (ds18b20ReadBitData(port) == 0) {}
}

void ds18b20ConvertTMatchRom(uint64_t rom, Ds18b20Port *port) {
    ds18b20ConvertT(&rom, port);
}

void ds18b20ConvertTSkipRom(Ds18b20Port *port) {
    ds18b20ConvertT(NULL, port);
}

float ds18b20ReadTemperature(uint64_t *rom, Ds18b20Port *port) {
    uint8_t readAttemptsLeft = READ_SCRATCHPAD_ATTEMPTS;

    while (readAttemptsLeft > 0) {
        ds18b20ConvertT(rom, port);
        Ds18b20ScratchpadFull *scratchpad = ds18b20ReadScratchpadFull(rom, port);

        if (ds18b20CheckScratchpadCrc(scratchpad->data, scratchpad->crc)) {
            float temperature = ds18b20Parse16BitTemperature((uint16_t) scratchpad->data);
            free(scratchpad);
            return temperature;
        } else {
            free(scratchpad);
        }

        readAttemptsLeft--;
    }

    return DS18B20_TEMPERATURE_CRC_ERROR;
}

float ds18b20ReadTemperatureMatchRom(uint64_t rom, Ds18b20Port *port) {
    return ds18b20ReadTemperature(&rom, port);
}

float ds18b20ReadTemperatureSkipRom(Ds18b20Port *port) {
    return ds18b20ReadTemperature(NULL, port);
}

Ds18b20Resolution ds18b20ReadResolution(uint64_t *rom, Ds18b20Port *port) {
    Ds18b20ScratchpadFull *scratchpad = ds18b20ReadScratchpadFull(rom, port);

    uint8_t resolutionBits = (scratchpad->data >> 37) & 3;
    free(scratchpad);
    switch (resolutionBits) {
        case 0:
            return DS18B20_NINE;
        case 1:
            return DS18B20_TEN;
        case 2:
            return DS18B20_ELEVEN;
        case 3:
            return DS18B20_TWELVE;
        default:
            return 0;
    }
}

Ds18b20Scratchpad *ds18b20ReadScratchpad(uint64_t *rom, Ds18b20Port *port) {
    uint8_t readAttemptsLeft = READ_SCRATCHPAD_ATTEMPTS;

    while (readAttemptsLeft > 0) {
        Ds18b20ScratchpadFull *scratchpad = ds18b20ReadScratchpadFull(rom, port);

        if (ds18b20CheckScratchpadCrc(scratchpad->data, scratchpad->crc)) {
            Ds18b20Scratchpad *configuration = malloc(sizeof(Ds18b20Scratchpad));

            uint8_t thByte = scratchpad->data >> 16;
            uint8_t tlByte = scratchpad->data >> 24;

            configuration->highTempThreshold = ds18b20Parse8BitTemperature(thByte);
            configuration->lowTempThreshold = ds18b20Parse8BitTemperature(tlByte);

            Ds18b20Resolution resolution;

            switch ((scratchpad->data >> 37) & 3) {
                case 0:
                    resolution = DS18B20_NINE;
                    break;
                case 1:
                    resolution = DS18B20_TEN;
                    break;
                case 2:
                    resolution = DS18B20_ELEVEN;
                    break;
                case 3:
                    resolution = DS18B20_TWELVE;
                    break;
                default:
                    resolution = 0;
            }

            configuration->resolution = resolution;

            return configuration;
        }

        free(scratchpad);

        readAttemptsLeft--;
    }

    return NULL;
}

Ds18b20Scratchpad *ds18b20ReadScratchpadMatchRom(uint64_t rom, Ds18b20Port *port) {
    return ds18b20ReadScratchpad(&rom, port);
}

Ds18b20Scratchpad *ds18b20ReadScratchpadSkipRom(Ds18b20Port *port) {
    return ds18b20ReadScratchpad(NULL, port);
}

void ds18b20WriteScratchpad(Ds18b20Scratchpad *configuration, uint64_t *rom, Ds18b20Port *port) {
    ds18b20Reset(port);
    ds18b20MatchOrSkipRom(rom, port);
    ds18b20SendCommand(WRITE_SCRATCHPAD_COMMAND, port);

    for (uint8_t index = 0; index < 8; index++) {
        ds18b20SendBitData((uint64_t) configuration->highTempThreshold, index, port);
    }

    for (uint8_t index = 0; index < 8; index++) {
        ds18b20SendBitData((uint64_t) configuration->lowTempThreshold, index, port);
    }

    for (uint8_t index = 0; index < 8; index++) {
        ds18b20SendBitData(0, index, port);
    }

    ds18b20Reset(port);
}

void ds18b20WriteScratchpadMatchRom(Ds18b20Scratchpad *configuration, uint64_t rom, Ds18b20Port *port) {
    ds18b20WriteScratchpad(configuration, &rom, port);
}

void ds18b20WriteScratchpadSkipRom(Ds18b20Scratchpad *configuration, Ds18b20Port *port) {
    ds18b20WriteScratchpad(configuration, NULL, port);
}

void ds18b20CopyScratchpad(uint64_t *rom, Ds18b20Port *port) {
    ds18b20Reset(port);
    ds18b20MatchOrSkipRom(rom, port);
    ds18b20SendCommand(COPY_SCRATCHPAD, port);
    while (ds18b20ReadBitData(port) == 0) {}

    ds18b20Reset(port);
}

void ds18b20CopyScratchpadMatchRom(uint64_t rom, Ds18b20Port *port) {
    ds18b20CopyScratchpad(&rom, port);
}
void ds18b20CopyScratchpadSkipRom(Ds18b20Port *port) {
    ds18b20CopyScratchpad(NULL, port);
}

void ds18b20RecallE2(uint64_t *rom, Ds18b20Port *port) {
    ds18b20Reset(port);
    ds18b20MatchOrSkipRom(rom, port);
    ds18b20SendCommand(RECALL_E2, port);
    while (ds18b20ReadBitData(port) == 0) {}

    ds18b20Reset(port);
}

void ds18b20RecallE2MatchRom(uint64_t *rom, Ds18b20Port *port) {
    ds18b20RecallE2(rom, port);
}

void ds18b20RecallE2SkipRom(Ds18b20Port *port) {
    ds18b20RecallE2(NULL, port);
}

Ds18b20PowerMode ds18b20ReadPowerSupply(uint64_t *rom, Ds18b20Port *port) {
    ds18b20Reset(port);
    ds18b20MatchOrSkipRom(rom, port);
    ds18b20SendCommand(READ_POWER_SUPPLY, port);
    Ds18b20PowerMode powerMode = (Ds18b20PowerMode) ds18b20ReadBitData(port);
    ds18b20Reset(port);

    return powerMode;
}

Ds18b20PowerMode ds18b20ReadPowerSupplyMatchRom(uint64_t rom, Ds18b20Port *port) {
    ds18b20ReadPowerSupply(&rom, port);
}

Ds18b20PowerMode ds18b20ReadPowerSupplySkipRom(Ds18b20Port *port) {
    return ds18b20ReadPowerSupply(NULL, port);
}