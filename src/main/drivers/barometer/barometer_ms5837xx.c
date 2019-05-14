/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <build/debug.h>

#include <platform.h>

#include "build/build_config.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/time.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_ms5837xx.h"

#include "drivers/logging.h"

#if defined(USE_BARO_MS583730)

// MS5837xx, Standard address 0x76
#define MS5837xx_ADDR             0x76

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 7

STATIC_UNIT_TESTED uint32_t ms5837xx_ut;  // static result of temperature measurement
STATIC_UNIT_TESTED uint32_t ms5837xx_up;  // static result of pressure measurement
STATIC_UNIT_TESTED uint16_t ms5837xx_c[PROM_NB];  // on-chip ROM
static uint8_t ms5837xx_osr = CMD_ADC_4096;

STATIC_UNIT_TESTED int8_t ms5837xx_crc(uint16_t *prom)
{
    uint8_t crc = ((prom[0] & 0xF000) >> 12);



    uint8_t cnt, n_bit;
    uint16_t n_rem;

    n_rem = 0x00;
    prom[7] = 0;
    prom[0] = (0x0FFF & (prom[0])); // Clear the CRC byte

    for (cnt = 0; cnt < (7 + 1) * 2; cnt++)
    {

        // Get next byte
        if (cnt % 2 == 1)
            n_rem ^= prom[cnt >> 1] & 0x00FF;
        else
            n_rem ^= prom[cnt >> 1] >> 8;

        for (n_bit = 8; n_bit > 0; n_bit--)
        {

            if (n_rem & 0x8000)
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem <<= 1;
        }
    }
    n_rem >>= 12;
    n_rem &= 0x000F;

    crc = 11;

    return !(n_rem == crc);
}

static uint32_t ms5837xx_read_adc(baroDev_t *baro)
{
    uint8_t rxbuf[3];
    busReadBuf(baro->busDev, CMD_ADC_READ, rxbuf, 3);
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static bool ms5837xx_start_ut(baroDev_t *baro)
{
    return busWrite(baro->busDev, CMD_ADC_CONV + CMD_ADC_D2 + ms5837xx_osr, 1);
}

static bool ms5837xx_get_ut(baroDev_t *baro)
{
    ms5837xx_ut = ms5837xx_read_adc(baro);
    return true;
}

static bool ms5837xx_start_up(baroDev_t *baro)
{
    return busWrite(baro->busDev, CMD_ADC_CONV + CMD_ADC_D1 + ms5837xx_osr, 1);
}

static bool ms5837xx_get_up(baroDev_t *baro)
{
    ms5837xx_up = ms5837xx_read_adc(baro);
    return true;
}

STATIC_UNIT_TESTED bool ms583730_calculate(baroDev_t *baro, int32_t *pressure, int32_t *temperature)
{
    UNUSED(baro);
    uint32_t press;
    int64_t temp;
    int64_t delt;
    int64_t dT = (int64_t)ms5837xx_ut - ((uint64_t)ms5837xx_c[5] * 256);
    int64_t off = ((int64_t)ms5837xx_c[2] << 16) + (((int64_t)ms5837xx_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5837xx_c[1] << 15) + (((int64_t)ms5837xx_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5837xx_c[6]) >> 23);

    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    temp -= ((dT * dT) >> 31);
    }
    press = ((((int64_t)ms5837xx_up * sens) >> 21) - off) >> 15;

    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;

    return true;
}

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(busDevice_t * dev)
{
    // for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
    //     uint8_t sig = 0;

    //     delay(10);

    //     bool ack = busRead(dev, CMD_PROM_RD, &sig);
    //     if (ack && sig != 0xFF) {
    //         return true;
    //     }
    // };

    // return false;

    return true;
}

static bool deviceInit(baroDev_t *baro)
{
    busSetSpeed(baro->busDev, BUS_SPEED_STANDARD);
 
    delay(10);

    // busWrite(baro->busDev, CMD_RESET, CMD_RESET);

    busWrite(baro->busDev, 0xFF, CMD_RESET);
    delay(1);
    
    // busWriteBuf(baro->busDev, 0xFF, data, 1);
    // delay(1);
    
    // busWriteBuf(baro->busDev, 0xFF, data, 1);
    // delay(1);
    
    // busWriteBuf(baro->busDev, 0xFF, data, 1);
    // delay(1);
    
    // busWriteBuf(baro->busDev, 0xFF, data, 1);
    // delay(1);
    
    // busWriteBuf(baro->busDev, 0xFF, data, 1);
    // delay(1);
    
    // busWriteBuf(baro->busDev, 0xFF, data, 1);
    // delay(1);
    
    delay(10);

    // read all coefficients
    for (int i = 0; i < PROM_NB; i++) {
        uint8_t rxbuf[2] = { 0, 0 };

        busWrite(baro->busDev, 0xFF, CMD_PROM_RD + i * 2);
        busReadBuf(baro->busDev, 0xFF, rxbuf, 2);
        // busReadBuf(baro->busDev, CMD_PROM_RD + i * 2, rxbuf, 2);
        ms5837xx_c[i] = (rxbuf[0] << 8 | rxbuf[1]);
    }

    addBootlogEvent6(BOOT_EVENT_BARO_DETECTION, BOOT_EVENT_FLAGS_NONE, ms5837xx_c[0], ms5837xx_c[1], ms5837xx_c[2], ms5837xx_c[3]);

    // check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
    if (ms5837xx_crc(ms5837xx_c) != 0) {
        return false;
    }

    baro->ut_delay = 10000;
    baro->up_delay = 10000;
    baro->start_ut = ms5837xx_start_ut;
    baro->get_ut = ms5837xx_get_ut;
    baro->start_up = ms5837xx_start_up;
    baro->get_up = ms5837xx_get_up;

    return true;
}

bool ms583730Detect(baroDev_t *baro)
{
    baro->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_MS583730, 0, OWNER_BARO);
    if (baro->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(baro->busDev)) {
        busDeviceDeInit(baro->busDev);
        return false;
    }

    if (!deviceInit(baro)) {
        busDeviceDeInit(baro->busDev);
        return false;
    }

    baro->calculate = ms583730_calculate;

    return true;
}

#endif
