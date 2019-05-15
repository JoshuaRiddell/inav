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

#define CMD_PRESSURE        0x20
#define CMD_TEMPERATURE     0x21
#define CMD_WHOAMI          0x77
#define WHOAMI_ID               0x69

static bool ms5837xx_start_ut(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool ms5837xx_get_ut(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool ms5837xx_start_up(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool ms5837xx_get_up(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

STATIC_UNIT_TESTED bool ms583730_calculate(baroDev_t *baro, int32_t *pressure, int32_t *temperature)
{
    bool ack_temp = busReadBuf(baro->busDev, CMD_TEMPERATURE, (uint8_t *)temperature, 4);
    bool ack_pres = busReadBuf(baro->busDev, CMD_PRESSURE, (uint8_t *)pressure, 4);

    return ack_temp && ack_pres;
}

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(busDevice_t * dev)
{
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        uint8_t sig = 0;

        delay(10);

        bool ack = busRead(dev, CMD_WHOAMI, &sig);
        if (ack && sig == WHOAMI_ID) {
            return true;
        }
    };

    return false;
}

bool ms583730Detect(baroDev_t *baro)
{
    baro->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_ARDUINO, 0, OWNER_BARO);
    if (baro->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(baro->busDev)) {
        busDeviceDeInit(baro->busDev);
        return false;
    }

    baro->ut_delay = 10000;
    baro->up_delay = 10000;
    baro->start_ut = ms5837xx_start_ut;
    baro->get_ut = ms5837xx_get_ut;
    baro->start_up = ms5837xx_start_up;
    baro->get_up = ms5837xx_get_up;

    baro->calculate = ms583730_calculate;

    return true;
}

#endif
