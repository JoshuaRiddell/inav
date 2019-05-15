/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "drivers/time.h"

#include "drivers/logging.h"

#if defined(USE_RANGEFINDER_BRPING)
#include "drivers/rangefinder/rangefinder_brping.h"
#include "io/rangefinder.h"

#define CMD_PRESSURE        0x20
#define CMD_TEMPERATURE     0x21
#define CMD_DISTANCE        0x22
#define CMD_CONFIDENCE      0x23
#define CMD_WHOAMI          0x77
#define WHOAMI_ID           0x69

#define DETECTION_CONE_DECIDEGREES 300
#define DETECTION_CONE_EXTENDED_DECIDEGREES 450

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

static void brpingRangefinderInit(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
}

static void brpingRangefinderUpdate(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
}

static int32_t brpingRangefinderGetDistance(rangefinderDev_t *rangefinder)
{
    uint32_t distance_raw = 0;

    busReadBuf(rangefinder->busDev, CMD_DISTANCE, (uint8_t *)&distance_raw, 4);

    return (int32_t)distance_raw/10;
}

bool brpingDetect(rangefinderDev_t *rangefinder)
{
    rangefinder->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_ARDUINO, 0, OWNER_RANGEFINDER);

    if (rangefinder->busDev == NULL) {
    addBootlogEvent6(BOOT_EVENT_RANGEFINDER_DETECTION, BOOT_EVENT_FLAGS_NONE, 10, 0, 0, 0);
        return false;
    }

    if (!deviceDetect(rangefinder->busDev)) {
    addBootlogEvent6(BOOT_EVENT_RANGEFINDER_DETECTION, BOOT_EVENT_FLAGS_NONE, 11, 0, 0, 0);
        busDeviceDeInit(rangefinder->busDev);
        return false;
    }

    rangefinder->delayMs = RANGEFINDER_BRPING_TASK_PERIOD_MS;
    rangefinder->maxRangeCm = 999;
    rangefinder->detectionConeDeciDegrees = DETECTION_CONE_DECIDEGREES;
    rangefinder->detectionConeExtendedDeciDegrees = DETECTION_CONE_EXTENDED_DECIDEGREES;

    rangefinder->init = &brpingRangefinderInit;
    rangefinder->update = &brpingRangefinderUpdate;
    rangefinder->read = &brpingRangefinderGetDistance;

    return true;
}

#endif
