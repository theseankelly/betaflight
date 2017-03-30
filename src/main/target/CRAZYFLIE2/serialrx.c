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

// This file implements the Crazyflie "Syslink" protocol. This is a UART
// bridge between the NRF51 MCU and the STM32 MCU. Details on the
// protocol can be found on the Crazyflie wiki:
// https://wiki.bitcraze.io/doc:crazyflie:syslink:index?s[]=syslink
//
// This implementation is a subset of the full protocol targeting Rx
// scenarios. Code is largely adapted from the Crazyflie 2.0 source.

#include <stdbool.h>
#include <stdint.h>

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/serialtarget.h"
#include "syslink.h"


syslinkRxState_e rxState = waitForFirstStart;

static syslinkPacket_t slp;
static uint8_t dataIndex = 0;
static uint8_t cksum[2] = {0};
static uint8_t counter = 0;

static rxRuntimeConfig_t *rxRuntimeConfigPtr;
static serialPort_t *serialPort;

#define SUPPORTED_CHANNEL_COUNT 12
static uint32_t channelData[SUPPORTED_CHANNEL_COUNT];
static bool rcFrameComplete = false;

#define NEEDED_FRAME_INTERVAL 5000

void routeIncommingPacket(syslinkPacket_t* slp)
{
    // Only care about type zero (raw radio)
    if(slp->type == 0)
    {
        crtpCommander_t *commanderPacket = (crtpCommander_t*)(slp->data);

        // Only care about port 7 (generic setpoint) type 2 (RC PWM)
        if( commanderPacket->hdr.port == 0x07 && commanderPacket->type == 0x02)
        {
            // need to assume some bounds for rpy -- -50 to 50? for r, p
            // -400 to 400 for yaw?

            // convert from -50 through 50 to 1000 to 2000

            // Need to scale the range to a 1000 swing, and apply offset to 1500.
            // scale from current swing up to -500 to 500

            // Using RX_CHANNELS_TAER


            // Remap channels 0-3 from AERT (RPYT) to TAER
            channelData[0] = commanderPacket->channels[3];
            channelData[1] = commanderPacket->channels[0];
            channelData[2] = commanderPacket->channels[1];
            channelData[3] = commanderPacket->channels[2];

            // Rest of the channels
            uint8_t i;
            for (i = 4; i < SUPPORTED_CHANNEL_COUNT; i++)
            {
                channelData[i] = commanderPacket->channels[i];
            }

//            // Thrust is a little different -- lets just pass it straight for now
//            // 0 to UINT32_MAX maps from 1000 to 2000.
//            channelData[0] = (commanderPacket->thrust * 1000 / UINT16_MAX) + 1000;
//
//                    //1500;
//
//            // roll
//            channelData[1] = (uint16_t)((commanderPacket->roll / 100 * 1000) + 1500);
//
//            // Pitch -- this is negated
//            channelData[2] = (uint16_t)(-1 * (commanderPacket->pitch / 100 * 1000) + 1500);
//
//            // Yaw
//            channelData[3] = (uint16_t)((commanderPacket->yaw / 800 * 1000) + 1500);



            rcFrameComplete = true;
        }
    }
}

// Receive ISR callback
static void dataReceive(uint16_t c)
{
    counter++;
    switch(rxState)
    {
    case waitForFirstStart:
        rxState = (c == SYSLINK_START_BYTE1) ? waitForSecondStart : waitForFirstStart;
        break;
    case waitForSecondStart:
        rxState = (c == SYSLINK_START_BYTE2) ? waitForType : waitForFirstStart;
        break;
    case waitForType:
        cksum[0] = c;
        cksum[1] = c;
        slp.type = c;
        rxState = waitForLengt;
        break;
    case waitForLengt:
        if (c <= SYSLINK_MTU)
        {
            slp.length = c;
            cksum[0] += c;
            cksum[1] += cksum[0];
            dataIndex = 0;
            rxState = (c > 0) ? waitForData : waitForChksum1;
        }
        else
        {
            rxState = waitForFirstStart;
        }
        break;
    case waitForData:
        slp.data[dataIndex] = c;
        cksum[0] += c;
        cksum[1] += cksum[0];
        dataIndex++;
        if (dataIndex == slp.length)
        {
            rxState = waitForChksum1;
        }
        break;
    case waitForChksum1:
        if (cksum[0] == c)
        {
            rxState = waitForChksum2;
        }
        else
        {
            rxState = waitForFirstStart; //Checksum error
        }
        break;
    case waitForChksum2:
        if (cksum[1] == c)
        {
            routeIncommingPacket(&slp);
        }
        else
        {
            rxState = waitForFirstStart; //Checksum error
        }
        rxState = waitForFirstStart;
        break;
    default:
        break;
    }


//    if (framePosition < SPEK_FRAME_SIZE) {
//        spekFrame[spekFramePosition++] = (uint8_t)c;
//        if (spekFramePosition < SPEK_FRAME_SIZE) {
//            rcFrameComplete = false;
//        } else {
//            rcFrameComplete = true;
//        }
//    }
}



static uint8_t frameStatus(void)
{
    if (!rcFrameComplete) {
        return RX_FRAME_PENDING;
    }

    // Set rcFrameComplete to false so we don't process this one twice
    rcFrameComplete = false;

    return RX_FRAME_COMPLETE;
}

static uint16_t readRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }
    return channelData[chan];
}


bool targetRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfigPtr = rxRuntimeConfig;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    switch (rxConfig->serialrx_provider) {
    case SERIALRX_TARGET:
        break;
    }

    rxRuntimeConfig->channelCount = SUPPORTED_CHANNEL_COUNT;
    rxRuntimeConfig->rxRefreshRate = 10000; // ?? this is from devo
    rxRuntimeConfig->rcReadRawFn = readRawRC;
    rxRuntimeConfig->rcFrameStatusFn = frameStatus;

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        dataReceive,
        1000000,
        MODE_RX,
        SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO
        );

    return serialPort != NULL;
}

