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

#pragma once

#define SYSLINK_MTU 32

#define CRTP_START_BYTE  0xAA
#define SYSLINK_START_BYTE1 0xBC
#define SYSLINK_START_BYTE2 0xCF

typedef struct syslinkPacket_s
{
  uint8_t type;
  uint8_t length;
  char data[SYSLINK_MTU];
} __attribute__((packed)) syslinkPacket_t;

typedef enum
{
  waitForFirstStart,
  waitForSecondStart,
  waitForType,
  waitForLengt,
  waitForData,
  waitForChksum1,
  waitForChksum2
} syslinkRxState_e;

typedef struct crtpCommander_s
{
    struct{
        uint8_t chan : 2;
        uint8_t link : 2;
        uint8_t port : 4;
    }hdr;
    uint8_t type;
    uint8_t numChannels;
    uint16_t channels[14];

} __attribute__((packed)) crtpCommander_t;
