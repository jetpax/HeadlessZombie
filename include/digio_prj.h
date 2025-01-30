/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
 * Changes by Tom de Bree <tom@voltinflux.com> 2024
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

#define DIG_IO_LIST \
    DIG_IO_ENTRY(HV_req,    GPIOE, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(start_in,  GPIOE, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(brake_in,  GPIOE, GPIO7, PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(fwd_in,    GPIOE, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(rev_in,    GPIOE, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(dcsw_out,  GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,   GPIOA, GPIO15,  PinMode::OUTPUT)     \
    DIG_IO_ENTRY(gp_out1,   GPIOE, GPIO7, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out2,   GPIOE, GPIO7, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out3,   GPIOE, GPIO7, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(sw_mode0,  GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(sw_mode1,  GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(lin_wake,  GPIOE, GPIO7, PinMode::OUTPUT)      \
    BUS_IO_ENTRY(lin_nslp, BusType::MCP2515, 0x01, BusPinMode::OUTPUT) \
    DIG_IO_ENTRY(prec_out,  GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(inv_out,   GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SL1_out,   GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SL2_out,   GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SP_out,    GPIOE, GPIO7, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gear1_in,  GPIOE, GPIO7,  PinMode::INPUT_FLT_INV)   \
    DIG_IO_ENTRY(gear2_in,  GPIOE, GPIO7,  PinMode::INPUT_FLT_INV)   \
    DIG_IO_ENTRY(gear3_in,  GPIOE, GPIO7,  PinMode::INPUT_FLT_INV)   \
    DIG_IO_ENTRY(req_out,   GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(pot1_cs,   GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(pot2_cs,   GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(mcp_sby,   GPIOE, GPIO7, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(PWM3,      GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(PWM2,      GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(PWM1,      GPIOE, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(t15_digi,  GPIOE, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(gp_12Vin,  GPIOE, GPIO7,  PinMode::INPUT_FLT)   \
    BUS_IO_ENTRY(can2_term, BusType::MCP2515, 0x00, BusPinMode::OUTPUT) \
    DIG_IO_ENTRY(dummypin,  GPIOE, GPIO7,  PinMode::INPUT_PD)   \

//dummypin is used by IOMatrix class for unused functions. Must be set to a pin that has no effect

#endif // PinMode_PRJ_H_INCLUDED
