/*
 * This file is part of the Headless Zombie project.
 *
 * Copyright (C) 2025 Jonathan Peace <jep@retrovms.com>
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

#include "busio.h"


void BusIo::Configure(BusType btype, uint8_t pin, BusPinMode pmode)
{ 
    _pin = pin;
    
   switch (pmode)
    {
        default:
        case BusPinMode::HI_Z:
            MCP2515_PinEn(pin, false);         
            break;

        case BusPinMode::OUTPUT:
            MCP2515_PinEn(pin, true);         
            break;
    }
}
bool BusIo::Get()    { return _last; }
void BusIo::Set()    { MCP2515_Out_Pin(_pin, true), _last = true; }
void BusIo::Clear()  { MCP2515_Out_Pin(_pin, false), _last = false;  }   
void BusIo::Toggle() { MCP2515_Out_Pin(_pin, _last ? false :true), _last ^= 1; }
