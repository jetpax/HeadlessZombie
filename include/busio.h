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

#include <stdint.h>

enum class BusType : uint8_t
{
    MCP2515,
    TIC12400,
    DRV8912,
    UJA1023
};

// Bus IO pin modes 
enum class BusPinMode : uint8_t
{
    HI_Z,
    OUTPUT
};// pin modes 


class BusIo
{
public:
    void Configure( BusType btype, uint8_t pin, BusPinMode pmode);
    
    bool Get();
    void Set();
    void Clear();
    void Toggle();

private:
    uint16_t _pin;
    bool _last=0;
    BusPinMode _mode = BusPinMode::HI_Z;
    BusType _btype = BusType::MCP2515;
};