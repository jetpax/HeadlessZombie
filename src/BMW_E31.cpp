/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire
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
 #include "BMW_E31.h"
/*
*E31 840CI Tacho:
*1000RPM = 70Hz
*2000RPM = 140Hz
*5000RPM = 345Hz
*6000RPM = 413Hz
*/

//We use this as an init function
void BMW_E31::SetCanInterface(CanHardware* c)
{
   c = c;
<<<<<<< Updated upstream
=======
   tim_setup();//Fire up timer one...
   timer_disable_counter(TIM1);//...but disable until needed
   //note we are trying to reuse the lexus gs450h oil pump pwm output here to drive the tach
   //Be aware this will prevent combo of E31 and GS450H for now ...
   timerIsRunning=false;

   can->RegisterUserMessage(0x153);//ASC message.
//-B0
//-B1 Speed LSB
//-B2 Speed MSB [Signal startbit: 12, Bit length: 12, 0x0008 = 1 km/hr]
//-B3
//-B4
//-B5
//-B6
//-B7

   can->RegisterUserMessage(0x1F0);//ABS message.
//Individual wheel speeds:
//Signal wheel 1: startbit 0, bit length 12, Intel LSB, unsigned, gain 1/16 (0.0625) (byte0 + next 4 bits of Byte1)
//Signal wheel 2: startbit 16, bit length 12, Intel LSB, unsigned, gain 1/16 (0.0625) (byte2 + next 4 bits of byte3)
//Signal wheel 3: startbit 32, bit length 12, Intel LSB, unsigned, gain 1/16 (0.0625)
//Signal wheel 4: startbit 48, bit length 12, Intel LSB, unsigned, gain 1/16 (0.0625)
}


void Bmw_E31::SetRevCounter(int speed)
{

   uint16_t speed_input = speed;
   speed_input = MAX(750, speed_input);//
   speed_input = MIN(7500, speed_input);
   timerPeriod = 30000000 / speed_input; //TODO: find correct factor or make parameter. current gives 52Hz at 750rpm.
   timer_set_period(TIM1, timerPeriod);
   timer_set_oc_value(TIM1, TIM_OC1, timerPeriod / 2); //always stay at 50% duty cycle

}

>>>>>>> Stashed changes



}


 void BMW_E31::Task1Ms()
{
   uint16_t speed_input = speed;

}


 bool BMW_E31::Ready()
{
   return DigIo::t15_digi.Get();
}

bool BMW_E31::Start()
{
   return Param::GetBool(Param::din_start);
}
