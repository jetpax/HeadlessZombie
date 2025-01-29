/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
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

#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

#define NUM_SAMPLES 12
#ifdef STM32F1
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC  
#else
#define SAMPLE_TIME ADC_SMPR_SMP_15CYC
#endif

#define ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, GPIOA, 1) \
   ANA_IN_ENTRY(throttle2, GPIOA, 1) \
   ANA_IN_ENTRY(uaux,      GPIOA, 1) \
   ANA_IN_ENTRY(GP_analog1,GPIOA, 1) \
   ANA_IN_ENTRY(GP_analog2,GPIOA, 1) \
   ANA_IN_ENTRY(MG1_Temp,  GPIOA, 1) \
   ANA_IN_ENTRY(MG2_Temp,  GPIOA, 1) \
   ANA_IN_ENTRY(dummyAnal, GPIOA, 1) \

//dummyAnal is used by IOMatrix class for unused functions. 
//Must be set to a pin that has no effect
// PA1 corresponds to BATT_VIN which is not connected on Headless
#endif // ANAIN_PRJ_H_INCLUDED
