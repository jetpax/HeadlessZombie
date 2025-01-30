#ifndef UTILS_H
#define UTILS_H

#include "vehicle.h"
#include "shifter.h"
#include "canhardware.h"
#include "errormessage.h"

namespace utils
{
    inline int32_t change(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    float GetUserThrottleCommand(CanHardware*);
    float ProcessThrottle(int);
    float ProcessUdc(int);
    void CalcSOC();
    void GetDigInputs(CanHardware*);
    void PostErrorIfRunning(ERROR_MESSAGE_NUM);
    void SelectDirection(Vehicle* , Shifter*);
    void displayThrottle();
    void ProcessCruiseControlButtons();
    void CpSpoofOutput();
    void SpeedoSet(uint16_t speed);
    void SpeedoStart();
    #ifndef H_Z
    // void GS450hOilPump(uint16_t pumpdc);
    #endif
}

#endif
