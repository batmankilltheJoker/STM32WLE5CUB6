
#ifndef __WATERPUMP_H__
#define __WATERPUMP_H__
/**
 * !!! WARNING !!!
 * Currently, only 4V bistable pulse solenoid valves are supported
 */

/* Water pump control module API */

//#define WATERPUMP_DUAL_IO
#define WATERPUMP_Single_IO
/*
 * Purpose: Initialize water pump control GPIOs.
 * Behavior: Drive both ON_CTRL and OFF_CTRL low to enter a safe idle state.
 */
void Water_pump_Init(void);

/*
 * Purpose: Generate a start pulse for the water pump.
 * Timing: Set ON_CTRL high (keep OFF_CTRL low) for ~300 ms, then pull ON_CTRL low.
 * Note: Uses blocking HAL_Delay(300). Replace with timer/state machine for non-blocking operation if required.
 */
void Water_pump_On(void);

/*
 * Purpose: Generate a stop pulse for the water pump.
 * Timing: Set OFF_CTRL high (keep ON_CTRL low) for ~300 ms, then pull OFF_CTRL low.
 * Note: Uses blocking HAL_Delay(300). Replace if a non-blocking approach is needed.
 */
void Water_pump_Off(void);

#endif // __WATERPUMP_H__
