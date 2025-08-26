/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Waterpump.c
  * @brief   This file provides code for the configuration
  *          of the Waterpump instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Waterpump.h"
#include "gpio.h"
/* Define WATERPUMP_DUAL_IO (e.g. in project settings or here) to enable dual GPIO control logic */
/* #define WATERPUMP_DUAL_IO */

#ifdef WATERPUMP_DUAL_IO			// Only two IO ports are used for control, no other hardware requirements, and only the IO port needs to add a diode
/*
 * Water pump control implementation.
 * Description:
 *   Uses two GPIO control lines (ON_CTRL / OFF_CTRL) to generate 300 ms pulses
 *   for start and stop operations. Both lines remain low in idle (safe) state.
 */

/* Initialize: enter safe idle state (both control lines low) */
void Water_pump_Init(void)
{
	HAL_GPIO_WritePin(ON_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OFF_CTRL_GPIO_Port, OFF_CTRL_Pin, GPIO_PIN_RESET);
}

/* Start pump: output ON_CTRL pulse (~300 ms) */
void Water_pump_On(void)
{
	HAL_GPIO_WritePin(ON_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OFF_CTRL_GPIO_Port, OFF_CTRL_Pin, GPIO_PIN_RESET);
	HAL_Delay(300); // 0.3s 阻塞延时
	HAL_GPIO_WritePin(ON_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_RESET);
}

/* Stop pump: output OFF_CTRL pulse (~300 ms) */
void Water_pump_Off(void)
{
	HAL_GPIO_WritePin(OFF_CTRL_GPIO_Port, OFF_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ON_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_RESET);
	HAL_Delay(300); // 0.3s 阻塞延时
	HAL_GPIO_WritePin(OFF_CTRL_GPIO_Port, OFF_CTRL_Pin, GPIO_PIN_RESET);
}

#endif /* WATERPUMP_DUAL_IO */

#ifdef WATERPUMP_Single_IO		// Hardware requirements: When the IO port pulls up the level, it can output positive (negative) voltage

/* Single-IO (fallback) stub implementations.
 * Adjust these according to your hardware if only one control line is used.
 * Current behavior: Use ON_CTRL line to emulate start (pulse) and stop (ensure low).
 */
void Water_pump_Init(void)
{
	HAL_GPIO_WritePin(ON_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OFF_CTRL_GPIO_Port, OFF_CTRL_Pin, GPIO_PIN_RESET);
}

void Water_pump_On(void)
{
  HAL_GPIO_WritePin(ON_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_SET);	// Output positive voltage
  HAL_Delay(300);
  HAL_GPIO_WritePin(ON_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_RESET);
}

void Water_pump_Off(void)
{
  HAL_GPIO_WritePin(OFF_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_SET);	// Output negative voltage
  HAL_Delay(300);
  HAL_GPIO_WritePin(OFF_CTRL_GPIO_Port, ON_CTRL_Pin, GPIO_PIN_RESET);
}

#endif /* WATERPUMP_Single_IO */
