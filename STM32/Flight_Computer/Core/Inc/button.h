/*
 * button.h
 *
 *  Created on: 29-Jul-2026
 *      Author: sumed
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include <stdint.h>

#define BUTTON_EVENT_STATE (1UL << 0)
#define BUTTON_EVENT_FAULT (1UL << 1)

void GPIO_Fault_Button_Callback(void);
void GPIO_State_Button_Callback(void);

uint32_t GPIO_Button_TakePendingEvents(void);
void GPIO_Button_ProcessEvents(uint32_t events);

#endif /* INC_BUTTON_H_ */
