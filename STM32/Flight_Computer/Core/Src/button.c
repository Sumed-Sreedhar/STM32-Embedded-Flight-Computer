/*
 * button.c
 *
 *  Created on: 29-Jul-2026
 *      Author: sumed
 */

#include "button.h"

#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "system_health.h"
#include "system_state.h"

static volatile uint32_t pending_button_events = 0U;

void GPIO_State_Button_Callback(void)
{
    pending_button_events |= BUTTON_EVENT_STATE;
}

void GPIO_Fault_Button_Callback(void)
{
    pending_button_events |= BUTTON_EVENT_FAULT;
}

uint32_t GPIO_Button_TakePendingEvents(void)
{
    uint32_t events;

    taskENTER_CRITICAL();
    events = pending_button_events;
    pending_button_events = 0U;
    taskEXIT_CRITICAL();

    return events;
}

void GPIO_Button_ProcessEvents(uint32_t events)
{
    if ((events & BUTTON_EVENT_STATE) != 0U)
    {
        switch (current_system_state)
        {
        case BOOTING:
            current_system_state = SELF_TEST;
            led_status_reset();
            break;

        case SELF_TEST:
            current_system_state = ACTIVE;
            led_status_reset();
            break;

        case ACTIVE:
            current_system_state = BOOTING;
            led_status_reset();
            break;

        default:
            SystemHealth_CycleTestFault();
            break;
        }
    }

    if ((events & BUTTON_EVENT_FAULT) != 0U)
    {
        SystemHealth_CycleTestFault();
        led_fault_reset();
    }
}
