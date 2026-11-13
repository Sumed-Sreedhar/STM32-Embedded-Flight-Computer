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
static uint32_t state_button_tick;
static uint32_t fault_button_tick;

void GPIO_State_Button_Callback(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - state_button_tick) < 200U) return;
    state_button_tick = now;
    pending_button_events |= BUTTON_EVENT_STATE;
}

void GPIO_Fault_Button_Callback(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - fault_button_tick) < 200U) return;
    fault_button_tick = now;
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
        case READY:
            current_system_state = ARMED;
            break;
        case ARMED:
            current_system_state = FLIGHT;
            break;
        case FLIGHT:
            current_system_state = LANDING;
            break;
        case LANDING:
            current_system_state = SHUTDOWN;
            break;
        case SHUTDOWN:
        case FAULT_STATE:
            current_system_state = READY;
            break;
        case BOOTING:
        case SELF_TEST:
            current_system_state = READY;
            break;
        }
        led_status_reset();
    }

    if ((events & BUTTON_EVENT_FAULT) != 0U)
    {
        SystemHealth_CycleTestFault();
        led_fault_reset();
    }
}
