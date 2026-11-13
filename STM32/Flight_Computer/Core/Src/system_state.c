/*
 * system_state.c
 *
 *  Created on: 29-Jul-2026
 *      Author: sumed
 */
#include "main.h"
#include "led.h"
#include "system_state.h"
#include "system_health.h"

volatile system_state_t current_system_state = BOOTING;
void SystemState_Update(void)
{
	if (current_system_state == BOOTING) current_system_state = SELF_TEST;
	else if (current_system_state == SELF_TEST) current_system_state = READY;
}

void handle_system_state(void) {
	switch(current_system_state)
	{
	case BOOTING:
		led_status_blink_slow();
		break;
	case SELF_TEST:
		led_status_blink_fast();
		break;
	case READY:
	case ARMED:
	case FLIGHT:
	case LANDING:
		led_status_on();
		break;
	case SHUTDOWN:
	case FAULT_STATE:
		led_status_off();
		break;
	default:
		current_health_state = CRITICAL_FAULT;
		break;
	}
}

