/*
 * system_state.h
 *
 *  Created on: 29-Jul-2026
 *      Author: sumed
 */

#ifndef INC_SYSTEM_STATE_H_
#define INC_SYSTEM_STATE_H_

typedef enum {
	BOOTING=0,
	SELF_TEST,
	ACTIVE
}system_state_t;

void handle_system_state(void);
void BMP280_SelfTest(void);
extern volatile system_state_t current_system_state;

#endif /* INC_SYSTEM_STATE_H_ */
