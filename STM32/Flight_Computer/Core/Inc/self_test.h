/*
 * self_test.h
 *
 *  Created on: 31-Jul-2026
 *      Author: sumed
 */

#ifndef INC_SELF_TEST_H_
#define INC_SELF_TEST_H_

void SelfTest(void);
void SelfTest_PrintResults(void);

typedef enum
{
	SELF_TEST_IDLE = 0,

	SELF_TEST_BMP,
	SELF_TEST_MPU,
	SELF_TEST_MLX,

	SELF_TEST_COMPLETE
}SelfTestState_t;

typedef enum
{
	BMP_OK,
	BMP_READID_FAIL,
	BMP_ID_FAIL,
	BMP_INIT_FAIL,
	BMP_LOAD_FAIL,
	BMP_READ_FAIL
}BMP_TestResult_t;

typedef enum
{
	MPU_OK,
	MPU_READID_FAIL,
	MPU_ID_FAIL,
	MPU_INIT_FAIL,
	MPU_READ_FAIL

}MPU_TestResult_t;

typedef enum
{
	MLX_OK,
	MLX_I2C_FAIL,
	MLX_CONFIG_FAIL

}MLX_TestResult_t;
#endif /* INC_SELF_TEST_H_ */
