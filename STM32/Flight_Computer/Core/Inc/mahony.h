#ifndef INC_MAHONY_H_
#define INC_MAHONY_H_
typedef struct {
    float q0, q1, q2, q3;
    float roll_deg, pitch_deg, yaw_deg;
} mahony_t;
void Mahony_Init(mahony_t *filter);
void Mahony_Update(mahony_t *filter, float gx, float gy, float gz,
                   float ax, float ay, float az, float mx, float my, float mz,
                   float dt);
#endif
