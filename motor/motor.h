#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void motor_init(void);
void motor_spin_forward(int speed);
void motor_spin_reverse(int speed);
void motor_stop(void);

#endif // MOTOR_CONTROL_H