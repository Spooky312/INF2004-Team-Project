#ifndef PID_H
#define PID_H

void pid_init(void);
void pid_set_target_rpm(float left_rpm, float right_rpm);
void pid_set_target_heading(float heading);
void pid_controller_run(void);

#endif // PID_H