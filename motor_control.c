/**
 * @file motor_control.c
 * @brief Motor control implementation
 * 
 * IMPROVED VERSION - ISR-safe timing and config.h integration
 */

#include "motor_control.h"
#include "config.h"  // ✓ ADDED
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include <stdlib.h>
#include <math.h>

// PID constants - use config values
#define KP CFG_PID_KP
#define KI CFG_PID_KI
#define KD CFG_PID_KD

// Internal state
static volatile uint32_t pulse_count_left = 0;
static volatile uint32_t pulse_count_right = 0;

// ✓ IMPROVED: Use uint32_t instead of absolute_time_t for ISR safety
static volatile uint32_t last_time_left = 0;
static volatile uint32_t last_time_right = 0;
static volatile uint32_t pulse_period_us_left = 0;
static volatile uint32_t pulse_period_us_right = 0;

static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;

/**
 * @brief Initialize PWM for a motor pin
 */
static void init_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    static bool slice_initialized[NUM_PWM_SLICES] = {false};

    if (!slice_initialized[slice]) {
        pwm_set_wrap(slice, 6249);     // 20 kHz
        pwm_set_clkdiv(slice, 1.0f);
        pwm_set_enabled(slice, true);
        slice_initialized[slice] = true;
    }
}

/**
 * @brief Set individual motor speed
 */
static void set_robo_motor(float speed_percent, uint pinA, uint pinB) {
    uint sliceA = pwm_gpio_to_slice_num(pinA);
    uint chanA  = pwm_gpio_to_channel(pinA);
    uint sliceB = pwm_gpio_to_slice_num(pinB);
    uint chanB  = pwm_gpio_to_channel(pinB);

    uint16_t top = pwm_hw->slice[sliceA].top;

    if (speed_percent > 0) {
        pwm_set_chan_level(sliceA, chanA, (uint16_t)(speed_percent * top));
        pwm_set_chan_level(sliceB, chanB, 0);
    } else if (speed_percent < 0) {
        pwm_set_chan_level(sliceA, chanA, 0);
        pwm_set_chan_level(sliceB, chanB, (uint16_t)(-speed_percent * top));
    } else {
        pwm_set_chan_level(sliceA, chanA, 0);
        pwm_set_chan_level(sliceB, chanB, 0);
    }
}

/**
 * @brief Encoder ISR handler
 * ✓ IMPROVED: Uses ISR-safe time_us_32() instead of get_absolute_time()
 */
static void encoder_isr(uint gpio, uint32_t events) {
    uint32_t now = time_us_32();  // ✓ ISR-safe timing
    
    if (gpio == ENCODER_PIN_L) {
        pulse_count_left++;
        // Calculate period safely (handles overflow)
        uint32_t elapsed = now - last_time_left;
        if (elapsed > 0 && elapsed < 1000000) {  // Sanity check: < 1 second
            pulse_period_us_left = elapsed;
        }
        last_time_left = now;
    } 
    else if (gpio == ENCODER_PIN_R) {
        pulse_count_right++;
        uint32_t elapsed = now - last_time_right;
        if (elapsed > 0 && elapsed < 1000000) {  // Sanity check: < 1 second
            pulse_period_us_right = elapsed;
        }
        last_time_right = now;
    }
}

bool motor_control_init(void) {
    // Initialize PWM for all motor pins
    init_pwm(PWM_M1A);
    init_pwm(PWM_M1B);
    init_pwm(PWM_M2A);
    init_pwm(PWM_M2B);

    // Initialize encoders
    gpio_init(ENCODER_PIN_L);
    gpio_set_dir(ENCODER_PIN_L, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_L);

    gpio_init(ENCODER_PIN_R);
    gpio_set_dir(ENCODER_PIN_R, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_R);

    // Register interrupt handler
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_L, GPIO_IRQ_EDGE_RISE, 
                                       true, &encoder_isr);
    gpio_set_irq_enabled(ENCODER_PIN_R, GPIO_IRQ_EDGE_RISE, true);

    // Initialize ADC for line sensor
    adc_init();
    adc_gpio_init(LINE_ADC_LEFT);
    
    // Note: LINE_ADC_RIGHT removed to avoid conflict with ENCODER_PIN_L
    // If you need dual sensors, use GPIO28 (ADC2) or GPIO29 (ADC3)

    #if CFG_DEBUG_MOTOR
    printf("[MOTOR] Init complete - Pins: M1A=%d M1B=%d M2A=%d M2B=%d\n",
           PWM_M1A, PWM_M1B, PWM_M2A, PWM_M2B);
    printf("[MOTOR] Encoders: L=%d R=%d\n", ENCODER_PIN_L, ENCODER_PIN_R);
    printf("[MOTOR] Line sensor: %d (ADC0)\n", LINE_ADC_LEFT);
    #endif

    return true;
}

void motor_set_state(motor_state_t state, float speed) {
    // Clamp speed to max
    speed = fminf(fabsf(speed), MAX_SPEED);
    
    #if CFG_DEBUG_MOTOR
    printf("[MOTOR] State=%d Speed=%.2f\n", state, speed);
    #endif
    
    switch (state) {
        case MOTOR_STOP:
            motor_stop();
            break;
        case MOTOR_FORWARD:
            motor_forward(speed);
            break;
        case MOTOR_BACKWARD:
            motor_backward(speed);
            break;
        case MOTOR_TURN_LEFT:
            motor_turn_left(speed);
            break;
        case MOTOR_TURN_RIGHT:
            motor_turn_right(speed);
            break;
        case MOTOR_PIVOT_LEFT:
            motor_pivot_left(speed);
            break;
        case MOTOR_PIVOT_RIGHT:
            motor_pivot_right(speed);
            break;
    }
}

void motor_stop(void) {
    set_robo_motor(0.0f, PWM_M1A, PWM_M1B);
    set_robo_motor(0.0f, PWM_M2A, PWM_M2B);
}

void motor_forward(float speed) {
    set_robo_motor(speed, PWM_M1A, PWM_M1B);
    set_robo_motor(speed, PWM_M2A, PWM_M2B);
}

void motor_backward(float speed) {
    set_robo_motor(-speed, PWM_M1A, PWM_M1B);
    set_robo_motor(-speed, PWM_M2A, PWM_M2B);
}

void motor_turn_left(float speed) {
    // Left motor slower, right motor faster
    set_robo_motor(speed * 0.3f, PWM_M1A, PWM_M1B);
    set_robo_motor(speed, PWM_M2A, PWM_M2B);
}

void motor_turn_right(float speed) {
    // Right motor slower, left motor faster
    set_robo_motor(speed, PWM_M1A, PWM_M1B);
    set_robo_motor(speed * 0.3f, PWM_M2A, PWM_M2B);
}

void motor_pivot_left(float speed) {
    // Left motor backward, right motor forward
    set_robo_motor(-speed * 0.5f, PWM_M1A, PWM_M1B);
    set_robo_motor(speed * 0.5f, PWM_M2A, PWM_M2B);
}

void motor_pivot_right(float speed) {
    // Left motor forward, right motor backward
    set_robo_motor(speed * 0.5f, PWM_M1A, PWM_M1B);
    set_robo_motor(-speed * 0.5f, PWM_M2A, PWM_M2B);
}

line_data_t motor_read_line_sensors(void) {
    line_data_t result = {0};
    
    // Read left sensor (only sensor in use)
    adc_select_input(0);  // ADC0 = GPIO26
    result.left_sensor = adc_read();
    
    // Single sensor: simple threshold detection
    // WHITE surface = high ADC value
    // BLACK line = low ADC value
    result.on_line = (result.left_sensor > LINE_THRESHOLD);
    
    // For single sensor, error is simple: on or off
    result.error = result.on_line ? 0 : 100;  // Centered or lost
    
    // Right sensor not used (would conflict with encoder)
    result.right_sensor = 0;
    
    #if CFG_DEBUG_LINE
    printf("[LINE] ADC=%u Threshold=%u OnLine=%d\n",
           result.left_sensor, LINE_THRESHOLD, result.on_line);
    #endif
    
    return result;
}

void motor_line_follow(line_data_t *line_data) {
    if (!line_data->on_line) {
        // Lost the line - stop and search
        motor_stop();
        return;
    }

    // Simple proportional control for single sensor
    // With dual sensors, you'd calculate position error
    float error = (float)line_data->error / 100.0f;
    
    // PID calculation
    pid_integral += error;
    
    // Anti-windup: limit integral
    if (pid_integral > 100.0f) pid_integral = 100.0f;
    if (pid_integral < -100.0f) pid_integral = -100.0f;
    
    float derivative = error - pid_last_error;
    
    float correction = KP * error + KI * pid_integral + KD * derivative;
    pid_last_error = error;
    
    // Limit correction
    correction = fmaxf(-0.5f, fminf(0.5f, correction));
    
    // Apply differential steering
    float left_speed = LINE_FOLLOWING_SPEED - correction;
    float right_speed = LINE_FOLLOWING_SPEED + correction;
    
    // Clamp speeds
    left_speed = fmaxf(0.0f, fminf(MAX_SPEED, left_speed));
    right_speed = fmaxf(0.0f, fminf(MAX_SPEED, right_speed));
    
    set_robo_motor(left_speed, PWM_M1A, PWM_M1B);
    set_robo_motor(right_speed, PWM_M2A, PWM_M2B);
}

motor_telemetry_t motor_get_telemetry(void) {
    motor_telemetry_t telem = {0};
    
    // Calculate RPM from encoder periods
    // ✓ IMPROVED: Added safety checks
    uint32_t period_left = pulse_period_us_left;
    uint32_t period_right = pulse_period_us_right;
    
    if (period_left > 0 && period_left < 1000000) {  // Valid range
        float freq_left = 1e6f / period_left;
        telem.rpm_left = (freq_left * 60.0f) / PULSES_PER_REV;
    }
    
    if (period_right > 0 && period_right < 1000000) {  // Valid range
        float freq_right = 1e6f / period_right;
        telem.rpm_right = (freq_right * 60.0f) / PULSES_PER_REV;
    }
    
    telem.encoder_left = pulse_count_left;
    telem.encoder_right = pulse_count_right;
    
    return telem;
}

void motor_reset_encoders(void) {
    // Disable interrupts while resetting
    uint32_t save = save_and_disable_interrupts();
    
    pulse_count_left = 0;
    pulse_count_right = 0;
    pulse_period_us_left = 0;
    pulse_period_us_right = 0;
    last_time_left = 0;
    last_time_right = 0;
    
    restore_interrupts(save);
    
    #if CFG_DEBUG_MOTOR
    printf("[MOTOR] Encoders reset\n");
    #endif
}