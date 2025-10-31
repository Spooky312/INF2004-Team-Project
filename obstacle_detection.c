/**
 * @file obstacle_detection.c
 * @brief Obstacle detection implementation
 */

#include "obstacle_detection.h"
#include "hardware/pwm.h"
#include <math.h>
#include "config.h"

/**
 * @brief Send trigger pulse to ultrasonic sensor
 */
static void send_trig_pulse(void) {
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
}

/**
 * @brief Measure echo pulse duration
 * @return Echo time in microseconds, 0 if timeout
 */
static uint32_t measure_echo(void) {
    absolute_time_t start = get_absolute_time();
    
    // Wait for echo to go HIGH
    while (!gpio_get(ECHO_PIN)) {
        if (absolute_time_diff_us(start, get_absolute_time()) > 30000) {
            return 0; // timeout
        }
    }

    absolute_time_t echo_start = get_absolute_time();
    
    // Wait for echo to go LOW
    while (gpio_get(ECHO_PIN)) {
        if (absolute_time_diff_us(echo_start, get_absolute_time()) > 30000) {
            return 0; // timeout
        }
    }
    
    absolute_time_t echo_end = get_absolute_time();
    return absolute_time_diff_us(echo_start, echo_end);
}

bool obstacle_detection_init(void) {
    // Initialize ultrasonic pins
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Initialize servo PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);

    // 50Hz PWM (20ms period) for servo
    pwm_set_wrap(slice, 20000);
    pwm_set_clkdiv(slice, 125.0f);  // 125MHz / 125 = 1MHz → 1 tick = 1us
    pwm_set_enabled(slice, true);

    // Center the servo
    obstacle_set_servo_angle(SERVO_CENTER);
    sleep_ms(500);  // Let servo reach position

    return true;
}

void obstacle_set_servo_angle(uint8_t angle) {
    if (angle > 180) angle = 180;
    
    // Convert angle to pulse width (1ms-2ms for 0-180°)
    float pulse_width_us = 1000.0f + (angle / 180.0f) * 1000.0f;
    pwm_set_gpio_level(SERVO_PIN, (uint16_t)pulse_width_us);
}

float obstacle_measure_distance(void) {
    send_trig_pulse();
    uint32_t echo_time_us = measure_echo();

    if (echo_time_us == 0) {
        return MAX_DISTANCE_CM;
    }

    // Speed of sound: 343 m/s = 0.0343 cm/us
    // Distance = (time * speed) / 2
    // Simplified: time_us / 58 ≈ distance_cm
    float distance_cm = echo_time_us / 58.0f;
    
    return (distance_cm < MAX_DISTANCE_CM) ? distance_cm : MAX_DISTANCE_CM;
}

void obstacle_look_direction(scan_direction_t direction) {
    switch (direction) {
        case SCAN_LEFT:
            obstacle_set_servo_angle(SERVO_LEFT_MAX);
            break;
        case SCAN_CENTER:
            obstacle_set_servo_angle(SERVO_CENTER);
            break;
        case SCAN_RIGHT:
            obstacle_set_servo_angle(SERVO_RIGHT_MAX);
            break;
    }
    sleep_ms(300);  // Wait for servo to reach position
}

bool obstacle_check_ahead(void) {
    obstacle_set_servo_angle(SERVO_CENTER);
    sleep_ms(200);
    
    float distance = obstacle_measure_distance();
    return (distance < SAFE_DISTANCE_CM);
}

obstacle_scan_t obstacle_perform_scan(void) {
    obstacle_scan_t scan = {0};
    uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Scan center
    obstacle_look_direction(SCAN_CENTER);
    scan.center_distance_cm = obstacle_measure_distance();
    sleep_ms(100);

    // Scan left
    obstacle_look_direction(SCAN_LEFT);
    scan.left_clearance_cm = obstacle_measure_distance();
    sleep_ms(100);

    // Scan right
    obstacle_look_direction(SCAN_RIGHT);
    scan.right_clearance_cm = obstacle_measure_distance();
    sleep_ms(100);

    // Return to center
    obstacle_look_direction(SCAN_CENTER);

    scan.scan_time_ms = to_ms_since_boot(get_absolute_time()) - start_time;

    // Determine if obstacle is present
    scan.obstacle_present = (scan.center_distance_cm < SAFE_DISTANCE_CM);

    // Determine which side is clear
    scan.left_clear = (scan.left_clearance_cm > SAFE_DISTANCE_CM);
    scan.right_clear = (scan.right_clearance_cm > SAFE_DISTANCE_CM);

    // Recommend path
    if (!scan.obstacle_present) {
        scan.recommended_path = SCAN_CENTER;
    } else if (scan.left_clear && scan.right_clear) {
        // Both clear, pick the one with more clearance
        scan.recommended_path = (scan.left_clearance_cm > scan.right_clearance_cm) 
                                ? SCAN_LEFT : SCAN_RIGHT;
    } else if (scan.left_clear) {
        scan.recommended_path = SCAN_LEFT;
    } else if (scan.right_clear) {
        scan.recommended_path = SCAN_RIGHT;
    } else {
        // Both blocked - prefer left
        scan.recommended_path = SCAN_LEFT;
    }

    return scan;
}

float obstacle_measure_width(void) {
    // Start from left, sweep right until we find both edges
    int left_edge_angle = -1;
    int right_edge_angle = -1;
    
    bool was_blocked = false;
    
    // Sweep from left to right
    for (int angle = SERVO_LEFT_MAX; angle <= SERVO_RIGHT_MAX; angle += SERVO_SCAN_STEP) {
        obstacle_set_servo_angle(angle);
        sleep_ms(200);  // Wait for servo and measurement
        
        float distance = obstacle_measure_distance();
        bool is_blocked = (distance < SAFE_DISTANCE_CM);
        
        if (is_blocked && !was_blocked && left_edge_angle == -1) {
            // Found left edge of obstacle
            left_edge_angle = angle;
        }
        
        if (!is_blocked && was_blocked && right_edge_angle == -1) {
            // Found right edge of obstacle
            right_edge_angle = angle;
            break;
        }
        
        was_blocked = is_blocked;
    }
    
    // Return to center
    obstacle_set_servo_angle(SERVO_CENTER);
    
    if (left_edge_angle == -1 || right_edge_angle == -1) {
        return 0.0f;  // Couldn't find edges
    }
    
    // Estimate width based on angle difference
    // This is approximate - actual width depends on distance
    int angle_span = right_edge_angle - left_edge_angle;
    
    // Rough estimate: at 30cm distance, 15° ≈ 8cm
    // width ≈ distance * tan(angle_span)
    float distance = obstacle_measure_distance();
    float width_cm = distance * tanf((angle_span * M_PI) / 180.0f);
    
    return width_cm;
}

obstacle_reading_t obstacle_get_reading(void) {
    obstacle_reading_t reading;
    reading.distance_cm = obstacle_measure_distance();
    reading.obstacle_detected = (reading.distance_cm < SAFE_DISTANCE_CM);
    reading.timestamp_us = time_us_32();
    
    // Determine direction based on current servo position
    // (This would need actual servo position tracking)
    reading.direction = SCAN_CENTER;
    
    return reading;
}
