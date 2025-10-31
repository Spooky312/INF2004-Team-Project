/**
 * @file test_basic.c
 * @brief MINIMAL TEST - Verify hardware works without FreeRTOS
 * 
 * This is a simplified test to check:
 * 1. USB serial output works
 * 2. Motors work
 * 3. Line sensor works
 * 4. Ultrasonic sensor works
 * 5. Servo works
 * 
 * NO FreeRTOS - just simple while loop
 * NO MQTT - no network complexity
 * 
 * Use this to verify hardware before trying full Demo 3
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "config.h"
#include "motor_control.h"
#include "obstacle_detection.h"

// Test mode selection
typedef enum {
    TEST_SERIAL_OUTPUT = 0,
    TEST_MOTORS = 1,
    TEST_LINE_SENSOR = 2,
    TEST_OBSTACLE = 3,
    TEST_FULL = 4
} test_mode_t;

// Change this to select which test to run
#define TEST_MODE TEST_FULL

/**
 * @brief Test 0: Serial output only
 */
void test_serial_output(void) {
    printf("\n=== Test 0: Serial Output ===\n");
    printf("If you can see this, USB serial is working!\n\n");
    
    int counter = 0;
    while (1) {
        printf("Counter: %d (If this increments, everything is OK!)\n", counter++);
        
        // Blink LED as visual confirmation
        gpio_put(25, 1);  // Pico has LED on GPIO 25
        sleep_ms(500);
        gpio_put(25, 0);
        sleep_ms(500);
    }
}

/**
 * @brief Test 1: Motors only
 */
void test_motors(void) {
    printf("\n=== Test 1: Motor Test ===\n");
    printf("Motors will run in sequence...\n");
    
    while (1) {
        printf("\n1. Forward at 50%% for 2 seconds\n");
        motor_forward(0.5f);
        sleep_ms(2000);
        
        printf("2. Stop for 1 second\n");
        motor_stop();
        sleep_ms(1000);
        
        printf("3. Backward at 50%% for 2 seconds\n");
        motor_backward(0.5f);
        sleep_ms(2000);
        
        printf("4. Stop for 1 second\n");
        motor_stop();
        sleep_ms(1000);
        
        printf("5. Turn left for 1 second\n");
        motor_turn_left(0.5f);
        sleep_ms(1000);
        
        printf("6. Turn right for 1 second\n");
        motor_turn_right(0.5f);
        sleep_ms(1000);
        
        motor_stop();
        printf("7. Stop for 2 seconds before repeating\n");
        sleep_ms(2000);
        
        // Get telemetry
        motor_telemetry_t telem = motor_get_telemetry();
        printf("Encoders: L=%lu R=%lu | RPM: L=%.1f R=%.1f\n",
               telem.encoder_left, telem.encoder_right,
               telem.rpm_left, telem.rpm_right);
    }
}

/**
 * @brief Test 2: Line sensor only
 */
void test_line_sensor(void) {
    printf("\n=== Test 2: Line Sensor Test ===\n");
    printf("Reading line sensor continuously...\n");
    printf("Move sensor over white/black surfaces\n\n");
    
    while (1) {
        line_data_t data = motor_read_line_sensors();
        
        printf("ADC=%4u | Threshold=%u | OnLine=%s | Error=%4d\n",
               data.left_sensor,
               LINE_THRESHOLD,
               data.on_line ? "YES" : "NO ",
               data.error);
        
        sleep_ms(200);
    }
}

/**
 * @brief Test 3: Obstacle detection only
 */
void test_obstacle(void) {
    printf("\n=== Test 3: Obstacle Detection Test ===\n");
    printf("Testing ultrasonic sensor and servo...\n\n");
    
    printf("1. Testing servo sweep...\n");
    obstacle_set_servo_angle(45);   // Left
    printf("   Servo at 45° (left)\n");
    sleep_ms(1000);
    
    obstacle_set_servo_angle(90);   // Center
    printf("   Servo at 90° (center)\n");
    sleep_ms(1000);
    
    obstacle_set_servo_angle(135);  // Right
    printf("   Servo at 135° (right)\n");
    sleep_ms(1000);
    
    obstacle_set_servo_angle(90);   // Center
    printf("   Servo back to center\n");
    sleep_ms(1000);
    
    printf("\n2. Reading distance continuously...\n");
    printf("   (Move hand in front of sensor)\n\n");
    
    while (1) {
        float distance = obstacle_measure_distance();
        bool obstacle_present = (distance < SAFE_DISTANCE_CM);
        
        printf("Distance: %6.1f cm | Obstacle: %s\n",
               distance,
               obstacle_present ? "YES ⚠️" : "NO");
        
        sleep_ms(300);
    }
}

/**
 * @brief Test 4: Full integration without FreeRTOS
 */
void test_full(void) {
    printf("\n=== Test 4: Full Integration (No FreeRTOS) ===\n");
    printf("Simple line following with obstacle avoidance\n\n");
    
    typedef enum {
        STATE_FOLLOWING,
        STATE_OBSTACLE_AVOID
    } simple_state_t;
    
    simple_state_t state = STATE_FOLLOWING;
    uint32_t last_obstacle_check = 0;
    
    while (1) {
        // Read line sensor
        line_data_t line_data = motor_read_line_sensors();
        
        // Get telemetry
        motor_telemetry_t telem = motor_get_telemetry();
        
        // Check for obstacle every 500ms
        uint32_t now = to_ms_since_boot(get_absolute_time());
        bool check_obstacle = (now - last_obstacle_check > 500);
        
        switch (state) {
            case STATE_FOLLOWING:
                if (line_data.on_line) {
                    // Follow line
                    motor_line_follow(&line_data);
                    printf("[FOLLOW] ADC=%u Error=%d RPM: L=%.1f R=%.1f\n",
                           line_data.left_sensor, line_data.error,
                           telem.rpm_left, telem.rpm_right);
                } else {
                    // Lost line
                    motor_stop();
                    printf("[FOLLOW] ✗ Line lost\n");
                }
                
                // Check for obstacles
                if (check_obstacle) {
                    last_obstacle_check = now;
                    if (obstacle_check_ahead()) {
                        printf("\n[OBSTACLE] ⚠️ Detected! Stopping...\n");
                        motor_stop();
                        sleep_ms(500);
                        state = STATE_OBSTACLE_AVOID;
                    }
                }
                break;
                
            case STATE_OBSTACLE_AVOID:
                printf("[OBSTACLE] Scanning...\n");
                obstacle_scan_t scan = obstacle_perform_scan();
                
                printf("[SCAN] L=%.1f C=%.1f R=%.1f\n",
                       scan.left_clearance_cm,
                       scan.center_distance_cm,
                       scan.right_clearance_cm);
                
                if (scan.recommended_path == SCAN_LEFT) {
                    printf("[AVOID] Going LEFT\n");
                    motor_pivot_left(0.4f);
                    sleep_ms(800);
                    motor_forward(0.5f);
                    sleep_ms(1500);
                    motor_pivot_right(0.4f);
                    sleep_ms(800);
                } else {
                    printf("[AVOID] Going RIGHT\n");
                    motor_pivot_right(0.4f);
                    sleep_ms(800);
                    motor_forward(0.5f);
                    sleep_ms(1500);
                    motor_pivot_left(0.4f);
                    sleep_ms(800);
                }
                
                motor_stop();
                printf("[AVOID] Complete, resuming line follow\n\n");
                state = STATE_FOLLOWING;
                break;
        }
        
        sleep_ms(50);  // Main loop rate
    }
}

/**
 * @brief Main
 */
int main(void) {
    // Initialize GPIO for LED (Pico has LED on GPIO 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(3000);  // IMPORTANT: Wait for USB
    
    printf("\n\n");
    printf("═══════════════════════════════════════════════════════\n");
    printf("  HARDWARE TEST PROGRAM (No FreeRTOS)\n");
    printf("═══════════════════════════════════════════════════════\n\n");
    
    printf("Current Test Mode: ");
    switch (TEST_MODE) {
        case TEST_SERIAL_OUTPUT:
            printf("SERIAL OUTPUT ONLY\n\n");
            // No hardware needed
            test_serial_output();
            break;
            
        case TEST_MOTORS:
            printf("MOTOR TEST\n\n");
            printf("Initializing motor control...\n");
            if (!motor_control_init()) {
                printf("ERROR: Motor init failed!\n");
                while(1) {
                    gpio_put(25, 1);
                    sleep_ms(100);
                    gpio_put(25, 0);
                    sleep_ms(100);
                }
            }
            printf("✓ Motor control ready\n");
            test_motors();
            break;
            
        case TEST_LINE_SENSOR:
            printf("LINE SENSOR TEST\n\n");
            printf("Initializing motor control (for ADC)...\n");
            if (!motor_control_init()) {
                printf("ERROR: Motor init failed!\n");
                while(1) {
                    gpio_put(25, 1);
                    sleep_ms(100);
                    gpio_put(25, 0);
                    sleep_ms(100);
                }
            }
            printf("✓ Line sensor ready\n");
            test_line_sensor();
            break;
            
        case TEST_OBSTACLE:
            printf("OBSTACLE DETECTION TEST\n\n");
            printf("Initializing obstacle detection...\n");
            if (!obstacle_detection_init()) {
                printf("ERROR: Obstacle init failed!\n");
                while(1) {
                    gpio_put(25, 1);
                    sleep_ms(100);
                    gpio_put(25, 0);
                    sleep_ms(100);
                }
            }
            printf("✓ Obstacle detection ready\n");
            test_obstacle();
            break;
            
        case TEST_FULL:
            printf("FULL INTEGRATION TEST\n\n");
            printf("Initializing all systems...\n");
            
            if (!motor_control_init()) {
                printf("ERROR: Motor init failed!\n");
                while(1) {
                    gpio_put(25, 1);
                    sleep_ms(100);
                    gpio_put(25, 0);
                    sleep_ms(100);
                }
            }
            printf("✓ Motor control ready\n");
            
            if (!obstacle_detection_init()) {
                printf("ERROR: Obstacle init failed!\n");
                while(1) {
                    gpio_put(25, 1);
                    sleep_ms(100);
                    gpio_put(25, 0);
                    sleep_ms(100);
                }
            }
            printf("✓ Obstacle detection ready\n");
            
            printf("\n*** ALL SYSTEMS READY ***\n\n");
            test_full();
            break;
    }
    
    // Should never reach here
    printf("Test ended unexpectedly\n");
    while (1) {
        tight_loop_contents();
    }
    
    return 0;
}