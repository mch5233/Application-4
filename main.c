/* --------------------------------------------------------------
   Application: Healthcare Heart Rate Monitor
   Release Type: Use of Memory Based Task Communication
   Class: Real Time Systems - Su 2025
   Theme: Healthcare Systems - Heart Rate Monitoring with Emergency Response
   Author: Mya Camacho-Hill
   Email: my062925@ucf.edu
   Company: [University of Central Florida]
   Website: theDRACOlab.com
   AI Use: Structure and synchronization patterns adapted from provided example
   and troubleshooting purposes
---------------------------------------------------------------*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

// Hardware Pin Definitions
#define LED_STATUS_GREEN GPIO_NUM_5    // System status indicator
#define LED_ALERT_RED    GPIO_NUM_4    // Emergency alert indicator
#define EMERGENCY_BUTTON GPIO_NUM_18   // Emergency call button
#define HEART_RATE_ADC_CHANNEL ADC1_CHANNEL_6 // GPIO34 - Heart rate sensor

// System Configuration Constants
#define MAX_EMERGENCY_QUEUE 5          // Maximum queued emergency events
#define HEART_RATE_THRESHOLD_HIGH 3000 // Dangerous heart rate threshold
#define HEART_RATE_THRESHOLD_LOW 500   // Dangerously low heart rate threshold
#define DEBOUNCE_DELAY_MS 50          // Button debounce delay
#define HEART_RATE_SAMPLE_MS 100      // Heart rate sampling frequency
#define HEARTBEAT_BLINK_MS 500        // System heartbeat LED frequency

// FreeRTOS Synchronization Primitives
SemaphoreHandle_t sem_emergency_button;    // Binary semaphore for emergency calls
SemaphoreHandle_t sem_heart_rate_alert;    // Counting semaphore for heart rate events
SemaphoreHandle_t medical_log_mutex;       // Mutex for shared medical logging

// Global System State
volatile int emergency_queue_count = 0;    // Track queued emergency events
volatile int current_heart_rate = 0;       // Current heart rate reading
volatile bool critical_condition = false;  // System critical state flag

/**
 * System Heartbeat Task - Indicates system is operational
 * Priority: 1 (Lowest) - Can be starved to detect blocking issues
 */
void system_heartbeat_task(void *pvParameters) {
    while (1) {
        // Green LED indicates system operational status
        gpio_set_level(LED_STATUS_GREEN, 1);
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_BLINK_MS));
        gpio_set_level(LED_STATUS_GREEN, 0);
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_BLINK_MS));
    }
}

/**
 * Heart Rate Monitor Task - Continuously monitors patient vitals
 * Priority: 2 (Medium) - Critical for patient safety
 */
void heart_rate_monitor_task(void *pvParameters) {
    static bool threshold_alert_active = false;
    
    while (1) {
        // Read heart rate from analog sensor (potentiometer simulating ECG)
        int heart_rate_raw = adc1_get_raw(HEART_RATE_ADC_CHANNEL);
        current_heart_rate = heart_rate_raw;
        
        // Protected logging of heart rate data
        xSemaphoreTake(medical_log_mutex, portMAX_DELAY);
        printf("Heart Rate Monitor: %d BPM (raw: %d)\n", 
               heart_rate_raw / 40, heart_rate_raw); // Convert to simulated BPM
        xSemaphoreGive(medical_log_mutex);
        
        // Check for dangerous heart rate conditions
        bool is_critical = (heart_rate_raw > HEART_RATE_THRESHOLD_HIGH) || 
                          (heart_rate_raw < HEART_RATE_THRESHOLD_LOW);
        
        if (is_critical && !threshold_alert_active) {
            // New critical condition detected
            if (emergency_queue_count < MAX_EMERGENCY_QUEUE) {
                emergency_queue_count++;
            }
            threshold_alert_active = true;
            critical_condition = true;
            
            // Signal medical response team
            xSemaphoreGive(sem_heart_rate_alert);
            
            xSemaphoreTake(medical_log_mutex, portMAX_DELAY);
            if (heart_rate_raw > HEART_RATE_THRESHOLD_HIGH) {
                printf("CRITICAL: Tachycardia detected! Heart rate too high!\n");
            } else {
                printf("CRITICAL: Bradycardia detected! Heart rate too low!\n");
            }
            xSemaphoreGive(medical_log_mutex);
            
        } else if (!is_critical && threshold_alert_active) {
            // Patient condition stabilized
            threshold_alert_active = false;
            critical_condition = false;
            
            xSemaphoreTake(medical_log_mutex, portMAX_DELAY);
            printf("RECOVERY: Heart rate normalized\n");
            xSemaphoreGive(medical_log_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(HEART_RATE_SAMPLE_MS));
    }
}

/**
 * Emergency Button Task - Handles patient/staff emergency calls
 * Priority: 3 (High) - Highest priority for immediate response
 */
void emergency_button_task(void *pvParameters) {
    static TickType_t last_press_time = 0;
    
    while (1) {
        int button_state = gpio_get_level(EMERGENCY_BUTTON);
        TickType_t current_time = xTaskGetTickCount();
        
        // Debounced button press detection
        if (button_state == 0 && 
            (current_time - last_press_time) * portTICK_PERIOD_MS > DEBOUNCE_DELAY_MS) {
            
            last_press_time = current_time;
            
            // Signal emergency response system
            xSemaphoreGive(sem_emergency_button);
            
            xSemaphoreTake(medical_log_mutex, portMAX_DELAY);
            printf("EMERGENCY CALL: Manual emergency button activated!\n");
            xSemaphoreGive(medical_log_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Fast polling for emergency response
    }
}

/**
 * Medical Response Task - Coordinates emergency responses
 * Priority: 2 (Medium) - Handles both automated and manual alerts
 */
void medical_response_task(void *pvParameters) {
    while (1) {
        // Check for heart rate alerts (non-blocking)
        if (xSemaphoreTake(sem_heart_rate_alert, 0) == pdTRUE) {
            emergency_queue_count--;
            
            xSemaphoreTake(medical_log_mutex, portMAX_DELAY);
            printf("MEDICAL RESPONSE: Automated heart rate alert processed\n");
            printf("Patient Status: HR=%d, Critical=%s, Queue=%d\n", 
                   current_heart_rate/40, critical_condition ? "YES" : "NO", emergency_queue_count);
            xSemaphoreGive(medical_log_mutex);
            
            // Alert LED sequence for medical emergency
            gpio_set_level(LED_ALERT_RED, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_ALERT_RED, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_ALERT_RED, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
          
            // BONUS - INDUCED FAILURE: Simulate medical response processing delay
            // This mimics real-world scenarios where medical staff may be busy
            // or medical equipment takes time to respond, potentially causing
            // task starvation of lower-priority tasks like system heartbeat
            //vTaskDelay(pdMS_TO_TICKS(3000)); // STARVATION SIMULATION
            
            gpio_set_level(LED_ALERT_RED, 0);
        }
        
        // Check for emergency button presses (non-blocking)
        if (xSemaphoreTake(sem_emergency_button, 0) == pdTRUE) {
            xSemaphoreTake(medical_log_mutex, portMAX_DELAY);
            printf("EMERGENCY DISPATCH: Manual emergency call processed\n");
            printf("Notifying: Medical staff, Emergency response team\n");
            xSemaphoreGive(medical_log_mutex);
            
            // Quick response LED flash for manual emergency
            gpio_set_level(LED_ALERT_RED, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_ALERT_RED, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_ALERT_RED, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_ALERT_RED, 0);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Minimal delay for responsiveness
    }
}

void app_main(void) { 
    // Configure LED GPIO pins as outputs
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_STATUS_GREEN) | (1ULL << LED_ALERT_RED),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_config);
    
    // Configure emergency button as input with pull-up
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << EMERGENCY_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&button_config);
    
    // Configure ADC for heart rate sensor
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(HEART_RATE_ADC_CHANNEL, ADC_ATTEN_DB_11);
    
    // Create FreeRTOS synchronization primitives
    sem_emergency_button = xSemaphoreCreateBinary();
    sem_heart_rate_alert = xSemaphoreCreateCounting(MAX_EMERGENCY_QUEUE, 0);
    medical_log_mutex = xSemaphoreCreateMutex();
    
    // Create FreeRTOS tasks with appropriate priorities
    // Priority 1: System heartbeat (lowest - can be starved for testing)
    xTaskCreate(system_heartbeat_task, "system_heartbeat", 2048, NULL, 1, NULL);
    
    // Priority 2: Medical monitoring and response (medium)
    xTaskCreate(heart_rate_monitor_task, "heart_rate_monitor", 2048, NULL, 2, NULL);
    xTaskCreate(medical_response_task, "medical_response", 2048, NULL, 2, NULL);
    
    // Priority 3: Emergency button (highest - most critical)
    xTaskCreate(emergency_button_task, "emergency_button", 2048, NULL, 3, NULL);

}
