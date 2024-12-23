#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gptimer.h"
#include "esp_http_client.h"

#include <lwip/sockets.h>  // Required for low-level socket API (sendto, socket, etc.)

#define TAG "QUEST_5"

// LEDC Configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_IO_A        (5)  // GPIO for Motor A Enable
#define LEDC_OUTPUT_IO_B        (19) // GPIO for Motor B Enable
#define LEDC_CHANNEL_A          LEDC_CHANNEL_0
#define LEDC_CHANNEL_B          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define LEDC_DUTY               (512) // 50% duty cycle
#define LEDC_FREQUENCY          (5000) // 5 kHz

// Motor A Configuration
#define MOTOR_A_EN_GPIO         (25)
#define MOTOR_A_IN1_GPIO        (22)
#define MOTOR_A_IN2_GPIO        (23)

// Motor B Configuration
#define MOTOR_B_EN_GPIO         (4)
#define MOTOR_B_IN1_GPIO        (21)
#define MOTOR_B_IN2_GPIO        (17)

#define F 1     // Forward
#define B -1    // Backward
// stop is 0

// Define motor states
typedef enum {
    STATE_MOVE_FORWARD,         // F F
    STATE_GO_LEFT,              // 0 F
    STATE_GO_RIGHT,             // F 0
    STATE_MOVE_BACKWARD,        // B B
    STATE_STOP,                 // 0 0
    STATE_SPIN_LEFT,            // B F
    STATE_SPIN_RIGHT,           // F B
    STATE_COUNT                 // Total number of states
} motor_state_t;

// Wi-Fi Configuration
#define WIFI_SSID       "TP-Link_EE49"
#define WIFI_PASSWORD   "34383940"

// UDP Configuration
#define UDP_PORT        8081
#define RX_BUFFER_SIZE  64

// Sharp IR configs
// Define ADC1 Channel 6 (GPIO34)
#define EXAMPLE_ADC1_CHAN   ADC_CHANNEL_6
// Set attenuation to ADC_ATTEN_DB_11 to read up to 3.3V
#define EXAMPLE_ADC_ATTEN   ADC_ATTEN_DB_12 //may need to set back to 11 although 11 is deprecated
static char current_mode = 'M'; // 'M' for Manual, 'A' for Autonomous
float error, derivative, output = 0.0;

// PID configuration
#define Kp 0.75      // Proportional gain
#define Ki 0.0       // Integral gain
#define Kd 0.0       // Derivative gain
float distance_reading = 0.0;
float heading_error = 0.0;

// Timer configuration
#define TIMER_PERIOD_MS 100
#define dt (TIMER_PERIOD_MS / 1000.0)  // Convert to seconds

// Define waypoints for autonomous mode
#define NUM_WAYPOINTS 4
#define WAYPOINT_THRESHOLD 50.0 // Threshold in mm to consider waypoint reached

const double waypoints[NUM_WAYPOINTS][2] = {
    { -728, 730 },
    { 765, 748 },
    { -725, -455 },
    { 745, -467 }
};
static int current_waypoint = 0;

// const double waypoints[NUM_WAYPOINTS][2] = {
//     { -725, -455 },
//     { 765, 748 },
//     { -728, 730 },
//     { 745, -467 }
// };
// static int current_waypoint = 0;

//requests from optitrack
#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR "192.168.0.167"
#endif

// #define ESP_ADDRESS "192.168.0.149"
#define ESP_ADDRESS "192.168.0.127" //robot11

#define PORT 41234

//control statics
// static const char *payload = "ROBOTID 12";
static const char *payload = "ROBOTID 11";
typedef struct {
    int id;
    float x;
    float z;
    float theta;
    char status[10];
} RobotData;
RobotData robot;
void send_to_server(RobotData *robot);

float destination[2];

int parseRobotData(const char *input, RobotData *data) {
    // Parse the input string and populate the RobotData structure
    int parsed = sscanf(input, "%d,%f,%f,%f,%9s", &data->id, &data->x, &data->z, &data->theta, data->status);

    // Check if all fields were successfully parsed
    if (parsed == 5) {
        return 1; // Success
    } else {
        return 0; // Parsing failed
    }
}

void printRobotData(const RobotData* robot) {
    if (robot == NULL) {
        printf("Invalid robot data\n");
        return;
    }

    printf("Robot ID: %d\n", robot->id);
    printf("Position X: %.2f\n", robot->x);
    printf("Position Z: %.2f\n", robot->z);
    printf("Orientation (theta): %.2f\n", robot->theta);
    printf("Status: %s\n", robot->status);
}

// Global Variables
static int server_socket;
static struct sockaddr_in server_addr, client_addr;
static socklen_t addr_len = sizeof(client_addr);
static char rx_buffer[RX_BUFFER_SIZE];  // Buffer to hold incoming data
float measured_value = 0;   // Measured distance in mm
bool timer_flag = false;    // Timer interrupt flag
float previous_error = 0.0; // Previous PID error
float integral = 0.0;       // Integral accumulator
static adc_oneshot_unit_handle_t adc_handle; //adc globals
static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_calibration = false; //end adc globals

//ADC calibration
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid argument or no memory");
    }

    return calibrated;
}

// Initialize LEDC for PWM
static void ledc_init(void)
{
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC channel for Motor A
    ledc_channel_config_t ledc_channel_a = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_A,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_A_EN_GPIO,
        .duty           = 0, // Start with 0% duty
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_a));

    // Configure LEDC channel for Motor B
    ledc_channel_config_t ledc_channel_b = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_B_EN_GPIO,
        .duty           = 0, // Start with 0% duty
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));
}

// Initialize Motor GPIOs
static void motor_init(void)
{
    // Initialize GPIOs for Motor A
    gpio_set_direction(MOTOR_A_IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_A_IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);

    // Initialize GPIOs for Motor B
    gpio_set_direction(MOTOR_B_IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_B_IN1_GPIO, 0);
    gpio_set_level(MOTOR_B_IN2_GPIO, 0);
}

// Set Motor Direction
static void set_motor_direction(int motor, int direction)
{
    if (motor == 0) { // Motor A
        if (direction == F) {
            gpio_set_level(MOTOR_A_IN1_GPIO, 1);
            gpio_set_level(MOTOR_A_IN2_GPIO, 0);
        }
        else if (direction == B) {
            gpio_set_level(MOTOR_A_IN1_GPIO, 0);
            gpio_set_level(MOTOR_A_IN2_GPIO, 1);
        }
        else { // stop
            gpio_set_level(MOTOR_A_IN1_GPIO, 0);
            gpio_set_level(MOTOR_A_IN2_GPIO, 0);
        }
    }
    else if (motor == 1) { // Motor B
        if (direction == F) {
            gpio_set_level(MOTOR_B_IN1_GPIO, 1);
            gpio_set_level(MOTOR_B_IN2_GPIO, 0);
        }
        else if (direction == B) {
            gpio_set_level(MOTOR_B_IN1_GPIO, 0);
            gpio_set_level(MOTOR_B_IN2_GPIO, 1);
        }
        else { // stop
            gpio_set_level(MOTOR_B_IN1_GPIO, 0);
            gpio_set_level(MOTOR_B_IN2_GPIO, 0);
        }
    }
}

// Set Motor Speed using LEDC
static void set_motor_speed(int motor, uint32_t speed)
{
    if (speed > 1023) speed = 1023; // Maximum speed limit

    if (motor == 0) { // Motor A
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, speed));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A));
    }
    else if (motor == 1) { // Motor B
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, speed));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B));
    }
}

// Finite State Machine for Motor Control
static void fsm(motor_state_t state)
{
    switch(state) {
        case STATE_MOVE_FORWARD:
            ESP_LOGI(TAG, "Moving forward...");
            set_motor_direction(0, F);
            set_motor_direction(1, F);
            set_motor_speed(0, 1023);
            set_motor_speed(1, 1023);
            break;

        case STATE_GO_LEFT:
            ESP_LOGI(TAG, "Turning left...");
            set_motor_direction(0, 0);
            set_motor_direction(1, F);
            set_motor_speed(0, 0);
            set_motor_speed(1, 1023);
            break;

        case STATE_GO_RIGHT:
            ESP_LOGI(TAG, "Turning right...");
            set_motor_direction(0, F);
            set_motor_direction(1, 0);
            set_motor_speed(0, 1023);
            set_motor_speed(1, 0);
            break;

        case STATE_MOVE_BACKWARD:
            ESP_LOGI(TAG, "Reversing...");
            set_motor_direction(0, B);
            set_motor_direction(1, B);
            set_motor_speed(0, 1023);
            set_motor_speed(1, 1023);
            break;

        case STATE_STOP:
            ESP_LOGI(TAG, "Stopping both motors...");
            set_motor_direction(0, 0);
            set_motor_direction(1, 0);
            set_motor_speed(0, 0);
            set_motor_speed(1, 0);
            break;

        case STATE_SPIN_LEFT:
            ESP_LOGI(TAG, "Spinning left...");
            set_motor_direction(0, B);
            set_motor_direction(1, F);
            set_motor_speed(0, 1023);
            set_motor_speed(1, 1023);
            break;

        case STATE_SPIN_RIGHT:
            ESP_LOGI(TAG, "Spinning right...");
            set_motor_direction(0, F);
            set_motor_direction(1, B);
            set_motor_speed(0, 1023);
            set_motor_speed(1, 1023);
            break;

        default:
            ESP_LOGI(TAG, "Unknown state. Stopping motors.");
            set_motor_direction(0, 0);
            set_motor_direction(1, 0);
            set_motor_speed(0, 0);
            set_motor_speed(1, 0);
            break;
    }
}

// Wi-Fi Event Handler
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "Wi-Fi started, attempting to connect...");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                {
                    wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                    ESP_LOGI(TAG, "Disconnected from Wi-Fi, reason: %d", event->reason);
                    ESP_LOGI(TAG, "Retrying to connect to Wi-Fi...");
                    esp_wifi_connect();
                    break;
                }
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                {
                    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                    ESP_LOGI(TAG, "Got IP address: %s", ip4addr_ntoa((const ip4_addr_t*)&event->ip_info.ip));
                    break;
                }
            default:
                break;
        }
    }
}

// Initialize Wi-Fi
static void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));  // Correct interface
    ESP_ERROR_CHECK(esp_wifi_start());
}

// UDP Receive Task
static void udp_receive_task(void* pvParameters) {
    static motor_state_t current_state = STATE_STOP; // Default motor state
    ESP_LOGI(TAG, "UDP Receive Task started");

    while (true) {
        // Receive data using recvfrom (blocking)
        int len = recvfrom(server_socket, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr*)&client_addr, &addr_len);
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay before retrying
            continue;
        }

        if (len > 0) {
            rx_buffer[len] = '\0';  // Null terminate the received data
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            // Check if the received data is a destination command
            if (rx_buffer[0] == '[') {
                // Parse the destination values from the format [x,z]
                float x, z;
                if (sscanf(rx_buffer, "[%f,%f]", &x, &z) == 2) {
                    destination[0] = x;
                    destination[1] = z;
                    ESP_LOGI(TAG, "Updated destination: X=%.2f, Z=%.2f", destination[0], destination[1]);
                } else {
                    ESP_LOGW(TAG, "Failed to parse destination from input: %s", rx_buffer);
                }
            } else {
                // Process individual character commands (e.g., WASD, P, etc.)
                for (int i = 0; i < len; i++) {
                    char command = rx_buffer[i]; // Process each character one by one
                    ESP_LOGI(TAG, "Processing command: %c", command);

                    if (command == 'p') {
                        // Toggle between manual and autonomous modes
                        current_mode = (current_mode == 'M') ? 'A' : 'M';
                        ESP_LOGI(TAG, "Mode switched to: %s", current_mode == 'M' ? "Manual" : "Autonomous");

                        // Stop motors when switching modes
                        current_state = STATE_STOP;
                        fsm(current_state);
                    } else if (current_mode == 'M') {
                        // Manual mode: Process WASD commands
                        if (command == 'w') {
                            current_state = STATE_MOVE_FORWARD;
                        } else if (command == 'a') {
                            current_state = STATE_GO_LEFT;
                        } else if (command == 'd') {
                            current_state = STATE_GO_RIGHT;
                        } else if (command == 's') {
                            current_state = STATE_MOVE_BACKWARD;
                        } else if (command == 'x') { // Stop the car
                            current_state = STATE_STOP;
                        } else {
                            ESP_LOGW(TAG, "Unknown command in Manual mode: %c. Stopping motors.", command);
                            current_state = STATE_STOP;
                        }

                        // Update motor state for manual mode
                        fsm(current_state);
                    } else if (current_mode == 'A') {
                        // Autonomous mode: Ignore manual commands
                        ESP_LOGI(TAG, "Autonomous mode active. Command ignored: %c", command);
                        // Autonomous control is handled separately
                    }
                }
            }
        }

        // If in Autonomous mode, handle waypoint navigation
        if (current_mode == 'A') {
            // Set destination to current waypoint
            destination[0] = waypoints[current_waypoint][0];
            destination[1] = waypoints[current_waypoint][1];

            // Calculate distance to destination
            float x_diff = destination[0] - robot.x;
            float z_diff = destination[1] - robot.z;
            float distance_to_destination = sqrt(x_diff * x_diff + z_diff * z_diff);

            ESP_LOGI(TAG, "Distance to waypoint %d: %.2f mm", current_waypoint, distance_to_destination);

            if (distance_to_destination < WAYPOINT_THRESHOLD) {
                ESP_LOGI(TAG, "Reached waypoint %d", current_waypoint);
                // Move to next waypoint
                current_waypoint = (current_waypoint + 1) % NUM_WAYPOINTS;
                ESP_LOGI(TAG, "Next waypoint: %d", current_waypoint);
                ESP_LOGI(TAG, "Next destination: X=%.2f, Z=%.2f", waypoints[current_waypoint][0], waypoints[current_waypoint][1]);
            }
        }

        printf("CURRENT_MODE IS %c \n", current_mode);
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Small delay to avoid overloading the task
    }
}


//timer callback
static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    timer_flag = true;
    return true;
}

// Initialize timer
static void timer_init() {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000  // 1 MHz resolution
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = TIMER_PERIOD_MS * 1000,  // 100ms
        .flags.auto_reload_on_alarm = true
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

//adc init
static void adc_init(void){
    // ADC Unit Initialization
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // ADC Channel Configuration
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, EXAMPLE_ADC1_CHAN, &chan_config));

    // ADC Calibration Initialization
    do_calibration = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN, EXAMPLE_ADC_ATTEN, &adc_cali_handle);
}

static float read_sensor(void) {
    int adc_raw;
    esp_err_t err = adc_oneshot_read(adc_handle, EXAMPLE_ADC1_CHAN, &adc_raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC value: %s", esp_err_to_name(err));
        return -1.0; // Return error value or handle it accordingly
    }

    float voltage_v = 0.0;
    if (do_calibration) {
        int voltage_mv;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv));
        voltage_v = voltage_mv / 1000.0; // Convert mV to V
    } else {
        voltage_v = (float)adc_raw / 4095.0 * 3.3; // Approximation
    }

    // Avoid division by zero
    if (voltage_v > 0.1) {
        return ((56.864 / voltage_v) - 6.562) * 10; // Distance in mm
    }

    return 0.0; // Return 0 if voltage is too low
}

static void orient_cart(int left_speed, int right_speed, float distance_reading){
    ESP_LOGI(TAG, "Distance Reading: %.2f", distance_reading);
    if(distance_reading < 200){
        motor_state_t current_state = STATE_SPIN_RIGHT;
        fsm(current_state);
        vTaskDelay(pdMS_TO_TICKS(500));
        current_state = STATE_MOVE_FORWARD;
        fsm(current_state);
        vTaskDelay(pdMS_TO_TICKS(500));
        current_state = STATE_STOP;
        fsm(current_state);
    }else{
        set_motor_direction(0, F);
        set_motor_direction(1, F);
        set_motor_speed(0, (int)left_speed);  // 0 = left wheel
        set_motor_speed(1, (int)right_speed); // 1 = right wheel
    }
}

// Single-Execution PID Control
static void pid_control() {
    int left_speed = 1023;
    int right_speed = 1023;  
    
    // PID calculations
    error = heading_error;  // Difference between current and target heading
    integral += error * dt;
    derivative = (error - previous_error) / dt;
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // Map PID output to motor speeds
    int change = (int)(25.0 * fabs(output));
    if(output < 0){
        left_speed -= change;
    }else if(output > 0){
        right_speed -= change;
    }
    // Constrain speeds within valid range
    left_speed = left_speed < 0 ? 0 : (left_speed > 1023 ? 1023 : left_speed);
    right_speed = right_speed < 0 ? 0 : (right_speed > 1023 ? 1023 : right_speed);

    // Set motor speeds
    if (current_mode == 'A') {
        orient_cart(left_speed, right_speed, read_sensor());
    }

    // Log PID and motor values
    ESP_LOGI(TAG, "Error: %.2f, Integral: %.2f, Derivative: %.2f, Output: %.2f",
                error, integral, derivative, output);
    ESP_LOGI(TAG, "Motor Speeds: Left = %d, Right = %d", left_speed, right_speed);

}

static void request_location(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }

            if (parseRobotData(rx_buffer, &robot)) {
                printRobotData(&robot);
                // send_to_server(&robot);  // Send parsed data to Flask server

                if (current_mode == 'A') {
                    // Set destination to current waypoint
                    destination[0] = waypoints[current_waypoint][0];
                    destination[1] = waypoints[current_waypoint][1];

                    // Calculate distance to destination
                    float x_diff = destination[0] - robot.x;
                    float z_diff = destination[1] - robot.z;
                    float distance_to_destination = sqrt(x_diff * x_diff + z_diff * z_diff);

                    ESP_LOGI(TAG, "Distance to waypoint %d: %.2f mm", current_waypoint, distance_to_destination);

                    if (distance_to_destination < WAYPOINT_THRESHOLD) {
                        ESP_LOGI(TAG, "Reached waypoint %d", current_waypoint);
                        // Move to next waypoint
                        current_waypoint = (current_waypoint + 1) % NUM_WAYPOINTS;
                        ESP_LOGI(TAG, "Next waypoint: %d", current_waypoint);
                        ESP_LOGI(TAG, "Next destination: X=%.2f, Z=%.2f", waypoints[current_waypoint][0], waypoints[current_waypoint][1]);
                    }
                }

                float curr_heading = robot.theta;
                float x_diff = destination[0] - robot.x;
                float z_diff = destination[1] - robot.z;
                ESP_LOGI(TAG, "Robot parsed: X:%.2f, Z:%.2f, Theta: %.2f,    X_diff, Z_diff: %.2f, %.2f", robot.x, robot.z, robot.theta, x_diff, z_diff);
                ESP_LOGI(TAG, "Robot Destination: x: %.2f, z: %.2f, curr_heading: %.2f", destination[0], destination[1], curr_heading);

                // Calculate heading error
                heading_error = atan2(x_diff, z_diff) * (180.0 / M_PI) - curr_heading;

                // Normalize heading_error to [-180, 180]
                while (heading_error > 180.0) heading_error -= 360.0;
                while (heading_error < -180.0) heading_error += 360.0;

                pid_control();
                
                ESP_LOGI(TAG, "Heading Error: %.2f", heading_error);
            } else {
                ESP_LOGW(TAG, "Failed to parse robot data from input: %s", rx_buffer);
            }

            vTaskDelay(200 / portTICK_PERIOD_MS);  // Delay before retrying

        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


void send_to_server(RobotData *robot) {
    esp_http_client_config_t config = {
        .url = "http://192.168.0.167:5001/update_position",
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    char post_data[256];
    snprintf(post_data, sizeof(post_data), 
             "{\"robot_id\": \"robot_%d\", \"x\": %.2f, \"z\": %.2f, \"theta\": %.2f}",
             robot->id, robot->x, robot->z, robot->theta);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Data sent to server: %s", post_data);
    } else {
        ESP_LOGE(TAG, "Failed to send data: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}


// Application Entry Point
void app_main(void) {
    // Set log level to INFO
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Initializing system...");

    // Initialize Wi-Fi
    wifi_init();

    // Initialize LEDC and Motor GPIOs
    ledc_init();
    motor_init();

    //timer init
    timer_init();

    //ADC init
    adc_init();

    // Create a UDP socket
    server_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (server_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    // Setup the server address structure
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on all available interfaces
    server_addr.sin_port = htons(UDP_PORT);

    // Bind the socket to the UDP port
    int err = bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(server_socket);
        return;
    }
    ESP_LOGI(TAG, "Socket bound to port %d", UDP_PORT);

    // Create a FreeRTOS task for receiving UDP data
    xTaskCreate(udp_receive_task, "UDP Receive Task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "UDP Receive Task created");

    xTaskCreate(request_location, "request_location", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Request Location Task created");
}
