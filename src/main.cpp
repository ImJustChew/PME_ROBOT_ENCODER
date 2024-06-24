#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

//pins 34-39 cannot use for pwm
// Define motor and encoder pins for one motor
#define IN1_1 23
#define IN1_2 22
#define DC1_1 35
#define DC1_2 34

#define IN2_1 32  // PWM_1 pin for motor 2
#define IN2_2 33  // PWM_2 pin for motor 2
#define DC2_1 21  // Encoder pin A for motor 2
#define DC2_2 19  // Encoder pin B for motor 2

#define IN3_1 25  // PWM_1 pin for motor 3
#define IN3_2 26  // PWM_2 pin for motor 3
#define DC3_1 18  // Encoder pin A for motor 3
#define DC3_2 5   // Encoder pin B for motor 3

#define IN4_1 27  // PWM_1 pin for motor 4
#define IN4_2 15  // PWM_2 pin for motor 4
#define DC4_1 4   // Encoder pin A for motor 4
#define DC4_2 2   // Encoder pin B for motor 4

// Define PWM channels
#define PWM_CHANNEL_1_1 0
#define PWM_CHANNEL_1_2 1

#define PWM_CHANNEL_2_1 2
#define PWM_CHANNEL_2_2 3

#define PWM_CHANNEL_3_1 4
#define PWM_CHANNEL_3_2 5

#define PWM_CHANNEL_4_1 6
#define PWM_CHANNEL_4_2 7

// variables
double control_period = 0.01;


// speed of 255 is already max.

volatile double input1, output1, setpoint1 = 0, kp1 = 0, ki1 = 0, kd1 = 0; 
volatile double last_integral1 = 0, rps_before1 = 0.0;  // MOTOR 1
int encoder_resolution1 = 512;
double speed_reduction_ratio1 = 1;

volatile double input2, output2, setpoint2 = 0, kp2 = 0, ki2 = 0, kd2 = 0;
volatile double last_integral2 = 0, rps_before2 = 0.0;  // MOTOR 2
int encoder_resolution2 = 512;
double speed_reduction_ratio2 = 1;

volatile double input3, output3, setpoint3 = 0, kp3 = 0, ki3 = 0, kd3 = 0; 
volatile double last_integral3 = 0, rps_before3 = 0.0;  // MOTOR 3
int encoder_resolution3 = 512;
double speed_reduction_ratio3 = 1;

volatile double input4, output4, setpoint4 = 0, kp4 = 0, ki4 = 0, kd4 = 0;
volatile double last_integral4 = 0, rps_before4 = 0.0;  // MOTOR 4
int encoder_resolution4 = 512;
double speed_reduction_ratio4 = 1;

int pwm_resolution = 255;

// Encoder objects
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;


// Speed variables
volatile double speed1 = 0;
volatile double speed2 = 0;
volatile double speed3 = 0;
volatile double speed4 = 0;


void controlMotor(int output, int pwmChannel1, int pwmChannel2) {
    if (output > 0) {
        ledcWrite(pwmChannel1, (int)output);
        ledcWrite(pwmChannel2, 0);
    } else {
        ledcWrite(pwmChannel1, 0);
        ledcWrite(pwmChannel2, (int)-output);
    }
}

const int TASK_FREQ_MS = control_period * 1000;

void getEncoderSpeed(void *pvParameters) {
    ESP32Encoder* encoder = (ESP32Encoder*)pvParameters;
    volatile double* speed = NULL;

    if (encoder == &encoder1) speed = &speed1;
    else if (encoder == &encoder2) speed = &speed2;
    else if (encoder == &encoder3) speed = &speed3;
    else if (encoder == &encoder4) speed = &speed4;
    for (;;) {
        uint64_t start = esp_timer_get_time();
        double count = encoder->getCount();
        encoder->clearCount();
        *speed = (double) count / TASK_FREQ_MS;
        if(encoder == &encoder1){
            input1 = *speed;
            double rps = count / (2 * encoder_resolution1 * speed_reduction_ratio1 * control_period);

            double err = setpoint1 - rps;
            double propotional = kp1 * err; 
            last_integral1 += ki1 * err * control_period;
            double differential =  kd1 * (rps-rps_before1) / control_period;
            double pid_duty = propotional + last_integral1 + differential;
            pid_duty = (pid_duty > 1) ? 1.0 : pid_duty;
            pid_duty = (pid_duty < -1) ? -1.0 : pid_duty;
            rps_before1 = rps;
            controlMotor(pid_duty*pwm_resolution, PWM_CHANNEL_1_1, PWM_CHANNEL_1_2);
            output1 = pid_duty*pwm_resolution;
        }
        else if(encoder == &encoder2){
            input2 = *speed;
            double rps = count / (2 * encoder_resolution2 * speed_reduction_ratio2 * control_period);

            double err = setpoint2 - rps;
            double propotional = kp2 * err;
            last_integral2 += ki2 * err * control_period;
            double differential =  kd2 * (rps-rps_before2) / control_period;
            double pid_duty = propotional + last_integral2 + differential;
            pid_duty = (pid_duty > 1) ? 1.0 : pid_duty;
            pid_duty = (pid_duty < -1) ? -1.0 : pid_duty;
            rps_before2 = rps;
            controlMotor(pid_duty*pwm_resolution, PWM_CHANNEL_2_1, PWM_CHANNEL_2_2);
            output2 = pid_duty*pwm_resolution;
        }
        else if(encoder == &encoder3){
            input3 = *speed;
            double rps = count / (2 * encoder_resolution3 * speed_reduction_ratio3 * control_period);

            double err = setpoint3 - rps;
            double propotional = kp3 * err;
            last_integral3 += ki3 * err * control_period;
            double differential =  kd3 * (rps-rps_before3) / control_period;
            double pid_duty = propotional + last_integral3 + differential;
            pid_duty = (pid_duty > 1) ? 1.0 : pid_duty;
            pid_duty = (pid_duty < -1) ? -1.0 : pid_duty;
            rps_before3 = rps;
            controlMotor(pid_duty*pwm_resolution, PWM_CHANNEL_3_1, PWM_CHANNEL_3_2);
            output3 = pid_duty*pwm_resolution;
        }
        else if(encoder == &encoder4){
            input4 = *speed;
            double rps = count / (2 * encoder_resolution4 * speed_reduction_ratio4 * control_period);

            double err = setpoint4 - rps;
            double propotional = kp4 * err;
            last_integral4 += ki4 * err * control_period;
            double differential =  kd4 * (rps-rps_before4) / control_period;
            double pid_duty = propotional + last_integral4 + differential;
            pid_duty = (pid_duty > 1) ? 1.0 : pid_duty;
            pid_duty = (pid_duty < -1) ? -1.0 : pid_duty;
            rps_before4 = rps;
            controlMotor(pid_duty*pwm_resolution, PWM_CHANNEL_4_1, PWM_CHANNEL_4_2);
            output4 = pid_duty*pwm_resolution;
        }
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void logEncoderSpeed(void *pvParameters) {
    Serial.print("Speed1 Speed2 Speed3 Speed4\n");
    for (;;) {
      Serial.printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", speed1, speed2, speed3, speed4, output1, output2, output3, output4);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    // Initialize encoders
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder1.attachSingleEdge(DC1_1, DC1_2);
    encoder2.attachSingleEdge(DC2_1, DC2_2);
    encoder3.attachSingleEdge(DC3_1, DC3_2);
    encoder4.attachSingleEdge(DC4_1, DC4_2);
    // Initialize PWM channels
    ledcSetup(PWM_CHANNEL_1_1, 5000, 8);  // 5 kHz frequency, 8-bit resolution
    ledcSetup(PWM_CHANNEL_1_2, 5000, 8);
    ledcSetup(PWM_CHANNEL_2_1, 5000, 8);
    ledcSetup(PWM_CHANNEL_2_2, 5000, 8);
    ledcSetup(PWM_CHANNEL_3_1, 5000, 8);
    ledcSetup(PWM_CHANNEL_3_2, 5000, 8);
    ledcSetup(PWM_CHANNEL_4_1, 5000, 8);
    ledcSetup(PWM_CHANNEL_4_2, 5000, 8);

    // Attach PWM channels to GPIO pins
    ledcAttachPin(IN1_1, PWM_CHANNEL_1_1);
    ledcAttachPin(IN1_2, PWM_CHANNEL_1_2);
    ledcAttachPin(IN2_1, PWM_CHANNEL_2_1);
    ledcAttachPin(IN2_2, PWM_CHANNEL_2_2);
    ledcAttachPin(IN3_1, PWM_CHANNEL_3_1);
    ledcAttachPin(IN3_2, PWM_CHANNEL_3_2);
    ledcAttachPin(IN4_1, PWM_CHANNEL_4_1);
    ledcAttachPin(IN4_2, PWM_CHANNEL_4_2);

    //set all to 0
    ledcWrite(PWM_CHANNEL_1_1, 0);
    ledcWrite(PWM_CHANNEL_1_2, 0);
    ledcWrite(PWM_CHANNEL_2_1, 0);
    ledcWrite(PWM_CHANNEL_2_2, 0);
    ledcWrite(PWM_CHANNEL_3_1, 0);
    ledcWrite(PWM_CHANNEL_3_2, 0);
    ledcWrite(PWM_CHANNEL_4_1, 0);
    ledcWrite(PWM_CHANNEL_4_2, 0);

    setpoint1 = 0;
    setpoint2 = 0;
    setpoint3 = 0;
    setpoint4 = 0;

    Serial.begin(921600);
    
    delay(2550);
    //clear all counts
    encoder1.clearCount();
    encoder2.clearCount();
    encoder3.clearCount();
    encoder4.clearCount();

    // create tasks to run getencoderspeed at 1kHz
    xTaskCreatePinnedToCore(getEncoderSpeed, "getEncoderSpeed1", 2048, (void*)&encoder1, 1, NULL, 0);
    xTaskCreatePinnedToCore(getEncoderSpeed, "getEncoderSpeed2", 2048, (void*)&encoder2, 1, NULL, 0);
    xTaskCreatePinnedToCore(getEncoderSpeed, "getEncoderSpeed3", 2048, (void*)&encoder3, 1, NULL, 0);
    xTaskCreatePinnedToCore(getEncoderSpeed, "getEncoderSpeed4", 2048, (void*)&encoder4, 1, NULL, 0);
    xTaskCreatePinnedToCore(logEncoderSpeed, "logEncoderSpeed", 2048, NULL, 1, NULL, 0);

    Serial.println("Setup done");
    
}

void loop() {
  // read kp, ki, kd from serial for motor 1
  if (Serial.available() > 0) {
    // setpoint1 = Serial.parseFloat();
    kp1 = Serial.parseFloat();
    ki1 = Serial.parseFloat();
    kd1 = Serial.parseFloat();
  }

  // switch setpoint1 0 and 30 every 5 seconds
    if (millis() % 10000 < 5000) {
        setpoint1 = 0;
    } else {
        setpoint1 = 30;
    }
}
