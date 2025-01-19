// Existing includes and RemoteXY setup
#define REMOTEXY_MODE__WIFI_POINT
#include <WiFi.h>
#include <RemoteXY.h>
#include <driver/ledc.h>

#define REMOTEXY_WIFI_SSID "RemoteXY_OpenAP"
#define REMOTEXY_WIFI_PASSWORD ""  // Leave blank for an open network
#define REMOTEXY_SERVER_PORT 6377
#define REMOTEXY_ACCESS_PASSWORD ""  // Leave blank for no RemoteXY password

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 188 bytes
  { 255, 7, 0, 0, 0, 181, 0, 19, 0, 0, 0, 77, 89, 95, 67, 79, 78, 84, 82, 79,
    76, 0, 18, 1, 106, 200, 1, 1, 10, 0, 4, 77, 65, 21, 115, 0, 2, 26, 12, 18,
    151, 40, 10, 192, 30, 26, 102, 111, 114, 119, 97, 114, 100, 0, 114, 101, 118, 101, 114, 115,
    101, 0, 2, 32, 14, 44, 22, 0, 2, 26, 31, 31, 79, 78, 0, 79, 70, 70, 0, 10,
    3, 24, 24, 24, 48, 4, 26, 31, 79, 78, 0, 31, 79, 70, 70, 0, 10, 81, 25, 24,
    24, 48, 4, 26, 31, 79, 78, 0, 31, 79, 70, 70, 0, 1, 11, 96, 24, 24, 3, 2,
    31, 116, 117, 114, 110, 95, 108, 101, 102, 116, 0, 1, 50, 96, 24, 24, 3, 2, 31, 116,
    117, 114, 110, 95, 114, 105, 103, 104, 116, 0, 129, 69, 182, 33, 12, 64, 24, 115, 112, 101,
    101, 100, 0, 129, 8, 77, 68, 23, 64, 24, 83, 84, 69, 69, 82, 0, 129, 38, 41, 33,
    12, 64, 24, 76, 65, 77, 80, 0 };
struct {
  int8_t speed;          // from 0 to 100
  uint8_t move;          // 0 = stop, 1 = forward, 2 = reverse
  uint8_t headlamp;      // =1 if switch ON and =0 if OFF
  uint8_t left_ind;      // =1 if state is ON, else =0
  uint8_t right_ind;     // =1 if state is ON, else =0
  uint8_t left;          // =1 if button pressed, else =0
  uint8_t right;         // =1 if button pressed, else =0
  uint8_t connect_flag;  // =1 if wire connected, else =0
} RemoteXY;
#pragma pack(pop)

// Pin assignments
const int headlampPin = 16;
const int rightIndPin = 17;
const int leftIndPin = 18;

// Motor control pins for L298N
const int IN1 = 19;  // Speed Motor PWM Control
const int IN2 = 21;  // Motor Reverse
const int IN3 = 22;  // Steering Motor Left
const int IN4 = 23;  // Steering Motor Right

// LEDC channel for PWM control
ledc_channel_config_t ledcChannel;
ledc_channel_config_t ledcSteering;

void setup() {
  RemoteXY_Init();

  // Initialize pins for headlamp and indicators
  pinMode(headlampPin, OUTPUT);
  pinMode(rightIndPin, OUTPUT);
  pinMode(leftIndPin, OUTPUT);

  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configure PWM for speed control on IN1
  ledcChannel.channel = LEDC_CHANNEL_0;
  ledcChannel.duty = 0;        // Start with duty cycle 0
  ledcChannel.gpio_num = IN1;  // PWM pin
  ledcChannel.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledcChannel.hpoint = 0;
  ledcChannel.timer_sel = LEDC_TIMER_0;

  // Configure PWM for steering control on IN3 and IN4
  ledcSteering.channel = LEDC_CHANNEL_1;
  ledcSteering.duty = 0;         // Start with duty cycle 0
  ledcSteering.gpio_num = IN3;   // PWM pin for steering left
  ledcSteering.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledcSteering.hpoint = 0;
  ledcSteering.timer_sel = LEDC_TIMER_1;

  // Configure LEDC timer for motor speed
  ledc_timer_config_t ledcTimer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,  // 8-bit resolution
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000  // Frequency of 1kHz
  };

  // Configure LEDC timer for steering
  ledc_timer_config_t ledcTimerSteering = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,  // 8-bit resolution
    .timer_num = LEDC_TIMER_1,
    .freq_hz = 500  // Frequency of 500Hz for smoother steering control
  };

  ledc_timer_config(&ledcTimer);
  ledc_timer_config(&ledcTimerSteering);
  ledc_channel_config(&ledcChannel);
  ledc_channel_config(&ledcSteering);

  // Initialize all outputs to OFF
  digitalWrite(headlampPin, LOW);
  digitalWrite(rightIndPin, LOW);
  digitalWrite(leftIndPin, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  RemoteXY_Handler();

  // Headlamp control
  digitalWrite(headlampPin, RemoteXY.headlamp ? HIGH : LOW);

  // Right indicator blinking at 0.5-second intervals
  static unsigned long lastRightBlinkTime = 0;
  if (RemoteXY.right_ind) {
    if (millis() - lastRightBlinkTime >= 500) {
      lastRightBlinkTime = millis();
      digitalWrite(rightIndPin, !digitalRead(rightIndPin));
    }
  } else {
    digitalWrite(rightIndPin, LOW);
  }

  // Left indicator blinking at 0.5-second intervals
  static unsigned long lastLeftBlinkTime = 0;
  if (RemoteXY.left_ind) {
    if (millis() - lastLeftBlinkTime >= 500) {
      lastLeftBlinkTime = millis();
      digitalWrite(leftIndPin, !digitalRead(leftIndPin));
    }
  } else {
    digitalWrite(leftIndPin, LOW);
  }

  // Motor speed and direction control based on 'move'
  int pwmValue = map(RemoteXY.speed, 0, 100, 0, 255);  // Map slider (0-100) to PWM (0-255)

  switch (RemoteXY.move) {
    case 1:                    // Forward
      digitalWrite(IN2, LOW);  // IN2 LOW for forward
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, pwmValue);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
      break;
    case 2:                                                                 // Reverse
      digitalWrite(IN2, HIGH);                                              // IN2 HIGH for reverse
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 255 - pwmValue);  // Adjusted PWM for reverse
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
      break;
    default:                                                  //stop
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);  // Stop PWM
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
      break;
  }

  // Steering control with PWM
  int steeringDuty = 128; // 50% duty cycle (reduced speed)

  if (RemoteXY.left) {  // Turn left
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, steeringDuty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    digitalWrite(IN4, LOW);
  } else if (RemoteXY.right) {  // Turn right
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, steeringDuty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    digitalWrite(IN4, HIGH);
  } else {  // No steering
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}
