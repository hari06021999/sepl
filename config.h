// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN 95
#define PWMRANGE 180
int frequency_right = 1000; //frequency (in Hz)
int frequency_left = 380; //frequency (in Hz)

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
void onfront(const std_msgs::Bool &msg);
void onfont_multi(const std_msgs::Int16 &msg);
float mapPwm(float x, float out_min, float out_max);


// Pins
const uint8_t R_PWM = 6;
const uint8_t right_relay = 3;
const uint8_t left_relay = 4;
const uint8_t L_PWM = 7;
const uint8_t front_light = 17;
const uint8_t front_light_white = 15;
const uint8_t front_light_yellow = 16;
const uint8_t forward_brake = 8;
const uint8_t reverse_brake = 9;
const uint8_t rear_light = 10;
const uint8_t limit_switch = 46;
bool _connected = false;
int limit_sw_State;
int stflag=0;
uint16_t lPwm;
uint16_t rPwm;
uint8_t d_L_flag=0;
uint8_t d_R_flag=0;
typedef enum {ZERO,ONE,TWO,THREE,FOUR,FIVE}number;
