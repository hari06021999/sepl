// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN 60 // Minimum pwm
#define PWMRANGE 130 //maximum pwm
uint16_t frequency_right = 380; //frequency (in Hz)
uint16_t frequency_left = 400; //frequency (in Hz)

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);// joystick callback function
void onfront(const std_msgs::Bool &msg);// front light callback function
void onfont_multi(const std_msgs::Int16 &msg);// Multicolor light callback function
float mapPwm(float x, float out_min, float out_max);// Mapping pwm function


/*
Settiup all GPIOs 
GPIO2------> Right Direction Relay
GPIO4------> Left Direction Relay
A0------> Front light relay
A1------> Front white light relay
A2------> Front yellow light Relay
GPIO10-------> forward_brake Relay
GPIO11------> reverse_brake Relay rear_light
A3------> rear_light Relay 
GPIO5 ------> Limit switch
*/
const uint8_t R_PWM = 9;//D3
const uint8_t L_PWM = 3;//D9
const uint8_t right_relay = 2;// D2 RIGHT MOTOR
const uint8_t left_relay = 4;// D4 RIGHT  MOTOR
const uint8_t front_light = A0;
const uint8_t front_light_white = A1;
const uint8_t front_light_yellow = A2;
const uint8_t rear_light = A3;
const uint8_t forward_brake = 10;
const uint8_t reverse_brake = 11;
const uint8_t limit_switch = 5;

/* Left pwm and right pwm variable to access global */
uint16_t lPwm;
uint16_t rPwm;
/* Industrial pc publish linear and angular value store in below variable to access through out program*/
float linear_velocity_ref;
float angular_velocity_ref;

uint8_t d_flag=0; // direction flag
uint8_t speed_right=25,speed_left=25; // speed increase variable
uint8_t stflag=0; //stop flag

typedef enum {ZERO,ONE,TWO,THREE,FOUR,FIVE}number;


