// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN 60
#define PWMRANGE 130
uint16_t frequency_right = 380; //frequency (in Hz)
uint16_t frequency_left = 400; //frequency (in Hz)

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
void onfront(const std_msgs::Bool &msg);
void onfont_multi(const std_msgs::Int16 &msg);
float mapPwm(float x, float out_min, float out_max);


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
const uint8_t encoder_right_a=A4;
const uint8_t encoder_right_b=A5;
const uint8_t encoder_left_a=6;
const uint8_t encoder_left_b=7;
bool _connected = false;
uint16_t lPwm;
uint16_t rPwm;
float linear_velocity_ref;
float angular_velocity_ref;

uint8_t d_flag=0;
uint8_t speed_right=25,speed_left=25;
uint8_t stflag=0;

typedef enum {ZERO,ONE,TWO,THREE,FOUR,FIVE}number;

/*  PID Controller variables */
long prevT=0;
float eprev=0;
float eintegral = 0;
volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

double left_kp = 1 , left_ki = 0 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 1 , right_ki = 0 , right_kd = 0.0;

