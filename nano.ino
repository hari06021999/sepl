#include <PWM.h>

#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#define PWM_MIN 100
#define PWMRANGE 200
int frequency_right = 380; //frequency (in Hz)
int frequency_left = 400; //frequency (in Hz)
const uint8_t R_PWM = 9;//D3
const uint8_t L_PWM = 3;//D9
const uint8_t right_relay = 2;// D2 RIGHT 1ST MOTOR
const uint8_t left_relay = 4;// D4 RIGHT 2ND MOTOR
uint16_t lPwm;
uint16_t rPwm;
float linear_velocity_ref;
float angular_velocity_ref;
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);
bool _connected = false;
uint8_t d_flag=0;
int i=25,j=25;
int range_time=0;
void setup() {
  // put your setup code here, to run once:
  InitTimersSafe();
  bool success = SetPinFrequencySafe(L_PWM, frequency_left);
  SetPinFrequencySafe(R_PWM, frequency_right);
   node.initNode();
  node.subscribe(sub);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  node.spinOnce();
  if(linear_velocity_ref)
  {
   // d_flag=0;
      digitalWrite(LED_BUILTIN, HIGH);
      
         if(i<=lPwm)
         {
          pwmWrite(L_PWM, i);
          i++;
         }
         if(j<=rPwm)
         {
          pwmWrite(R_PWM, j);
          j++;
         }
         delay(50);
         //range_time=millis()+20;
   
   }
   else
   {
    i=25;
    j=25;
    pwmWrite(L_PWM, 0);
    pwmWrite(R_PWM, 0);
     digitalWrite(LED_BUILTIN, LOW);
    }

}
void onTwist(const geometry_msgs::Twist &msg)
{
  linear_velocity_ref  = msg.linear.x;
  angular_velocity_ref = msg.angular.z;
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
  digitalWrite(left_relay, l>0);
  digitalWrite(right_relay, r>0);

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
   lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
   rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

}
bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
  
   // portOne.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
