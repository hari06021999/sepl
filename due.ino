
#define USE_USBCON

#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <DuePWM.h>
#include <stdint.h>
#include "teleop_config.h"

//Arduino Due pwm library downloaded from forum.
DuePWM pwm( frequency_right, frequency_left );

// ROS serial server
ros::NodeHandle node;
ros::Subscriber<std_msgs::Bool> sub_light1("front/light",&onfront);//subscriber topic for front light
ros::Subscriber<std_msgs::Int16> sub_light2("front/light/multi",&onfont_multi);//subscriber topic for front light fog or white
ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);////subscriber topic for linear and angular values
int i=25;
int j=25;

//setup function setting up all the functions,

void setup()
{

  node.getHardware()->setBaud(115200);
  pwm.setFreq1( frequency_right );//First frquency used for left motors
  pwm.setFreq2( frequency_left );// Second frequency used for right motor
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);// onboard led for testing
  //digitalWrite(LED_BUILTIN, LOW);
  pwm.pinFreq1( R_PWM );  // Pin 6 freq set to "pwm_freq2" on clock B
  pwm.pinFreq2( L_PWM );  // Pin 7 freq set to "pwm_freq2" on clock B
  
  setupPins();
  node.initNode();
  node.subscribe(sub);
  node.subscribe(sub_light1);
  node.subscribe(sub_light2);
  
}
/*
Settiup all GPIOs 
GPIO3------> Right Direction Relay
GPIO4------> Left Direction Relay
GPIO14------> Front light relay
GPIO15------> Front white light relay
GPIO16------> Front yellow light Relay
GPIO8------> forward_brake Relay
GPIO6------> reverse_brake Relay rear_light
GPI10------> rear_light Relay 

*/
void setupPins()
{
   
  pinMode(right_relay, OUTPUT);
  pinMode(left_relay, OUTPUT);
  pinMode(front_light, OUTPUT);
  pinMode(front_light_white, OUTPUT);
  pinMode(front_light_yellow, OUTPUT);
  pinMode(forward_brake, OUTPUT);
  pinMode(reverse_brake, OUTPUT);
  pinMode(rear_light, OUTPUT);
  pinMode(limit_switch, INPUT_PULLUP);
  stop();
  
}


/*
Stop Function is used to disable all relays initially
PWM initialized as zero in boot up condition
*/
void stop()
{
  digitalWrite(right_relay, HIGH);
  digitalWrite(left_relay, HIGH);
  digitalWrite(front_light, HIGH);
  digitalWrite(front_light_white, HIGH); 
  digitalWrite(front_light_yellow, HIGH);
  digitalWrite(forward_brake, HIGH); 
  digitalWrite(reverse_brake, HIGH);
  digitalWrite(rear_light, HIGH);
  pwm.pinDuty(L_PWM, 0);
  pwm.pinDuty(R_PWM, 0);
  
}
/*
Twist call back function, this is used for the purpose of receiving or subscribing the values of linear and angular.
linear and angular values publishing by the camera (Autonomous Mode) or (Teleop Mode)

THis block contains code of converting linear in to motion.
we are finding l value and r value to drive motors in corrosponding action
if the l value is greater than 0, left direction relay is off
       r value is greater than 0, right direction relay is off
       l value is lesser than 0, left direction relay is on
       r value is lesser than 0, right direction relay is on
*/
void onTwist(const geometry_msgs::Twist &msg)
{
  digitalWrite(LED_BUILTIN, HIGH);
  if (!_connected)
  {
    stop();
    return;
  }
    

if(msg.linear.x>0||msg.linear.x<0)
{
  i=25;
  j=25;
  // Cap values at [-1 .. 1]
  
  // Calculate the intensity of left and right wheels. Simple version.
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);
  d_L_flag=1;
  d_R_flag=1;
  // Set direction pins and PWM
   digitalWrite(left_relay, l>0);
   digitalWrite(right_relay, r>0);

  if(l<0 && r<0)
   {
     digitalWrite(rear_light, LOW);
   }
   else
   {
    digitalWrite(rear_light, HIGH);
    }
   
}
else//if(msg.linear.x==0)
{
  d_L_flag=0;
  d_R_flag=0;
 pwm.pinDuty(L_PWM, 0);
 pwm.pinDuty(R_PWM, 0);
  digitalWrite(right_relay, HIGH);
  digitalWrite(left_relay, HIGH);
  digitalWrite(rear_light, HIGH);
  stflag=1;
 // digitalWrite(LED_BUILTIN, LOW);
}
}
void onfront(const std_msgs::Bool &msg)
{

  uint8_t frontlight_data = msg.data;
  if(frontlight_data == ONE)
  {
   

    digitalWrite(front_light, LOW);
   
  }
   if(frontlight_data == ZERO)
  {
    digitalWrite(front_light, HIGH);
  
  }
}
void onfont_multi(const std_msgs::Int16 &msg)
{
  uint8_t  front_mutli_light_data = msg.data;
  if(front_mutli_light_data == ONE)
  {
   
    digitalWrite(front_light_white, LOW);
    digitalWrite(front_light_yellow, HIGH);

  }
   if(front_mutli_light_data == TWO)
  {
    digitalWrite(front_light_yellow, LOW);
    digitalWrite(front_light_white, HIGH);
  }
  if(front_mutli_light_data == ZERO)
  {
    digitalWrite(front_light_white, HIGH);
    digitalWrite(front_light_yellow, HIGH);
  }
}

void loop()
{
  if (!rosConnected())
    stop();
  node.spinOnce();
   
      if(i<=lPwm&&d_L_flag)
       {
        i++;
        digitalWrite(LED_BUILTIN, HIGH);
         pwm.pinDuty(L_PWM, i);
       }
       
       if(j<=rPwm&&d_R_flag)
       {
            j++;
            pwm.pinDuty(R_PWM, j);
       }
       
       if(i>lPwm)
          d_L_flag=0;
       if(j>rPwm)
          d_R_flag=0;
     delay(75);
 
  
   // read the state of the pushbutton value:
    limit_sw_State = digitalRead(limit_switch);
  
    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (limit_sw_State == LOW && stflag) {
      digitalWrite(forward_brake, LOW); 
      digitalWrite(reverse_brake, HIGH);
    }
    if(limit_sw_State==HIGH && stflag)
    {
      stflag=0;
      digitalWrite(forward_brake, HIGH); 
      digitalWrite(reverse_brake, LOW);
      delay(2000);
      digitalWrite(forward_brake, HIGH); 
      digitalWrite(reverse_brake, HIGH);
    }
}

bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
