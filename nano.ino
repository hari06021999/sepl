#include <PWM.h>

#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include <SoftwareSerial.h>

SoftwareSerial mySerial(6, 7); // RX, TX
#define PWM_MIN 60
#define PWMRANGE 130
int frequency_right = 380; //frequency (in Hz)
int frequency_left = 400; //frequency (in Hz)
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
//const uint8_t limit_switch1 = 6;

uint16_t lPwm;
uint16_t rPwm;
float linear_velocity_ref;
float angular_velocity_ref;
void onTwist(const geometry_msgs::Twist &msg);
void onfront(const std_msgs::Bool &msg);
void onfont_multi(const std_msgs::Int16 &msg);
float mapPwm(float x, float out_min, float out_max);
ros::NodeHandle node;
ros::Subscriber<std_msgs::Bool> sub_light1("front/light",&onfront);
ros::Subscriber<std_msgs::Int16> sub_light2("front/light/multi",&onfont_multi);
ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);
std_msgs::Int16 Encoder_right;
std_msgs::Int16 Encoder_left;
ros::Publisher right_side("/right/encoder",&Encoder_right);    
ros::Publisher left_side("/left/encoder",&Encoder_left);
bool _connected = false;
uint8_t d_flag=0;
int i=25,j=25;
const long pwm_interval=50;
long break_interval=0;
long break_fed_interval=0;
long break_actuate_interval=0;
unsigned long sig_started=0;
int limit_sw_State;
int stflag=0;
void setup() {
  setupPins();
   mySerial.begin(9600);
  mySerial.println("Hello, world?");
  // put your setup code here, to run once:
  InitTimersSafe();
  bool success = SetPinFrequencySafe(L_PWM, frequency_left);
  SetPinFrequencySafe(R_PWM, frequency_right);
  //node.getHardware()->setBaud(115200);
   node.initNode();
  node.subscribe(sub);
 node.subscribe(sub_light1);
  node.subscribe(sub_light2);
   node.advertise(right_side);                  //prepare to publish speed in ROS topic
   node.advertise(left_side);                  //prepare to publish speed in ROS topic

}

void loop() {
  unsigned long currentMillis=millis();
  // put your main code here, to run repeatedly:
  node.spinOnce();

  if(linear_velocity_ref)
  {
      digitalWrite(LED_BUILTIN, HIGH);
      if(currentMillis-sig_started>=pwm_interval)
      {
        sig_started=currentMillis;
        //digitalWrite(LED_BUILTIN, HIGH);
         if(i<lPwm)
         {
          pwmWrite(L_PWM, i);
          i++;
         }
         if(i>lPwm)
         {
          pwmWrite(L_PWM, i);
          i--;
         }
         if(j<rPwm)
         {
          pwmWrite(R_PWM, j);
          j++;
         }
         if(j>rPwm)
         {
          pwmWrite(R_PWM, j);
          j--;
         }

          node.spinOnce();
  
   }
  }
   else
   {
    i=25;
    j=25;
    pwmWrite(L_PWM, 0);
    pwmWrite(R_PWM, 0);
    digitalWrite(LED_BUILTIN, LOW);
    }

    limit_sw_State = digitalRead(limit_switch);
  
   // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
   if (0==stflag) {
      digitalWrite(forward_brake, LOW); //brake foward
      digitalWrite(reverse_brake, HIGH);
      stflag=1;
    }
    if(1==stflag)
    {
      break_fed_interval++;
    if(limit_sw_State==HIGH ||break_fed_interval>30000 )
    {
      break_fed_interval=0;
      stflag=2;
      digitalWrite(forward_brake, HIGH); //brake hold
      digitalWrite(reverse_brake, HIGH);   //brake hold
    }
    }
    if(2==stflag)
    {
      break_actuate_interval++;
      if(break_actuate_interval>60000)
      {
        break_actuate_interval=0;
        stflag=3;
        digitalWrite(forward_brake, HIGH); //brake stop
        digitalWrite(reverse_brake, LOW);   //brake reverse
      }
      node.spinOnce();

    }
    if(3==stflag)
    {
    
      break_interval++;
      if(break_interval>30000)
      {
      break_interval=0;
      stflag=4;
      digitalWrite(forward_brake, HIGH); //brake stop
      digitalWrite(reverse_brake, HIGH);
      }
      
    }
    node.spinOnce();


}
void onTwist(const geometry_msgs::Twist &msg)
{
  linear_velocity_ref  = msg.linear.x;
  angular_velocity_ref = msg.angular.z;
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
  digitalWrite(left_relay, l<0);
  digitalWrite(right_relay, r<0);
 if(l<0&&r<0)
    {
       digitalWrite(rear_light, LOW);
    }
    else
    {
      digitalWrite(rear_light, HIGH);

    }
    if(l==0&&r==0)
      stflag=0;
  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
   lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
   rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);
//    mySerial.print("Linear:");
//  mySerial.println(msg.linear.x);

}
void onfront(const std_msgs::Bool &msg)
{
   mySerial.print("FRONT LIGHT:");
//  mySerial.println(msg.linear.x);
  int check = msg.data;
  if(check == 1)
  {
   
    digitalWrite(front_light, LOW);
   
  }
   if(check == 0)
  {
    digitalWrite(front_light, HIGH);
  
  }
}
void onfont_multi(const std_msgs::Int16 &msg)
{
  int check = msg.data;
  if(check == 1)
  {
   
    digitalWrite(front_light_white, LOW);
    digitalWrite(front_light_yellow, HIGH);

  }
   if(check == 2)
  {
    digitalWrite(front_light_yellow, LOW);
    digitalWrite(front_light_white, HIGH);
  }
  if(check == 0)
  {
    digitalWrite(front_light_white, HIGH);
    digitalWrite(front_light_yellow, HIGH);
  }
}
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
void stop()
{
  digitalWrite(rear_light, HIGH);
  digitalWrite(front_light, HIGH);
  digitalWrite(front_light_white, HIGH); 
  digitalWrite(front_light_yellow, HIGH);
  digitalWrite(forward_brake, HIGH); 
  digitalWrite(reverse_brake, HIGH);
  
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
