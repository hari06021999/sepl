#include <PWM.h>
#include <stdint.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <ros/time.h>
#include "teleop_config.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(6, 7); // RX, TX

ros::NodeHandle node; //initializing ros nodes
ros::Subscriber<std_msgs::Bool> sub_light1("front/light",&onfront); //calling the subscriber for front lights
ros::Subscriber<std_msgs::Int16> sub_light2("front/light/multi",&onfont_multi);//calling the subscriber for front multi-coloured lights
ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);//calling the subscriber for receiving the data from industrial PC
std_msgs::Int16 Encoder_right;
std_msgs::Int16 Encoder_left;
ros::Publisher right_side("/right/encoder",&Encoder_right);    
ros::Publisher left_side("/left/encoder",&Encoder_left);
 
void setup() {
  setupPins();
  mySerial.begin(9600);  //setting the baudrate for debugging purpose
  mySerial.println("Hello, world?");
  InitTimersSafe();
  bool success = SetPinFrequencySafe(L_PWM, frequency_left);//setting frequency for the left wheels
  SetPinFrequencySafe(R_PWM, frequency_right);//setting frequency for the right wheels
   node.initNode();
  node.subscribe(sub);//variable for subscribering the data receiving for industrail PC
 node.subscribe(sub_light1);
  node.subscribe(sub_light2);
  node.advertise(right_side);                  //prepare to publish speed in ROS topic
   node.advertise(left_side);                  //prepare to publish speed in ROS topic
 
  
}

void loop() {

const long pwm_interval=50;
long break_interval=0;
long break_fed_interval=0;
unsigned long sig_started=0;
uint8_t limit_sw_State;
unsigned long currentMillis=millis();// To calculate the delay using timer
  node.spinOnce();

  if(linear_velocity_ref) 
  {
      digitalWrite(LED_BUILTIN, HIGH);
      if(currentMillis-sig_started>=pwm_interval)
      {
        sig_started=currentMillis;
        //digitalWrite(LED_BUILTIN, HIGH);
         if(speed_right<lPwm)
         {
          pwmWrite(L_PWM, speed_right);
          speed_right++;
         }
         if(speed_right>lPwm)
         {
          pwmWrite(L_PWM, speed_right);
          speed_right--;
         }
         if(speed_left<rPwm)
         {
          pwmWrite(R_PWM, speed_left);
          speed_left++;
         }
         if(speed_left>rPwm)
         {
          pwmWrite(R_PWM, speed_left);
          speed_left--;
         }

          node.spinOnce();
  
   }
  }
   else
   {
    speed_right=25;
    speed_left=25;
    pwmWrite(L_PWM, 0);
    pwmWrite(R_PWM, 0);
    digitalWrite(LED_BUILTIN, LOW);
    }

    limit_sw_State = digitalRead(limit_switch);
  
   // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
   if (stflag==0) {
      digitalWrite(forward_brake, LOW); //brake foward
      digitalWrite(reverse_brake, HIGH);
      stflag=1;
    }
    if(stflag==1)
    {
      break_fed_interval++;
    if(limit_sw_State==HIGH ||break_fed_interval>20000 )
    {
      break_fed_interval=0;
      stflag=2;
      digitalWrite(forward_brake, HIGH); //brake stop
      digitalWrite(reverse_brake, LOW);   //brake reverse
    }
    }
    if(stflag==2)
    {
    
      break_interval++;
      if(break_interval>30000)
      {
      break_interval=0;
      stflag=3;
      digitalWrite(forward_brake, HIGH); 
      digitalWrite(reverse_brake, HIGH);
      }
      
    }

    // int target = 10000;
    // long currT = micros();
    // long et = currT-prevT;
    // float deltaT = ((float)(currT-prevT))/1.0e6;
    // prevT = currT;
    // //error
    // int e = encoder0Pos-target;
    // //derivative
    // float dedt = (e-eprev)/(deltaT);
    // //integral
    // eintegral = eintegral + e*deltaT;
    // //store previous error
    // eprev = e;
  Encoder_right.data = encoder0Pos;
  right_side.publish(&Encoder_right);
  Encoder_right.data = encoder1Pos;
  left_side.publish(&Encoder_left);
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
  uint8_t front_light_data = msg.data;
  if(front_light_data == ONE)
  {
   
    digitalWrite(front_light, LOW);
   
  }
   if(front_light_data == ZERO)
  {
    digitalWrite(front_light, HIGH);
  
  }
}
void onfont_multi(const std_msgs::Int16 &msg)
{
  uint8_t light_data = msg.data;
  if(light_data == ONE)
  {
   
    digitalWrite(front_light_white, LOW);
    digitalWrite(front_light_yellow, HIGH);

  }
   if(light_data == TWO)
  {
    digitalWrite(front_light_yellow, LOW);
    digitalWrite(front_light_white, HIGH);
  }
  if(light_data == ZERO)
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
  pinMode(encoder_right_a, INPUT_PULLUP); 
  pinMode(encoder_right_b, INPUT_PULLUP); 
  pinMode(encoder_left_a, INPUT_PULLUP); 
  pinMode(encoder_left_b, INPUT_PULLUP); 
  attachInterrupt(0, encoder_right_A, RISING);
  attachInterrupt(1, encoder_right_B, RISING);
  attachInterrupt(2, encoder_left_A, RISING);
  attachInterrupt(3, encoder_left_B, RISING);
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
// ************** encoders interrupts **************

// ************** encoder 1 *********************


void encoder_left_A(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder_left_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_left_b) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder_left_b) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void encoder_left_B(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder_left_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder_left_a) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder_left_a) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void encoder_right_A(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder_right_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_right_b) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder_right_b) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void encoder_right_B(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder_right_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder_right_a) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder_right_b) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

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
