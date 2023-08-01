/* Header files Include for PWM, ros and Geometry message, Boolean,Integer */
#include <PWM.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include <SoftwareSerial.h> // Debug serial
SoftwareSerial mySerial(6, 7); // RX, TX
/* Initialize node handle for ros communication */ 
ros::NodeHandle node;
ros::Subscriber<std_msgs::Bool> sub_light1("front/light",&onfront);// front light subscribe topic creation
ros::Subscriber<std_msgs::Int16> sub_light2("front/light/multi",&onfont_multi);// front light multicolor subscribe topic creation
ros::Subscriber<geometry_msgs::Twist> sub("/linear/angular", &onTwist);// command velocity subscribe topic creation

/*
set time interval for acheive respective delay
--> PWM 
--> break
--> break forward
--> break actuate holding
*/
const long pwm_interval=50;
long break_interval=0;
long break_fed_interval=0;
long break_actuate_interval=0;
unsigned long sig_started=0;
int limit_sw_State;// state of limit switch
void setup() {
  setupPins(); // gpio pins initialize
   mySerial.begin(9600);// baudrate for debug serial communcation
  // Pwm timer initialize
  InitTimersSafe();
  bool success = SetPinFrequencySafe(L_PWM, frequency_left); // set pwm frequency for left side motor
  SetPinFrequencySafe(R_PWM, frequency_right);// set pwm frequency for left side motor

  node.initNode(); // ros node intialize
  node.subscribe(sub); //subcribe joystick value
  node.subscribe(sub_light1);//subscribe front toplight topic
  node.subscribe(sub_light2);//subscribe front multicolor topic
}

void loop() {
  unsigned long currentMillis=millis(); // measure the time elapsed since the program started 
  node.spinOnce(); // where all of the ROS communication callbacks are handled
/*
  If description - linear value greater than zero will execute below statement
  purpose - While starting vehicle motor has to take the pwm gradually. Based on movement of joystick will 
  increase and decrease speed of vehicle as well */
  
  if(linear_velocity_ref) 
  {
      digitalWrite(LED_BUILTIN, HIGH);
      if(currentMillis-sig_started>=pwm_interval)
      {
        sig_started=currentMillis;
        //digitalWrite(LED_BUILTIN, HIGH);
         if(speed_left<lPwm)
         {
          pwmWrite(L_PWM, speed_left);
          speed_left++;
         }
         if(speed_left>lPwm)
         {
          pwmWrite(L_PWM, speed_left);
          speed_left--;
         }
         if(speed_right<rPwm)
         {
          pwmWrite(R_PWM, speed_right);
          speed_right++;
         }
         if(speed_right>rPwm)
         {
          pwmWrite(R_PWM, speed_right);
          speed_right--;
         }

          node.spinOnce();// where all of the ROS communication callbacks are handled
  
   }
  }
 /* When it is joystick value as zero, The motor speed is zero and motor doesnot run as well */    
   else
   {
    speed_left=25;
    speed_right=25;
    pwmWrite(L_PWM, 0);
    pwmWrite(R_PWM, 0);
    digitalWrite(LED_BUILTIN, LOW);
    }
 /* Read the limit switch status is High to low for changing the state of break condition */
    limit_sw_State = digitalRead(limit_switch);
 /* If stop flag is zero break actuator going to forward direction and apply break */
   if (stflag==ZERO) {
      digitalWrite(forward_brake, LOW); //brake foward
      digitalWrite(reverse_brake, HIGH);
      stflag=1; // setting stop flag one to holding break for sometime 
    }

 /* If stop flag is one break appling  for particular time */
    if(stflag==ONE)
    {
      break_fed_interval++;
    if(limit_sw_State==HIGH ||break_fed_interval>30000 )
    {
      break_fed_interval=0;
      stflag=2; // setting stop flag two for break comes to home position
      digitalWrite(forward_brake, HIGH); //brake hold
      digitalWrite(reverse_brake, HIGH);   //brake hold
    }
    }
  /* If stop flag is Two break relesing and comes to home position */
    if(stflag==TWO)
    {
      break_actuate_interval++;
      if(break_actuate_interval>60000)
      {
        break_actuate_interval=0;
        stflag=3;// setting stop flag three for break relay off 
        digitalWrite(forward_brake, HIGH); //brake stop
        digitalWrite(reverse_brake, LOW);   //brake reverse
      }
      node.spinOnce();// where all of the ROS communication callbacks are handled

    }
  /* If stop flag is Three  for break relay off */
    if(stflag==THREE)
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
    node.spinOnce();// where all of the ROS communication callbacks are handled


}
/* Callback function of Joystick value coming from industrial pc node */
void onTwist(const geometry_msgs::Twist &msg)
{
  /* Assigning global variable to access in loop*/
  linear_velocity_ref  = msg.linear.x;
  angular_velocity_ref = msg.angular.z;
  /* find the left and right motor how much distance should be travel */
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
  /* setting the direction of motor */
  digitalWrite(left_relay, l<0);
  digitalWrite(right_relay, r<0);
  /* Both left and right motor value less than zero back light should glow  */
 if(l<0&&r<0)
    {
       digitalWrite(rear_light, LOW);
    }
    else
    {
      digitalWrite(rear_light, HIGH);

    }
  /* Both left and right motor value as zero setting stop flag is zero to execute break functionality  */
    if(l==0&&r==0)
      stflag=0;
  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
   lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
   rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);
  // Debug the linear value
//    mySerial.print("Linear:");
//  mySerial.println(msg.linear.x);

}
/* Front light callback function */
void onfront(const std_msgs::Bool &msg)
{

  uint8_t front_light_value = msg.data;
  /* if data is one from ROS Mobile - light will glow */
  if(front_light_value == ONE)
  {
   
    digitalWrite(front_light, LOW);
   
  }
   /* if data is zero from ROS Mobile - light will not glow */
   if(front_light_value == ZERO)
  {
    digitalWrite(front_light, HIGH);
  
  }
}
/* Multicolor light callback function */
void onfont_multi(const std_msgs::Int16 &msg)
{
  uint8_t fog_white_light = msg.data;
  /* if data is one from ROS Mobile app - white light will  glow */
  if(fog_white_light == ONE)
  {
   
    digitalWrite(front_light_white, LOW);
    digitalWrite(front_light_yellow, HIGH);

  }
    /* if data is two from ROS Mobile app - fog light will  glow */

   if(fog_white_light == TWO)
  {
    digitalWrite(front_light_yellow, LOW);
    digitalWrite(front_light_white, HIGH);
  }
    /* if data is zero from ROS Mobile app - both light will not glow */

  if(fog_white_light == ZERO)
  {
    digitalWrite(front_light_white, HIGH);
    digitalWrite(front_light_yellow, HIGH);
  }
}
/* Set all gpio pins */
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
/* Initially all relay are stop*/
void stop()
{
  digitalWrite(rear_light, HIGH);
  digitalWrite(front_light, HIGH);
  digitalWrite(front_light_white, HIGH); 
  digitalWrite(front_light_yellow, HIGH);
  digitalWrite(forward_brake, HIGH); 
  digitalWrite(reverse_brake, HIGH);
  
}


// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
