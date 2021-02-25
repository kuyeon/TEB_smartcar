#include "L_S_Encoder_count.h"
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h> 
#include "math.h"

Servo servo;

int Motor[3] = { 24, 25, 5 };

unsigned int ENCODER_CNT = 0;
unsigned int Timer_flag = 0;
int Encoder_value = 0;

double vth = 0.0;
double vx = 0.0;
int servo_angle = 85;
int dc_vel = 0;

void motor(const geometry_msgs::Twist& msg){
    vth = msg.angular.z;
    vx = msg.linear.x;
    servo_angle = 85 + msg.angular.z * 50; //30, 85, 140 angle -> msg.angular.z * 57
    dc_vel = abs(510*msg.linear.x);

  if(msg.linear.x > 0){
    digitalWrite(25, HIGH);
    digitalWrite(24, LOW);
    analogWrite(5, dc_vel);
    servo.write(servo_angle);
    delay(1); // use delay to avoid disturbance
  }
  else if(msg.linear.x < 0){
    digitalWrite(25, LOW);
    digitalWrite(24, HIGH);
    analogWrite(5, dc_vel);
    servo.write(servo_angle);
    delay(1);
  }
  else{
    digitalWrite(25, LOW);
    digitalWrite(24, LOW);
    analogWrite(5, dc_vel);
    servo.write(servo_angle);
    delay(1);
  }

}

ros::NodeHandle n;
std_msgs::Float64 encoder_msg;
ros::Publisher pub_encoder("encoder_pulse", &encoder_msg);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor);

void setup() {
  // Motor Setup
  servo.attach(7); //attach it to pin 9
  servo.write(85);
  int z;
  for (z = 0; z < 3; z++) {
    pinMode(Motor[z], OUTPUT);
    digitalWrite(Motor[z], LOW);
  }
  attachInterrupt(7, Encoder_count, RISING);
  Timer1::set(500000, Timer1_ISR);
  Timer1::start();
  
  // ROS Node
  n.initNode();
  n.subscribe(sub);
  n.advertise(pub_encoder);
}

void loop() {
  if (Timer_flag) {
    if(vx < 0) {
    Encoder_value = (-1)*ENCODER_CNT;
    }
    else if(vx > 0) {
    Encoder_value = ENCODER_CNT;
    }
    else {
    Encoder_value = 0;
    }
    ENCODER_CNT = 0;
    Timer_flag = 0;
  }
  
  encoder_msg.data = Encoder_value;
  pub_encoder.publish(&encoder_msg);
  n.spinOnce();
}

void Encoder_count() {

  ENCODER_CNT++;
}

void Timer1_ISR() {
  Timer_flag = 1;
}
