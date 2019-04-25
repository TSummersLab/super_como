//#include <EnableInterrupt.h>

#include <ros.h>
#include <ros/time.h>
#include <hardware_interface/encoders.h>
#include <hardware_interface/ultrasonics.h>
//#include <EnableInterrupt.h>


ros::NodeHandle nh; // ROS node

// ultrasonic and encoder messages
hardware_interface::ultrasonics us;
hardware_interface::encoders enc;

// ROS publishers
ros::Publisher ultrasonicsPub("ultrasonics",&us);
ros::Publisher encodersPub("encoders", &enc);

// --------------Pins defined below--------------------------
// Front Side encoders (left to right)
// US1
int trigpin1 = 1;
int echopin1 = 0;
// US2
int trigpin2 = 3;
int echopin2 = 2;
// US3
int trigpin3 = 5;
int echopin3 = 4;
// US4
int trigpin4 = 7;
int echopin4 = 6;

// Rear Side encoders (left to right)
// US5
int trigpin5 = 22;
int echopin5 = 21;
// US6
int trigpin6 = 20;
int echopin6 = 19;
// US7
int trigpin7 = 18;
int echopin7 = 17;
// US8
int trigpin8 = 16;
int echopin8 = 15;

// Left wheel encoder
int leftencpin = 23;
// Right wheel encoder
int rightencpin = 14;
// --------------Pins defined above--------------------------


float duration1, duration2, duration3, duration4, duration5, duration6, duration7, duration8; //ultrasonic distance time measurement
float inches1, inches2, inches3, inches4, inches5, inches6, inches7, inches8; // ultrasonic distance in inches
int leftcnt, rightcnt; // encoder color change counters (each 8 are a full wheel rotation)
float leftvel, rightvel; // estimated instanteneous wheel speed
int delay1= 5; 
int delay2= 10;

//class Encoders {
//  public:
//    void 
//}
//
//void incleftcb() {
//  
//}
//
//void incrightcb() {
//  
//}


void setup()
{
  // start ROS node
  nh.initNode(); 

  // Publish topics
  nh.advertise(ultrasonicsPub); 
  nh.advertise(encodersPub);
  
  // Pin modes
  pinMode(trigpin1, OUTPUT);
  pinMode(echopin1, INPUT);
  pinMode(trigpin2, OUTPUT);
  pinMode(echopin2, INPUT);
  pinMode(trigpin3, OUTPUT);
  pinMode(echopin3, INPUT);
  pinMode(trigpin4, OUTPUT);
  pinMode(echopin4, INPUT);
  pinMode(trigpin5, OUTPUT);
  pinMode(echopin5, INPUT);
  pinMode(trigpin6, OUTPUT);
  pinMode(echopin6, INPUT);
  pinMode(trigpin7, OUTPUT);
  pinMode(echopin7, INPUT);
  pinMode(trigpin8, OUTPUT);
  pinMode(echopin8, INPUT);
  pinMode(leftencpin, INPUT_PULLUP);
  pinMode(rightencpin, INPUT_PULLUP);
  //enableInterrupt(leftencpin, incleftcb, CHANGE); // interrupt for left encoder
  //enableInterrupt(rightencpin, incrightcb, CHANGE); // interrupt for right encoder

  //Serial.begin(57600);
}
 
void loop()
{ 
  digitalWrite(trigpin1, LOW);
  delayMicroseconds(delay1);
  digitalWrite(trigpin1, HIGH);
  delayMicroseconds(delay2);
  digitalWrite(trigpin1, LOW);
  pinMode(echopin1, INPUT);
  duration1 = pulseIn(echopin1, HIGH);

  digitalWrite(trigpin2, LOW);
  delayMicroseconds(delay1);
  digitalWrite(trigpin2, HIGH);
  delayMicroseconds(delay2);
  digitalWrite(trigpin2, LOW);
  pinMode(echopin2, INPUT);
  duration2 = pulseIn(echopin2, HIGH);

  digitalWrite(trigpin3, LOW);
  delayMicroseconds(delay1);
  digitalWrite(trigpin3, HIGH);
  delayMicroseconds(delay2);
  digitalWrite(trigpin3, LOW);
  pinMode(echopin3, INPUT);
  duration3 = pulseIn(echopin3, HIGH);

  digitalWrite(trigpin4, LOW);
  delayMicroseconds(delay1);
  digitalWrite(trigpin4, HIGH);
  delayMicroseconds(delay2);
  digitalWrite(trigpin4, LOW);
  pinMode(echopin4, INPUT);
  duration4 = pulseIn(echopin4, HIGH);

  digitalWrite(trigpin5, LOW);
  delayMicroseconds(delay1);
  digitalWrite(trigpin5, HIGH);
  delayMicroseconds(delay2);
  digitalWrite(trigpin5, LOW);
  pinMode(echopin5, INPUT);
  duration5 = pulseIn(echopin5, HIGH);

  digitalWrite(trigpin6, LOW);
  delayMicroseconds(delay1);
  digitalWrite(trigpin6, HIGH);
  delayMicroseconds(delay2);
  digitalWrite(trigpin6, LOW);
  pinMode(echopin6, INPUT);
  duration6 = pulseIn(echopin6, HIGH);

  digitalWrite(trigpin7, LOW);
  delayMicroseconds(delay1);
  digitalWrite(trigpin7, HIGH);
  delayMicroseconds(delay2);
  digitalWrite(trigpin7, LOW);
  pinMode(echopin7, INPUT);
  duration7 = pulseIn(echopin7, HIGH);

  digitalWrite(trigpin8, LOW);
  delayMicroseconds(delay1);
  digitalWrite(trigpin8, HIGH);
  delayMicroseconds(delay2);
  digitalWrite(trigpin8, LOW);
  pinMode(echopin8, INPUT);
  duration8 = pulseIn(echopin8, HIGH);

  inches1 = (duration1/2)/ 74;
  inches2 = (duration2/2)/ 74;
  inches3 = (duration3/2)/ 74;
  inches4= (duration4/2)/ 74;
  inches5 = (duration5/2)/ 74;
  inches6 = (duration6/2)/ 74;
  inches7= (duration7/2)/ 74;
  inches8= (duration8/2)/ 74;

//    ultrasonic1.range = inches1;
//    ultrasonic1.radiation_type = 1;
//    ultrasonic2.range = inches2;
//    ultrasonic2.radiation_type = 2;
//    ultrasonic3.range = inches3;
//    ultrasonic3.radiation_type = 3;
//    ultrasonic4.range = inches4;
//    ultrasonic4.radiation_type = 4;
//    ultrasonic5.range = inches5;
//    ultrasonic5.radiation_type = 5;
//    ultrasonic6.range = inches6;
//    ultrasonic6.radiation_type = 6;
//    ultrasonic7.range = inches7;
//    ultrasonic7.radiation_type = 7;
//    ultrasonic8.range = inches8;
//    ultrasonic8.radiation_type = 8;


    us.us1 = inches1/39.37; //divide by 39.37 to convert to metric scale
    us.us2 = inches2/39.37;
    us.us3 = inches3/39.37;
    us.us4 = inches4/39.37;
    us.us5 = inches5/39.37;
    us.us6 = inches6/39.37;
    us.us7 = inches7/39.37;
    us.us8 = inches8/39.37;
    us.timestamp = nh.now();
    
//    chatter1.publish(&ultrasonic1);
//    chatter2.publish(&ultrasonic2);
//    chatter3.publish(&ultrasonic3);
//    chatter4.publish(&ultrasonic4);
//    chatter5.publish(&ultrasonic5);
//    chatter6.publish(&ultrasonic6);
//    chatter7.publish(&ultrasonic7);
//    chatter8.publish(&ultrasonic8);
   ultrasonicsPub.publish(&us);
    

  nh.spinOnce();
  /*
  unsigned long time;
  time = millis()/1000;
  Serial.print(time);
  Serial.print(": ");
  Serial.print(inches1);
  Serial.print("in 1, ");
  Serial.print(inches2);
  Serial.print("in 2, ");
  Serial.print(inches3);
  Serial.print("in 3, ");
  Serial.print(inches4);
  Serial.print("in 4, ");
  Serial.print(inches5);
  Serial.print("in 5, ");
  Serial.print(inches6);
  Serial.print("in 6, ");
  Serial.print(inches7);
  Serial.print("in 7, ");
  Serial.print(inches8);
  Serial.print("in 8, ");
  Serial.println();
  */
  delay(60);
}
