#include <ros.h>
//#include <ros.h>
#include <ros/time.h>
#include <hardware_interface/encoders.h>
//#include </home/nvidia/arduino-1.8.7/hardware/tools/avr/avr/include/avr/interrupt.h>

#include </home/nvidia/arduino-1.8.7/hardware/teensy/avr/cores/teensy3/avr/interrupt.h>
//#include <EnableInterrupt.h>

const int ENC_BL_PIN = 23;
const int ENC_BR_PIN = 14;
volatile uint8_t updateFlagsShared;
uint8_t updateFlags;
const int BL_FLAG = 1;
const int BR_FLAG = 2;

    volatile int BL_count_shared = 0;
    volatile int BR_count_shared = 0;
    int BL_count = 0;
    int BR_count = 0;
    int BL_count_old = 0;
    int BR_count_old = 0;
    float vel_BL = 0;
    float vel_BR = 0;
    
    volatile unsigned long BL_new_time = 0;
    volatile unsigned long BR_new_time = 0;
    volatile unsigned long BL_old_time = 0;
    volatile unsigned long BR_old_time = 0;
    unsigned long BL_DeltaTime = 0;
    unsigned long BR_DeltaTime = 0;
    
float pi  = 3.141593;
float R   = 0.051; 
volatile unsigned long dt;
volatile unsigned long t0;

hardware_interface::encoders enc;
hardware_interface::encoders enc_vel;

ros::NodeHandle nh;
ros::Publisher pub_encoder("encoders_count", &enc);
ros::Publisher pub_vel_est("encoders_vel", &enc_vel);

void incBL() {
  BL_count_shared++;
  BL_old_time = BL_new_time;
  BL_new_time = micros();
  updateFlagsShared |= BL_FLAG;
}

void incBR() {
  BR_count_shared++;
  BR_old_time = BR_new_time;
  BR_new_time = micros();
  updateFlagsShared |= BR_FLAG;
}

void readAndCopyInputs() {
  if (updateFlagsShared) {
    // Turn off interrupts, make local copies of variables set by interrupts,
    // then turn interrupts back on. Without doing this, an interrupt could
    // update a shared multibyte variable while the loop is in the middle of
    // reading it
    //noInterrupts();
    // make local copies
    updateFlags = updateFlagsShared;
    if(updateFlags & BL_FLAG) {
      BL_count = BL_count_shared;
      BL_DeltaTime = BL_new_time - BL_old_time;
    }
    if(updateFlags & BR_FLAG) {
      BR_count = BR_count_shared;
      BR_DeltaTime = BR_new_time - BR_old_time;
    }
    // clear shared update flags and turn interrupts back on
    updateFlagsShared = 0;
    //interrupts();
  }
}

void calcVelocityEstimate() {
    if(BL_count_old != BL_count){
        vel_BL = 0.25*pi*R/(BL_DeltaTime/1000000.0);    }
    else{ vel_BL = 0.0; }

    if(BR_count_old != BR_count){
        vel_BR = 0.25*pi*R/(BR_DeltaTime/1000000.0);    }
    else{ vel_BR = 0.0; }

    // update history
    BL_count_old = BL_count;
    BR_count_old = BR_count;
}

void setup() {
  nh.initNode();
  pinMode(ENC_BR_PIN, INPUT_PULLUP);
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_PIN), incBR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_PIN), incBL, CHANGE);
  nh.advertise(pub_encoder);
  nh.advertise(pub_vel_est);
  t0 = millis();
}

void loop() {
  dt = millis() - t0;
if (dt > 50) {
    readAndCopyInputs();

    // publish velocity estimate
    calcVelocityEstimate();
    //vel_est.BL  = car.getVelEstBL();
    //vel_est.BR  = car.getVelEstBR();
    //pub_vel_est.publish(&vel_est); 

    // publish encoder ticks
    enc.BL = BL_count;
    enc.BR = BR_count;
    enc.timestamp = nh.now();
    pub_encoder.publish(&enc);

    // publish encoder estimated vel
    enc_vel.BL = vel_BL;
    enc_vel.BR = vel_BR;
    enc_vel.timestamp = nh.now();
    pub_vel_est.publish(&enc_vel);

    t0 = millis();
  }
  nh.spinOnce();
}
