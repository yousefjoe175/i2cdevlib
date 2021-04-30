

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define M_PI    3.14159265358979323846
#define OUTPUT_READABLE_YAWPITCHROLL

/**************************************************************************************************
 *                                     GLOBAL VARIABLES 
 * ************************************************************************************************/
bool blinkState = false;
/* MPU control/status vars */
 bool dmpReady = false;  /*set true if DMP init was successful */
uint8_t mpuIntStatus;    /*holds actual interrupt status byte from MPU */
uint8_t devStatus;       /* return status after each device operation (0 = success, !0 = error) */
uint16_t packetSize;     /* expected DMP packet size (default is 42 bytes) */
uint16_t fifoCount;      /*count of all bytes currently in FIFO */
uint8_t fifoBuffer[64];  /* FIFO storage buffer */

/* orientation/motion variables */
Quaternion q;           /* [w, x, y, z]         quaternion container  */
VectorFloat gravity;    /* [x, y, z]            gravity vector        */
float euler[3];         /* [psi, theta, phi]    Euler angle container */
int16_t gyro[3];
int16_t accel[3];  
float ypr[3];           /* [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector */
float  x = 0 ;          /* variable to store offset*/ 
float  yaw   ;          /* variable to store the required yaw angle*/ 
long unsigned int i = 0;/*counter for calculting offset*/
//String message;   
//++++
int message3;

/**************************************************************************************************
 *                                    
 * DMP Function Description : indicates whether MPU interrupt pin has gone high
 * *************************************************************************************************/
 volatile bool mpuInterrupt = false;     
  void dmpDataReady() {
    mpuInterrupt = true ;
   }


int main(){

    /* Initialize I2C and serial communications */
    //  ++++


    mpu.initialize();

    devStatus = mpu.dmpInitialize();	
    /* supply your own gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

      if (devStatus == 0) {
       
        /* set the user ctrl register which contains the bit for enable DMP */
        mpu.setDMPEnabled(true);	
        
        
        /* enable ARM interrupt detection with ISR dmpDataReady() */
        // ++++++

        /* set our DMP Ready flag so the main loop() function knows it's okay to use it */   
        dmpReady = true;
        /*  get expected DMP packet size for later comparison */
        packetSize = mpu.dmpGetFIFOPacketSize();
    }


    while(1){
        /* label that the code will go to while sensor still calibrates */
    label_1:
   
    /* if programming failed, don't try to do anything */
    if (!dmpReady){ 
        return 0;
    }


    /* wait for MPU interrupt or extra packet(s) available */
    while (!mpuInterrupt && fifoCount < packetSize) { }

    /* reset interrupt flag and get INT_STATUS byte */
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    /* get current FIFO count */
    fifoCount = mpu.getFIFOCount();

    /* check for overflow (this should never happen unless our code is too inefficient) */
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        /* reset so we can continue cleanly */
        mpu.resetFIFO();
       // Serial.println(F("FIFO overflow!"));

    /* otherwise, check for DMP data ready interrupt (this should happen frequently) */
    } 
    else if (mpuIntStatus & 0x02)
    {
        /* wait for correct available data length, should be a VERY short wait */
        while (fifoCount < packetSize){
            fifoCount = mpu.getFIFOCount();
        }

        /* read a packet from FIFO */
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        /* track FIFO count here in case there is > 1 packet available
            (this lets us immediately read more without waiting for an interrupt) */
        fifoCount -= packetSize;
        

        /* getting the GYRO and Accelerometer */
        mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetAccel(accel, fifoBuffer);
        
        /* getting YAW angle */
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


        if(i<900)
            {
               i++ ;
               x=(ypr[0]*180/M_PI);
               goto label_1;
            }
            else
            {
              yaw = ((int)( (ypr[0] * 180/M_PI)- x ) );
              /*check if the angle is in range */ 
              if(yaw >= 180) yaw -= 360;
              if(yaw < -180) yaw += 360;

             /* 
              message3 = int(yaw);
              if(message3>-180 && message3 <=-100)
              {
                message = String(message3); 
              }
              else if(message3 >= -99 && message3 <=-10)
              {
                message = "-0"  + String(-1*message3) ;
              }
              else if(message3 >=-9 && message3 <= -1)
              {
                message = "-00"  + String(-1*message3);
              }
              else if(message3 >=0 && message3<=9)
              {
                message = "000" + String(message3)  ; 
              }
              else if(message3 >=10 && message3 <=99)
              {
                message = "00" + String(message3)  ;
              }
              else
              {
                message = "0" + String(message3) ;
              }
             /* delay(200);
             ++++   */


             
            }


    }


}