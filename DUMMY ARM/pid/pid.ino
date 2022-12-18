#include <Wire.h>
#include <Servo.h>


Servo right_prop;
Servo left_prop;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];




float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

double kp= 100;
double ki=0.654;
double kd= 50;


double throttle=1300; // value of throttle to given to motors
float desired_angle = 0; //This is the angle in which we want the test rig to be at
                         


void setup() {
  Wire.begin(); //Begin a transmission to the I2C slave device with the given address.
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  right_prop.attach(3); //attach the right motor to pin 3
  left_prop.attach(5);  //attach the left motor to pin 5

  time = millis(); //Start counting time in milliseconds
 
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  delay(6000); //Not understood this part of the code make sure to ask seniors
}

void loop() {


    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; //That will give you the elapsed time in milliseconds
  //As the elapsed time is in miliseconds we divide by 1000 to convert to seconds
 
  
     Wire.beginTransmission(0x68); //because slave address of IMU is 0x68
     Wire.write(0x3B); //wire.write helps in writing data from a slave device in response to a request from a master, or queues bytes for transmission from a master to slave device
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); //Used by the master to request bytes from a slave device
   
   
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg; //x-axis
     
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg; //y-axis
 
   Serial.print("Acceleration_angle[1]"); Serial.print(Acceleration_angle[1]);
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first address
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //there are 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); 
   Gyr_rawY=Wire.read()<<8|Wire.read();
   
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   Gyro_angle[0] = Gyr_rawX/131.0; 

   Gyro_angle[1] = Gyr_rawY/131.0;
   Serial.print("Gyro_angle[1]= ");Serial.print(Gyro_angle[1]);

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   value by the elapsedTime./
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that affects the angles and ofcourse multiply by 0.98 */

   //X axis angle
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   //Y axis angle
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   Serial.print(" Total_angle[0]") ; Serial.print( Total_angle[0]);

   
  




//x-axis will be parallel to the test rig
error = Total_angle[1] - desired_angle;// error between the desired angle and the real measured angle
Serial.print(error);    
/*Next the proportional value of the PID is just a proportional constant
multiplied by the error*/

 pid_p = kp*error;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For that we will use a variable caWe substract that value from the actual error and divide all by the elapsed time. 
lled previous_error.
Finaly we multiply the result by the derivate constant*/

pid_d = kd*((error - previous_error)/elapsedTime);

//The final PID values is the sum of each of these 3 parts
PID = pid_p + pid_i + pid_d;

/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
// since PWM signals has a vale of 1000 micro sec and the max value is 2000 so the wave oscilates between -1000 and 1000
if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}
//Calculate PWM width
pwmLeft = throttle + PID;
pwmRight = throttle - PID;



//Right
if(pwmRight < 1000)
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}
//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}
if(error==0)
{pwmLeft=throttle;
 pwmRight=throttle;}
//Finaly using the servo function we create the PWM pulses with the calculated width of each pulse

left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = error;


Serial.print("PWMLeft:");
Serial.print(pwmLeft);
Serial.print("PWMRight:");
Serial.print(pwmRight);
Serial.println();
