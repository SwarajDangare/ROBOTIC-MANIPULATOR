/***************************************************************************

*/

//#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
//#include "Wire.h"

#define INT_PIN_2 2

//NAxisMotion mySensor;         //Object that for the sensor 
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 40;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
unsigned char incommingByte = 0;
char reg[10]={0};
unsigned char write_str[5] = {0};
unsigned char RD_Quater[4] = {0xAA, 0x01, 0x08, 26};
unsigned char quater_data[100] = {0};
unsigned char feedbk[2];
int i; 
//int data_num = 0;
//#include <SoftwareSerial.h>
//SoftwareSerial Serial(0, 1); // RX, TX
//SoftwareSerial Serial1(2, 3); // RX, TX
void setup() //This code is executed once
{    
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  Serial.println("RT begi1...2...3...");
  Serial1.begin(115200);    //Enable Serial1 of Arduino board, Baud Rate = 115200;
  delay(100);
  BNO055_Init();
  
//  initBNO055();    
//  setBNO055_OPMode();   //Set BNO to work at NDOF mode.
 /*   Serial1.readBytes(reg,2);
  Serial.print("Reg1::");
  Serial.println(reg[0]);
  Serial.print("Reg2::");
  Serial.println(reg[1]);*/
}

void loop() //This code is looped forever
{
//  Serial.println("RT begi");
  unsigned char Sel_PG0[]={0xAA, 0x00,0x07,0x01,0x00};
  Serial1.write(Sel_PG0,5);     //select page0
   if(Serial1.available()>0){
    Serial1.readBytes(feedbk,2);  
  }
}
/*if ((millis() - lastStreamTime) >= streamPeriod)
  { 
  Serial1.write(RD_Quater,4);    //{0xAA, 0x01, 0x20, 0x08}
  if(Serial1.available()>0){
    Serial1.readBytes(feedbk,2);  
    }
  
 if(feedbk[0] == 0xBB)
   {
   if(Serial1.available()>0)
     {
     Serial1.readBytes(quater_data,feedbk[1]);
     }
/*   for(i=0;i<feedbk[1];i++)
   {
   Serial.print(quater_data[i]);
   Serial.print("\t");
   }
   Serial.println("");*/

 /*else
  {
   judgement(feedbk[1]);
  }
 
// data_num++;
// Serial.println(data_num);
//    lastStreamTime = millis();    
 //   readEuler();
  }  
}*/

void BNO055_Init(void)
{
  unsigned char Sel_PG0[]={0xAA, 0x00, 0x07, 0x01, 0x00};
  Serial1.begin(115200);
  Serial1.write(Sel_PG0,5);
  if(Serial1.available()>0){
    Serial1.readBytes(feedbk,2);  
  }
  if(feedbk[0] == 0xEE)
  {
    if(feedbk[1] == 1)
    {
    Serial.println("Page0 selected");
    }
    else
  {
  judgement(feedbk[1]);
  }

  }
  unsigned char Set_PW[]={0xAA, 0x00, 0x3E, 0x01, 0x00};
  Serial1.write(Set_PW,5);
  if(Serial1.available()>0){
    Serial1.readBytes(feedbk,2);  
  }
    if(feedbk[0] == 0xEE)
  {
    if(feedbk[1] == 1)
    {
    Serial.println("Normal mode selected!!");
    }
    else
  {
  judgement(feedbk[1]);
  }

  }

  unsigned char Set_OPMD[]={0xAA, 0x00, 0x3D, 0x01, 0x07};
  Serial1.write(Set_OPMD,5);
  if(Serial1.available()>0){
    Serial1.readBytes(feedbk,2);  
  }
    if(feedbk[0] == 0xEE)
  {
    if(feedbk[1] == 1)
    {
    Serial.println("OP mode NDOF selected!");
    delay(650);
    }
    else
  {
  judgement(feedbk[1]);
  }

  }


}
void judgement(unsigned char a)
{
  switch(a)
  {
  case 1: Serial.println("Write success");break;
  case 3: Serial.println("Write Fail");break;
  case 4: Serial.println("Regmap invalid address");break;
  case 5: Serial.println("regmap write disabled");break;
  case 6: Serial.println("wrong start byte");break;
  case 7: Serial.println("Bus over run error");break;
  case 8: Serial.println("Max length error");break;
  case 9: Serial.println("Min length error");break;
  case 10: Serial.println("Receive character timeout"); break;
  default: Serial.println("failure with no reason");break;
  }

}


/*void initBNO055(void)
{
  pinMode(INT_PIN_2, INPUT_PULLUP);
  resetBNO055();
}

void resetBNO055(void)
{
  digitalWrite(RESET_PIN, LOW);    //Set the Reset pin LOW
  delay(RESET_PERIOD);        //Hold it for a while
  digitalWrite(RESET_PIN, HIGH);    //Set the Reset pin HIGH
  delay(INIT_PERIOD);         //Pause for a while to let the sensor initialize completely (Anything >500ms should be fine)
  
}
unsigned char setOP_Mode[5] = {0xAA,0x00,0x3D,1,0x01};
void setBNO055_OPMode(void)
{
    Serial1.write(setOP_Mode,5);
}

void readEuler(void)
{
  unsigned char ReadEuler[4] = {0xAA, 0x01, 0x1A,6};
  Serial1.write(ReadEuler,4);

}
*/
