

#define LMdir 2
#define LMpwm 3
#define RMdir 4
#define RMpwm 5

#define Fled 6
#define Bled 9
#define Lled 10
#define Rled 11

#define potX A0
#define potY A1


int xPosition = 0;
int yPosition = 0;
int SW_state = 0;
int lastSW_state =0;
int mode= 1;
int mapX = 0;
int mapY = 0;
//
//#define LW 7
//#define HI 8
//int LWmode;
//int HImode;

unsigned long buttonPressedTime;
unsigned long currentTime;

void setup() 
{
  pinMode(LMdir,OUTPUT);
  pinMode(LMpwm,OUTPUT);
  pinMode(RMdir,OUTPUT);
  pinMode(RMpwm,OUTPUT);

  pinMode(Fled,OUTPUT);
  pinMode(Bled,OUTPUT);
  pinMode(Lled,OUTPUT);
  pinMode(Rled,OUTPUT);
  
  pinMode(potX, INPUT);
  pinMode(potY, INPUT);
//  pinMode(LW, INPUT);
//  pinMode(HI, INPUT);
  
  Serial.begin(9600);

  

}

void loop() 
{
  xPosition = analogRead(potX);
  yPosition = analogRead(potY);

//  LWmode = digitalRead(LW);
//  HImode = digitalRead(HI);
  
//  if(LWmode ==1)
//  {
//  mapX = map(xPosition, 0, 1023, -255, 255);
//  mapY = map(yPosition, 0, 1023, -255, 255);
//  }
//  else if(HImode ==1)
//  {
  mapX = map(xPosition, 0, 1023, -127, 127);
  mapY = map(yPosition, 0, 1023, -127, 127);
//  }
//  else
//  {
//    mapX =0;
//    mapY =0;
//  }

  
  if (mapX<10 && mapX>-10)
  {
    mapX=0;
  }
  if (mapY<10 && mapY>-10)
  {
    mapY=0;
  }

  
  if (mapX>0)
  {
    digitalWrite(LMdir,HIGH);
    analogWrite(LMpwm,mapX);
    digitalWrite(RMdir,HIGH);
    analogWrite(RMpwm,mapX);

    analogWrite(Fled,mapX);
    analogWrite(Bled,0);
  }
  else
  {
    digitalWrite(LMdir,LOW);
    analogWrite(LMpwm,-mapX);
    digitalWrite(RMdir,LOW);
    analogWrite(RMpwm,-mapX);

    analogWrite(Fled,0);
    analogWrite(Bled,-mapX);
    
  }
  
  if (mapY>0)
  {
    digitalWrite(LMdir,HIGH);
    analogWrite(LMpwm,mapY);
    digitalWrite(RMdir,LOW);
    analogWrite(RMpwm,mapY);

    analogWrite(Lled,0);
    analogWrite(Rled,mapY);
    
  }
  else
  {
    digitalWrite(LMdir,LOW);
    analogWrite(LMpwm,-mapY);
    digitalWrite(RMdir,HIGH);
    analogWrite(RMpwm,-mapY);

    analogWrite(Lled,-mapY);
    analogWrite(Rled,0);
  }

//  if
//  {
//    digitalWrite(RMdir,LOW);
//    analogWrite(RMpwm,-mapY);
//
//    analogWrite(Lled,-mapY);
//    analogWrite(Rled,0);
//  }





  
  Serial.print("X: ");
  Serial.print(mapX);
  Serial.print(" | Y: ");
  Serial.println(mapY);


  delay(100);
  
}
