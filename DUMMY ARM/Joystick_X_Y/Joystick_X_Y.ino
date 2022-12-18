
const int joyLink1 = A0; 
const int joyLink2 = A1; 

int angLink1 = 0;
int angLink2 = 0;
int target1 = 45;
int target2 = 45;

void setup() 
{
  Serial.begin(9600);
}

void loop() {
  angLink1 = analogRead(joyLink1);
  angLink2 = analogRead(joyLink2);
  angLink1 = map(angLink1, 0, 1023,-5, 5);
  angLink2 = map(angLink2, 0, 1023, -5, 5);

  target1+=angLink1;
  target2+=angLink2;
  if(target1>=135)
  {
    target1=135;
  }
  if(target1<=-45)
  {
    target1=-45;
  }

  if(target2>=105)
  {
    target2=105;
  }
  if(target2<=-105)
  {
    target2=-105;
  }
  
  Serial.print("Link1 = ");
  Serial.print(target1);
  Serial.print("\t Link2 = ");
  Serial.println(target2);

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);
}
