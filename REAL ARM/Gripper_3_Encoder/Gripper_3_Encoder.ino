#include <analogWrite.h>
#include "pid.h"

#include <WiFi.h>

const char *ssid = "Swaraj Laptop";
const char *password = "12345678";
WiFiServer wifiServer(27339);
char rxData;

#define Left_EA 25
#define Left_EB 26
#define Right_EA 34
#define Right_EB 35
#define Grip_EA 32
#define Grip_EB 33

#define Left_PWM 27
#define Left_DIR 14
#define Right_PWM 12
#define Right_DIR 13
#define Grip_PWM 17
#define Grip_DIR 16



// How many motors
#define NMOTORS 3
int target[NMOTORS];

// Pins
const int enca[] = {Left_EA, Right_EA, Grip_EA};
const int encb[] = {Left_EB, Right_EB, Grip_EB};
const int pwm[] = {Left_PWM , Right_PWM , Grip_PWM };
const int DIR[] = {Left_DIR , Right_DIR , Grip_DIR };

// Globals
long prevT = 0;
volatile int posi[] = {0, 0, 0};

// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    //Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();


  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(DIR[k], OUTPUT);

    pid[k].setParams(1, 0, 0, 255);
  }

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);

  Serial.println("target pos");
}

void loop()
{
  WiFiClient client = wifiServer.available();
  if (client)
  {
    //    Serial.println("Client Found");
    while (client.connected())
    {
      while (client.available() > 0)
      {
        rxData = client.read();
        Serial.println(rxData);
        client.write(rxData);

        if (rxData == 'w')
        {
          target[0] -= 3;
          target[1] += 3;
        }
        else if (rxData == 's')
        {
          target[0] += 3;
          target[1] -= 3;
        }
        else if (rxData == 'a')
        {
          target[0] += 3;
          target[1] += 3;
        }
        else if (rxData == 'd')
        {
          target[0] -= 3;
          target[1] -= 3;
        }
        code();
      }
      code();
    }
  }
}
void code()
{
  // set target position

  //  target[0] = 750 * sin(prevT / 1e6);
  //  target[1] = 750 * sin(prevT / 1e6);
  //  target[2] = sin(prevT / 1e6);

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for (int k = 0; k < NMOTORS; k++) {
    pos[k] = posi[k];
  }
  interrupts(); // turn interrupts back on

  // loop through the motors
  for (int k = 0; k < NMOTORS; k++) {
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);
    // signal the motor
    setMotor(dir, pwr, pwm[k], DIR[k]);
  }

  for (int k = 0; k < NMOTORS; k++) {
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
  }
  Serial.println();
}


void setMotor(int dir, int pwmVal, int pwm, int DIR)
{
  analogWrite(pwm, pwmVal);
  digitalWrite(DIR, dir);
  Serial.print(DIR);
  Serial.print(" ");
  Serial.print(pwmVal);
  Serial.print(" ");

}

template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b > 0) {
    posi[j]++;
  }
  else {
    posi[j]--;
  }
}
