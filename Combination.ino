#include <Wire.h>                     // Used for OLED 
#include <Adafruit_GFX.h>             // Used for OLED
#include <Adafruit_SSD1306.h>         // Used for OLED
#include <FlexiTimer2.h>

#define trigPin 12                    // TrigPin of ultrasonic sensor
#define echoPin 13                    // EchoPin of ultrasonic sensor
#define A_INA 9                       // Forward signal to the right wheel
#define A_INB 6                       // Backward signal to the right wheel
#define B_INA 11                      // Forward signal to the left wheel
#define B_INB 10                      // Backward signal to the left wheel
#define IR1 8                         // H3 on PCB.                        
#define IR5 1                         // H7 on PCB. Cannot coexist with serial communication statements!!                       
#define AOB 3                         // Encoder on the left wheel
#define AOA 2                         // Encoder on the right wheel
#define bcf A1                        // The 1st beacon detector
#define bcs A0                        // The 2nd beacon detector   
int myangle;
int pulsewidth;
int a=60;
int servopin=5;
int exl = 30;                         // Expected speed of wheel B (L)
int exr = 30;                         // Expected speed of wheel A (R)
int sensor1, sensor2;                 // Output of 2 IR sensors
int speedl, speedr;                   // Actual speed of wheel B & A
int countl, countr;                   // the number of pulses of B & A in each 30ms
int PWMA, PWMB, errorr, errorl;       // the PWM value given to A & B, the error between exl or exr and speedl or speedr
int B1v, B2v;
int button, flag;
float displace;
long duration;
long cm = 100;
volatile long countA, countB;         // Total number of pulses, used for detecting the path length
float Kp = 5;

#define OLED_RESET -1                 // The default is -1. Any other value is N/A for this programme.
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

void setup(){ 
  pinMode(servopin, OUTPUT);
  pinMode(IR1, INPUT);
  pinMode(IR5, INPUT);
  pinMode(echoPin, INPUT);
  pinMode(A2, INPUT);
  pinMode(AOA, INPUT);
  pinMode(AOB, INPUT);
  pinMode(A_INA, OUTPUT);
  pinMode(A_INB, OUTPUT);
  pinMode(B_INA, OUTPUT);
  pinMode(B_INB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(A3, OUTPUT);
  
  for(int i=0;i<=50;i++)//给予舵机足够的时间让它转到指定角度
  {
  servopulse(servopin,0);//引用脉冲函数 
  }
  
  attachInterrupt(0,SpeedA,FALLING);  // Interrupt from Pin 2
  attachInterrupt(1,SpeedB,FALLING);  // Interrupt from Pin 3
  display.begin(SSD1306_SWITCHCAPVCC,0x3C);
  display.setTextColor(WHITE);        // Set the colour of pixels
  display.clearDisplay();             // Clear the OLED screen
  display.setTextSize(2);             // Set the size of font 
   
  FlexiTimer2::set(30, 1.0/1000, ontimer30ms);
  FlexiTimer2::start();
  
}

void loop(){
  displace = (countA + countB)*204.1*1.29/11700;
  display.setCursor(20, 20);            //Display location
  display.print(displace);
  display.print("cm");
  display.display();
  display.clearDisplay();

    if(flag==2){
    digitalWrite(A3, HIGH);
    delay(2000);
    digitalWrite(A3, LOW);      
    delay(18000);
    flag++;
    }
    
    if(flag==3){
    B2v=analogRead(bcs);
  
    if (B2v >= 600)                       // Beacon approached
    {
        analogWrite(A_INA, 0);
        analogWrite(A_INB, 0);
        analogWrite(B_INA, 0); 
        analogWrite(B_INB, 0);
       for(int i=0;i<=50;i++)//给予舵机足够的时间让它转到指定角度
     {
        servopulse(servopin,180);//引用脉冲函数 
     }
    while(1);
    }
    else if(B2v <= a){                       // The beacon detecting
        analogWrite(A_INA, 0);
        analogWrite(A_INB, 0);
        analogWrite(B_INA, 0); 
        analogWrite(B_INB, 60);                   // Spin around
    }
    else if(B2v > a && B2v <= 600){
        analogWrite(A_INA, 68);
        analogWrite(A_INB, 0);
        analogWrite(B_INA, 0); 
        analogWrite(B_INB, 45);
       delay(1700);
    }
    }
}

//ISR for Pin 2
void SpeedA(){
  countA++;
  countr++;
}

//ISR for Pin 3
void SpeedB(){
  countB++;
  countl++;
}

void ontimer30ms(){ 
  button = digitalRead(A2);

  if(button==1){
    flag=1;
  }

  if(flag==1){    
    speedl = countl;
    countl = 0;
    speedr = countr;
    countr = 0;
    ultrasonic();
    if(cm > 16){
      turnctrl();
      speedPID();
      digitalWrite(A3, LOW);
    }
  }
}
  
void speedPID(){
  errorl = exl-speedl;
  errorr = exr-speedr;
  PWMA = Kp*errorr+50;
  PWMB = Kp*errorl+50;

  if(PWMA>200){
    PWMA=200;
  }
  else if(PWMA<0){
    PWMA=0;
  }
  else if(PWMB>200){
    PWMB=200;
  }
  else if(PWMB<0){
    PWMB=0;
  }

  analogWrite(B_INA, 0);
  analogWrite(B_INB, PWMB);
  analogWrite(A_INA, PWMA);
  analogWrite(A_INB, 0);
}

void turnctrl(){
  sensor1= digitalRead(IR1);
  sensor2= digitalRead(IR5);

  if(sensor1==1 && sensor2==1){
  //Forward
  exl=36;
  exr=36;  
  }

  else if(sensor1==0 && sensor2==1){
  //Turn right
  exl=36;
  exr=8;  
  }

  else if(sensor1==1 && sensor2==0){
  //Turn left
  exl=8;
  exr=36;  
  }  
}

void ultrasonic(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); 
  cm = (duration/2) / 29.1;
  
  if(cm <= 16){
    analogWrite(A_INA, 0);
    analogWrite(A_INB, 0);
    analogWrite(B_INA, 0);
    analogWrite(B_INB, 0);
    flag = 2;
  }
}

void servopulse(int servopin,int myangle)
{
  pulsewidth=(myangle*11)+500;
  digitalWrite(servopin,HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servopin,LOW);
  delay(20-pulsewidth/1000);
}
