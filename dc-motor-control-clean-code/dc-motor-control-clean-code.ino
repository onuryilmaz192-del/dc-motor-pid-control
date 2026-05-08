#include <Arduino.h>

constexpr uint8_t ENCODER_PIN = 2;
constexpr uint8_t MOTOR_IN1 = 8;
constexpr uint8_t MOTOR_IN2 = 7;
constexpr uint8_t MOTOR_ENA = 6;
 
constexpr float DISK_RADIUS_CM = 3.5;
constexpr float PULSE_PER_ROTATION = 4.0;
constexpr uint32_t CONTROL_INTERVAL_MS = 1000;
constexpr uint32_t DEBOUNCE_MS = 10;
constexpr uint8_t MAX_PWM = 255;

constexpr float KP = 30;
constexpr float KI = 13;
constexpr float KD = 5;
constexpr float setPoint = 5; //km/h

void moveForward();
void countPulse();
void printGraph(float speed,float setPoint);
float getSpeed(float dtS);
float computePID(float dtS,float speed);

volatile uint32_t pulseCount = 0;
volatile uint32_t lastInterruptTime = 0;
uint32_t lastTime = 0;
float output = 0;

void setup(){
  Serial.begin(9600);
  pinMode(ENCODER_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN),countPulse,RISING);
  moveForward();
}

void loop(){
  uint32_t now = millis();
    if(now - lastTime > CONTROL_INTERVAL_MS){
      float dtS = (now - lastTime)/1000.0; //divided by 1000 milliseconds to get seconds as a result
      float speed = getSpeed(dtS);
      output = computePID(dtS , speed);
      analogWrite(MOTOR_ENA , output);
      lastTime = now;
      printGraph(speed , setPoint);
    }
}

void moveForward(){
  pinMode(MOTOR_IN1,OUTPUT);
  pinMode(MOTOR_IN2,OUTPUT);
  pinMode(MOTOR_ENA,OUTPUT);
  digitalWrite(MOTOR_IN1,HIGH);
  digitalWrite(MOTOR_IN2,LOW);
}
void countPulse(){
  uint32_t interruptTime = millis();
   if(interruptTime - lastInterruptTime > DEBOUNCE_MS){
     pulseCount++;
     lastInterruptTime = interruptTime;
  }
}
float getSpeed(float dtS){
  noInterrupts();
  uint32_t pulseCountCopy = pulseCount;
  pulseCount = 0;
  interrupts();
  float diskPerimeter = 2 * PI * DISK_RADIUS_CM;
  float rpm = (pulseCountCopy / PULSE_PER_ROTATION) * (60.0/dtS); // multiplied by 60 to get minutes as a result
  float speed =  (diskPerimeter * rpm * 60) / 100000.0; // multiplied by 60 and divided by 10000 to get km/h as a result;
  return speed;
}
float computePID(float dtS , float speed){
  double error = setPoint - speed;
  double Pout = KP * error;
  static double integralSum = 0;
  integralSum = integralSum + (error * dtS);
  integralSum = constrain(output , -MAX_PWM , MAX_PWM);
  double Iout = KI * integralSum;
  static double prevError = 0;
  double deriative = (error - prevError)/dtS;
  prevError = error;
  double Dout = KD * deriative;
  output = Pout + Iout + Dout;
  output = constrain(integralSum , 0 , MAX_PWM);
  return output;
}
void printGraph(float speed , float setPoint){
  Serial.print(speed);
  Serial.print(" ");
  Serial.println(setPoint);
}
