#include <Arduino.h>

constexpr uint8_t SYSTEM_MAX_PWM = 255;
constexpr uint32_t CONTROL_INTERVAL_MS = 1000;
constexpr float MILLISECONDS_IN_SECONDS = 1000.0;

void printGraph(float speedKmH , float setPointKmh);

class Motor{
private:
uint8_t in1Pin;
uint8_t in2Pin;
uint8_t enaPin;

public:
Motor(uint8_t in1,uint8_t in2,uint8_t ena){
  in1Pin = in1;
  in2Pin = in2;
  enaPin = ena;
}

void moveForward(){
  pinMode(in1Pin,OUTPUT);
  pinMode(in2Pin,OUTPUT);
  pinMode(enaPin,OUTPUT);
  digitalWrite(in1Pin,HIGH);
  digitalWrite(in2Pin,LOW);
}

void setSpeed(double output){
  analogWrite(enaPin, output);
}
};

class Encoder{
private:
  uint8_t encoderPin;
  static volatile uint32_t pulseCount;
  static volatile uint32_t lastInterruptTime;
  float diskRadiusCM;
  float pulsePerRotation; //Number of spaces in your disk
  static const uint8_t DEFAULT_DEBOUNCE_MS = 10;

public:
  Encoder(uint8_t encoderpin,float diskradiuscm,float ppr){
  encoderPin = encoderpin;
  diskRadiusCM = diskradiuscm;
  pulsePerRotation = ppr;
  }

  static void countPulse(){
  uint32_t interruptTime = millis();
  if(interruptTime - lastInterruptTime > DEFAULT_DEBOUNCE_MS){
    pulseCount++;
    lastInterruptTime = interruptTime;
    }
  }

  void begin(){
  pinMode(encoderPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin),countPulse,RISING);
  }

  float getSpeed(float dtS){
  noInterrupts();
  uint32_t pulseCountCopy = pulseCount;
  pulseCount = 0;
  interrupts();
  constexpr float SECONDS_IN_A_MINUTE = 60.0;
  constexpr float MINUTES_IN_AN_HOUR = 60.0;
  constexpr float CM_IN_A_KM = 100000.0;
  float diskPerimeter = 2 * PI * diskRadiusCM;
  float rpm = (pulseCountCopy / pulsePerRotation) * (SECONDS_IN_A_MINUTE/dtS);
  float speedKmH =  (diskPerimeter * rpm * MINUTES_IN_AN_HOUR) / CM_IN_A_KM;
  return speedKmH;
 }
}; 
volatile uint32_t Encoder::pulseCount = 0;
volatile uint32_t Encoder::lastInterruptTime = 0;

class PIDController{
private:
float Kp;
float Ki;
float Kd;
float setPointKmh;
double prevError = 0.0;
double integralSum = 0.0;
public:
PIDController(float kp ,float ki,float kd,float setpointkmh){
  Kp = kp;
  Ki = ki;
  Kd = kd;
  setPointKmh = setpointkmh;
}
float computePID(float dtS, float speedKmH){
  double error = setPointKmh - speedKmH;
  double Pout = Kp * error;
  double output = 0;
  integralSum = integralSum + (error * dtS);
  double Iout = Ki * integralSum;
  Iout = constrain(Iout, -SYSTEM_MAX_PWM,SYSTEM_MAX_PWM);
  double deriative = (error - prevError)/dtS;
  prevError = error;
  double Dout = Kd * deriative;
  output = Pout + Iout + Dout;
  output = constrain(output , 0 , SYSTEM_MAX_PWM);
  return output;
}
float getSetPoint(){
  return setPointKmh;
}
};

uint32_t lastTime = 0;

Motor mainMotor(8,7,6);
Encoder mainEncoder(2,3.5,4.0);
PIDController mainPID(30,13,5,5);

void setup(){
  Serial.begin(115200);
  mainMotor.moveForward();
  mainEncoder.begin();
}

void loop() {
  uint32_t now = millis();

  if(now - lastTime > CONTROL_INTERVAL_MS){
    float dtS = (now-lastTime)/MILLISECONDS_IN_SECONDS;
    lastTime = now;

    float speedKmH = mainEncoder.getSpeed(dtS);
    float output = mainPID.computePID(dtS,speedKmH);
    mainMotor.setSpeed(output);

    float setpointKmH = mainPID.getSetPoint();
    printGraph(speedKmH,setpointKmH);
  }
}

void printGraph(float speedKmH , float setPointKmh){
Serial.print(speedKmH);
Serial.print(" ");
Serial.println(setPointKmh);
}