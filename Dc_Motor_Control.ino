  const byte sensorpin = 2;
  const byte in1pin = 8;
  const byte in2pin = 7;
  const byte enapin = 6;
  const float radius_of_spin = 3.5; //cm
  const float pulse_per_rotation = 4.0; // spaces in the disk
  int time_reset = 1000; //milliseconds
  volatile int pulse_count = 0;
  unsigned long last_time = 0;
  volatile unsigned long last_interrupt_time = 0;
  double setpoint = 5.0; // km/hour
  double preverror , output , integral = 0;
  double kp = 30;
  double ki = 13;
  double kd = 5;
void countpulse() {
  unsigned long interrupt_time = millis(); 
  if (interrupt_time - last_interrupt_time > 10) { //to avoid double pulses readed on my optic encoder
    pulse_count++;
    last_interrupt_time = interrupt_time;
  }
}
void printspeed(float rpm , float speed){
  Serial.print(rpm);
  Serial.println("rpm");
  Serial.print(speed);
  Serial.println("km per hour");
}
void setup() {
  Serial.begin(9600);
  pinMode(sensorpin , INPUT);
  pinMode(in1pin , OUTPUT);
  pinMode(in2pin , OUTPUT);
  pinMode(enapin , OUTPUT);
  attachInterrupt(digitalPinToInterrupt(sensorpin), countpulse , RISING);
  digitalWrite(in1pin , HIGH);
  digitalWrite(in2pin , LOW);
}
void loop(){
unsigned long now_time = millis();
if((now_time-last_time)>= time_reset){
  noInterrupts();
  float temp_pulse = pulse_count;
  pulse_count = 0;
  interrupts();
  double dt = (now_time - last_time)/1000.0;
  last_time = now_time;
  float rpm = (temp_pulse / pulse_per_rotation) * (60000.0 / time_reset);
  float speed = (2 * 3.14 * radius_of_spin * rpm * 60) / 100000.0;
  double error = setpoint - speed;
  double pout = kp * error;
  integral += error * dt;
  double iout = ki * integral;
  double deriative = (error - preverror)/dt;
  double dout = kd * deriative;
  output = pout + iout + dout;
  output = constrain(output , 0 , 255);
  preverror = error;
  analogWrite(enapin, output);
  printspeed(rpm,speed);
}
}
 