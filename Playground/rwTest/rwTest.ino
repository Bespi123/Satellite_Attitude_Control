//Current sensor Pins
#define Sensor_IM1  28
#define Sensor_IM2  27
#define Sensor_IM3  26

//Reaction Wheels PWM Pins
#define pin_PWM_M1  3
#define pin_PWM_M2  39
#define pin_PWM_M3  38

//Reaction Wheels direction pines
#define pin_dir_M1  2
#define pin_dir_M2  12
#define pin_dir_M3  11

//Reaction Wheel ecoder Pines
#define pin_enc_M1  36
#define pin_enc_M2  35
#define pin_enc_M3  33

//Debug flag
#define DEBUG true

void setup() {
// Define inputs and outputs
Serial.begin(9600);
pinMode(pin_PWM_M1,OUTPUT);
pinMode(pin_dir_M1,OUTPUT);
}

void loop() {
  Serial.println("CW");
  digitalWrite(pin_dir_M1,HIGH);
  for(int i=255;i>0;i=i-50){
    Serial.println(i);
    analogWrite(pin_PWM_M1,i);
    delay(3000);
  }
  for(int i=0;i<255;i=i+50){
    Serial.println(i);
    analogWrite(pin_PWM_M1,i);
    delay(3000);
  }
  delay(1000);
  Serial.println("CCW");
  digitalWrite(pin_dir_M1,LOW);
  for(int i=255;i>0;i=i-50){
    Serial.println(i);
    analogWrite(pin_PWM_M1,i);
    delay(3000);
  }
  for(int i=0;i<255;i=i+50){
    Serial.println(i);
    analogWrite(pin_PWM_M1,i);
    delay(3000);
  }
delay(1000);
}
