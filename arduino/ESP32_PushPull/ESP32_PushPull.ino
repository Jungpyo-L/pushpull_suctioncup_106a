#define PWM_pin 26  // A0 in ESP32

const int freq = 30;
const int pwmChannel = 1;
const int resolution = 8;

int pwm;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM_pin, pwmChannel);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available()) {
    int val = Serial.parseInt();
    pwm = val;
  }

  int pwm_8bit = map(pwm, 0, 100, 0, 255);
  ledcWrite(pwmChannel, pwm_8bit);
  Serial.println(pwm);

  delay(10);

}