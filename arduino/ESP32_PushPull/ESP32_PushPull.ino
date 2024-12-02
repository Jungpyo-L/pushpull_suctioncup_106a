#define PWM_pin 26  // A0 in ESP32
#define Push_pin 25 // A1
#define Pull_pin 4 // A5
#define PULL_state 1
#define PUSH_state 2
#define STOP 0

const int freq = 30;
const int pwmChannel = 1;
const int resolution = 8;

int state = 0; // Variable to store state
int pwm = 0;   // Variable to store PWM

void setup() {
  Serial.begin(115200);

  // Setup PWM channel
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM_pin, pwmChannel);

  // Setup push and pull pins
  pinMode(Push_pin, OUTPUT);
  pinMode(Pull_pin, OUTPUT);
}

void loop() {
  // Check if there is serial data available
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Read input until newline character
    int separatorIndex = input.indexOf('_');    // Find the position of '_'

    if (separatorIndex > 0) {
      // Parse the state and PWM from the input
      state = input.substring(0, separatorIndex).toInt();
      pwm = input.substring(separatorIndex + 1).toInt();
      
    }else {
      int val = Serial.parseInt();
      pwm = val;
      state = 0;
    }
  }

  // Map the PWM value (assume input range is 0-100) to 8-bit range (0-255)
  int pwm_8bit = map(pwm, 0, 100, 0, 255);

  // Write the PWM value to the PWM pin
  // ledcWrite(pwmChannel, pwm_8bit);

  switch (state)
  {
  case STOP:
    ledcWrite(pwmChannel, 0);
    digitalWrite(Push_pin, LOW);
    digitalWrite(Pull_pin, LOW);
    break;
  case PULL_state:
    ledcWrite(pwmChannel, pwm_8bit);
    digitalWrite(Push_pin, HIGH);
    digitalWrite(Pull_pin, LOW);
    break;
  
  case PUSH_state:
    ledcWrite(pwmChannel, pwm_8bit);
    digitalWrite(Push_pin, LOW);
    digitalWrite(Pull_pin, HIGH);
    break;
  }

  // delay(10);
}



// #define PWM_pin 26  // A0 in ESP32

// const int freq = 30;
// const int pwmChannel = 1;
// const int resolution = 8;

// int pwm;

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);

//   ledcSetup(pwmChannel, freq, resolution);
//   ledcAttachPin(PWM_pin, pwmChannel);
  
// }

// void loop() {
//   // put your main code here, to run repeatedly:

//   if(Serial.available()) {
//     int val = Serial.parseInt();
//     pwm = val;
//   }

//   int pwm_8bit = map(pwm, 0, 100, 0, 255);
//   ledcWrite(pwmChannel, pwm_8bit);
//   Serial.println(pwm);

//   delay(10);

// }