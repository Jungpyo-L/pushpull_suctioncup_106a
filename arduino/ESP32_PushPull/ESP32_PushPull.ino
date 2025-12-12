// esp downgraded into 2.0.17 // ESP32 Dev Module // /dev/ttyS0
#define PWM_pin 26
#define Push_pin 25
#define Pull_pin 4
#define Pull_pin_below 15

#define STOP 0
#define PULL_state 1
#define PUSH_state 2

const int freq = 30;
const int pwmChannel = 1;
const int resolution = 8;

int state = STOP;
int pwm = 0;   // 0–100

void setup() {
  Serial.begin(115200);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM_pin, pwmChannel);
  ledcWrite(pwmChannel, 0);

  pinMode(Push_pin, OUTPUT);
  pinMode(Pull_pin, OUTPUT);
  pinMode(Pull_pin_below, OUTPUT);

  digitalWrite(Push_pin, LOW);
  digitalWrite(Pull_pin, LOW);
  digitalWrite(Pull_pin_below, LOW);

  Serial.println("ESP32 READY");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();   // IMPORTANT

    int idx = input.indexOf('_');
    if (idx > 0) {
      state = input.substring(0, idx).toInt();
      pwm   = input.substring(idx + 1).toInt();
    }

    Serial.print("RX state="); Serial.print(state);
    Serial.print(" pwm="); Serial.println(pwm);
  }

  if (pwm < 0) pwm = 0;
  if (pwm > 100) pwm = 100;

  int pwm_8bit = map(pwm, 0, 100, 0, 255);

  // 🔑 CRITICAL LINE — makes {state:2,pwm:0} STOP
  if (pwm == 0) state = STOP;

  switch (state) {
    case STOP:
      ledcWrite(pwmChannel, 0);
      digitalWrite(Push_pin, LOW);
      digitalWrite(Pull_pin, LOW);
      digitalWrite(Pull_pin_below, LOW);
      break;

    case PUSH_state:
      ledcWrite(pwmChannel, pwm_8bit);
      digitalWrite(Push_pin, HIGH);
      digitalWrite(Pull_pin, LOW);
      digitalWrite(Pull_pin_below, LOW);
      break;

    case PULL_state:
      ledcWrite(pwmChannel, pwm_8bit);
      digitalWrite(Push_pin, LOW);
      digitalWrite(Pull_pin, HIGH);
      digitalWrite(Pull_pin_below, HIGH);
      break;
  }
}



// #define PWM_pin 26  // A0 in ESP32
// #define Push_pin 25 // A1
// #define Pull_pin 4  // A5
// #define Pull_pin_below 15 //D15
// #define PULL_state 1
// #define PUSH_state 2
// #define STOP 0

// const int freq = 30;
// const int pwmChannel = 1;   // (kept, but unused with ledcAttach)
// const int resolution = 8;

// int state = 0;
// int pwm = 0;

// void setup() {
//   Serial.begin(115200);

//   // NEW (ESP32 core 3.x style)
//   ledcAttach(PWM_pin, freq, resolution);

//   pinMode(Push_pin, OUTPUT);
//   pinMode(Pull_pin, OUTPUT);
//   pinMode(Pull_pin_below, OUTPUT);
// }

// void loop() {
//   // --- keep exactly same ---
//   if (Serial.available()) {
//     String input = Serial.readStringUntil('\n');
//     int separatorIndex = input.indexOf('_');

//     if (separatorIndex > 0) {
//       state = input.substring(0, separatorIndex).toInt();
//       pwm = input.substring(separatorIndex + 1).toInt();
//     } else {
//       int val = Serial.parseInt();
//       pwm = val;
//       if (pwm == 0) state = 0;
//       else state = 1;
//     }
//   }
//   // -------------------------

//   int pwm_8bit = map(pwm, 0, 100, 0, 255);

//   switch (state) {
//     case STOP:
//       ledcWrite(PWM_pin, 0);
//       digitalWrite(Push_pin, LOW);
//       digitalWrite(Pull_pin, LOW);
//       digitalWrite(Pull_pin_below, LOW);
//       break;

//     case PUSH_state:
//       ledcWrite(PWM_pin, pwm_8bit);
//       digitalWrite(Push_pin, HIGH);
//       digitalWrite(Pull_pin, LOW);
//       digitalWrite(Pull_pin_below, LOW);
//       break;

//     case PULL_state:
//       ledcWrite(PWM_pin, pwm_8bit);
//       digitalWrite(Push_pin, LOW);
//       digitalWrite(Pull_pin, HIGH);
//       digitalWrite(Pull_pin_below, HIGH);
//       break;
//   }
// }









// // #define PWM_pin 26  // A0 in ESP32
// // #define Push_pin 25 // A1
// // #define Pull_pin 4 // A5
// // #define Pull_pin_below 15 //D15
// // #define PULL_state 1
// // #define PUSH_state 2
// // #define STOP 0

// // const int freq = 30;
// // const int pwmChannel = 1;
// // const int resolution = 8;

// // int state = 0; // Variable to store state
// // int pwm = 0;   // Variable to store PWM

// // void setup() {
// //   Serial.begin(115200);

// //   // Setup PWM channel
// //   ledcSetup(pwmChannel, freq, resolution);
// //   ledcAttachPin(PWM_pin, pwmChannel);

// //   // Setup push and pull pins
// //   pinMode(Push_pin, OUTPUT);
// //   pinMode(Pull_pin, OUTPUT);
// //   pinMode(Pull_pin_below, OUTPUT);
// // }

// // void loop() {
// //   // Check if there is serial data available
// //   if (Serial.available()) {
// //     String input = Serial.readStringUntil('\n'); // Read input until newline character
// //     int separatorIndex = input.indexOf('_');    // Find the position of '_'

// //     if (separatorIndex > 0) {
// //       // Parse the state and PWM from the input
// //       state = input.substring(0, separatorIndex).toInt();
// //       pwm = input.substring(separatorIndex + 1).toInt();
      
// //     }else {
// //       int val = Serial.parseInt();
// //       pwm = val;
// //       if (pwm == 0) {
// //         state = 0;
// //       }else {
// //         state = 1;
// //       }
// //     }
// //   }

// //   // Map the PWM value (assume input range is 0-100) to 8-bit range (0-255)
// //   int pwm_8bit = map(pwm, 0, 100, 0, 255);

// //   // Write the PWM value to the PWM pin
// //   // ledcWrite(pwmChannel, pwm_8bit);

// //   switch (state)
// //   {
// //   case STOP:
// //     ledcWrite(pwmChannel, 0);
// //     digitalWrite(Push_pin, LOW);
// //     digitalWrite(Pull_pin, LOW);
// //     digitalWrite(Pull_pin_below, LOW);
// //     break;
// //   case PUSH_state:
// //     ledcWrite(pwmChannel, pwm_8bit);
// //     digitalWrite(Push_pin, HIGH);
// //     digitalWrite(Pull_pin, LOW);
// //     digitalWrite(Pull_pin_below, LOW);
// //     break;
  
// //   case PULL_state:
// //     ledcWrite(pwmChannel, pwm_8bit);
// //     digitalWrite(Push_pin, LOW);
// //     digitalWrite(Pull_pin, HIGH);
// //     digitalWrite(Pull_pin_below, HIGH);
// //     break;
// //   }

// //   // delay(10);
// // }



// // // #define PWM_pin 26  // A0 in ESP32

// // // const int freq = 30;
// // // const int pwmChannel = 1;
// // // const int resolution = 8;

// // // int pwm;

// // // void setup() {
// // //   // put your setup code here, to run once:
// // //   Serial.begin(115200);

// // //   ledcSetup(pwmChannel, freq, resolution);
// // //   ledcAttachPin(PWM_pin, pwmChannel);
  
// // // }

// // // void loop() {
// // //   // put your main code here, to run repeatedly:

// // //   if(Serial.available()) {
// // //     int val = Serial.parseInt();
// // //     pwm = val;
// // //   }

// // //   int pwm_8bit = map(pwm, 0, 100, 0, 255);
// // //   ledcWrite(pwmChannel, pwm_8bit);
// // //   Serial.println(pwm);

// // //   delay(10);

// // // }
