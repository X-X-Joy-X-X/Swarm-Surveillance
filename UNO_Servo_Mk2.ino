#include <Servo.h>

#define SERVO_1_PIN 2  // PWM pin for servo control
#define SERVO_2_PIN 3  // PWM pin for servo control
#define SERVO_3_PIN 4  // PWM pin for servo control
#define SERVO_4_PIN 5  // PWM pin for servo control
#define SERVO_5_PIN 6  // PWM pin for servo control
#define SERVO_6_PIN 7  // PWM pin for servo control
#define SERVO_7_PIN 8  // PWM pin for servo control
#define SERVO_8_PIN 9  // PWM pin for servo control


Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;
Servo Servo6;
Servo Servo7;
Servo Servo8;

int currentSpeed = 90;
char currentCommand = 'S';

void setup() {
  Serial.begin(9600);     // Serial communication with ESP32

  Servo1.attach(SERVO_1_PIN);
  Servo2.attach(SERVO_2_PIN);
  Servo3.attach(SERVO_3_PIN);
  Servo4.attach(SERVO_4_PIN);
  Servo5.attach(SERVO_5_PIN);
  Servo6.attach(SERVO_6_PIN);
  Servo7.attach(SERVO_7_PIN);
  Servo8.attach(SERVO_8_PIN);
  
  Servo1.write(currentSpeed);
  Servo2.write(currentSpeed);
  Servo3.write(80);
  Servo4.write(currentSpeed);
  Servo5.write(currentSpeed);
  Servo6.write(currentSpeed);
  Servo7.write(currentSpeed);
  Servo8.write(currentSpeed);
  delay(3000);

  Serial.println("Running");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch(command) {
      case 'F':  // Forward
        if (currentCommand != 'F'){
          currentCommand = 'F';
          startForward();
        }
        break;

      case 'R':  // Right
        if (currentCommand == 'F'){
          endForward();
        }
        if (currentCommand != 'R'){
          currentCommand = 'R';
          right();
        }
        break;

      case 'L':  // Left
        if (currentCommand == 'F'){
          endForward();
        }
        if (currentCommand != 'L'){
          currentCommand = 'L';
          left();
        }

      case 'S':  // Stop
        if (currentCommand == 'F'){
          endForward();
        }
        currentCommand = 'S';
        break;
    }
  }
  switch(currentCommand){
    case 'F':
      forward();
      break;
    case 'S':  // Stop
      //Servo1.write(90);
      break;
  }
  Serial.println(currentCommand);

  // while(1){
  //   startForward();
  //   forward();
  //   forward();
  //   endForward();
  //   left();
  //   left();
  // }
}

void startForward(){
  for (int i = 0; i <= 50; i += 1){
    Servo1.write(90 - i);                 // Forward
    Servo2.write(90 + i * .5);            // Straight
    delay(15);
  }
  for (int i = 0; i <= 50; i += 1){
    Servo5.write(90 + i);                 // Forward
    Servo6.write(90 - i * .5);            // Straight
    delay(15);
  }

    for (int i = 0; i <= 50; i += 1){
    Servo3.write(90 - i * .5);            // Half Back
    Servo4.write(90 - i * .5);            // Straight
    delay(15);
  }
  for (int i = 0; i <= 50; i += 1){       
    Servo7.write(90 + i * .5);             // Half Back
    Servo8.write(90 + i * .5);            // Straight
    delay(15);
  }  
}


void forward(){
  for (int i = 0; i <= 50; i += 1){
    Servo1.write(40 + i);                   // Back
    //Servo2.write(120 - i * .5);           // Stay Straight
    Servo3.write(65 + i * .5);              // Half Forward
    Servo4.write(65 + i * .5);              // Bend
    Servo5.write(140 - i);                  // Back
    //Servo6.write(60 + i * .5);            // Stay Straight
    Servo7.write(115 - i * .5);             // Half Forward
    Servo8.write(115 - i * .5);             // Bend
    delay(20);
  }

  for (int i = 0; i <= 50; i += 1){
    Servo1.write(90 + i * .5);              // Half Back
    //Servo2.write(120 - i * .5);           // Stay Straight
    Servo3.write(90 + i);                   // Forward
    Servo4.write(90 - i * .5);              // Straight
    Servo5.write(90 - i * .5);              // Half Back
    //Servo6.write(60 - i * .5);            // Stay Straight
    Servo7.write(90 - i);                   // Forward
    Servo8.write(90 + i * .5);              // Straight
    delay(20);
  }

    for (int i = 0; i <= 50; i += 1){
    Servo1.write(115 - i * .5);             // Half Forward
    Servo2.write(115 - i * .5);             // Bend
    Servo3.write(140 - i);                  // Back
    //Servo4.write(60 + i * .5);            // Stay Straight
    Servo5.write(65 + i * .5) ;             // Half Forward
    Servo6.write(65 + i * .5);              // Bend
    Servo7.write(40 + i );                  // Back
    //Servo8.write(120 - i * .5);           // Stay Straight
    delay(20);
  }

  for (int i = 0; i <= 50; i += 1){
    Servo1.write(90 - i);                   // Forward
    Servo2.write(90 + i * .5);              // Straight
    Servo3.write(90 - i * .5);              // Half Back
    //Servo4.write(60 - i * .5);            // Stay Straight
    Servo5.write(90 + i);                   // Forward
    Servo6.write(90 - i * .5);              // Straight
    Servo7.write(90 + i * .5);              // Half Back
    //Servo8.write(120 + i * .5);           // Stay Straight
    delay(20);
  }
  
}


void endForward(){
  for (int i = 0; i <= 50; i += 1){
    Servo1.write(40 + i);                     // Back             
    Servo2.write(115 - i * .5);               // Bend
    Servo5.write(140 - i);                    // Back
    Servo6.write(65 + i * .5);                // Bend
    Servo3.write(65 + i * .5);                // Forward
    Servo4.write(65 + i * .5);                // Bend
    Servo7.write(115 - i * .5);                // Forward
    Servo8.write(115 - i * .5);               // Bend
    delay(20);
  }
}


void right(){
  for (int i = 0; i <= 50; i += 1){
    Servo1.write(90 - i * .5);            // Forward
    Servo2.write(90 + i * .5);            // Straight
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){
    Servo3.write(90 - i * .5);            // Half Back
    Servo4.write(90 + i * .5);            // Bent
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){       
    Servo4.write(115 - i);                // Straight
    delay(15);
  }
  
  for (int i = 0; i <= 50; i += 1){       
    Servo5.write(90 - i * .5);            // Half Back
    Servo6.write(90 + i * .5);            // Bent
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){       
    Servo6.write(115 - i);                // Straight
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){
    Servo7.write(90 - i * .5);            // Forward
    Servo8.write(90 + i * .5);            // Straight
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){
    Servo1.write(65 + i * .5);                // Back             
    Servo2.write(115 - i * .5);               // Bend
    Servo5.write(65 + i * .5);                // Forward
    Servo6.write(65 + i * .5);                // Bend
    Servo3.write(65 + i * .5);                // Forward
    Servo4.write(65 + i * .5);                // Bend
    Servo7.write(65 + i * .5);                // Back
    Servo8.write(115 - i * .5);               // Bend
    delay(20);
  }
}

void left(){
  for (int i = 0; i <= 50; i += 1){
    Servo3.write(90 + i * .5);            // Forward
    Servo4.write(90 - i * .5);            // Straight
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){
    Servo1.write(90 + i * .5);            // Half Back
    Servo2.write(90 - i * .5);            // Bent
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){       
    Servo2.write(65 + i);                 // Straight
    delay(15);
  }
  
  for (int i = 0; i <= 50; i += 1){       
    Servo7.write(90 + i * .5);            // Half Back
    Servo8.write(90 - i * .5);            // Bent
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){       
    Servo8.write(65 + i);                 // Straight
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){
    Servo5.write(90 + i * .5);            // Forward
    Servo6.write(90 - i * .5);            // Straight
    delay(15);
  }

  for (int i = 0; i <= 50; i += 1){
    Servo3.write(115 - i * .5);              // Back             
    Servo4.write(65 + i * .5);               // Bend
    Servo7.write(115 - i * .5);              // Forward
    Servo8.write(115 - i * .5);              // Bend
    Servo1.write(115 - i * .5);              // Forward
    Servo2.write(115 - i * .5);              // Bend
    Servo5.write(115 - i * .5);              // Back
    Servo6.write(65 + i * .5);               // Bend
    delay(20);
  }
}

void back(){
  //Servo1.write(90);
}
