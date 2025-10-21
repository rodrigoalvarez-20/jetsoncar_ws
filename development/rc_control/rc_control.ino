#include <Servo.h>

Servo steeringServo;
Servo throttleServo;

const int steeringPin = 9;
const int throttlePin = 10;
const int claxonPin = 8;

int steeringAngle = 90;
int throttleAngle = 90;
String inputString = "";

void setup() {
  Serial.begin(115200);
  steeringServo.attach(steeringPin);
  throttleServo.attach(throttlePin);
  steeringServo.write(steeringAngle);
  throttleServo.write(throttleAngle);
  //Serial.println("Arduino ready: send 'steering,throttle' (e.g. 120,95)");
  pinMode(claxonPin, OUTPUT);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        processCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  int commaIndex = cmd.indexOf(',');
  if (commaIndex == -1){
    // Posible claxon
    if (cmd.startsWith("B:")){
      char clx = cmd[2];
      if (clx == '1'){
        tone(claxonPin, 920);
      }else{
        noTone(claxonPin);
      }
      return;
    }else{
      return;
    }
  }

  String sSteering = cmd.substring(0, commaIndex);
  String sThrottle = cmd.substring(commaIndex + 1);

  int steeringVal = constrain(sSteering.toInt(), 0, 180);
  int throttleVal = constrain(sThrottle.toInt(), 0, 180);

  steeringServo.write(steeringVal);
  throttleServo.write(throttleVal);

  steeringAngle = steeringVal;
  throttleAngle = throttleVal;

  //Serial.print("Steering="); Serial.print(steeringVal);
  //Serial.print(" | Throttle="); Serial.println(throttleVal);
}
