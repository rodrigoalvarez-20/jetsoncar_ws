#include <Servo.h>

Servo steeringServo;
Servo throttleServo;

const int steeringPin = 9;
const int throttlePin = 10;
const int claxonPin   = 8;

int steeringAngle = 90;
int throttleAngle = 90;

// ===== BUFFER SERIAL =====
#define BUFFER_SIZE 32
char inputBuffer[BUFFER_SIZE];
byte bufferIndex = 0;

void setup() {
  Serial.begin(115200);

  steeringServo.attach(steeringPin);
  throttleServo.attach(throttlePin);

  steeringServo.write(steeringAngle);
  throttleServo.write(throttleAngle);

  pinMode(claxonPin, OUTPUT);

  Serial.println("Arduino ready: send 'steering,throttle' (e.g. 120,95)");
}

void loop() {

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      if (bufferIndex > 0) {
        inputBuffer[bufferIndex] = '\0';  // terminar string
        processCommand(inputBuffer);
        bufferIndex = 0;
      }

    } else {
      if (bufferIndex < BUFFER_SIZE - 1) {
        inputBuffer[bufferIndex++] = c;
      }
      // Si se llena el buffer, ignora caracteres extra
    }
  }
}

void processCommand(char* cmd) {

  // ===== CLAXON =====
  if (strncmp(cmd, "B:", 2) == 0) {
    if (cmd[2] == '1') {
      tone(claxonPin, 920);
    } else {
      noTone(claxonPin);
    }
    return;
  }

  // ===== BUSCAR COMA =====
  char* commaPtr = strchr(cmd, ',');
  if (commaPtr == NULL) {
    return;  // comando inválido
  }

  *commaPtr = '\0';  // separar en dos strings

  int steeringVal = atoi(cmd);
  int throttleVal = atoi(commaPtr + 1);

  steeringVal = constrain(steeringVal, 45, 135);
  throttleVal = constrain(throttleVal, 45, 135);

  steeringServo.write(steeringVal);
  throttleServo.write(throttleVal);

  steeringAngle = steeringVal;
  throttleAngle = throttleVal;
}
