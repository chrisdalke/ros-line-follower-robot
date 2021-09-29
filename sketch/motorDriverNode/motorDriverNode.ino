int leftMotorEnablePin = 5;
int leftMotorDirPin1 = 16;
int leftMotorDirPin2 = 10;
int rightMotorEnablePin = 6;
int rightMotorDirPin1 = 8;
int rightMotorDirPin2 = 7;

bool hasTarget = false;
float targetSpeed = 0;
float targetDirection = 0;
float tempDirRad = 0;

const byte numChars = 64;
char receivedChars[numChars];

String speedKey = "wss.speed=";
String dirKey = "wss.dir=";


void setup() {
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT);
  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);
  pinMode(rightMotorDirPin2, OUTPUT);
  Serial.begin(115200);
}

// Write -1 -> 1 as a motor speed
void writeLeftMotor(float scaledSpeed) {
  if (scaledSpeed < 0) {
    digitalWrite(leftMotorDirPin1, HIGH);
    digitalWrite(leftMotorDirPin2, LOW);
  } else {
    digitalWrite(leftMotorDirPin1, LOW);
    digitalWrite(leftMotorDirPin2, HIGH);
  }
  analogWrite(leftMotorEnablePin, abs(scaledSpeed) * 255);
}
void writeRightMotor(float scaledSpeed) {
  if (scaledSpeed < 0) {
    digitalWrite(rightMotorDirPin1, HIGH);
    digitalWrite(rightMotorDirPin2, LOW);
  } else {
    digitalWrite(rightMotorDirPin1, LOW);
    digitalWrite(rightMotorDirPin2, HIGH);
  }
  analogWrite(rightMotorEnablePin, abs(scaledSpeed * 4) * 255);
}


void loop() {
  // Parse serial stream for speed and direction assignments
  bool newData = false;
  char currentChar;
  static byte bufferIndex = 0;
  while (Serial.available() > 0 && newData == false) {
    currentChar = Serial.read();
    if (currentChar != '\n') {
      receivedChars[bufferIndex++] = currentChar;
      if (bufferIndex >= numChars) {
          bufferIndex = numChars - 1;
      }
    } else {
      receivedChars[bufferIndex++] = '\0';
      bufferIndex = 0;
      newData = true;
    }
  }
  if (newData) {
    String keyValue =String(receivedChars);
    if (keyValue.startsWith(speedKey)) {
      targetSpeed = keyValue.substring(speedKey.length()).toFloat();
    }
    if (keyValue.startsWith(dirKey)) {
      targetDirection = keyValue.substring(dirKey.length()).toFloat();
    }
  }
        
  float leftSpeed = 0.0;
  float rightSpeed = 0.0;

  // Speed and direction should both be [-1, 1]
  // Speed: full backwards to full forwards
  // Direction: full turn left, center, full turn right
  float dirAngleRad = ((targetDirection * 90.0) * 71.0) / 4068.0;
  if (dirAngleRad < 0) {
    leftSpeed = cos(dirAngleRad * 2.0);
    rightSpeed = 1.0;
  } else {
    leftSpeed = 1.0;
    rightSpeed = cos(dirAngleRad * 2.0);
  }
  Serial.print(leftSpeed);
  Serial.print(" ");
  Serial.println(rightSpeed);
  
  leftSpeed *= targetSpeed;
  rightSpeed *= targetSpeed;
  writeLeftMotor(leftSpeed);
  writeRightMotor(rightSpeed * -1.0);

  delay(10);
}