#include <Servo.h>
#include <ArduinoJson.h>

Servo servo1;
Servo servo2;
Servo servo3;

int servo1Pin = 5;
int servo2Pin = 3;
int servo3Pin = 11;

void setup() {
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  Serial.begin(115200);
  // 歸零
  servo1.write(90);
  servo2.write(90);
  servo3.write(180);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    StaticJsonDocument<200> doc;

    // 尝试解析 JSON
    DeserializationError error = deserializeJson(doc, data);

    // 检查是否解析成功
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.f_str());
      return;
    }

    int servoNumber = doc["servoNumber"]; 
    int position = doc["position"]; 

    // 控制伺服电机
    if (servoNumber == 1) {
      servo1.write(position);
    } else if (servoNumber == 2) {
      servo2.write(position);
    } else if (servoNumber == 3) {
      servo3.write(position);
    }

    // 发送 JSON 数据回 Python
    Serial.print("{\"servoNumber\":");
    Serial.print(servoNumber);
    Serial.print(",\"position\":");
    Serial.print(position);
    Serial.println("}");
    
    delay(20);
    while (Serial.available() > 0) {
      Serial.read(); // 清空串行缓冲区
    }
  }
}
