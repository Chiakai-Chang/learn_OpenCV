#include <ArduinoJson.h>
#include <ESP32Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

int minUs = 1000;
int maxUs = 2000;

int servo1Pin = 10;
int servo2Pin = 18;
int servo3Pin = 3;

void setup() {
  // 分配所有定时器
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Serial.begin(115200);

  // 设置伺服频率
  servo1.setPeriodHertz(50);    // 标准 50 Hz 伺服
  servo2.setPeriodHertz(50);    // 标准 50 Hz 伺服
  servo3.setPeriodHertz(50);    // 标准 50 Hz 伺服

  servo1.attach(servo1Pin, minUs, maxUs);
  servo2.attach(servo2Pin, minUs, maxUs);
  servo3.attach(servo3Pin, minUs, maxUs);
  
  // 歸零
  servo1.write(90);
  delay(20);
  servo2.write(90);
  delay(20);
  servo3.write(180);
  delay(20);
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
