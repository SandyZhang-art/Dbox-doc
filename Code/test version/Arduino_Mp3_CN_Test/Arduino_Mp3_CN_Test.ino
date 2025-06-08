#include "DFRobotDFPlayerMini.h"

DFRobotDFPlayerMini myDFPlayer;

void setup() {
  Serial.begin(9600);   // USB 串口用于调试
  while (!Serial) {
    ;
  }
  Serial.println("Starting MP3 Player...");

  Serial1.begin(9600);  // 硬件串口 Serial1 (D0 RX, D1 TX)
  Serial.println("Serial1 initialized at 9600 baud.");

  // 初始化 MP3 模块，启用 ACK 和重置
  if (!myDFPlayer.begin(Serial1, true, true)) {
    Serial.println("MP3 Player failed to initialize!");
    Serial.println("Check: 1. Wiring (D0->TX, D1->RX), 2. Power (3.3V/5V), 3. SD card");
    while (true);  // 停止运行，便于调试
  }
  Serial.println("MP3 Player initialized successfully!");

  myDFPlayer.volume(30);  // 音量设定为 30 (0-30)
  Serial.println("Volume set to 30.");
  myDFPlayer.play(1);     // 播放第 3 个文件 (0003.mp3)
  Serial.println("Playing file 0003.mp3...");
}

void loop() {
  if (myDFPlayer.available()) {
    int type = myDFPlayer.readType();
    int value = myDFPlayer.read();
    Serial.print("Event Type: ");
    Serial.print(type);
    Serial.print(", Value: ");
    Serial.println(value);

    if (type == DFPlayerPlayFinished && value == 1) {
      Serial.println("File 0003.mp3 finished, replaying...");
      myDFPlayer.play(1);  // 循环播放 0003.mp3
    }
  }
}