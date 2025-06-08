/*!
 * @file  piezoVibrationSensor.ino
 * @brief This example detects vibration/pressure using a thin film pressure sensor
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  linfeng(490289303@qq.com)
 * @modifier  Grok (for adding EMA filter and integer output)
 * @version  V1.2
 * @date  2025-04-13
 */

#define sensorPin A0

// EMA滤波参数
float alpha = 0.1;  // 滤波因子，0.1表示较强的平滑，调整范围0到1
float filteredValue = 0;  // 存储滤波后的值

void setup()
{
  Serial.begin(115200);
  // 初始化滤波值为第一次读取
  filteredValue = analogRead(sensorPin);
}

void loop()
{
  int rawValue = analogRead(sensorPin);  // 读取原始值
  // 应用EMA滤波
  filteredValue = alpha * rawValue + (1 - alpha) * filteredValue;

  // 输出滤波后的值，转换为整数
  Serial.println((int)filteredValue);

  delay(50);  // 每50ms读取一次
}