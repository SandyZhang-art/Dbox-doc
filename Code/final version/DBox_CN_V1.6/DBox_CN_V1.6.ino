/*
 * 文件名: DBox
 * 
 * 功能描述:
 *          1、检测挥手动作与手翻转动作
 *          2、挥手动作播放（打鼓）音频，手翻转动作播放（金属薄片镲）音频
 *          3、挥手动作速度越快，声音越大并且灯越亮，速度越慢灯越暗，声音越小。手翻转动作也是一样的
 *          4、挥手动作幅度越大，灯的颜色越偏红紫，幅度越小，灯的颜色越偏蓝绿。手翻转动作也是一样的。
 *          5、薄膜压力传感器可以识别出目前是成人按还是儿童按压，从而修改检测速度的参数。（因为速度参数影响音量、颜色、时间等）
 *
 * 硬件需求:
 *         1. Arduino Nano 33 BLE Sense Rev2 (with BMI270 IMU)
 * 
 * 版本: V1.6
 * 
 * 日期: 2025-04-17
 * 
 * 库需求:
 *         - Arduino_BMI270_BMM150
 *         - Adafruit_NeoPixel
 *         - DFRobotDFPlayerMini
 *
 * 注意事项:
 *         - 确保硬件连接正确，尤其是MP3模块的串口连接（D0 RX, D1 TX）和电源（3.3V/5V）
 *         - SD卡需格式化为FAT32，并包含正确的音频文件
 *         - 压力传感器需校准，确保读取值在预期范围内
 */

#include "DFRobotDFPlayerMini.h"    // 引入DFRobot DFPlayer Mini库，用于控制MP3模块
#include "Arduino_BMI270_BMM150.h"  // 引入Arduino BMI270 IMU库，用于读取加速度计数据
#include <Adafruit_NeoPixel.h>      // 引入Adafruit NeoPixel库，用于控制RGB LED灯条

// =============================================== Pressure =================================================
// 压力传感器相关定义
#define PRESSURE_PIN A0         // 压力传感器连接的模拟引脚
#define KID_RANGE_MIN 170       // 儿童按压值的最小范围
#define KID_RANGE_MAX 512       // 儿童按压值的最大范围
#define ALDULT_RANGE_MIN 682    // 成人按压值的最小范围
#define ALDULT_RANGE_MAX 929    // 成人按压值的最大范围
#define PRESS_COMPLETE_TIMES 3  // 完成按压所需的次数
#define PRESS_THRESHOLD 170     // 按压检测的阈值
#define PRESS_PERIOD 50         // 按压周期

// EMA滤波参数
float alphaPressure = 0.1;         // EMA滤波因子，值越小平滑效果越强（范围0到1）
float filteredValuePressure = 0;   // 存储EMA滤波后的压力值
int pressCount = 0;                // 按压次数计数器
unsigned long pressStartTime = 0;  // 记录按压开始的时间戳（毫秒）
bool isPressing = false;           // 标记是否正在按压
const long pressDuration = 5000;   // 按压持续时间阈值（毫秒），5秒
bool boolCalibrationData = false;  // 校准数据标志（未使用）
int pressureAverage = 0;           // 按压值的累加平均值
// ==========================================================================================================

// ================================================= MP3 ====================================================
// MP3模块相关定义
#define MP3_VOLUME 30            // MP3默认音量（范围0-30）
DFRobotDFPlayerMini myDFPlayer;  // 创建DFPlayer Mini对象，用于控制MP3模块
// ==========================================================================================================

// ========================================= Arm Swing Detection =============================================
// 挥手动作检测相关定义
#define ARM_SWING_EMA_ALPHA 0.1                  // 挥手动作EMA滤波系数，控制平滑程度
#define ARM_SWING_TRACK 1                        // 挥手动作对应的MP3音轨编号（打鼓音效）
#define ARM_SWING_AMPLITUDE_MIN 0.02             // 挥手动作最小幅度阈值
#define ARM_SWING_AMPLITUDE_MAX 1                // 挥手动作最大幅度阈值
#define ARM_SWING_SAMPLE_INTERVAL_MS 10          // 挥手动作采样间隔（毫秒）
#define ARM_SWING_X_ACCEL_MIN_THRESHOLD 0.6      // X轴加速度最小阈值，用于检测挥手波形（单位：g）
#define ARM_SWING_Y_ACCEL_MAX_THRESHOLD 0.5      // Y轴加速度最大阈值，用于检测挥手波形（单位：g）
#define ARM_SWING_Z_ACCEL_MAX_THRESHOLD 0.2      // Z轴加速度最大阈值，用于检测挥手波形（单位：g）
#define ARM_SWING_X_TREND_DELTA_THRESHOLD 0.03   // X轴加速度变化阈值，用于趋势检测（单位：g）
#define ARM_SWING_RISING_Y_DELTA_THRESHOLD 0.02  // Y轴加速度上升阶段结束的最小变化阈值（单位：g）

bool refreshTimeLock = true;                           // 时间锁标志，防止重复记录挥手开始时间
bool isArmSwingRisingPhase = false;                    // 标记是否处于挥手动作的上升阶段
float prevAccelX, prevAccelY;                          // 前一次的X、Y轴加速度值
float minAccelX, minAccelY;                            // 挥手动作中的最小X、Y轴加速度值
unsigned long armSwingStartTime = 0;                   // 记录挥手动作开始的时间戳（毫秒）
unsigned int armSwingTrendCount = 0;                   // 挥手趋势一致性计数器
unsigned int armSwingWaveStartCount = 0;               // 挥手波形开始检测计数器
unsigned long lastSampleTimestamBMI270 = 0;            // 上次BMI270传感器采样的时间戳
unsigned long lastSampleTimestamPressure = 0;          // 上次压力传感器采样的时间戳
unsigned long armSwingLightOnTime = 0;                 // 挥手动作点亮RGB灯的时间戳
float armSwingDuration = 0.0;                          // 挥手动作的持续时间（秒）
float oldAccX;                                         // 记录挥手动作初始的X轴加速度值
float smoothedAccelX, smoothedAccelY, smoothedAccelZ;  // 平滑后的X、Y、Z轴加速度值

float minWaveSpeed = 0.2;  
float maxWaveSpeed = 2.0;  

// 挥手动作状态机枚举
enum SwingMotionState {
  SWING_IDLE_STATE,          // 空闲状态：等待挥手动作开始
  SWING_VERTICAL_COMPLETED,  // 手臂垂直状态完成
  SWING_HAND_COMPLETED,      // 挥手动作完成
  SWING_LIGHT_ON             // 点亮RGB灯状态
};
SwingMotionState swingCurrentState = SWING_IDLE_STATE;  // 当前挥手动作状态，初始化为空闲状态
// ============================================================================================================

// ========================================= Arm Flip Detection ===============================================
// 手翻转动作检测相关定义
#define SMOOTHED_ACCEL_Z_ERROR_MAX 1.04  // Z轴加速度稳定状态的最大误差范围
#define SMOOTHED_ACCEL_Z_ERROR_MIN 0.97  // Z轴加速度稳定状态的最小误差范围
#define LIGHT_TIME 500                   // RGB灯点亮的持续时间（毫秒）
#define FLIP_AMPLITUDE_MIN 0.15          // 手翻转动作最小幅度阈值
#define FLIP_AMPLITUDE_MAX 1             // 手翻转动作最大幅度阈值
#define BRIGHTNESS_MAX 255               // RGB灯最大亮度
#define BRIGHTNESS_MIN 0                 // RGB灯最小亮度
#define FLIP_TRACK 2                     // 手翻转动作对应的MP3音轨编号（金属薄片镲音效）

// 手翻转动作状态机枚举
enum FlipMotionState {
  FLIP_IDLE_STATE,        // 空闲状态：等待手翻转动作开始
  FLIP_STABLE_COMPLETED,  // 手稳定状态完成
  FLIP_COMPLETE,          // 手翻转动作完成
  FLIP_LIGHT_ON           // 点亮RGB灯状态
};
FlipMotionState flipCurrentState = FLIP_IDLE_STATE;  // 当前手翻转动作状态，初始化为空闲状态

unsigned long flipLightOnTime = 0;     // 记录手翻转动作点亮RGB灯的时间戳
unsigned long flipStartTimestamp = 0;  // 记录手翻转动作开始的时间戳（毫秒）

float minFlipSpeed = 0.12;    // 翻转动作的最小速度（秒）
float maxFlipSpeed = 0.4;     // 翻转动作的最大速度（秒）
float flip_prevAccelZ;        // 前一次的Z轴加速度值
float accelZAmplitude = 0;    // Z轴加速度的幅度变化
float armFlipDuration = 0.0;  // 手翻转动作的持续时间（秒）

uint8_t flipStableCnt = 0;     // 手翻转稳定状态计数器
uint8_t flipDowntrendCnt = 0;  // Z轴加速度下降趋势计数器

bool assistedJudgment = false;            // 辅助判断标志，用于手翻转动作检测
bool flipAccZDowntrendFlag = false;       // Z轴加速度下降趋势标志
bool flipYDowntrendCompleteFlag = false;  // Y轴下降趋势完成标志
// ============================================================================================================

// =============================================== Pressure ===================================================
/**
 * 初始化压力传感器
 * - 初始化EMA滤波的初始值为第一次读取的压力值
 */
void initPressureSensor() {
  filteredValuePressure = analogRead(PRESSURE_PIN);  // 读取初始压力值作为EMA滤波的起点
}

/**
 * 获取并处理压力传感器数据
 * - 使用EMA滤波平滑压力值
 * - 检测按压动作，记录按压次数和平均值
 * - 根据按压值判断用户类型（儿童或成人），调整挥手速度参数
 */
void getDataPressure() {
  int rawValue = analogRead(PRESSURE_PIN);  // 读取原始压力值
  // 应用EMA滤波公式：新值 = α * 当前值 + (1 - α) * 前值
  filteredValuePressure = alphaPressure * rawValue + (1 - alphaPressure) * filteredValuePressure;

  int storedFilteredValue = (int)filteredValuePressure;  // 将滤波值转换为整数

  // 检测压力传感器按压
  if (storedFilteredValue > PRESS_THRESHOLD && !isPressing) {
    // 检测到新的按压动作
    isPressing = true;
    pressStartTime = millis();  // 记录按压开始时间
  } else if (storedFilteredValue > PRESS_THRESHOLD && isPressing) {
    // 持续按压中，检查是否达到指定持续时间
    if (millis() - pressStartTime >= pressDuration) {
      pressCount++;                                             // 增加按压次数
      pressureAverage = pressureAverage + storedFilteredValue;  // 累加压力值
      isPressing = false;                                       // 重置按压状态
      pressStartTime = millis();                                // 记录按压结束时间
      Serial.println("Press completed! Count: " + String(pressCount) + 
      "压力值" + String(pressureAverage));
    }
  } else if (storedFilteredValue <= PRESS_THRESHOLD && isPressing) {
    // 压力释放，重置按压状态
    isPressing = false;
    pressStartTime = millis();  // 记录释放时间
  }

  // 检查按压间隔时间，超时重置计数
  if (pressCount > 0 && !isPressing && (millis() - pressStartTime >= 10000)) {
    pressCount = 0;       // 重置按压次数
    pressureAverage = 0;  // 重置压力累加值
    Serial.println("Interval too long! Resetting press count.");
  }

  // 检查是否完成3次按压
  if (pressCount >= PRESS_COMPLETE_TIMES) {
    int pressValue = pressureAverage / 3;  // 计算平均压力值
    // 根据压力值判断用户类型
    if ((KID_RANGE_MIN <= pressValue) && (pressValue <= KID_RANGE_MAX)) {
      minWaveSpeed = 0.5;  // 儿童模式，设置较慢的挥手速度阈值
      minFlipSpeed = 0.15;
    }
    if ((ALDULT_RANGE_MIN <= pressValue) && (pressValue <= ALDULT_RANGE_MIN)) {
      minWaveSpeed = 0.2;  // 成人模式，设置较快的挥手速度阈值
      minFlipSpeed = 0.12;
    }
    Serial.print("目前手臂摆动最快速度：");
    Serial.print(minWaveSpeed);
    Serial.print(" S");
    Serial.print(" | 目前手臂翻转最快速度：");
    Serial.print(minFlipSpeed);
    Serial.print(" S");
    Serial.print(" | 压力值：");
    Serial.print(pressValue);

    pressCount = 0;  // 重置计数以便下一次检测
  }
}
// ============================================================================================================

// ========================================== MP3 mini player =================================================
/**
 * 初始化MP3模块
 * - 配置串口通信
 * - 初始化DFPlayer Mini模块
 * - 设置默认音量
 */
void initMP3miniplayer() {
  Serial1.begin(9600);  // 初始化硬件串口Serial1（D0 RX, D1 TX）
  Serial.println("Serial1 initialized at 9600 baud.");
  // 初始化MP3模块，启用ACK和重置功能
  if (!myDFPlayer.begin(Serial1, true, true)) {
    Serial.println("MP3 Player failed to initialize!");
    Serial.println("Check: 1. Wiring (D0->TX, D1->RX), 2. Power (3.3V/5V), 3. SD card");
    while (true)
      ;  // 初始化失败，程序停止运行
  }
  Serial.println("MP3 Player initialized successfully!");
  myDFPlayer.volume(MP3_VOLUME);  // 设置MP3音量为默认值
  Serial.println("Volume set to 30.");
}
// ============================================================================================================

// =========================================== RGB Light Strip ================================================
// RGB灯条相关定义
#define RGB_PIN 6     // RGB灯条连接的数字引脚
#define NUMPIXELS 24  // RGB灯条的LED数量

Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);  // 创建NeoPixel对象，配置为GRB颜色顺序和800kHz频率

/**
 * 初始化RGB灯条
 * - 调用NeoPixel库的begin方法，启用灯条
 */
void initializeRGB() {
  pixels.begin();  // 初始化NeoPixel灯条
}

/**
 * 清除RGB灯条颜色
 * - 关闭所有LED并更新显示
 */
void cleanColor() {
  pixels.clear();  // 清除所有LED颜色
  pixels.show();   // 更新灯条显示
}

/**
 * 将动作幅度映射到RGB颜色
 * - 幅度小时偏蓝绿，幅度大时偏红紫
 * - 使用线性插值计算红、绿、蓝分量
 * @param amplitude 当前动作幅度
 * @param minAmplitude 最小幅度阈值
 * @param maxAmplitude 最大幅度阈值
 * @return 32位颜色值（RGB）
 */
uint32_t mapAmplitudeToColor(float amplitude, float minAmplitude, float maxAmplitude) {
  if (amplitude <= minAmplitude) amplitude = minAmplitude;  // 限制幅度下限
  if (amplitude >= maxAmplitude) amplitude = maxAmplitude;  // 限制幅度上限

  float t = (amplitude - minAmplitude) / (maxAmplitude - minAmplitude);  // 归一化幅度

  uint8_t red = (uint8_t)(0 + t * 255);      // 红色分量：0到255
  uint8_t green = (uint8_t)(150 - t * 150);  // 绿色分量：150到0
  uint8_t blue = (uint8_t)(150 + t * 105);   // 蓝色分量：150到255

  return pixels.Color(red, green, blue);  // 返回RGB颜色值
}
// ============================================================================================================

// ======================================= Motion Detection Core ==============================================
/**
 * 初始化IMU传感器
 * - 初始化串口通信
 * - 初始化BMI270 IMU传感器
 * - 输出加速度计采样率
 * - 关闭RGB灯条
 */
void initializeSensors() {
  while (!Serial)
    ;  // 等待串口连接
  Serial.println("Starting operation");

  if (!IMU.begin()) {  // 初始化IMU传感器
    Serial.println("IMU initialization failed!");
    while (1)
      ;  // 初始化失败，程序停止运行
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());  // 输出加速度计采样率
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Smoothed acceleration values (g) and changes");
  Serial.println("X\tY\tZ\tdeltaX\tdeltaY");
  cleanColor();  // 关闭RGB灯条
}

/**
 * 检测挥手动作
 * - 使用状态机实现挥手动作的检测逻辑
 * - 根据加速度数据判断动作阶段
 * - 控制RGB灯和MP3音效
 */
void detectArmSwing() {
  switch (swingCurrentState) {
    case SWING_LIGHT_ON:
      // 检查RGB灯点亮时间是否超过指定持续时间
      if (millis() - armSwingLightOnTime >= LIGHT_TIME) {
        cleanColor();                          // 关闭RGB灯
        swingCurrentState = SWING_IDLE_STATE;  // 切换到空闲状态
      }
      break;
    case SWING_IDLE_STATE:
      // 判断手臂是否处于垂直状态（基于加速度阈值）
      if (abs(smoothedAccelX) > ARM_SWING_X_ACCEL_MIN_THRESHOLD && 
      abs(smoothedAccelY) < ARM_SWING_Y_ACCEL_MAX_THRESHOLD && 
      abs(smoothedAccelZ) < ARM_SWING_Z_ACCEL_MAX_THRESHOLD) {
        armSwingWaveStartCount++;  // 增加垂直状态计数
      } else {
        armSwingWaveStartCount = 0;  // 重置计数
      }
      // 确认手臂垂直状态完成
      if (armSwingWaveStartCount == 50) {
        oldAccX = abs(smoothedAccelX);                 // 记录初始X轴加速度
        swingCurrentState = SWING_VERTICAL_COMPLETED;  // 切换到垂直完成状态
        Serial.println("Hand vertical ready");
      }
      break;

    case SWING_VERTICAL_COMPLETED:
      // 检测挥手动作（X轴加速度下降趋势）
      if ((prevAccelX - abs(smoothedAccelX)) > ARM_SWING_X_TREND_DELTA_THRESHOLD) {
        armSwingTrendCount++;  // 增加趋势计数
        if (armSwingTrendCount >= 3) {
          // 确认挥手动作开始
          if (refreshTimeLock) {
            Serial.println("Record time start");
            armSwingStartTime = millis();  // 记录挥手开始时间
            refreshTimeLock = false;       // 锁定时间记录
          }
          isArmSwingRisingPhase = true;  // 标记进入上升阶段
          Serial.println("Waving~~");
        }
        minAccelX = prevAccelX;  // 记录最小X轴加速度
        minAccelY = prevAccelY;  // 记录最小Y轴加速度
      }
      prevAccelX = abs(smoothedAccelX);  // 更新前一次X轴加速度
      prevAccelY = abs(smoothedAccelY);  // 更新前一次Y轴加速度

      // 判断挥手动作是否结束（上升阶段后Y轴加速度增加）
      if ((isArmSwingRisingPhase) && (abs(smoothedAccelX) > minAccelX) 
      && ((abs(smoothedAccelY) - minAccelY) >= ARM_SWING_RISING_Y_DELTA_THRESHOLD)) {
        swingCurrentState = SWING_HAND_COMPLETED;  // 切换到挥手完成状态
      }

      // 超时检测（2秒未完成动作）
      if (!refreshTimeLock && (millis() - armSwingStartTime >= 2000)) {
        refreshTimeLock = true;
        armSwingWaveStartCount = 0;
        armSwingTrendCount = 0;
        isArmSwingRisingPhase = false;
        swingCurrentState = SWING_IDLE_STATE;  // 切换到空闲状态
      }
      break;

    case SWING_HAND_COMPLETED:
      // 挥手动作完成，处理结果
      refreshTimeLock = true;
      armSwingWaveStartCount = 0;
      armSwingTrendCount = 0;
      isArmSwingRisingPhase = false;
      armSwingDuration = (millis() - armSwingStartTime) / 1000.0;  // 计算挥手持续时间

      Serial.print("Wave it over");
      Serial.print(" | Duration: ");
      Serial.print(armSwingDuration);
      Serial.println("s");

      // 超时或误触检测
      if ((armSwingDuration > 2) || (armSwingDuration < 0.2)) {
        cleanColor();       // 关闭RGB灯
        myDFPlayer.stop();  // 停止MP3播放
        armSwingDuration = 0;
        swingCurrentState = SWING_IDLE_STATE;  // 切换到空闲状态
        break;
      }

      // 设置RGB灯亮度（速度越快亮度越高）
      int brightness = map(armSwingDuration * 1000, minWaveSpeed * 1000, 
      maxWaveSpeed * 1000, BRIGHTNESS_MAX, BRIGHTNESS_MIN);
      pixels.setBrightness(brightness);

      // 设置RGB灯颜色（幅度越大越偏红紫）
      float amplitude = abs(abs(smoothedAccelX) - oldAccX);
      uint32_t color = mapAmplitudeToColor(amplitude, ARM_SWING_AMPLITUDE_MIN, 
      ARM_SWING_AMPLITUDE_MAX);

      // 设置MP3音量（速度越快音量越大）
      int mp3Volume = map(armSwingDuration * 1000, minWaveSpeed * 1000, 
      maxWaveSpeed * 1000, MP3_VOLUME, 0);
      myDFPlayer.volume(mp3Volume);
      myDFPlayer.play(ARM_SWING_TRACK);  // 播放打鼓音效

      Serial.print("amplitude:");
      Serial.print(amplitude);
      Serial.print(" | ");
      Serial.println(brightness);

      // 更新RGB灯颜色
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, color);
      }
      pixels.show();                       // 显示RGB灯效果
      armSwingLightOnTime = millis();      // 记录灯点亮时间
      swingCurrentState = SWING_LIGHT_ON;  // 切换到点亮状态
      break;
  }
}

/**
 * 检测手翻转动作
 * - 使用状态机实现手翻转动作的检测逻辑
 * - 根据加速度数据判断动作阶段
 * - 控制RGB灯和MP3音效
 */
void detectArmFlip() {
  float absSmoothedAccelZ = abs(smoothedAccelZ);

  switch (flipCurrentState) {
    case FLIP_LIGHT_ON:
      // 检查RGB灯点亮时间是否超过指定持续时间
      if (millis() - flipLightOnTime >= LIGHT_TIME) {
        myDFPlayer.stop();                   // 停止MP3播放
        cleanColor();                        // 关闭RGB灯
        flipCurrentState = FLIP_IDLE_STATE;  // 切换到空闲状态
      }
      break;

    case FLIP_IDLE_STATE:
      // 判断手臂是否稳定（Z轴接近1g，X、Y轴接近0）
      if ((SMOOTHED_ACCEL_Z_ERROR_MIN < absSmoothedAccelZ) && (absSmoothedAccelZ < SMOOTHED_ACCEL_Z_ERROR_MAX) && (abs(smoothedAccelX) < 0.3) && (abs(smoothedAccelY) < 0.3)) {
        flipStableCnt++;  // 增加稳定状态计数
      } else {
        flipStableCnt = 0;  // 重置计数
      }
      // Serial.print("absSmoothedAccelZ: ");
      // Serial.print(absSmoothedAccelZ);
      // Serial.print(" | flip_prevAccelZ: ");
      // Serial.println(flip_prevAccelZ);

      // 确认稳定状态
      if (flipStableCnt == 5) {
        flipStableCnt = 0;
        flipCurrentState = FLIP_STABLE_COMPLETED;  // 切换到稳定完成状态
        flip_prevAccelZ = absSmoothedAccelZ;       // 记录当前Z轴加速度
        Serial.println("目前处于稳定状态");
      }
      break;

    case FLIP_STABLE_COMPLETED:
      smoothedAccelZ = absSmoothedAccelZ;    // 取Z轴加速度绝对值
      smoothedAccelY = abs(smoothedAccelY);  // 取Y轴加速度绝对值

      // 检测Y轴加速度变化，触发辅助判断
      if ((smoothedAccelY > 0.08) && (!assistedJudgment)) {
        assistedJudgment = true;
      }

      // 检测Z轴加速度下降趋势
      if ((abs(absSmoothedAccelZ - flip_prevAccelZ) > 0.04) && (assistedJudgment)) {
        flipAccZDowntrendFlag = true;
        Serial.print("absSmoothedAccelZ: ");
        Serial.print(absSmoothedAccelZ);
        Serial.print(" | flip_prevAccelZ: ");
        Serial.print(flip_prevAccelZ);
        Serial.print(" | SmoothedAccelX: ");
        Serial.print(abs(smoothedAccelX));
        Serial.print(" | SmoothedAccelY: ");
        Serial.println(abs(smoothedAccelY));
      }
      if ((abs(smoothedAccelY) > 0.10) && (!flipYDowntrendCompleteFlag)) {
        flipAccZDowntrendFlag = false;
      }
      // 记录下降开始时间和初始Z轴幅度
      if ((flipAccZDowntrendFlag) && (!flipYDowntrendCompleteFlag)) {
        flipYDowntrendCompleteFlag = true;
        flipStartTimestamp = millis();        // 记录翻转开始时间
        accelZAmplitude = absSmoothedAccelZ;  // 记录初始Z轴加速度
      }

      // 检测Z轴加速度稳定（下降趋势结束）
      if ((abs(absSmoothedAccelZ - flip_prevAccelZ) < 0.04) && (flipYDowntrendCompleteFlag)) {
        flipDowntrendCnt++;  // 增加稳定计数
      } else {
        flipDowntrendCnt = 0;  // 重置计数
      }

      // 确认翻转动作完成
      if ((flipDowntrendCnt >= 10) && (flipYDowntrendCompleteFlag)) {
        flipDowntrendCnt = 0;
        flipCurrentState = FLIP_COMPLETE;  // 切换到翻转完成状态
      }
      flip_prevAccelZ = absSmoothedAccelZ;  // 更新前一次Z轴加速度

      if (smoothedAccelX > 0.18) {
        flipCurrentState = FLIP_COMPLETE;  // 切换到翻转完成状态
      }
      break;

    case FLIP_COMPLETE:
      // 翻转动作完成，处理结果
      unsigned long duration = millis() - flipStartTimestamp;
      armFlipDuration = duration / 1000.0;  // 计算翻转持续时间
      Serial.print("Duration: ");
      Serial.print(armFlipDuration);
      Serial.println("S");

      // 设置RGB灯亮度（速度越快亮度越高）
      int brightness = map(armFlipDuration * 1000, minFlipSpeed * 1000, maxFlipSpeed * 1000, BRIGHTNESS_MAX, BRIGHTNESS_MIN);
      // Serial.print("brightness: ");
      // Serial.println(brightness);
      brightness = constrain(brightness, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
      pixels.setBrightness(brightness);

      // 计算Z轴加速度幅度
      accelZAmplitude = abs(accelZAmplitude - absSmoothedAccelZ);

      // 设置RGB灯颜色（幅度越大越偏红紫）
      uint32_t color = mapAmplitudeToColor(accelZAmplitude, FLIP_AMPLITUDE_MIN, FLIP_AMPLITUDE_MAX);
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, color);
      }
      // 设置MP3音量（速度越快音量越大）
      int mp3Volume = map(armFlipDuration * 1000, minFlipSpeed * 1000, maxFlipSpeed * 1000, MP3_VOLUME, 0);
      myDFPlayer.volume(mp3Volume);
      myDFPlayer.play(FLIP_TRACK);  // 播放金属薄片镲音效

      pixels.show();  // 显示RGB灯效果

      Serial.print("brightness: ");
      Serial.println(brightness);
      Serial.print("accelZAmplitude: ");
      Serial.println(accelZAmplitude);

      // Serial.print("absSmoothedAccelZ: ");
      // Serial.print(absSmoothedAccelZ);
      // Serial.print(" | flip_prevAccelZ: ");
      // Serial.println(flip_prevAccelZ);
      // Serial.println(smoothedAccelY);
      // Serial.println(smoothedAccelX);


      flipLightOnTime = millis();  // 记录灯点亮时间
      assistedJudgment = false;
      flipAccZDowntrendFlag = false;
      flipYDowntrendCompleteFlag = false;
      flipCurrentState = FLIP_LIGHT_ON;  // 切换到点亮状态
      break;
  }
}

/**
 * 处理传感器数据
 * - 读取加速度计数据
 * - 应用EMA滤波平滑数据
 * - 调用挥手和翻转检测函数
 */
void processSensorData() {
  float rawAccelX, rawAccelY, rawAccelZ;  // 原始加速度值

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(rawAccelX, rawAccelY, rawAccelZ);  // 读取加速度数据

    // 应用EMA滤波平滑加速度值
    smoothedAccelX = ARM_SWING_EMA_ALPHA * rawAccelX + (1 - ARM_SWING_EMA_ALPHA) * smoothedAccelX;
    smoothedAccelY = ARM_SWING_EMA_ALPHA * rawAccelY + (1 - ARM_SWING_EMA_ALPHA) * smoothedAccelY;
    smoothedAccelZ = ARM_SWING_EMA_ALPHA * rawAccelZ + (1 - ARM_SWING_EMA_ALPHA) * smoothedAccelZ;

    detectArmSwing();  // 检测挥手动作
    detectArmFlip();   // 检测手翻转动作
  }
}
// ============================================================================================================

// ================================================ Main Code =================================================
/**
 * Arduino初始化函数
 * - 初始化串口通信
 * - 初始化MP3模块
 * - 初始化IMU传感器
 * - 初始化压力传感器
 */
void setup() {
  Serial.begin(115200);  // 初始化串口，波特率115200
  initMP3miniplayer();   // 初始化MP3模块
  initializeSensors();   // 初始化IMU传感器
  initPressureSensor();  // 初始化压力传感器
}

/**
 * Arduino主循环函数
 * - 定期采集压力传感器数据（每50ms）
 * - 定期处理IMU传感器数据（每ARM_SWING_SAMPLE_INTERVAL_MS毫秒）
 */
void loop() {
  unsigned long currentTimestampBMI270 = millis();    // 当前时间戳（IMU）
  unsigned long currentTimestampPressure = millis();  // 当前时间戳（压力）

  // 每50ms采集一次压力数据
  if (currentTimestampPressure - lastSampleTimestamPressure >= PRESS_PERIOD) {
    getDataPressure();                                      // 处理压力传感器数据
    lastSampleTimestamPressure = currentTimestampPressure;  // 更新上次采样时间
  }

  // 每ARM_SWING_SAMPLE_INTERVAL_MS毫秒处理一次IMU数据
  if (currentTimestampBMI270 - lastSampleTimestamBMI270 >= ARM_SWING_SAMPLE_INTERVAL_MS) {
    processSensorData();                                // 处理IMU传感器数据
    lastSampleTimestamBMI270 = currentTimestampBMI270;  // 更新上次采样时间
  }
}