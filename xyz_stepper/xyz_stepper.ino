/*
 * ESP32 三轴步进滑台控制固件
 * 使用 AccelStepper 库，串口115200接收G-code
 *
 * 硬件配置：
 *   X轴: step=GPIO26, dir=GPIO16, 限位=GPIO5
 *   Y轴: step=GPIO25, dir=GPIO27, 限位=GPIO13
 *   Z轴: step=GPIO17, dir=GPIO14, 限位=GPIO23
 *   使能: GPIO12 (低电平使能)
 *   步进: 400步/mm, 最大速度1000mm/min, 加速度1000mm/s²
 *   X/Y行程195mm, Z行程100mm(可动范围95~195)
 *   限位开关在正方向末端，上拉输入，低电平触发
 */

#include <AccelStepper.h>

// === 引脚定义 ===
#define X_STEP_PIN    26
#define X_DIR_PIN     16
#define X_LIMIT_PIN   5

#define Y_STEP_PIN    25
#define Y_DIR_PIN     27
#define Y_LIMIT_PIN   13

#define Z_STEP_PIN    17
#define Z_DIR_PIN     14
#define Z_LIMIT_PIN   23

#define ENABLE_PIN    12

// === 参数 ===
#define STEPS_PER_MM        400.0
#define MAX_SPEED_MM_MIN    1000.0
#define ACCEL_MM_S2         1000.0
#define HOMING_SPEED_MM_MIN 1000.0
#define HOMING_FEED_MM_MIN  200.0
#define PULLOFF_MM          3.0

#define X_MAX_TRAVEL  195.0
#define Y_MAX_TRAVEL  195.0
#define Z_MAX_TRAVEL  100.0

// 归零后坐标 (195 - pulloff = 192)
#define HOME_POS_XY   192.0
#define HOME_POS_Z    192.0

// Z可动范围
#define Z_MIN_POS     95.0
#define Z_MAX_POS     195.0

// 方向反转
#define X_DIR_INVERT  false
#define Y_DIR_INVERT  true
#define Z_DIR_INVERT  true

// === 步进电机对象 ===
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// === 状态变量 ===
bool absoluteMode = true;   // G90绝对 / G91相对
bool isHomed = false;
bool motorsEnabled = true;

float currentX = 0, currentY = 0, currentZ = 0;  // 当前坐标(mm)
float feedRate = 1000.0;  // 默认进给速度 mm/min

String inputBuffer = "";

// === 辅助函数 ===
long mmToSteps(float mm) {
  return (long)(mm * STEPS_PER_MM);
}

float stepsToMm(long steps) {
  return (float)steps / STEPS_PER_MM;
}

float getMaxSpeed() {
  return MAX_SPEED_MM_MIN / 60.0 * STEPS_PER_MM;  // steps/s
}

float getAccel() {
  return ACCEL_MM_S2 * STEPS_PER_MM;  // steps/s²
}

float feedToStepsPerSec(float feed_mm_min) {
  if (feed_mm_min > MAX_SPEED_MM_MIN) feed_mm_min = MAX_SPEED_MM_MIN;
  return feed_mm_min / 60.0 * STEPS_PER_MM;
}

bool xLimitTriggered() { return digitalRead(X_LIMIT_PIN) == LOW; }
bool yLimitTriggered() { return digitalRead(Y_LIMIT_PIN) == LOW; }
bool zLimitTriggered() { return digitalRead(Z_LIMIT_PIN) == LOW; }

void enableMotors() {
  digitalWrite(ENABLE_PIN, LOW);
  motorsEnabled = true;
}

void disableMotors() {
  digitalWrite(ENABLE_PIN, HIGH);
  motorsEnabled = false;
}

// === 限位保护中断 ===
volatile bool limitHit = false;
volatile char limitAxis = ' ';

void IRAM_ATTR onXLimit() {
  limitHit = true;
  limitAxis = 'X';
}

void IRAM_ATTR onYLimit() {
  limitHit = true;
  limitAxis = 'Y';
}

void IRAM_ATTR onZLimit() {
  limitHit = true;
  limitAxis = 'Z';
}

// === 归零 ===
bool homeAxis(AccelStepper &stepper, int limitPin, bool dirInvert,
              float maxTravel, char axisName) {
  Serial.print("[Homing ");
  Serial.print(axisName);
  Serial.println("]");

  float seekSpeed = HOMING_SPEED_MM_MIN / 60.0 * STEPS_PER_MM;
  float feedSpeed = HOMING_FEED_MM_MIN / 60.0 * STEPS_PER_MM;

  // 正方向寻找限位
  long seekSteps = mmToSteps(maxTravel + 10);
  if (dirInvert) seekSteps = -seekSteps;  // 反转方向时物理正方向是负步数
  // 修正：限位在正方向末端，需要向正方向移动
  // dirInvert=false: 正方向=正步数
  // dirInvert=true: 正方向=负步数（因为方向反转）

  stepper.setMaxSpeed(seekSpeed);
  stepper.setAcceleration(getAccel());

  // 第一次：快速寻找限位
  stepper.move(dirInvert ? -seekSteps : seekSteps);
  // 实际上：向物理正方向移动
  long targetSteps = (long)(maxTravel * STEPS_PER_MM + 4000);
  if (dirInvert) targetSteps = -targetSteps;
  stepper.move(targetSteps);

  while (!digitalRead(limitPin) == LOW) {  // 等待限位触发
    stepper.run();
    if (stepper.distanceToGo() == 0) {
      Serial.print("Error: ");
      Serial.print(axisName);
      Serial.println(" limit not found");
      return false;
    }
  }
  stepper.stop();
  stepper.setCurrentPosition(stepper.currentPosition());
  delay(100);

  // 回退一段距离
  long pulloff = mmToSteps(PULLOFF_MM + 2);
  if (dirInvert) pulloff = -pulloff;
  stepper.move(-pulloff);  // 反方向回退
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(100);

  // 第二次：慢速精确寻找
  stepper.setMaxSpeed(feedSpeed);
  long slowSeek = mmToSteps(PULLOFF_MM + 5);
  if (dirInvert) slowSeek = -slowSeek;
  stepper.move(slowSeek);

  while (digitalRead(limitPin) != LOW) {
    stepper.run();
    if (stepper.distanceToGo() == 0) {
      Serial.print("Error: ");
      Serial.print(axisName);
      Serial.println(" limit not found on 2nd pass");
      return false;
    }
  }
  stepper.stop();
  delay(50);

  // 最终回退 pulloff 距离
  long finalPulloff = mmToSteps(PULLOFF_MM);
  if (dirInvert) finalPulloff = -finalPulloff;
  stepper.setMaxSpeed(feedSpeed);
  stepper.move(-finalPulloff);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  Serial.print(axisName);
  Serial.println(" homed OK");
  return true;
}

void homeAll() {
  enableMotors();
  limitHit = false;

  // 临时禁用中断（归零时不需要紧急停止）
  detachInterrupt(X_LIMIT_PIN);
  detachInterrupt(Y_LIMIT_PIN);
  detachInterrupt(Z_LIMIT_PIN);

  // Z先归零
  if (!homeAxis(stepperZ, Z_LIMIT_PIN, Z_DIR_INVERT, Z_MAX_TRAVEL + 95, 'Z')) {
    Serial.println("Error: Z homing failed");
    goto reattach;
  }
  stepperZ.setCurrentPosition(mmToSteps(HOME_POS_Z) * (Z_DIR_INVERT ? -1 : 1));
  currentZ = HOME_POS_Z;

  // X归零
  if (!homeAxis(stepperX, X_LIMIT_PIN, X_DIR_INVERT, X_MAX_TRAVEL, 'X')) {
    Serial.println("Error: X homing failed");
    goto reattach;
  }
  stepperX.setCurrentPosition(mmToSteps(HOME_POS_XY) * (X_DIR_INVERT ? -1 : 1));
  currentX = HOME_POS_XY;

  // Y归零
  if (!homeAxis(stepperY, Y_LIMIT_PIN, Y_DIR_INVERT, Y_MAX_TRAVEL, 'Y')) {
    Serial.println("Error: Y homing failed");
    goto reattach;
  }
  stepperY.setCurrentPosition(mmToSteps(HOME_POS_XY) * (Y_DIR_INVERT ? -1 : 1));
  currentY = HOME_POS_XY;

  isHomed = true;
  Serial.println("ok Homing complete");

reattach:
  // 重新挂载限位中断
  attachInterrupt(X_LIMIT_PIN, onXLimit, FALLING);
  attachInterrupt(Y_LIMIT_PIN, onYLimit, FALLING);
  attachInterrupt(Z_LIMIT_PIN, onZLimit, FALLING);
}

// === 移动执行 ===
void moveTo3D(float targetX, float targetY, float targetZ, float speed_mm_min) {
  // 限位检查
  targetX = constrain(targetX, 0, X_MAX_TRAVEL);
  targetY = constrain(targetY, 0, Y_MAX_TRAVEL);
  targetZ = constrain(targetZ, Z_MIN_POS, Z_MAX_POS);

  long stepsX = mmToSteps(targetX) * (X_DIR_INVERT ? -1 : 1);
  long stepsY = mmToSteps(targetY) * (Y_DIR_INVERT ? -1 : 1);
  long stepsZ = mmToSteps(targetZ) * (Z_DIR_INVERT ? -1 : 1);

  float speedSteps = feedToStepsPerSec(speed_mm_min);

  stepperX.setMaxSpeed(speedSteps);
  stepperY.setMaxSpeed(speedSteps);
  stepperZ.setMaxSpeed(speedSteps);

  stepperX.moveTo(stepsX);
  stepperY.moveTo(stepsY);
  stepperZ.moveTo(stepsZ);

  // 运行直到全部到位
  while (stepperX.distanceToGo() != 0 ||
         stepperY.distanceToGo() != 0 ||
         stepperZ.distanceToGo() != 0) {

    // 限位保护
    if (limitHit) {
      stepperX.stop();
      stepperY.stop();
      stepperZ.stop();
      // 等待减速停止
      while (stepperX.distanceToGo() != 0 ||
             stepperY.distanceToGo() != 0 ||
             stepperZ.distanceToGo() != 0) {
        stepperX.run();
        stepperY.run();
        stepperZ.run();
      }
      Serial.print("ALARM: Limit hit on ");
      Serial.println(limitAxis);

      // 回退3mm
      long pullback = mmToSteps(3.0);
      if (limitAxis == 'X') {
        stepperX.move(X_DIR_INVERT ? pullback : -pullback);
        while (stepperX.distanceToGo() != 0) stepperX.run();
      } else if (limitAxis == 'Y') {
        stepperY.move(Y_DIR_INVERT ? pullback : -pullback);
        while (stepperY.distanceToGo() != 0) stepperY.run();
      } else if (limitAxis == 'Z') {
        stepperZ.move(Z_DIR_INVERT ? pullback : -pullback);
        while (stepperZ.distanceToGo() != 0) stepperZ.run();
      }

      limitHit = false;
      break;
    }

    stepperX.run();
    stepperY.run();
    stepperZ.run();
  }

  // 更新当前坐标
  currentX = stepsToMm(stepperX.currentPosition() * (X_DIR_INVERT ? -1 : 1));
  currentY = stepsToMm(stepperY.currentPosition() * (Y_DIR_INVERT ? -1 : 1));
  currentZ = stepsToMm(stepperZ.currentPosition() * (Z_DIR_INVERT ? -1 : 1));

  Serial.println("ok");
}

// === G-code解析 ===
float parseValue(String &line, char code, float defaultVal) {
  int idx = line.indexOf(code);
  if (idx < 0) {
    idx = line.indexOf((char)(code + 32));  // 小写
  }
  if (idx < 0) return defaultVal;

  int start = idx + 1;
  int end = start;
  while (end < line.length() &&
         (isDigit(line[end]) || line[end] == '.' || line[end] == '-')) {
    end++;
  }
  if (end == start) return defaultVal;
  return line.substring(start, end).toFloat();
}

bool hasCode(String &line, char code) {
  return line.indexOf(code) >= 0 || line.indexOf((char)(code + 32)) >= 0;
}

void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // 查询状态
  if (line.charAt(0) == '?') {
    Serial.print("<");
    Serial.print(isHomed ? "Idle" : "Unknown");
    Serial.print("|MPos:");
    Serial.print(currentX, 3);
    Serial.print(",");
    Serial.print(currentY, 3);
    Serial.print(",");
    Serial.print(currentZ, 3);
    Serial.print("|F:");
    Serial.print(feedRate, 0);
    Serial.print("|Mode:");
    Serial.print(absoluteMode ? "G90" : "G91");
    Serial.println(">");
    return;
  }

  // 归零
  if (line.startsWith("$H") || line.startsWith("$h")) {
    homeAll();
    return;
  }

  // G-code
  if (line.charAt(0) == 'G' || line.charAt(0) == 'g') {
    int gCode = (int)parseValue(line, line.charAt(0), -1);

    switch (gCode) {
      case 0:   // G0 快速移动
      case 1: { // G1 进给移动
        if (!motorsEnabled) enableMotors();

        float speed = (gCode == 0) ? MAX_SPEED_MM_MIN : feedRate;
        if (hasCode(line, 'F')) {
          speed = parseValue(line, 'F', speed);
          feedRate = speed;
        }

        float targetX = currentX;
        float targetY = currentY;
        float targetZ = currentZ;

        if (absoluteMode) {
          if (hasCode(line, 'X')) targetX = parseValue(line, 'X', currentX);
          if (hasCode(line, 'Y')) targetY = parseValue(line, 'Y', currentY);
          if (hasCode(line, 'Z')) targetZ = parseValue(line, 'Z', currentZ);
        } else {
          if (hasCode(line, 'X')) targetX += parseValue(line, 'X', 0);
          if (hasCode(line, 'Y')) targetY += parseValue(line, 'Y', 0);
          if (hasCode(line, 'Z')) targetZ += parseValue(line, 'Z', 0);
        }

        moveTo3D(targetX, targetY, targetZ, speed);
        break;
      }

      case 90:  // 绝对坐标
        absoluteMode = true;
        Serial.println("ok G90");
        break;

      case 91:  // 相对坐标
        absoluteMode = false;
        Serial.println("ok G91");
        break;

      case 92: { // 设置当前坐标
        if (hasCode(line, 'X')) {
          currentX = parseValue(line, 'X', currentX);
          stepperX.setCurrentPosition(mmToSteps(currentX) * (X_DIR_INVERT ? -1 : 1));
        }
        if (hasCode(line, 'Y')) {
          currentY = parseValue(line, 'Y', currentY);
          stepperY.setCurrentPosition(mmToSteps(currentY) * (Y_DIR_INVERT ? -1 : 1));
        }
        if (hasCode(line, 'Z')) {
          currentZ = parseValue(line, 'Z', currentZ);
          stepperZ.setCurrentPosition(mmToSteps(currentZ) * (Z_DIR_INVERT ? -1 : 1));
        }
        Serial.println("ok G92");
        break;
      }

      default:
        Serial.print("Error: Unknown G");
        Serial.println(gCode);
        break;
    }
    return;
  }

  // M-code
  if (line.charAt(0) == 'M' || line.charAt(0) == 'm') {
    int mCode = (int)parseValue(line, line.charAt(0), -1);

    switch (mCode) {
      case 17:  // 使能电机
        enableMotors();
        Serial.println("ok M17 Motors enabled");
        break;

      case 18:  // 释放电机
      case 84:
        disableMotors();
        Serial.println("ok M18 Motors disabled");
        break;

      default:
        Serial.print("Error: Unknown M");
        Serial.println(mCode);
        break;
    }
    return;
  }

  Serial.print("Error: Unknown command: ");
  Serial.println(line);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(500);

  // 使能引脚
  pinMode(ENABLE_PIN, OUTPUT);
  enableMotors();

  // 限位开关引脚（上拉）
  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  // 步进电机配置
  stepperX.setMaxSpeed(getMaxSpeed());
  stepperX.setAcceleration(getAccel());
  stepperX.setPinsInverted(X_DIR_INVERT, false, false);

  stepperY.setMaxSpeed(getMaxSpeed());
  stepperY.setAcceleration(getAccel());
  stepperY.setPinsInverted(Y_DIR_INVERT, false, false);

  stepperZ.setMaxSpeed(getMaxSpeed());
  stepperZ.setAcceleration(getAccel());
  stepperZ.setPinsInverted(Z_DIR_INVERT, false, false);

  // 限位中断
  attachInterrupt(X_LIMIT_PIN, onXLimit, FALLING);
  attachInterrupt(Y_LIMIT_PIN, onYLimit, FALLING);
  attachInterrupt(Z_LIMIT_PIN, onZLimit, FALLING);

  Serial.println("XYZ Stepper Controller Ready");
  Serial.println("Commands: $H(home) G0/G1(move) G90/G91 G92 M17/M18 ?(status)");
}

// === Loop ===
void loop() {
  // 读取串口
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  // 持续运行步进电机（处理减速停止等）
  stepperX.run();
  stepperY.run();
  stepperZ.run();
}
