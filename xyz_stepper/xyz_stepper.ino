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
#define X_LIMIT_PIN   5   // X轴限位

#define Y_STEP_PIN    25
#define Y_DIR_PIN     27
#define Y_LIMIT_PIN   13  // Y轴限位

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

// 方向反转 (与FluidNC配置一致: X有:low后缀需要反转，Y/Z不需要)
#define X_DIR_INVERT  false
#define Y_DIR_INVERT  false
#define Z_DIR_INVERT  false

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

// 去抖确认限位：5次读取中至少3次LOW即确认触发
bool confirmLimit(int limitPin) {
  int lowCount = 0;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(limitPin) == LOW) lowCount++;
    delayMicroseconds(200);
  }
  return (lowCount >= 3);
}

// 等待电机减速停止
void waitStop(AccelStepper &stepper) {
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    yield();
  }
}

bool homeAxis(AccelStepper &stepper, int limitPin, float maxTravel, char axisName, int homeDir) {
  // homeDir: +1=正步数方向找限位, -1=负步数方向找限位
  Serial.print("[Homing ");
  Serial.print(axisName);
  Serial.print("] limit_pin=");
  Serial.print(limitPin);
  Serial.print(" dir=");
  Serial.print(homeDir);
  Serial.print(" state=");
  Serial.println(digitalRead(limitPin) == LOW ? "TRIGGERED" : "open");

  float seekSpeed = HOMING_SPEED_MM_MIN / 60.0 * STEPS_PER_MM;
  float feedSpeed = HOMING_FEED_MM_MIN / 60.0 * STEPS_PER_MM;

  stepper.setAcceleration(getAccel());

  // 如果限位已经触发，先回退(反方向)
  if (confirmLimit(limitPin)) {
    Serial.println("  Limit already triggered, backing off...");
    stepper.setMaxSpeed(feedSpeed);
    stepper.move(-homeDir * mmToSteps(PULLOFF_MM + 5));
    waitStop(stepper);
    delay(100);
    if (confirmLimit(limitPin)) {
      Serial.println("  Error: still triggered after backoff");
      return false;
    }
  }

  // === 第一次：快速向限位方向寻找 ===
  stepper.setMaxSpeed(seekSpeed);
  long seekDist = homeDir * mmToSteps(maxTravel + 10);
  stepper.move(seekDist);

  {
    unsigned long lastPrint = 0;
    unsigned long startTime = millis();
    while (true) {
      stepper.run();
      yield();  // 防止看门狗复位
      // 先快速检查，只有读到LOW才做去抖确认
      if (digitalRead(limitPin) == LOW && confirmLimit(limitPin)) {
        Serial.println("  Seek: limit confirmed");
        break;
      }
      if (stepper.distanceToGo() == 0) {
        Serial.print("Error: ");
        Serial.print(axisName);
        Serial.println(" limit not found");
        return false;
      }
      // 超时保护：30秒未找到限位则停止
      if (millis() - startTime > 30000) {
        stepper.stop();
        waitStop(stepper);
        Serial.print("Error: ");
        Serial.print(axisName);
        Serial.println(" homing timeout (30s)");
        return false;
      }
      // 每500ms打印一次限位状态
      if (millis() - lastPrint > 500) {
        lastPrint = millis();
        Serial.print("  ");
        Serial.print(axisName);
        Serial.print(" pin");
        Serial.print(limitPin);
        Serial.print("=");
        Serial.println(digitalRead(limitPin));
      }
    }
  }
  // 减速停止
  stepper.stop();
  waitStop(stepper);
  delay(100);

  // === 回退一段距离(反方向=远离限位) ===
  stepper.setMaxSpeed(seekSpeed);
  stepper.move(-homeDir * mmToSteps(PULLOFF_MM + 2));
  waitStop(stepper);
  delay(100);

  // === 第二次：慢速精确寻找 ===
  stepper.setMaxSpeed(feedSpeed);
  stepper.move(homeDir * mmToSteps(PULLOFF_MM + 5));

  while (true) {
    stepper.run();
    yield();
    if (digitalRead(limitPin) == LOW && confirmLimit(limitPin)) {
      Serial.println("  Feed: limit confirmed");
      break;
    }
    if (stepper.distanceToGo() == 0) {
      Serial.print("Error: ");
      Serial.print(axisName);
      Serial.println(" limit not found on 2nd pass");
      return false;
    }
  }
  stepper.stop();
  waitStop(stepper);
  delay(50);

  // === 最终回退 pulloff 距离 ===
  stepper.setMaxSpeed(feedSpeed);
  stepper.move(-homeDir * mmToSteps(PULLOFF_MM));
  waitStop(stepper);

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

  // Z先归零：负步数方向=物理向上=限位方向
  if (!homeAxis(stepperZ, Z_LIMIT_PIN, Z_MAX_TRAVEL + 95, 'Z', -1)) {
    Serial.println("Error: Z homing failed");
    goto reattach;
  }
  // 归零后设置当前位置=192mm（限位在195，回退了3mm）
  stepperZ.setCurrentPosition(mmToSteps(HOME_POS_Z));
  currentZ = HOME_POS_Z;

  // X归零：正步数方向=限位方向
  if (!homeAxis(stepperX, X_LIMIT_PIN, X_MAX_TRAVEL, 'X', 1)) {
    Serial.println("Error: X homing failed");
    goto reattach;
  }
  stepperX.setCurrentPosition(mmToSteps(HOME_POS_XY));
  currentX = HOME_POS_XY;

  // Y归零：负步数方向=往后=限位方向
  if (!homeAxis(stepperY, Y_LIMIT_PIN, Y_MAX_TRAVEL, 'Y', -1)) {
    Serial.println("Error: Y homing failed");
    goto reattach;
  }
  stepperY.setCurrentPosition(mmToSteps(HOME_POS_XY));
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
  // 软限位检查（未归零时不约束Z下限，避免启动后Z强制跳到95）
  targetX = constrain(targetX, 0, X_MAX_TRAVEL);
  targetY = constrain(targetY, 0, Y_MAX_TRAVEL);
  if (isHomed) {
    targetZ = constrain(targetZ, Z_MIN_POS, Z_MAX_POS);
  } else {
    targetZ = constrain(targetZ, 0, Z_MAX_POS);
  }

  long stepsX = mmToSteps(targetX);
  long stepsY = mmToSteps(targetY);
  long stepsZ = mmToSteps(targetZ);

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
        yield();
      }
      Serial.print("ALARM: Limit hit on ");
      Serial.println(limitAxis);

      // 回退3mm（负方向=远离限位）
      long pullback = mmToSteps(3.0);
      if (limitAxis == 'X') {
        stepperX.move(-pullback);
        while (stepperX.distanceToGo() != 0) { stepperX.run(); yield(); }
      } else if (limitAxis == 'Y') {
        stepperY.move(-pullback);
        while (stepperY.distanceToGo() != 0) { stepperY.run(); yield(); }
      } else if (limitAxis == 'Z') {
        stepperZ.move(-pullback);
        while (stepperZ.distanceToGo() != 0) { stepperZ.run(); yield(); }
      }

      limitHit = false;
      break;
    }

    stepperX.run();
    stepperY.run();
    stepperZ.run();
    yield();
  }

  // 更新当前坐标
  currentX = stepsToMm(stepperX.currentPosition());
  currentY = stepsToMm(stepperY.currentPosition());
  currentZ = stepsToMm(stepperZ.currentPosition());

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

  // 诊断命令: $T 读限位状态, $TX/Y/Z 测试单轴正方向移动10mm
  if (line.startsWith("$T") || line.startsWith("$t")) {
    Serial.print("Limits: X(pin");
    Serial.print(X_LIMIT_PIN);
    Serial.print(")=");
    Serial.print(digitalRead(X_LIMIT_PIN) == LOW ? "TRIGGERED" : "open");
    Serial.print(" Y(pin");
    Serial.print(Y_LIMIT_PIN);
    Serial.print(")=");
    Serial.print(digitalRead(Y_LIMIT_PIN) == LOW ? "TRIGGERED" : "open");
    Serial.print(" Z(pin");
    Serial.print(Z_LIMIT_PIN);
    Serial.print(")=");
    Serial.println(digitalRead(Z_LIMIT_PIN) == LOW ? "TRIGGERED" : "open");

    // 单轴测试: $TX = X正方向10mm, $TZ- = Z负方向10mm
    if (line.length() >= 3) {
      char axis = line.charAt(2);
      float dir = 1.0;
      if (line.length() >= 4 && line.charAt(3) == '-') dir = -1.0;
      float testDist = 10.0 * dir;
      float speed = feedToStepsPerSec(200);

      if (axis == 'X' || axis == 'x') {
        Serial.print("Test X move ");
        Serial.print(testDist);
        Serial.println("mm (positive should go toward limit)");
        stepperX.setMaxSpeed(speed);
        stepperX.move(mmToSteps(testDist));
        while (stepperX.distanceToGo() != 0) { stepperX.run(); yield(); }
      } else if (axis == 'Y' || axis == 'y') {
        Serial.print("Test Y move ");
        Serial.print(testDist);
        Serial.println("mm");
        stepperY.setMaxSpeed(speed);
        stepperY.move(mmToSteps(testDist));
        while (stepperY.distanceToGo() != 0) { stepperY.run(); yield(); }
      } else if (axis == 'Z' || axis == 'z') {
        Serial.print("Test Z move ");
        Serial.print(testDist);
        Serial.println("mm (positive should go UP toward limit)");
        stepperZ.setMaxSpeed(speed);
        stepperZ.move(mmToSteps(testDist));
        while (stepperZ.distanceToGo() != 0) { stepperZ.run(); yield(); }
      }
      Serial.println("Test done. Check limits again:");
      Serial.print("  X=");
      Serial.print(digitalRead(X_LIMIT_PIN) == LOW ? "TRIGGERED" : "open");
      Serial.print(" Y=");
      Serial.print(digitalRead(Y_LIMIT_PIN) == LOW ? "TRIGGERED" : "open");
      Serial.print(" Z=");
      Serial.println(digitalRead(Z_LIMIT_PIN) == LOW ? "TRIGGERED" : "open");
    }
    Serial.println("ok");
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
          stepperX.setCurrentPosition(mmToSteps(currentX));
        }
        if (hasCode(line, 'Y')) {
          currentY = parseValue(line, 'Y', currentY);
          stepperY.setCurrentPosition(mmToSteps(currentY));
        }
        if (hasCode(line, 'Z')) {
          currentZ = parseValue(line, 'Z', currentZ);
          stepperZ.setCurrentPosition(mmToSteps(currentZ));
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
  Serial.println("Commands: $H(home) $T(diag) G0/G1(move) G90/G91 G92 M17/M18 ?(status)");
  Serial.println("Diag: $T(read limits) $TZ(test Z+10mm) $TZ-(test Z-10mm)");
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
