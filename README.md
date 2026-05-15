# 機器人自訂函式與 LemLib 使用手冊

[🇹🇼 中文版本 (Chinese)](README.md) | [🇺🇸 English Version](README_en.md)

本手冊紀錄本專案的底盤控制設定細節、PID 調參方法，以及所有自訂函式的用法，提供給後續寫自動程式 (Auton) 的人參考。

## 目錄
1. [LemLib 基礎設定 (main.cpp)](#1-lemlib-基礎設定-maincpp)
2. [LemLib PID 調參教學](#2-lemlib-pid-調參教學)
3. [LemLib 控制移動語法](#3-lemlib-控制移動語法)
4. [自訂函式 (functions.cpp)](#4-自訂函式-functionscpp)
    - 基礎除錯
    - 機構控制
    - 時間基礎移動
    - 感測器距離移動
    - 第一身分座標移動
    - 定位修正

---

## 1. LemLib 基礎設定 (main.cpp)

要讓機器人正常運作，必須先在 `main.cpp` 完成硬體與控制器的基礎參數設定。

### 1-1. 定義馬達群組
將驅動底盤的馬達分為左右兩組。負號代表馬達反轉，正號代表正轉，這需與實體車上的齒輪傳動方向對應。
```cpp
pros::MotorGroup leftMotors({-1, -2, 3}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({4, 5, -6}, pros::MotorGearset::blue);
```

### 1-2. 底盤動力參數
設定輪距與轉速，系統才能將座標換算成馬達轉動的圈數。
```cpp
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 
                              11,           // 左右輪距 (英吋)
                              lemlib::Omniwheel::NEW_325, // 輪子種類與直徑
                              450,          // 齒輪比運算後的實際轉速 (RPM)
                              2             // 甩尾摩擦係數 (橫向滑移用)
);
```

### 1-3. 建立 Chassis 物件
將感測器(陀螺儀與測距輪)綁定後，建立 `chassis` 物件。
```cpp
lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal, nullptr, &imu_sensor);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
```

---

## 2. LemLib PID 調參教學

在 `main.cpp` 會看到兩組 ControllerSettings (直線 linear 與轉向 angular)。PID 參數決定了車子移動時的精準度與平穩度。

### 參數解讀
- **kP (比例):** 推力大小。設太小會不到位，設太大會來回劇烈震盪。
- **kI (積分):** 通常設為 0。VEX 底盤很少用到。
- **kD (微分):** 煞車阻尼。用來抵銷 kP 造成的震盪。
- **Slew:** 起步加速度限制。數值越小，起步越柔和，0 代表無限制。

### 調參步驟
1. 將 kI, kD 設為 0。
2. 慢慢調高 kP 值，讓車子跑一段短距離或轉 90 度。持續增加 kP，直到車子到達目標時開始出現明顯的來回抖動。
3. 保持剛剛的 kP 值，開始調高 kD 值，直到抖動消失，車子能穩穩且精準地停點。
4. 常見數值參考：
   - 直線 (Linear): kP 通常在 10 左右，kD 是 kP 的幾倍，Slew 可以設 20 防止起步打滑。
   - 轉向 (Angular): kP 通常在 2~5 之間，kD 通常較大 (例如 7~35 之間)。

---

## 3. LemLib 控制移動語法

### 設定起始位置
寫 Auton 的第一行必加指令，告訴系統車子一開始擺放在場地的哪個絕對座標。
```cpp
chassis.setPose(0, 0, 90); // 座標(0,0)，面向右邊 90 度
```

### 絕對座標移動
系統會自動轉向、加速與煞車，抵達指定點。
```cpp
chassis.moveToPoint(0, 24, 2000); // 走到前方 (0, 24)，超時 2000ms

// 若要倒退前往該點，外掛 params 設定 forwards 為 false
chassis.moveToPoint(0, -24, 2000, {.forwards = false});
```

### 原地轉向
指定車頭面向特定角度，或直接瞄準特定座標。
```cpp
chassis.turnToHeading(180, 1000); // 車頭轉向正後方
chassis.turnToPoint(24, 24, 1000); // 轉向瞄準 (24,24)

// 用車尾瞄準物件
chassis.turnToPoint(24, 24, 1000, {.forwards = false}); 
```

---

## 4. 自訂函式 (functions.cpp)

底本專案中自行包裝的輔助功能。

### 基礎除錯
- `print_coord()`: 將現在的 X, Y 座標列印到 Terminal，供除錯定位使用。

### 機構控制
- `Outtaking(int timeMsec, bool blocking)`
  控制 Intake/Outtake 轉動指定時間 `timeMsec`。
  包含防卡機制，若卡彈會自動微反轉排除。
  `blocking` 設 `true` 代表原地做完才執行下一行；設 `false` 為背景執行(可邊走邊吸)。

### 時間基礎移動
- `moveTime(double speed_volt, double Timemsec, bool stop)`
  不理會感測器，純靠時間給馬達電壓。
  適合短距離粗糙移動或推牆壁。

### 感測器距離移動
- `drive_distance_front(float distance, float heading, float max_voltage, float timeout)`
- `drive_distance_back(float distance, float heading, float max_voltage, float timeout)`
  車體直行，直到前方(或後方)雷射測距感測器看到的距離小於 `distance` 停下。
  過程會強制鎖定在 `heading` 角度防止走偏。

### 第一身分座標移動
- `drive_distance(float distance, float heading, int timeout, lemlib::MoveToPointParams params, bool async)`
  不用自己算終點座標。以目前位子為基準，朝向 `heading` 角度走 `distance` 距離（內建呼叫 moveToPoint 轉換）。
- `driveToWallSmart(float targetGapInches, int timeout, bool useFrontSensor, ...)`
  結合測距感測器與 LemLib 座標。會讀取離牆距離並修正絕對座標，平滑開到離牆 `targetGapInches` 吋的位置。

### 定位與牆壁校準
- `reset(SensorSelect s1, SensorSelect s2, double angle)`
  使用雷射測距打向四周牆壁，重新計算並修正機器人的系統座標 (X 或 Y)。
  可只選一面牆 (如 `front`), 另一面填 `none`。
  `angle` 放目前已知的真實角度。若是 `-1000` 則直接採用 IMU 當前數據。
  ```cpp
  // 範例：靠牆微調，讀前牆修正距牆座標
  reset(front, none, -1000);
  ```
