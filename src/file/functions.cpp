#include "main.h"
#include <cmath>
#include <algorithm>


// 1. kp, ki, kd   : 基本 PID (P=力度, I=補誤差, D=煞車)
// 2. windup       : [積分範圍] 誤差小於此值才開始補 I (防暴衝)
// 3. small_err    : [精準誤差] 允許的誤差範圍 (例如 1)
// 4. small_time   : [精準計時] 在誤差內維持多久算停穩 (ms)
// 5. large_err    : [寬鬆誤差] 停不準沒關係，至少要在這範圍內 (例如 3)
// 6. large_time   : [寬鬆計時] 寬鬆範圍的超時時間 (ms)
// 7. slew         : [防翹孤輪] 加速度限制 (數值越小越柔和，127 為無限制)


// set_linear_constants(10, 0, 3, 3, 1, 100, 3, 500, 20);
// set_angular_constants(2, 0, 10, 3, 1, 100, 3, 500, 0);
void print_coord() {
    double x = chassis.getPose().x;
    double y = chassis.getPose().y;
    printf("X: %.f", x);
    printf("   Y: %.f\n", y);
}

struct OuttakeParams {
    int time;
};

void outtake_task_fn(void* param) {
    OuttakeParams* p = (OuttakeParams*)param;
    int timeMsec = p->time;
    delete p; // 釋放記憶體

    int startTime = pros::millis();
    Intake.move(127);
    Outtake.move(127);

    while (pros::millis() - startTime < timeMsec) {
        int elapsedTime = pros::millis() - startTime;
        double outtakeVel = std::abs(Outtake.get_actual_velocity());

        if (elapsedTime > 500) {
            if (outtakeVel > 480) {
                if (elapsedTime < (timeMsec * 0.8)) {
                    Intake.move(-127);
                    pros::delay(300);
                    Intake.move(127);
                    pros::delay(1300);
                } else {
                    break; 
                }
            }
        }
        pros::delay(10);
    }
    Outtake.move(0);
}

void Outtaking(int timeMsec, bool blocking ){
    if (blocking) {
        // 阻塞模式：直接執行原本的邏輯 (或是呼叫上面的 task 函式並等待)
        OuttakeParams* p = new OuttakeParams{timeMsec};
        outtake_task_fn(p);
    } else {
        // 非阻塞模式：啟動 Task 後立刻往下跑
        OuttakeParams* p = new OuttakeParams{timeMsec};
        pros::Task outtakeTask(outtake_task_fn, p, "Outtake Task");
    }
}

void moveTime(double speed_volt, double Timemsec,bool stop) {
    int startTime = pros::millis(); // 紀錄開始時間

    // 當 "現在時間" 減 "開始時間" 小於 "目標時間" 時，持續執行q
    while (pros::millis() - startTime < Timemsec) {
        chassis.tank(speed_volt, speed_volt); // 持續告訴馬達要轉
        pros::delay(20);            // 讓 CPU 休息一下，避免當機 (重要!)
    }
    if(stop){
        chassis.tank(0, 0); // 時間到，停止
    }
}

const double OFFSET_FRONT_F = 0.0;  // 前感測器靠前 10 吋
const double OFFSET_FRONT_R = 0.0;
const double OFFSET_BACK_F  = -10.0; // 後感測器靠後 10 吋
const double OFFSET_BACK_R  = 0.0;
const double OFFSET_RIGHT_F = 0.0;
const double OFFSET_RIGHT_R = 4.0;   // 右感測器靠右 4 吋
const double OFFSET_LEFT_F  = 0.0;
const double OFFSET_LEFT_R  = -4.0;  // 左感測器靠左 4 吋
void reset(SensorSelect s1, SensorSelect s2, double angle) {
    constexpr double halfField = 72.0;

    // 1. 角度處理
    double finalAngle = (angle == -1000.0) ? imu_sensor.get_heading() : angle;
    double theta = finalAngle * M_PI / 180.0; // 弧度

    lemlib::Pose currentPose = chassis.getPose();
    double newX = currentPose.x;
    double newY = currentPose.y;

    auto process_sensor = [&](SensorSelect s) {
        if (s == none) return;

        double d_mm = 0, offF = 0, offR = 0;
        
        if (s == front) { 
            d_mm = Front_Distance_sensor.get_distance(); 
            offF = OFFSET_FRONT_F; offR = OFFSET_FRONT_R; 
        } else if (s == back) { 
            d_mm = Back_Distance_sensor.get_distance(); 
            offF = OFFSET_BACK_F; offR = OFFSET_BACK_R; 
        } else if (s == right) { 
            d_mm = Right_Distance_sensor.get_distance(); 
            offF = OFFSET_RIGHT_F; offR = OFFSET_RIGHT_R; 
        } else if (s == left) { 
            d_mm = Left_Distance_sensor.get_distance(); 
            offF = OFFSET_LEFT_F; offR = OFFSET_LEFT_R; 
        }

        double d = d_mm / 25.4;
        if (d > 80.0) return; // 過濾無效值

        // 計算感測器在世界座標的方向 (與原代碼邏輯相同)
        double senseTheta = theta;
        if (s == right) senseTheta -= M_PI/2.0;
        else if (s == left) senseTheta += M_PI/2.0;
        else if (s == back) senseTheta += M_PI;

        double dirX = sin(senseTheta);
        double dirY = cos(senseTheta);

        if (std::abs(dirY) > std::abs(dirX)) { // 修正 Y 軸 (感測器朝向接近垂直)
            double wallY = (dirY > 0) ? halfField : -halfField;
            // 機器人中心 Y = 牆壁Y - (距離 * 方向) - 偏移量在Y軸的分量
            newY = wallY - (d * dirY) - (offF * cos(theta) - offR * sin(theta));
        } else { // 修正 X 軸 (感測器朝向接近水平)
            double wallX = (dirX > 0) ? halfField : -halfField;
            // 機器人中心 X = 牆壁X - (距離 * 方向) - 偏移量在X軸的分量
            newX = wallX - (d * dirX) - (offF * sin(theta) + offR * cos(theta));
        }
    };

    process_sensor(s1);
    process_sensor(s2);

    chassis.setPose(newX, newY, finalAngle);
}

// 輔助：角度修正 (-180 ~ 180)
float reduce_negative_180_to_180(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

void drive_distance_front(float distance, float heading, float max_voltage, float timeout) {
    
    lemlib::PID drivePID(linearController.kP, linearController.kI, 6, linearController.windupRange, true);
    
    lemlib::PID headingPID(angularController.kP, angularController.kI, angularController.kD, angularController.windupRange, true);

    // 2. 退出條件：也是自動抓取
    lemlib::ExitCondition exitCondition(linearController.smallError, linearController.smallErrorTimeout);

    // 3. 初始化
    float current_dist = Front_Distance_sensor.get() * 0.1; 
    float drive_error = current_dist - distance;
    long start_time = pros::millis();

    // 4. 執行迴圈 (使用你輸入的 timeout)
    while (!exitCondition.update(drive_error) && (pros::millis() - start_time < timeout)) {
        
        current_dist = Front_Distance_sensor.get() * 0.1;
        drive_error = current_dist - distance;

        float current_heading = chassis.getPose().theta;
        float heading_error = reduce_negative_180_to_180(heading - current_heading);

        float drive_output = drivePID.update(drive_error);
        float heading_output = headingPID.update(heading_error);

        // 限制電壓 (使用你輸入的 max_voltage)
        drive_output = std::clamp(drive_output, -max_voltage, max_voltage);
        heading_output = std::clamp(heading_output, -max_voltage, max_voltage);

        chassis.tank(drive_output + heading_output, drive_output - heading_output);

        pros::delay(10);
    }
    chassis.tank(0, 0);
}

void drive_distance_back(float distance, float heading, float max_voltage, float timeout) {
    lemlib::PID drivePID(linearController.kP, linearController.kI, linearController.kD, linearController.windupRange, true);
    
    lemlib::PID headingPID(angularController.kP, angularController.kI, angularController.kD, angularController.windupRange, true);

    // 2. 退出條件：也是自動抓取
    lemlib::ExitCondition exitCondition(linearController.smallError, linearController.smallErrorTimeout);

    // 3. 初始化
    float current_dist = Back_Distance_sensor.get() * 0.1; 
    float drive_error = current_dist - distance;
    long start_time = pros::millis();

    // 4. 執行迴圈 (使用你輸入的 timeout)
    while (!exitCondition.update(drive_error) && (pros::millis() - start_time < timeout)) {
        
        current_dist = Back_Distance_sensor.get() * 0.1;
        drive_error = distance - current_dist;

        float current_heading = chassis.getPose().theta;
        float heading_error = reduce_negative_180_to_180(heading - current_heading);

        float drive_output = drivePID.update(drive_error);
        float heading_output = headingPID.update(heading_error);

        // 限制電壓 (使用你輸入的 max_voltage)
        drive_output = std::clamp(drive_output, -max_voltage, max_voltage);
        heading_output = std::clamp(heading_output, -max_voltage, max_voltage);

        chassis.tank(drive_output + heading_output, drive_output - heading_output);

        pros::delay(10);
    }
    chassis.tank(0, 0);
}

void drive_distance(float distance, float heading, int timeout, lemlib::MoveToPointParams params, bool async ) {
    // 1. 取得目前座標
    lemlib::Pose currentPose = chassis.getPose();
    
    // 2. 角度轉弧度 (注意：VEX/LemLib 座標系 0度在正北方，順時針增加)
    float radHeading = heading * M_PI / 180.0;

    // 3. 計算目標 X, Y
    float targetX = currentPose.x + distance * sin(radHeading);
    float targetY = currentPose.y + distance * cos(radHeading);

    // 4. 自動判斷前後進 (如果使用者沒在 params 裡指定的話)
    // 雖然 moveToPointParams 預設 forwards 是 true，但我們依據 distance 正負號決定
    params.forwards = (distance >= 0);

    // 5. 呼叫原生的 moveToPoint
    chassis.moveToPoint(targetX, targetY, timeout, params, async);
}

/**
 * @brief 讀取距離感測器並移動 (支援 MoveToPoint 參數與 Async)
 * * @param targetGapInches 目標離牆距離 (英吋)
 * @param timeout 超時 (ms)
 * @param useFrontSensor true=前感測器, false=後感測器
 * @param params LemLib 移動參數 (maxSpeed, minSpeed 等)
 * @param async true=不等待直接回傳, false=等待直到移動完成
 */
void driveToWallSmart(float targetGapInches, int timeout, bool useFrontSensor, lemlib::MoveToPointParams params, bool async) {
    // 1. 讀取數據
    pros::Distance& sensor = useFrontSensor ? Front_Distance_sensor : Back_Distance_sensor;
    double currentDistMM = sensor.get();
    
    // 無效值處理：若 async=false 且讀不到值，直接 return 避免卡住
    if (currentDistMM > 3000) return; 

    double currentDistInches = currentDistMM / 25.4;
    
    // 2. 計算位移
    double diff = currentDistInches - targetGapInches;
    // 前鏡頭: 距離過大需前進(+); 後鏡頭: 距離過大需後退(-)
    double shift = useFrontSensor ? diff : -diff;

    // 誤差過小不執行
    if (std::abs(shift) < 0.5) return;

    // 3. 獲取並吸附角度
    lemlib::Pose pose = chassis.getPose();
    double theta = std::fmod(pose.theta, 360.0);
    if (theta < 0) theta += 360.0;

    int snappedAngle = 0;
    if (theta >= 45 && theta < 135) snappedAngle = 90;
    else if (theta >= 135 && theta < 225) snappedAngle = 180;
    else if (theta >= 225 && theta < 315) snappedAngle = 270;
    else snappedAngle = 0;

    // 4. 計算目標座標
    float targetX = pose.x;
    float targetY = pose.y;

    switch (snappedAngle) {
        case 0:   targetY += shift; break;
        case 90:  targetX += shift; break;
        case 180: targetY -= shift; break;
        case 270: targetX -= shift; break;
    }

    // 5. 設定方向 (用前鏡頭則車頭朝向目標，用後鏡頭則車尾朝向目標)
    params.forwards = useFrontSensor;

    // 6. 執行移動，傳入 async 參數
    chassis.moveToPoint(targetX, targetY, timeout, params, async);
}