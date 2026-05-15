#pragma once
void set_linear_constants(float kp, float ki, float kd, float windup, 
                          float small_error, float small_timeout, 
                          float large_error, float large_timeout, 
                          float slew);

// 宣告：設定「轉向」的所有 PID 參數
void set_angular_constants(float kp, float ki, float kd, float windup, 
                           float small_error, float small_timeout, 
                           float large_error, float large_timeout, 
                           float slew);
        


void moveTime(double speed_volt, double Timemsec,bool stop =1);

enum SensorSelect { front, back, left, right, none };

/**
 * @brief 自動判斷感測器朝向並重置座標 (原點為場地中央)
 * @param s1 第一顆感測器 (前後左右或none)
 * @param s2 第二顆感測器 (前後左右或none)
 * @param angle 手動設定的角度 (預設為 -1000 代表抓取 IMU)
 */
void reset(SensorSelect s1, SensorSelect s2, double angle = -1000.0);
void Outtaking(int timeMsec, bool blocking = true);
void print_coord();
void driveFor(float distance, float maxSpeed, int timeout, float minspeed = 0, float exit = 0) ;
// 宣告函式
// 注意：預設值 (= 110, = 2000) 寫在這裡就好
void drive_distance_front(float distance, float heading, float max_voltage = 110, float timeout = 2000);
void drive_distance_back(float distance, float heading, float max_voltage = 110, float timeout = 2000);
void drive_distance(float distance, float heading, int timeout, lemlib::MoveToPointParams params={}, bool async = false) ;

/**
 * @brief 讀取距離感測器並移動至目標距離
 * * @param targetGapInches 目標離牆距離 (英吋)
 * @param timeout 超時設定 (ms)
 * @param useFrontSensor true=前感測器, false=後感測器
 * @param params [可選] LemLib 移動參數 (如 maxSpeed, earlyExitRange)，預設為空
 * @param async [可選] true=非同步(背景執行), false=同步(等待完成)，預設為 true
 */
void driveToWallSmart(float targetGapInches, int timeout, bool useFrontSensor, lemlib::MoveToPointParams params = {}, bool async = true);