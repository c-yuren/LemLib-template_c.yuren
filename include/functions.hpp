#pragma once
#include "lemlib/api.hpp"

void print_coord();
void moveTime(double speed_volt, double Timemsec,bool stop =1);

enum SensorSelect { front, back, left, right, none };

/**
 * @brief 自動判斷感測器朝向並重置座標 (原點為場地中央)
 * @param s1 第一顆感測器 (前後左右或none)
 * @param s2 第二顆感測器 (前後左右或none)
 * @param angle 手動設定的角度 (預設為 -1000 代表抓取 IMU)
 */
void reset(SensorSelect s1, SensorSelect s2, double angle = -1000.0);
void drive_distance_front(float distance, float heading, float max_voltage = 110, float timeout = 2000);
void drive_distance_back(float distance, float heading, float max_voltage = 110, float timeout = 2000);
void drive_distance(float distance, float heading, int timeout,lemlib::MoveToPointParams params={}, bool async = false) ;