#include "main.h"
#include "auton.h"

// lateral motion controller


// 1. kp, ki, kd   : 基本 PID (P=力度, I=補誤差, D=煞車)
// 2. windup       : [積分範圍] 誤差小於此值才開始補 I (防暴衝)
// 3. small_err    : [精準誤差] 允許的誤差範圍 (例如 1)
// 4. small_time   : [精準計時] 在誤差內維持多久算停穩 (ms)
// 5. large_err    : [寬鬆誤差] 停不準沒關係，至少要在這範圍內 (例如 3)
// 6. large_time   : [寬鬆計時] 寬鬆範圍的超時時間 (ms)
// 7. slew         : [防翹孤輪] 加速度限制 (數值越小越柔和，127 為無限制)

void test() {
    pros::delay(3000);
    drive_distance_back(120, 0);
}

//-----資格賽----awp-------
void right_awp() {}

void left_awp() {}

void solo_awp() {}
//-----淘汰賽----score-----
void right_score() {}

void left_score() {}

void solo_score() {}