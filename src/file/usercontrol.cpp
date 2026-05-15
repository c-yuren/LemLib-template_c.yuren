#include "main.h"
#include "functions.hpp"

// =============================================================
//  功能 1: Loader 切換
// =============================================================
void Loader_switch(void* p) {
    bool state = false;
    while (true) {
        if(auton_start==0){
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                state = !state;
                Loader.set_value(state);
            }
            pros::delay(20); // 必加延遲
        }
    }
}

// =============================================================
//  功能 2: Center (Goal) 夾具
// =============================================================
void Center_switch(void* p) {
    while (true) {
        if(auton_start==0){
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                goal_switch.set_value(true);
            } else {
                goal_switch.set_value(false);
            }
            pros::delay(20);
        }
    }
}

// =============================================================
//  功能 3: Descore 裝置
// =============================================================
void Descore_switch(void* p) {
    bool state = 1;
    descore.set_value(1);
    if(auton_start==0){
        while (true) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                state = !state;
                descore.set_value(state);
            }
            pros::delay(20);
        }
    }
}

// =============================================================
//  功能 4: Intake 與 Outtake 控制
// =============================================================
void Outtake_spin(void* p) {
    if(auton_start==0){
        while (true) {
            bool L1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
            bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
            bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

            // 這是你原本 Outtake_spin 的邏輯翻譯
            if (L1) {
                // L1 按下：Intake 和 Outtake 一起轉
                Intake.move(127);
                Outtake.move(127);
            } 
            else if (R1) {
                // R1 按下：只轉 Intake
                Intake.move(127);
                Outtake.move(0);
            } 
            else if (R2) {
                // R2 按下：Intake 反轉
                Intake.move(-127);
                Outtake.move(0);
            } 
            else {
                // 都沒按：停止
                Intake.move(0);
                Outtake.move(0);
            }

            pros::delay(10); // 必加延遲
        }   
    }
}

