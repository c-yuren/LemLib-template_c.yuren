#include "main.h"
#include "auton.h"
#include "skill.h"
#include "usercontrol.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
// #include "liblvgl/lvgl.h"

// setting----------------------------------------------setting----------------------------------------
LV_IMAGE_DECLARE(my_logo);
int current_auton_selection = 7;
bool auton_start = 1;
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-1, -2, 3},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({4, 5, -6}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor Intake(7, pros::MotorGearset::blue);
pros::Motor Outtake(-8, pros::MotorGearset::blue);
pros::adi::Pneumatics Loader('A', false);
pros::adi::Pneumatics goal_switch('C', false);
pros::adi::Pneumatics descore('D', false);
pros::Distance Right_Distance_sensor(17);
pros::Distance Left_Distance_sensor(18);
pros::Distance Front_Distance_sensor(19);
pros::Distance Back_Distance_sensor(20);
#define COLOR_RED 0xFF0000
#define COLOR_BLUE 0x0000FF
#define COLOR_YELLOW 0xFFFF00
#define COLOR_GREEN 0x00FF00
#define COLOR_ORANGE 0xE06000
#define COLOR_DARK_GRAY 0x404040
#define COLOR_BACKGROUND 0x202020
#define COLOR_BLACK 0x000000
#define COLOR_WHITE 0xFFFFFF
#define COLOR_CYAN 0x00FFFF
// Inertial Sensor on port 10
pros::Imu imu_sensor(11);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-14);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(13);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -1.3);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 2.875);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             7, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             300, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, //&vertical,  //vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu_sensor // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// screen function---------------------------screen function------------------------------------
int timer_1 = 105;
bool screen_mode = 1;
bool pre_auton_break = 0;
bool reset_color = true; // true = red, false = blue
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
pros::Motor LF(1, pros::MotorGears::blue);
pros::Motor LB(2, pros::MotorGears::blue);
pros::Motor LU(3, pros::MotorGears::blue);
pros::Motor RF(4, pros::MotorGears::blue);
pros::Motor RB(5, pros::MotorGears::blue);
pros::Motor RU(6, pros::MotorGears::blue);
const char* btn_names[8] = {"1. R AWP",   "2. L AWP",   "3. Solo AWP",    "4. R SCORE",
                            "5. L SCORE", "6. S SCORE", "7. Auto Skills", "8. Driver"};

void controller_task_fn(void* p) {
    uint32_t start_time = pros::millis();

    while (true) {
        // --- 1. 共同數據讀取 ---
        double x = chassis.getPose().x;
        double y = chassis.getPose().y;
        double heading = chassis.getPose().theta;

        // --- 2. 模式判斷與顯示 ---

        // 情況 A：自動結束前 (包含 Disabled 準備期間 & Autonomous 自動期間)
        // 需求：只顯示「座標」和「程式名稱」
        if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
            // Line 0: 顯示程式名稱
            if (current_auton_selection == 0) {
                controller.print(0, 0, "Wait Select...  ");
            } else {
                // 顯示選好的程式 (例如 "Auto: R AWP")
                controller.print(0, 0, "Auto: %-10s", btn_names[current_auton_selection - 1]);
            }
            pros::delay(60);

            // Line 1: 顯示座標 (方便擺車確認位置)
            controller.print(1, 0, "X: %.0f  Y: %.0f   ", x, y);
            pros::delay(60);

            // Line 2: 清空 (保持簡潔)
            controller.print(2, 0, "heading: %.00f", heading);
        }

        // 情況 B：駕駛階段 (OpControl)
        // 需求：恢復計時、溫度、警告
        else {
            double total_temp = LF.get_temperature() + LB.get_temperature() + LU.get_temperature() +
                                RF.get_temperature() + RB.get_temperature() + RU.get_temperature();
            double avg_temp = total_temp / 6.0;

            int seconds = 105 - (pros::millis() - start_time) / 1000;
            if (seconds < 0) seconds = 0;

            // Line 0: 計時 + 溫度
            controller.print(0, 0, "T: %ds  Tmp: %.0f", seconds, avg_temp);
            pros::delay(60);

            // Line 1: 座標
            controller.print(1, 0, "X: %.0f  Y: %.0f   ", x, y);
            pros::delay(60);

            // Line 2: 警告訊息
            if (total_temp >= 55) controller.print(2, 0, "OVER HEAT !!!   ");
            else if (pros::battery::get_capacity() <= 20) controller.print(2, 0, "LOW BATTERY !!! ");
            else controller.print(2, 0, "");
        }

        pros::delay(200);
    }
}

void brain_screen_task(void* param) {
    // 1. 絕對等待：直到 initialize 結束前，不讓 LVGL 介入，避免黑屏
    while (!pre_auton_break) { pros::delay(20); }

    // 2. 清除舊畫面：移除 pros::screen 留下的九宮格
    lv_obj_clean(lv_screen_active());
    lv_obj_invalidate(lv_screen_active());

    // --- UI 建立 ---

    // Logo
    lv_obj_t* img1 = lv_image_create(lv_screen_active());
    lv_image_set_src(img1, &my_logo);
    lv_obj_align(img1, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    // Labels
    lv_obj_t* label_drive_L = lv_label_create(lv_screen_active());
    lv_obj_set_style_transform_scale(label_drive_L, 350, 0);
    lv_obj_align(label_drive_L, LV_ALIGN_TOP_LEFT, 5, 5);

    lv_obj_t* label_drive_R = lv_label_create(lv_screen_active());
    lv_obj_set_style_transform_scale(label_drive_R, 350, 0);
    lv_obj_align(label_drive_R, LV_ALIGN_TOP_LEFT, 70, 5);

    lv_obj_t* label_io = lv_label_create(lv_screen_active());
    lv_obj_align(label_io, LV_ALIGN_LEFT_MID, 5, -25);

    lv_obj_t* label_auto = lv_label_create(lv_screen_active());
    lv_obj_set_style_transform_scale(label_auto, 350, 0);
    lv_obj_align(label_auto, LV_ALIGN_TOP_MID, -60, 0);

    lv_obj_t* label_dist = lv_label_create(lv_screen_active());
    lv_obj_align(label_dist, LV_ALIGN_TOP_MID, -10, 50);

    lv_obj_t* label_pos = lv_label_create(lv_screen_active());
    lv_obj_set_style_transform_scale(label_pos, 350, 0);
    lv_obj_align(label_pos, LV_ALIGN_TOP_RIGHT, -200, 0);
    lv_obj_set_style_text_align(label_pos, LV_TEXT_ALIGN_RIGHT, 0);

    lv_obj_t* label_head = lv_label_create(lv_screen_active());
    lv_obj_set_style_transform_scale(label_head, 350, 0);
    lv_obj_align(label_head, LV_ALIGN_TOP_RIGHT, -60, 0);

    lv_obj_t* label_warn = lv_label_create(lv_screen_active());
    lv_obj_align(label_warn, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_obj_set_style_transform_scale(label_warn, 512, 0);
    lv_obj_set_style_transform_pivot_x(label_warn, 0, 0);
    lv_obj_set_style_transform_pivot_y(label_warn, 20, 0);

    while (true) {
        // printf("IMU pitch: %f\n", imu_sensor.get_roll());

        // 讀取數據
        int LF_t = LF.get_temperature();
        int RF_t = RF.get_temperature();
        int LB_t = LB.get_temperature();
        int RB_t = RB.get_temperature();
        int LU_t = LU.get_temperature();
        int RU_t = RU.get_temperature();

        int In_t = Intake.get_temperature();
        int Out_t = Outtake.get_temperature();

        double imu_y = imu_sensor.get_roll();

        double x = chassis.getPose().x;
        double y = chassis.getPose().y;

        double head = chassis.getPose().theta;
        double dist_front = Front_Distance_sensor.get_distance();
        double dist_back = Back_Distance_sensor.get_distance();
        double dist_left = Left_Distance_sensor.get_distance();
        double dist_right = Right_Distance_sensor.get_distance();

        int battery = (int)pros::battery::get_capacity();
        lemlib::Pose pose = chassis.getPose();

        if (LF_t == 2147483647) LF_t = 0;
        if (RF_t == 2147483647) RF_t = 0;
        if (LB_t == 2147483647) LB_t = 0;
        if (RB_t == 2147483647) RB_t = 0;
        if (LU_t == 2147483647) LU_t = 0;
        if (RU_t == 2147483647) RU_t = 0;
        if (In_t == 2147483647) In_t = 0;
        if (Out_t == 2147483647) Out_t = 0;

        bool is_drive_hot = (LF_t >= 55 || RF_t >= 55 || LB_t >= 55 || RB_t >= 55 || LU_t >= 55 || RU_t >= 55);
        bool is_io_hot = (In_t >= 55 || Out_t >= 55);
        // 1. 底盤馬達
        lv_label_set_text_fmt(label_drive_L, "LF: %d\nLB: %d\nLU: %d", LF_t, LB_t, LU_t);
        if (is_drive_hot) lv_obj_set_style_text_color(label_drive_L, lv_color_hex(0xFF0000), 0); // 紅字
        else lv_obj_set_style_text_color(label_drive_L, lv_color_hex(0xFFFFFF), 0); // 白字

        lv_label_set_text_fmt(label_drive_R, "RF: %d\nRB: %d\nRU: %d", RF_t, RB_t, RU_t);
        if (is_drive_hot) lv_obj_set_style_text_color(label_drive_R, lv_color_hex(0xFF0000), 0); // 紅字
        else lv_obj_set_style_text_color(label_drive_R, lv_color_hex(0xFFFFFF), 0); // 白字

        // 2. IO 機構
        lv_label_set_text_fmt(label_io, "Intake     :%d \nOuttaKe: %d ", In_t, Out_t);

        // auto
        lv_obj_set_style_text_color(label_auto, lv_color_hex(0x005ce6), 0);
        lv_label_set_text_fmt(label_auto, "%s", btn_names[current_auton_selection - 1]);

        if (is_io_hot) lv_obj_set_style_text_color(label_io, lv_color_hex(0xFF0000), 0);
        else lv_obj_set_style_text_color(label_io, lv_color_hex(0xFFFFFF), 0);

        // 3. 距離
        lv_obj_set_style_text_color(label_dist, lv_color_hex(0xFFFF00), 0);
        lv_label_set_text_fmt(label_dist, "         %d\n\n%d                      %d\n\n          %d", (int)dist_front,
                              (int)dist_left, (int)dist_right, (int)dist_back);
        // 4. 座標
        lv_label_set_text_fmt(label_pos, "X: %d.%d\nY: %d.%d", (int)pose.x, abs((int)(pose.x * 10) % 10), (int)pose.y,
                              abs((int)(pose.y * 10) % 10));

        lv_label_set_text_fmt(label_head, "Heading: %d\n roll: %d", (int)pose.theta, (int)imu_y);

        // 4. 警告訊息
        if ((is_drive_hot || is_io_hot) && battery <= 20) {
            lv_label_set_text(label_warn, "BATTERY LOW\nOVER HEAT");
            lv_obj_set_style_text_color(label_warn, lv_color_hex(0xFF00FF), 0); // 紫色
        } else if (battery <= 20) {
            lv_label_set_text(label_warn, "BATTERY LOW");
            lv_obj_set_style_text_color(label_warn, lv_color_hex(0xFFAA00), 0); // 橘色
        } else if (is_drive_hot || is_io_hot) {
            lv_label_set_text(label_warn, "OVER HEAT");
            lv_obj_set_style_text_color(label_warn, lv_color_hex(0xFF0000), 0); // 紅色
        } else {
            lv_label_set_text(label_warn, "");
        }

        // 觸控重置座標
        if (pros::screen::touch_status().touch_status == pros::E_TOUCH_PRESSED) {
            chassis.setPose(0, 0, 0);
            while (pros::screen::touch_status().touch_status == pros::E_TOUCH_PRESSED) pros::delay(20);
        }
        pros::delay(200);
    }
}

/* competition-------------------------------competition--------------------------------------------*/
/*
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Outtake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // 1. 先啟動遙控器畫面更新
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    imu_sensor.reset();
    pros::Task controller_task(controller_task_fn);

    // 2. 只有在設定為 0 時，才進入九宮格選單
    if (current_auton_selection == 0) {
        int temp_selection = 1; // 暫存選擇，預設為 1

        while (current_auton_selection == 0) {
            // --- 繪製九宮格 ---
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    int btn_index = row * 3 + col;
                    int x = col * 160;
                    int y = row * 80;

                    // --- 決定按鈕的背景顏色 ---
                    uint32_t bg_color;

                    // 特殊處理 CONFIRM 鍵 (第9格)
                    if (btn_index == 8) {
                        bg_color = COLOR_GREEN;
                    }
                    // 一般按鈕
                    else {
                        bool is_selected = ((btn_index + 1) == temp_selection);
                        if (is_selected) bg_color = COLOR_GREEN; // 選中：綠色
                        else if (btn_index < 6) bg_color = COLOR_WHITE; // 前6個：白色
                        else bg_color = COLOR_ORANGE; // 特殊：橘色
                    }

                    // 1. 畫背景方塊
                    pros::screen::set_pen(bg_color);
                    pros::screen::fill_rect(x, y, x + 155, y + 75);

                    // 2. 設定文字背景色 (關鍵修正：讓字底色 = 按鈕色)
                    pros::screen::set_eraser(bg_color);

                    // 3. 設定文字顏色 & 印字
                    if (btn_index == 8) { // CONFIRM
                        pros::screen::set_pen(COLOR_BLACK);
                        pros::screen::print(pros::E_TEXT_LARGE_CENTER, x + 5, y + 25, "CONFIRM");
                    } else { // 一般選項
                        bool is_selected = ((btn_index + 1) == temp_selection);
                        // if (is_selected) pros::screen::set_pen(COLOR_WHITE); // 綠底配白字
                        pros::screen::set_pen(COLOR_BLACK); // 白底配黑字

                        pros::screen::print(pros::E_TEXT_MEDIUM, x + 10, y + 30, btn_names[btn_index]);
                    }
                }
            }

            // --- 觸控偵測 ---
            pros::screen_touch_status_s status = pros::screen::touch_status();
            if (status.touch_status == pros::E_TOUCH_PRESSED) {
                int col = status.x / 160;
                int row = status.y / 80;
                int pressed_index = row * 3 + col;

                if (pressed_index == 8) current_auton_selection = temp_selection;
                else if (pressed_index < 8) temp_selection = pressed_index + 1;

                pros::delay(200);
            }
            pros::delay(20);
        }
    }

    // 3. 清除畫面並交棒給 Brain Screen Task
    pros::screen::erase();
    pros::delay(50);
    pre_auton_break = true;
    pros::delay(50);
    pros::Task Brain_screen(brain_screen_task);
}

// Runs while the robot is disabled
void disabled() {}

// runs after initialize if the robot is connected to field control
void competition_initialize() {
    switch (current_auton_selection) {
        case 1: chassis.setPose(0, 0, 0); break;
        case 2: chassis.setPose(0, 0, 0); break;
        case 3: chassis.setPose(0, 0, 0); break;
        case 4: chassis.setPose(0, 0, 0); break;
        case 5: chassis.setPose(0, 0, 0); break;
        case 6:
            // chassis.setPose(-51.120000, 17.040000, 90.000000);
            break;
        case 7: chassis.setPose(0, 0, 0); break;
    }
    // 設定旗標讓監控螢幕開始運作
}

// auton
void autonomous() {
    auton_start = 1;
    switch (current_auton_selection) {
        case 1: right_awp(); break;
        case 2: left_awp(); break;
        case 3: solo_awp(); break;
        case 4: right_score(); break;
        case 5: left_score(); break;
        case 6: solo_score(); break;
        case 7: skill_auton(); break;
    }
}

// Driver
void opcontrol() {
    auton_start = 0;
    pre_auton_break = 0;
    descore.set_value(1);
    chassis.setPose(0, 0, 0);
    reset(front, none, 0);
    if (current_auton_selection == 8) { skill_driver(); }
    // placeing task here
    // pros::Task task1(Loader_switch);
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        pros::delay(10);
    }
}