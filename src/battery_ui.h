#ifndef BatteryUI_H
#define BatteryUI_H
#pragma once

#include "PowerManagement.h"
#include <ui.h>

static int lastCategory = -999;
static PowerStatus lastPowerStatus = PowerStatus::Unknown;

void update_battery_icon(PowerManagement& power, float voltage) {
    PowerStatus pwrStatus = power.getPowerStatus();
    int category = 0;
    if (voltage >= 4.15) category = 6;
    else if (voltage >= 4.05) category = 5;
    else if (voltage >= 3.95) category = 4;
    else if (voltage >= 3.85) category = 3;
    else if (voltage >= 3.75) category = 2;
    else if (voltage >= 3.65) category = 1;
    else if (voltage >= 3.5) category = 0;
    else category = -1;

    bool needUpdate = (category != lastCategory) || (pwrStatus != lastPowerStatus);
    if (needUpdate) {
        const lv_image_dsc_t* bat_img = nullptr;
        if (pwrStatus == PowerStatus::USBPowered) {
            bat_img = &ui_img_battery_android_frame_alert_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else if (pwrStatus == PowerStatus::BatteryAndUSBPowered) {
            bat_img = &ui_img_battery_android_frame_bolt_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else if (category == 6) {
            bat_img = &ui_img_battery_android_frame_full_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else if (category == 5) {
            bat_img = &ui_img_battery_android_frame_6_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else if (category == 4) {
            bat_img = &ui_img_battery_android_frame_5_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else if (category == 3) {
            bat_img = &ui_img_battery_android_frame_4_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else if (category == 2) {
            bat_img = &ui_img_battery_android_frame_3_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else if (category == 1) {
            bat_img = &ui_img_battery_android_frame_2_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else if (category == 0) {
            bat_img = &ui_img_battery_android_frame_1_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        } else {
            bat_img = &ui_img_battery_android_frame_question_24dp_e3e3e3_fill0_wght400_grad0_opsz24_png;
        }
        lv_lock();
        lv_image_set_src(ui_batImg, bat_img);
        lv_unlock();

        lastCategory = category;
        lastPowerStatus = pwrStatus;
    }
}

#endif // BatteryUI_H