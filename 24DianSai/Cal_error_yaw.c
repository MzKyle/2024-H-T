#include "board.h"
#include "stdio.h"
#include "string.h"
#include "Cal_error_yaw.h"

extern float Yaw;  // 外部定义的当前偏航角变量(全局)

/**
 * 计算浮点数绝对值
 * @param value 输入值
 * @return 绝对值
 */
float myfabs(float value) {
    if (value < 0) {
        return -value;
    } else {
        return value;
    }
}

/**
 * 计算两个偏航角之间的最短误差(考虑-180°到180°的循环特性)
 * @param Target 目标偏航角(度)
 * @param Now 当前偏航角(度)
 * @return 最短角度误差(度)，负值表示需要左转，正值表示需要右转
 */
float Yaw_error(float Target, float Now) {
    static float error;  // 静态变量保存计算结果
    Target = -Target;
	Now = -Now;
    // 情况1：目标角度在右半平面(0°~180°)
    if (Target >= 0) {
        // 子情况1：当前角度在左半平面(-180°~0°)
        if (Now <= 0) {
            // 计算当前角度的绝对值
            float nowAbs = myfabs(Now);
            
            // 判断是顺时针旋转还是逆时针旋转更近
            if (nowAbs < (180 - Target)) {
                // 逆时针旋转更近
                error = nowAbs + Target;
            } else {
                // 顺时针旋转更近
                error = -(180 - Target) - (180 - nowAbs);
            }
        } 
        // 子情况2：当前角度在右半平面(0°~180°)
        else if (Now > 0) {
            error = Target - Now;  // 直接计算差值
        }
    } 
    // 情况2：目标角度在左半平面(-180°~0°)
    else if (Target < 0) {
        // 子情况1：当前角度在右半平面(0°~180°)
        if (Now > 0) {
            // 判断是顺时针旋转还是逆时针旋转更近
            if (Now > Target + 180) {
                // 逆时针旋转更近
                error = (180 - Now) + (180 - myfabs(Target));
            } else if (Now < Target + 180) {
                // 顺时针旋转更近
                error = -(myfabs(Target) + Now);
            }
        } 
        // 子情况2：当前角度在左半平面(-180°~0°)
        else if (Now < 0) {
            error = -(myfabs(Target) - myfabs(Now));  // 同方向角度差
        }
    }
    
    return error;  // 返回计算得到的最短角度误差
}