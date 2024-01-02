#pragma once

namespace thoht {

enum SplineOptimizeType
{
    None = 0,

    POINTS = 1,
    T_I_C = 1 << 1,
    IMU_BIASES = 1 << 2,
    IMU_INTRINSICS = 1 << 3,
    GRAVITY_DIR = 1 << 4,
    CAM_LINE_DELAY = 1 << 5,
    SPLINE = 1 << 6,
    ACC_BIAS = 1 << 7,
    GYR_BIAS = 1 << 8,

    All = POINTS | T_I_C | IMU_BIASES | IMU_INTRINSICS | GRAVITY_DIR |
          CAM_LINE_DELAY | SPLINE | ACC_BIAS | GYR_BIAS,

};

}
