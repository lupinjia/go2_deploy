/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <array>
#include <stdio.h>
#include <iostream>

/**
 * @brief wrap the angle to [-pi, pi]
 * @param angle the input angle
 * @return the wrapped angle
 */
float wrap_to_pi(float angle)
{
    while(angle > M_PI) angle -= 2*M_PI;
    while(angle < -M_PI) angle += 2*M_PI;
    return angle;
}

/**
 * @brief clip the value to the range [low_limit, high_limit]
 * @param value the input value
 * @param low_limit the lower limit
 * @param high_limit the upper limit
 * @return the clipped value
 */
float clip(float value, float low_limit, float high_limit)
{
    if(value <= low_limit) return low_limit;
    if(value >= high_limit) return high_limit;
    return value;
}

float max_abs(std::array<float, 12> arr)
{
    float max_abs = 0.0;
    for(int i=0; i<12; i++){
        if(fabs(arr[i]) > max_abs){
            max_abs = fabs(arr[i]);
        }
    }
    return max_abs;
}

template<typename T1, typename T2>
inline T1 max(const T1 a, const T2 b){
	return (a > b ? a : b);
}

template<typename T1, typename T2>
inline T1 min(const T1 a, const T2 b){
	return (a < b ? a : b);
}

#endif  // MATHTOOLS_H