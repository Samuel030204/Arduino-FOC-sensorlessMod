#include "highpass_filter.h"

HighPassFilter::HighPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
    , x_prev(0.0f)
{
    timestamp_prev = _micros();
}


float HighPassFilter::operator() (float x)
{
    unsigned long timestamp = _micros();
    float dt = (timestamp - timestamp_prev)*1e-6f;
    
    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        y_prev = 0.0f;
        x_prev = x;
        timestamp_prev = timestamp;
        return 0.0f;
    }
    
    float alpha = Tf/(Tf + dt);
    float y = alpha * (y_prev + x - x_prev);
    y_prev = y;
    x_prev = x;
    timestamp_prev = timestamp;
    return y;
}
