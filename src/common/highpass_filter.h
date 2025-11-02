#ifndef HIGHPASS_FILTER_H
#define HIGHPASS_FILTER_H


#include "time_utils.h"
#include "foc_utils.h"

/**
 *  High pass filter class
 */
class HighPassFilter
{
public:
    /**
     * @param Tf - High pass filter time constant
     */
    HighPassFilter(float Tf);
    ~HighPassFilter() = default;

    float operator() (float x);
    float Tf; //!< High pass filter time constant

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step 
    float x_prev; //!< input value in previous execution step
};

#endif // HIGHPASS_FILTER_H
