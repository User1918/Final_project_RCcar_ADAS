#include "EMA_FILTER.h" 
#include <Arduino.h>  
EMAFilter::EMAFilter(float alpha) 
{
    if (alpha < 0.0f) {
        _alpha = 0.0f;
    } else if (alpha > 1.0f) {
        _alpha = 1.0f;
    } else {
        _alpha = alpha;
    }
    _previousEMA = 0.0f;
    _initialized = false;
}
float EMAFilter::update(float currentRawValue) 
{
    if (!_initialized) {
        _previousEMA = currentRawValue;
        _initialized = true;
        return _previousEMA;
    }
    _previousEMA = (_alpha * currentRawValue) + ((1.0f - _alpha) * _previousEMA);
    return _previousEMA;
}
void EMAFilter::reset(float initialValue, bool startInitialized) 
{
    _previousEMA = initialValue;
    _initialized = startInitialized;
}
float EMAFilter::getCurrentFilteredValue() const 
{
    return _previousEMA;
}