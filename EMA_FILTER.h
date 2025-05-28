#ifndef EMA_FILTER_H
#define EMA_FILTER_H

class EMAFilter {
public:
    EMAFilter(float alpha);
    float update(float currentRawValue);
    void reset(float initialValue = 0.0f, bool startInitialized = false);
    float getCurrentFilteredValue() const;

private:
    float _alpha;        
    float _previousEMA; 
    bool _initialized;   
};

#endif 