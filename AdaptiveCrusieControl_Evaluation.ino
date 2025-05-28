//=================================Library Needed===========================
#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" 
#include "EMA_FILTER.h"
//===========================Set up Pins=============================
#define RPWM 2  //White
#define LPWM 15 //Orange
#define R_EN 0  //Blue
#define L_EN 4  //Green
#define IS_R 25 //Black
#define IS_L 26 //Yellow
#define SERVO_PIN 13
#define HALL_SENSOR_PIN 12
#define echoPin 16
#define trigPin 17

//=============================DefineClass===============================
EMAFilter FilterForMotor(1);
//=============================Global Parameters==========================
const float SOUND_SPEED = 0.0343f;  // cm/us
float distanceCm = 0.0f;
const uint8_t Max_distance = 40;
uint32_t duration;
const uint16_t MIN_PULSE_WIDTH = 500;
const uint16_t MAX_PULSE_WIDTH = 2500;
const uint16_t REFRESH_INTERVAL = 20000;
const uint8_t SERVO_MIN_ANGLE = 40;
const uint8_t SERVO_MAX_ANGLE = 130;
const uint16_t CURRENT_LIMIT_MV = 7000;
const uint8_t PULSES_PER_REV = 5;
volatile uint16_t pulseCount = 0;
volatile float currentRPM = 0.0f;
const uint16_t RPM_CALC_INTERVAL_MS = 500;
const uint32_t RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile uint32_t lastRpmCalcTime = 0;
volatile uint16_t lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;
float g_filteredRPM = 0.0f;   
float RPM_setpoint = 0.0f; 
float Kp = 1.5f;  
float Ki = 0.5f;  
float Kd = 0.0f; 
const uint8_t PID_INTERVAL_MS = 10;
const float PID_SAMPLE_TIME_S = (float)PID_INTERVAL_MS / 1000.0;
float integral = 0.0f;           
float previousErrorRPM = 0.0f;   
uint32_t lastPIDRunTime = 0;
uint8_t currentPWM = 0; 
const float RPM_TO_PWM_SLOPE = 0.05f;  
const float RPM_TO_PWM_OFFSET = 4.6f; 
const uint16_t MAX_RPM_ESTIMATE = 5000; 
bool motorRunning = false;
const uint8_t SERIAL_BUFFER_SIZE = 10;
byte serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialBufferIndex = 0;

//=======================Serial Compatibility=========================
int32_t steering_angle = 90;
int32_t received_speed = 0;

//======================Functions=====================================
void IRAM_ATTR hallSensorISR() //Interrupt function for encoder
{
    pulseCount++;
}
void IRAM_ATTR rpm_timer_callback(void *arg) //Interrupt timer for encoder
{
    uint32_t currentTime_us = micros();
    uint16_t currentPulseReading;
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();
    uint32_t deltaTime_us = currentTime_us - lastRpmCalcTime;
    uint16_t deltaPulses = currentPulseReading - lastRpmCalcPulseCount;
    float calculatedRPM = 0.0f;
    if (deltaTime_us > 0 && deltaPulses > 0) 
    { 
        float pulses_per_second = (float)deltaPulses * 1000000.0f / (float)deltaTime_us;
        float rps = pulses_per_second / (float)PULSES_PER_REV; 
        calculatedRPM = rps * 60.0f;                            
    } 
    else if (deltaTime_us > RPM_CALC_INTERVAL_US * 2) 
    { 
         if (abs(RPM_setpoint) < 0.1f)
         { 
              calculatedRPM = 0.0f;
         }
    }
    noInterrupts();
    currentRPM = calculatedRPM; 
    interrupts();
    lastRpmCalcTime = currentTime_us;
    lastRpmCalcPulseCount = currentPulseReading;
}
void setMotorPWM(int pwmValue) // Function to controll Motor
{
    pwmValue = constrain(pwmValue, 0, 255);
    currentPWM = pwmValue; 
    uint16_t raw_IS_R = analogRead(IS_R);
    float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);
    if (voltage_IS_R_mV < CURRENT_LIMIT_MV ) 
    {
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH);
        analogWrite(RPWM, pwmValue);
        digitalWrite(LPWM, LOW);
        motorRunning = (pwmValue > 0);
    } 
    else 
    {
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        analogWrite(LPWM, 0);
        digitalWrite(RPWM, LOW);
        motorRunning = false;
        integral = 0; 
        currentPWM = 0;
    }
}
float measureDistanceCM() //Measure the distance between ego car and a preceeding vehicle
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 5000); 
  if (duration == 0) {
    return -1;
  }
  float distance = duration * SOUND_SPEED / 2;
  if (distance < 0) distance = 0;
  if (distance > Max_distance) distance = Max_distance;
  return distance;
}
float adjustRPMWithDistance(float distance) //Based on the measured distances, this function will adjust RPM in order to remain safe distance from the preceeding vehicle 
{
    static float lastRPM = 0.0f;
    if (distance < 10.0f) 
    {
         lastRPM = 0.0f; 
    } 
    else if (distance >= 10.0f && distance <= 35.0f) 
    {
        if (distance >= 29.0f && distance <= 31.0f)
        {
            return lastRPM; 
        }
        float RPM_calculate = ((Max_distance - distance) * (RPM_setpoint / Max_distance));
        lastRPM = RPM_setpoint - RPM_calculate;
    } 
    else if (distance > 35.0f && distance < Max_distance) 
    {
        lastRPM += 5.0f;
        if (lastRPM > RPM_setpoint) lastRPM = RPM_setpoint;
    } 
    else 
    {
        lastRPM = RPM_setpoint; 
    }
    return lastRPM;
}
float calculatePID_RPM_Output(float setpointRPM, float measuredRPM) //PID algorithm 
{
    float errorRPM = setpointRPM - measuredRPM;
    float maxIntegralContribution_RPM = MAX_RPM_ESTIMATE; 
    float maxIntegral = (Ki > 0.001) ? maxIntegralContribution_RPM / Ki : 1e9; 
    integral += errorRPM * PID_SAMPLE_TIME_S;
    integral = constrain(integral, -maxIntegral, maxIntegral); 
    float derivative = 0.0f;
    if (PID_SAMPLE_TIME_S > 0) 
    {
        derivative = (errorRPM - previousErrorRPM) / PID_SAMPLE_TIME_S;
    }

    float p_term = Kp * errorRPM;
    float i_term = Ki * integral;
    float d_term = Kd * derivative;
    float pidOutput_RPM = p_term + i_term + d_term;
    previousErrorRPM = errorRPM; 
    return pidOutput_RPM;
}
uint8_t transformRPMtoPWM(float targetEffort_RPM) // Converting function from RPM to PWM
{
    float calculatedPWM_f = RPM_TO_PWM_SLOPE * targetEffort_RPM + RPM_TO_PWM_OFFSET;
    int pwmOut = constrain((int)round(calculatedPWM_f), 0, 255);
    return pwmOut;
}
void control_servo() //Function to contor servo (orientating the vehicle)
{
    static uint32_t lastServoUpdate = 0UL;
    uint32_t now = micros();
    if (now - lastServoUpdate >= REFRESH_INTERVAL) 
    {
        lastServoUpdate = now;
        uint8_t constrainedAngle = constrain(steering_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        uint16_t pulseWidth = map(constrainedAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulseWidth);
        digitalWrite(SERVO_PIN, LOW);
    }
}
void parseSerialData() //Receive data from Raspberry pi 
{
    if (serialBuffer[0] == '<' && serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') 
    {
        steering_angle = *((int32_t*)&serialBuffer[1]);
        received_speed = *((int32_t*)&serialBuffer[5]);
        RPM_setpoint = (float)received_speed;
    } 
    serialBufferIndex = 0;
}
void displayData() // Display data for tuning PID
{
    static uint32_t lastPrintTime = 0UL;
    const uint32_t PRINT_INTERVAL_MS = 200UL; 
    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) 
    {
        lastPrintTime = millis();
        float rawRPM_display;
        noInterrupts();
        rawRPM_display = currentRPM;
        interrupts();
        Serial.print("SetpointRPM:");
        Serial.print(RPM_setpoint);
        Serial.print(",");
        Serial.print("FilteredRPM:"); 
        Serial.print(g_filteredRPM,6);
        Serial.print(",");
        Serial.print("distance:");
        Serial.println(distanceCm,2);
    }
}
void setup() //Setup function 
{
    Serial.begin(115200);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT);
    pinMode(IS_L, INPUT);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0);
    digitalWrite(RPWM, LOW);
    setMotorPWM(0);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
    lastRpmCalcTime = micros();
    lastPIDRunTime = millis();
    noInterrupts();
    lastRpmCalcPulseCount = pulseCount;
    interrupts();
    esp_timer_create_args_t rpm_timer_args = { .callback = &rpm_timer_callback, .name = "rpm_calc"};
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
    FilterForMotor.reset();
}
void loop() // Main Loop 
{
    while (Serial.available() > 0) {
         uint8_t byteRead = Serial.read();
        if (serialBufferIndex == 0 && byteRead != '<') 
        {
            continue; 
        }
        if (serialBufferIndex < SERIAL_BUFFER_SIZE) 
        {
            serialBuffer[serialBufferIndex++] = byteRead;
        }
        if (serialBufferIndex == SERIAL_BUFFER_SIZE) 
        {
            if (serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') 
            {
                parseSerialData(); 
            } else {
                 Serial.println("Serial Frame Error: No end marker.");
            }
             serialBufferIndex = 0; 
        }
    }
    float rawRPM;
    noInterrupts();
    rawRPM = currentRPM;
    interrupts();
    //g_filteredRPM = updateEMA(rawRPM);
    //float Filtermeasured = updateEMA(measured);
    g_filteredRPM = FilterForMotor.update(rawRPM);
    float measured = measureDistanceCM();
    if (measured >= 0.0) 
    {
    distanceCm = measured; 
    }
    float adjustedSetpoint = adjustRPMWithDistance(distanceCm);
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) 
    {
        lastPIDRunTime = currentMillis; 
        if (abs(RPM_setpoint) < 0.1) 
        {
            integral = 0.0;
            previousErrorRPM = 0.0;
        }
        float targetEffort_RPM = calculatePID_RPM_Output(adjustedSetpoint, g_filteredRPM); 
        int targetPWM = transformRPMtoPWM(targetEffort_RPM);
        setMotorPWM(targetPWM);
    } 
    control_servo();
    static uint32_t lastSendTime = 0;
    const uint8_t sendInterval = 200; 
    if (millis() - lastSendTime >= sendInterval) 
    {
    lastSendTime = millis(); 
    sendFeedbackToPi(adjustedSetpoint, distanceCm, g_filteredRPM);
    }
    displayData();
}