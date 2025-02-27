#include "Arduino.h"
#include "PID_v1.h"

PID::PID(double *Input, double *Output, double *Setpoint,
         double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    // За замовчуванням встановлюємо діапазон 0–255
    SetOutputLimits(0, 255);
    SampleTime = 100;
    SetControllerDirection(ControllerDirection);
    SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis() - SampleTime;

    // Ініціалізація складових PID
    P_term = 0;
    I_term = 0;
    D_term = 0;

    // Ініціалізація змінних для рекурсивного обчислення
    lastError = 0;
    prevError = 0;
    lastOutput = 0;
}

PID::PID(double *Input, double *Output, double *Setpoint,
    double Kp, double Ki, double Kd, int POn, int ControllerDirection, int outMin, int outMax)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    SetOutputLimits(outMin, outMax);
    SampleTime = 100;
    SetControllerDirection(ControllerDirection);
    SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis() - SampleTime;

    // Ініціалізація складових PID
    P_term = 0;
    I_term = 0;
    D_term = 0;

    // Ініціалізація змінних для рекурсивного обчислення
    lastError = 0;
    prevError = 0;
    lastOutput = 0;
}

PID::PID(double *Input, double *Output, double *Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection)
    : PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {}

PID::PID(double *Input, double *Output, double *Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection, int outMin, int outMax)
    : PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection, outMin, outMax) {}

bool PID::Compute()
{
    if (!inAuto)
        return false;
    unsigned long now = millis();
    unsigned long timeChange = now - lastTime;
    if (timeChange >= SampleTime)
    {
        double input = *myInput;
        double error = *mySetpoint - input;

        // Обчислення приростів помилки
        double dError = error - lastError;
        double ddError = error - 2.0 * lastError + prevError;

        // Рекурсивна формула PID
        double deltaOutput = ki * error + kp * dError + kd * ddError;
        double output = lastOutput + deltaOutput;

        // Обчислення пропорційної складової
        P_term = kp * error;

        double allowedIMax = outMax - P_term;
        double allowedIMin = outMin - P_term;

        I_term += ki * error;
        I_term = constrain(I_term, allowedIMin, allowedIMax);

        D_term = output - (P_term + I_term);
        double allowedDMax = outMax - (P_term + I_term);
        double allowedDMin = outMin - (P_term + I_term);
        D_term = constrain(D_term, allowedDMin, allowedDMax);

        output = P_term + I_term + D_term;
        output = constrain(output, outMin, outMax);
        *myOutput = output;

        prevError = lastError;
        lastError = error;
        lastOutput = output;
        lastInput = input;
        lastTime = now;
        return true;
    }
    return false;
}

void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

    pOn = POn;
    pOnE = (POn == P_ON_E);
    dispKp = Kp;
    dispKi = Ki;
    dispKd = Kd;

    double SampleTimeInSec = ((double)SampleTime) / 1000;
    // Використовуємо scalingFactor для масштабування коефіцієнтів, виходячи з діапазону, заданого SetOutputLimits()
    kp = dispKp * scalingFactor;
    ki = dispKi * scalingFactor * SampleTimeInSec;
    kd = dispKd * scalingFactor / SampleTimeInSec;

    if (controllerDirection == REVERSE)
    {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
}

void PID::SetTunings(double Kp, double Ki, double Kd)
{
    SetTunings(Kp, Ki, Kd, pOn);
}

void PID::SetSampleTime(int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        double ratio = (double)NewSampleTime / (double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

void PID::SetOutputLimits(double Min, double Max)
{
    if (Min >= Max)
        return;
    outMin = Min;
    outMax = Max;
    
    // Розрахунок коефіцієнта масштабування: новий діапазон/256 (за замовчуванням 0–255)
    scalingFactor = (outMax - outMin) / 255.0;

    if (inAuto)
    {
        if (*myOutput > outMax)
            *myOutput = outMax;
        else if (*myOutput < outMin)
            *myOutput = outMin;

        if (I_term > outMax)
            I_term = outMax;
        else if (I_term < outMin)
            I_term = outMin;
            
        // Перерахунок внутрішніх коефіцієнтів PID із застосуванням нового коефіцієнта масштабування
        SetTunings(dispKp, dispKi, dispKd, pOn);
    }
}

void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto)
    {
        Initialize();
    }
    inAuto = newAuto;
}

void PID::Initialize()
{
    I_term = *myOutput;
    lastInput = *myInput;
    lastError = *mySetpoint - *myInput;
    prevError = lastError;
    lastOutput = *myOutput;
    if (I_term > outMax)
        I_term = outMax;
    else if (I_term < outMin)
        I_term = outMin;
}

void PID::SetControllerDirection(int Direction)
{
    if (inAuto && Direction != controllerDirection)
    {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
    controllerDirection = Direction;
}

double PID::GetKp() { return dispKp; }
double PID::GetKi() { return dispKi; }
double PID::GetKd() { return dispKd; }
int PID::GetMode() { return inAuto ? AUTOMATIC : MANUAL; }
int PID::GetDirection() { return controllerDirection; }

double PID::GetPterm() { return P_term; }
double PID::GetIterm() { return I_term; }
double PID::GetDterm() { return D_term; }
