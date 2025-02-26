#include "Arduino.h"
#include "PID_v1.h"

PID::PID(double* Input, double* Output, double* Setpoint, 
         double Kp, double Ki, double Kd, int POn, int ControllerDirection) {
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

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

PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection)
    : PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {}

bool PID::Compute() {
    if (!inAuto) return false;
    unsigned long now = millis();
    unsigned long timeChange = now - lastTime;
    if (timeChange >= SampleTime) {
        double input = *myInput;
        double error = *mySetpoint - input;
        
        // Обчислення різницевих приростів помилки:
        // dError = E(n) - E(n-1)
        // ddError = E(n) - 2E(n-1) + E(n-2)
        double dError = error - lastError;
        double ddError = error - 2.0 * lastError + prevError;
        
        // Рекурсивна формула PID:
        // ΔU = ki * E(n) + kp * (E(n) - E(n-1)) + kd * (E(n) - 2E(n-1) + E(n-2))
        double deltaOutput = ki * error + kp * dError + kd * ddError;
        double output = lastOutput + deltaOutput;
        
        // Обчислення пропорційної складової (найвищий пріоритет)
        P_term = kp * error;
        
        // Обчислення допустимих меж для інтегральної складової
        double allowedIMax = outMax - P_term;
        double allowedIMin = outMin - P_term;
        
        // Рекурсивне накопичення інтегральної складової з обмеженням
        I_term += ki * error;
        I_term = constrain(I_term, allowedIMin, allowedIMax);
        
        // Обчислення диференціальної складової як залишок від рекурсивного виходу
        D_term = output - (P_term + I_term);
        double allowedDMax = outMax - (P_term + I_term);
        double allowedDMin = outMin - (P_term + I_term);
        D_term = constrain(D_term, allowedDMin, allowedDMax);
        
        // Остаточний вихід – сума всіх складових із фінальним обмеженням
        output = P_term + I_term + D_term;
        output = constrain(output, outMin, outMax);
        *myOutput = output;
        
        // Оновлення історичних змінних для рекурсивного обчислення
        prevError = lastError;
        lastError = error;
        lastOutput = output;
        lastInput = input;
        lastTime = now;
        return true;
    }
    return false;
}

void PID::SetTunings(double Kp, double Ki, double Kd, int POn) {
    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    pOn = POn;
    pOnE = (POn == P_ON_E);
    dispKp = Kp;
    dispKi = Ki;
    dispKd = Kd;

    double SampleTimeInSec = ((double)SampleTime) / 1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;

    if (controllerDirection == REVERSE) {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
}

void PID::SetTunings(double Kp, double Ki, double Kd) {
    SetTunings(Kp, Ki, Kd, pOn);
}

void PID::SetSampleTime(int NewSampleTime) {
   if (NewSampleTime > 0) {
      double ratio = (double)NewSampleTime / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void PID::SetOutputLimits(double Min, double Max) {
    if (Min >= Max) return;
    outMin = Min;
    outMax = Max;

    if (inAuto) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;

        if (I_term > outMax) I_term = outMax;
        else if (I_term < outMin) I_term = outMin;
    }
}

void PID::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto) {
        Initialize();
    }
    inAuto = newAuto;
}

void PID::Initialize() {
    I_term = *myOutput;
    lastInput = *myInput;
    lastError = *mySetpoint - *myInput;
    prevError = lastError;
    lastOutput = *myOutput;
    if (I_term > outMax) I_term = outMax;
    else if (I_term < outMin) I_term = outMin;
}

void PID::SetControllerDirection(int Direction) {
    if (inAuto && Direction != controllerDirection) {
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
