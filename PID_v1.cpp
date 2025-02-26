#include "Arduino.h"
#include "PID_v1.h"

PID::PID(double* Input, double* Output, double* Setpoint, 
         double Kp, double Ki, double Kd, int POn, int ControllerDirection) {
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);
    SampleTime = 100;
    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis() - SampleTime;

    // Ініціалізація складових PID
    P_term = 0;
    I_term = 0;
    D_term = 0;
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
            double dInput = input - lastInput;
    
            // Розрахунок пропорційної складової (Kp має найвищий пріоритет)
            P_term = kp * error;
    
            // Обчислюємо допустимі межі для інтегральної складової (Ki)
            // Вона не може перевищити вихід, тому її ліміти – це залишок після врахування P_term
            double allowedIMax = outMax - P_term;
            double allowedIMin = outMin - P_term;
    
            // Оновлюємо інтегральну складову і обмежуємо її згідно з допустимими значеннями
            I_term += ki * error;
            I_term = constrain(I_term, allowedIMin, allowedIMax);
    
            // Розрахунок допустимих меж для диференціальної складової (Kd)
            // Вона повинна враховувати суму P_term та I_term
            double allowedDMax = outMax - (P_term + I_term);
            double allowedDMin = outMin - (P_term + I_term);
    
            // Обчислення диференціальної складової та її обмеження
            D_term = -kd * dInput;
            D_term = constrain(D_term, allowedDMin, allowedDMax);
    
            // Остаточний вихід – сума всіх складових
            double output = P_term + I_term + D_term;
            // Фінальне обмеження на випадок непередбачених ситуацій
            output = constrain(output, outMin, outMax);
            *myOutput = output;
    
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

void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
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
        PID::Initialize();
    }
    inAuto = newAuto;
}

void PID::Initialize() {
    I_term = *myOutput;
    lastInput = *myInput;
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

// Нові методи для отримання P, I, D складових
double PID::GetPterm() { return P_term; }
double PID::GetIterm() { return I_term; }
double PID::GetDterm() { return D_term; }
