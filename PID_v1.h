#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.2.1

class PID {
public:
    #define AUTOMATIC 1
    #define MANUAL 0
    #define DIRECT 0
    #define REVERSE 1
    #define P_ON_M 0
    #define P_ON_E 1

    PID(double*, double*, double*, double, double, double, int, int);
    PID(double*, double*, double*, double, double, double, int);
    
    void SetMode(int Mode);
    bool Compute();
    void SetOutputLimits(double, double);
    void SetTunings(double, double, double);
    void SetTunings(double, double, double, int);
    void SetControllerDirection(int);
    void SetSampleTime(int);
    
    double GetKp();
    double GetKi();
    double GetKd();
    int GetMode();
    int GetDirection();

    // Методи для отримання окремих складових PID
    double GetPterm();
    double GetIterm();
    double GetDterm();

private:
    void Initialize();
    
    double dispKp, dispKi, dispKd;
    double kp, ki, kd;
    int controllerDirection;
    int pOn;
    
    double *myInput, *myOutput, *mySetpoint;
    unsigned long lastTime;
    double outputSum, lastInput;
    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto, pOnE;

    // Змінні для збереження складових PID
    double P_term, I_term, D_term;
    
    // Нові змінні для рекурсивної (різницевої) форми
    double lastError;   // E(n-1)
    double prevError;   // E(n-2)
    double lastOutput;  // U(n-1)
};

#endif
