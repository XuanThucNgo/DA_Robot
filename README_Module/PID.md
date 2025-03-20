Return [README.md](../README.md)

Reference [PID.h](../PID/PID.h)

Reference [PID.c](../PID/PID.c)

## PID Paramters
```
typedef struct PID
{
  float err;
  float err_pre;

  float Kp;
  float Ki;
  float Kd;
  float Kb;

  float sample_time;
  float alpha;

  float PTerm;
  float ITerm;
  float DTerm;
  float DTermFil;
  float DTermFilPre;

  float uSat;
  float u;
  float uAboveLimit;
  float uUnderLimit;

} PID_TypeDef;
```
# Functions API
### PID_Init
+ The PID_Init function initializes the PID controller by setting its gain parameters (Kp, Ki, Kd), filter coefficient (alpha), sampling time, and output limits. It also resets the PID state to ensure proper operation
### PID_clear
+ The PID_clear function resets all PID terms and outputs to zero, ensuring a fresh start for control calculations.
### PID
+ The PID function computes the control output based on the PID algorithm. It calculates the error (Target - Current), proportional, integral, and filtered derivative terms, then applies output saturation to stay within predefined limits. The function returns the saturated control output.
### PID_reset
+ The PID_reset function resets the control output (u) and its saturated value (uSat) to zero, ensuring a fresh start for the PID controller.
