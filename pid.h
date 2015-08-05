//  ===========================================================================
//
//	Copyright Deng Qingyu 2015
//	Lisense: MIT Lisense
//  Author: Deng Qingyu (bitdqy@hotmail.com)
//
//                                  pid.h
//  ===========================================================================
//  Document of PID algorithm
//
//  ===========================================================================
//                             Revise History
//
//  V1.0.0    2015-8-4                        First Version
//
//  ===========================================================================

#ifndef PID_H
#define PID_H

//  ===========================================================================

//  Struct for target object you want to control with PID
//  For every object, independent instance of this struct shall be created.
//  WARNING:    All field SHALL NOT be modified DIRECTLY
typedef struct
{
    double target;                  //  Control target value
    double output;                  //  PID output control signal

    double upperlimit;              //  Upper limit of output signal
    double lowerlimit;              //  Lower limit of output signal

    double error_k_1;               //  e[k-1]
    double error_k_2;               //  e[k-2]

    double kp;                      //  Proportional constant
    double ti;                      //  Intergal time constant
    double td;                      //  Differential time constant
    double sampletime;              //  Sample time

    //  For quick calculation
    //  SHALL NOT MODIFIED DIRECTLY
    //  u[k] = u[k-1] + f_k * e[k] + f_k-1 * e[k-1] + f_k-2 * e[k-2]
    double factorError_k;
    double factorError_k_1;
    double factorError_k_2;

    int    statusBit;               //  Save some event such as 
                                    //  parameter change, which require 
                                    //  to recalculate the factors
} PID_t;

//  Mode of PID output limiter
typedef enum
{
    inhibit,
    onlyUpperLimiter,
    onlyLowerLimiter,
    both
} PID_Limiter;

//  MACROs
#define PID_DIRTY_BIT           0x00000001
#define PID_INITIAL_BIT         0x00000002

#define PID_LIMITER_INHIBIT     0x00000000
#define PID_LIMITER_ONLYUPPER   0x00000004
#define PID_LIMITER_ONLYLOWER   0x00000008
#define PID_LIMITER_BOTH        0x0000000C
#define PID_LIMITER_MASK        0x0000000C

//  

//  Global Function Declaration

//  Initial PID structure
//  Input:
//  PID_t       *pPID
//  WARNING:    To keep the correctess, the function MUST be called before
//              a PID_t instance is used.
void InitPID(PID_t *pPID);

//  Set control object
//  Input:
//  PID_t       *pPID
//  double      target
void SetPIDTarget(PID_t *pPID, double target);

//  Set the proportional constant of PID control
//  Input:
//  PID_t       *pPID
//  double      kp
void SetPIDKp(PID_t *pPID, double kp);

//  Set the time constant of intergal
//  Input:
//  PID_t       *pPID
//  double      ti
void SetPIDti(PID_t *pPID, double ti);

//  Set the time constant of differential
//  Input:
//  PID_t       *pPID
//  double      td
void SetPIDtd(PID_t *pPID, double td);

//  Set the sample time of PID algorithm
//  Input:
//  PID_t       *pPID
//  double      sampleTime
void SetPIDSampleTime(PID_t *pPID, double sampleTime);

//  Set the output limiter mode
//  Input:
//  PID_t       *pPID
//  PID_Limiter mode    (inhibit, onlyLowerLimiter, onlyUpperLimiter, both)
void ConfigurePIDLimiter(PID_t *pPID, PID_Limiter mode);

//  Set upper limit. When the output is more then upper limit,
//  the output would be the limit.
//  Input:
//  PID_t       *pPID
//  double      upperLimit
void SetPIDUpperLimit(PID_t *pPID, double upperLimit);

//  Set lower limit. When the output is less then lower limit,
//  the output would be the limit.
//  Input:
//  PID_t       *pPID
//  double      lowerLimit
void SetPIDLowerLimit(PID_t *pPID, double lowerLimit);
//void CalculatePIDFactor(PID_t *pPID);

//  Update the PID structure with actual status, which should 
//  be detected by sensors.
//  Input:
//  PID_t       *pPID
//  double      actual  The actual status of object
void UpdatePID(PID_t *pPID, double actual);

//  Get PID output control signal
//  Input:
//  PID_t       *pPID
//  Output:
//  double      the output control signal
//  WARNING:    MAKE SURE that UpdatePID is called before this function call
double GetPIDOutput(PID_t *pPID);

//

//  Local Function Declarations

//  Calculate the factors in expression of PID.
//  Required to call before procedure to calculate output, 
//  when user changes constants.
//  Input:
//  PID_t   *pPID
//
void CalculateFactors(PID_t *pPID);

//  Limit output based on setting
//  Input:
//  PID_t   *pPID
//  double  calcResult      Result calculated by PID algorithm   
//  Output:
//  double                  Output processed by limiter
double OutputLimiter(PID_t *pPID, double calcResult);

//  End of File
#endif
