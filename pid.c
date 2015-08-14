//  ===========================================================================
//
//	Copyright Deng Qingyu 2015
//	Lisense: MIT Lisense
//  Author: Deng Qingyu (bitdqy@hotmail.com)
//
//                                  pid.c
//  ===========================================================================
//  Implement of PID(Proportion, Integration, Differential) control algorithm.
//  As the parameters of PID have obvious physical meaning ,PID algorithm is 
//  widely applied to object control. 
//
//  ===========================================================================
//                             Revise History
//
//  V1.0.0    2015-8-4                        First Version
//  V1.0.1    2015-8-15                       Fix bug that the error signal
//                                            is mistakely calculated
//
//  ===========================================================================

#include "pid.h"


//  Global Functions

void InitPID(PID_t *pPID)
{
    pPID->statusBit = PID_INITIAL_BIT;
    pPID->error_k_1 = 0;
    pPID->error_k_2 = 0;
    pPID->output = 0;
}

void SetPIDTarget(PID_t *pPID, double target)
{
    pPID->target = target;
}

void SetPIDkp(PID_t *pPID, double kp)
{
    pPID->statusBit |= PID_DIRTY_BIT;
    pPID->kp = kp;
}

void SetPIDti(PID_t *pPID, double ti)
{
    pPID->statusBit |= PID_DIRTY_BIT;
    pPID->ti = ti;
}

void SetPIDtd(PID_t *pPID, double td)
{
    pPID->statusBit |= PID_DIRTY_BIT;
    pPID->td = td;
}

void SetPIDSampleTime(PID_t *pPID, double sampleTime)
{
    pPID->statusBit |= PID_DIRTY_BIT;
    pPID->sampletime = sampleTime;
}

void ConfigurePIDLimiter(PID_t *pPID, PID_Limiter mode)
{
    pPID->statusBit &= ~PID_LIMITER_MASK; //  Clear Limiter Configure bits
    switch (mode)
    {
    case inhibit:
        break;
    case onlyUpperLimiter:
        pPID->statusBit |= PID_LIMITER_ONLYUPPER;
        break;
    case onlyLowerLimiter:
        pPID->statusBit |= PID_LIMITER_ONLYLOWER;
        break;
    case both:
        pPID->statusBit |= PID_LIMITER_BOTH;
    }
}

void SetPIDUpperLimit(PID_t *pPID, double upperLimit)
{
    pPID->upperlimit = upperLimit;
}


void SetPIDLowerLimit(PID_t *pPID, double lowerLimit)
{
    pPID->lowerlimit = lowerLimit;
}

void UpdatePID(PID_t *pPID, double actual)
{
    double error;           // error to target
    double errorterm;
    double errortermk_1;
    double errortermk_2;
    double result;

    //  Detect change of constant
    if (pPID->statusBit & PID_DIRTY_BIT)
    {
        CalculateFactors(pPID);
    }


    //  u[k] = u[k-1] + f_k * e[k] + f_k_1 * e[k-1] + f_k_2 * e[k-2]
    error = pPID->target - actual;
    errorterm = pPID->factorError_k * error;
    errortermk_1 = pPID->factorError_k_1 * pPID->error_k_1;
    errortermk_2 = pPID->factorError_k_2 * pPID->error_k_2;

    result = pPID->output + errorterm + errortermk_1 + errortermk_2;
    
    //  Update output
    pPID->output = OutputLimiter(pPID, result);

    //  Prepare for next calculation
    pPID->error_k_2 = pPID->error_k_1;
    pPID->error_k_1 = error;
}

double GetPIDOutput(PID_t *pPID)
{
    return pPID->output;
}


//  Local Functions

void CalculateFactors(PID_t *pPID)
{
    pPID->factorError_k = pPID->kp * (1 + pPID->sampletime / pPID->ti 
                                       + pPID->td / pPID->sampletime);
    pPID->factorError_k_1 = -pPID->kp * (1 + 2 * pPID->td / pPID->sampletime);
    pPID->factorError_k_2 = pPID->kp * pPID->td / pPID->sampletime;
    pPID->statusBit &= ~PID_DIRTY_BIT;  //Clear dirty bit
}

double OutputLimiter(PID_t *pPID, double calcResult)
{
    switch (pPID->statusBit & PID_LIMITER_MASK)
    {
    case PID_LIMITER_INHIBIT:
        return calcResult;
        break;
    case PID_LIMITER_ONLYUPPER:
        return calcResult > pPID->upperlimit ? pPID->upperlimit : calcResult;
        break;
    case PID_LIMITER_ONLYLOWER:
        return calcResult < pPID->lowerlimit ? pPID->lowerlimit : calcResult;
        break;
    case PID_LIMITER_BOTH:
        if (calcResult > pPID->upperlimit)
            return pPID->upperlimit;
        else if (calcResult < pPID->lowerlimit)
            return pPID->lowerlimit;
        else
            return calcResult;
    default:
        return calcResult;
        break;
    }
}

//  End of File