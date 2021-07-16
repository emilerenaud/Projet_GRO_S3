/*
Projet S3 GRO
Class to control a PID
@author Jean-Samuel Lauzon
@version 1.0 23/04/2019
*/

#include "PIDCustom.h"

PIDCustom::PIDCustom(int addr)
{
    eeAdresse_ = addr;
    readValue();
}

void PIDCustom::saveValues()
{
    EEPROM.put(eeAdresse_ + (sizeof(float) * 0), p_);
    EEPROM.put(eeAdresse_ + (sizeof(float) * 1), i_);
    EEPROM.put(eeAdresse_ + (sizeof(float) * 2), d_);
}

void PIDCustom::readValue()
{
    EEPROM.get(eeAdresse_ + (sizeof(float) * 0), p_);
    EEPROM.get(eeAdresse_ + (sizeof(float) * 1), i_);
    EEPROM.get(eeAdresse_ + (sizeof(float) * 2), d_);
    setGains(p_, i_, d_);
}

void PIDCustom::setGains(double kp, double ki, double kd)
{
    p_ = kp;
    i_ = ki;
    d_ = kd;
    saveValues();
    PID::setGains(p_,i_,d_);
}

void PIDCustom::setKp(double kp)
{
    p_ = kp;
    saveValues();
    PID::setKp(p_);
}

void PIDCustom::setKi(double ki)
{
    i_ = ki;
    PID::setKi(i_);
}

void PIDCustom::setKd(double kd)
{
    d_ = kd;
    PID::setKd(d_);
}