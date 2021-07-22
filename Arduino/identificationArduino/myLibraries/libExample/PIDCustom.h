/*
Projet S3 GRO
Class to control a PID
@author Jean-Samuel Lauzon
@version 1.0 23/04/2019
*/

#ifndef PIDCUSTOM_H_
#define PIDCUSTOM_H_

#include <Arduino.h>
#include <LibS3GRO.h>
#include <EEPROM.h>

class PIDCustom : public PID
{
public:
    PIDCustom(int addr);
    void saveValues();
    void readValue();
    void setGains(double kp, double ki, double kd);
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);

    float getKp(){return p_;};
    float getKi(){return i_;};
    float getKd(){return d_;};

protected:
    int eeAdresse_;
    float p_;
    float i_;
    float d_;
};
#endif //PID
