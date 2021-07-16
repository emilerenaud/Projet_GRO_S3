/*
Projet S1 2019
Exemple de librairie pouvant etre ajoute au projet
@author Jean-Samuel Lauzon
@version 1.0 22/05/2019
*/
#include <SoftwareSerial.h>
#include <LibS3GRO.h>
#include <EEPROM.h>

#ifndef LibExample_H_
#define LibExample_H_

#define EE_ADRESSE 0

class tunerPID
{
public:
    tunerPID(PID *pid, int addr = 0);
    ~tunerPID();
    void tune();
    void saveValues();
    void readValue();

protected:
    String text_;
    SoftwareSerial* tunerSerie_;
    PID* pid;
    float p_;
    float i_;
    float d_;
    int eeAdresse_;
};
#endif // LibExample_H_