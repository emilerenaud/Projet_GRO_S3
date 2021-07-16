/*
Projet S1 2019
Exemple de librairie pouvant etre ajoute au projet
@author Jean-Samuel Lauzon
@version 1.0 22/05/2019
*/
#include <SoftwareSerial.h>
#include <LibS3GRO.h>
#include <EEPROM.h>
#include <PIDCustom.h>

#ifndef LibExample_H_
#define LibExample_H_


class tunerPID
{
public:
    tunerPID(PIDCustom *pid);
    ~tunerPID();
    void tune();


protected:
    String text_;
    SoftwareSerial* tunerSerie_;
    PIDCustom* pid;
    float p_;
    float i_;
    float d_;

};
#endif // LibExample_H_