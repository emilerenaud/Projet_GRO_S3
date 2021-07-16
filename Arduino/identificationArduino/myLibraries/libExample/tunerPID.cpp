/*
Exemple de librairie pouvant etre ajoute au projet
*/
#include <tunerPID.h>

// Class constructor
tunerPID::tunerPID(PID *pid,int addr)
{
    tunerSerie_ = new SoftwareSerial(2, 3); // RX, TX
    tunerSerie_->begin(115200);
    eeAdresse_ = addr;
    readValue();
    pid->setGains(p_, i_, d_);
}

void tunerPID::tune()
{
    if (tunerSerie_->available())
    {
        text_ = tunerSerie_->read();

        if (text_.indexOf("p") != -1)
        {
            p_ = text_.substring(text_.indexOf("p")).toFloat();
        }
        if (text_.indexOf("i") != -1)
        {
            p_ = text_.substring(text_.indexOf("i")).toFloat();
        }
        if (text_.indexOf("d") != -1)
        {
            p_ = text_.substring(text_.indexOf("d")).toFloat();
        }
        pid->setGains(p_, i_, d_);

        if (text_.indexOf("save") != -1)
        {
            saveValues();
        }
        if (text_.indexOf("read") != -1)
        {
            readValue();
        }
    }
}

void tunerPID::saveValues()
{
    EEPROM.put(eeAdresse_ + (sizeof(float) * 0), p_);
    EEPROM.put(eeAdresse_ + (sizeof(float) * 1), i_);
    EEPROM.put(eeAdresse_ + (sizeof(float) * 2), d_);
}

void tunerPID::readValue()
{
    EEPROM.get(eeAdresse_ + (sizeof(float) * 0), p_);
    EEPROM.get(eeAdresse_ + (sizeof(float) * 1), i_);
    EEPROM.get(eeAdresse_ + (sizeof(float) * 2), d_);
}
