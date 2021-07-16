/*
Exemple de librairie pouvant etre ajoute au projet
*/
#include <tunerPID.h>

// Class constructor
tunerPID::tunerPID(PID *pid)
{
    tunerSerie_ = new SoftwareSerial(2, 3); // RX, TX
    tunerSerie_->begin(115200);
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
    EEPROM.put(EE_ADRESSE + (sizeof(float) * 0), p_);
    EEPROM.put(EE_ADRESSE + (sizeof(float) * 1), i_);
    EEPROM.put(EE_ADRESSE + (sizeof(float) * 2), d_);
}

void tunerPID::readValue()
{
    EEPROM.get(EE_ADRESSE + (sizeof(float) * 0), p_);
    EEPROM.get(EE_ADRESSE + (sizeof(float) * 1), i_);
    EEPROM.get(EE_ADRESSE + (sizeof(float) * 2), d_);
}
