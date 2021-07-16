/*
Exemple de librairie pouvant etre ajoute au projet
*/
#include <tunerPID.h>

// Class constructor
tunerPID::tunerPID(PIDCustom *pid)
{
    tunerSerie_ = new SoftwareSerial(2, 3); // RX, TX
    tunerSerie_->begin(115200);
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
        pid->disable();
        pid->setGains(p_, i_, d_);
        pid->enable();
        if (text_.indexOf("save") != -1)
        {
            pid->saveValues();
        }
        if (text_.indexOf("read") != -1)
        {
            pid->disable();
            pid->readValue();
            pid->enable();
        }
    }
}
