/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <tunerPID.h> // Vos propres librairies
#include <EEPROM.h>
#include <PIDCustom.h>

/*------------------------------ Constantes ---------------------------------*/

#define BAUD 115200        // Frequence de transmission serielle
#define UPDATE_PERIODE 100 // Periode (ms) d'envoie d'etat general

#define MAGPIN 32 // Port numerique pour electroaimant
#define POTPIN A5 // Port analogique pour le potentiometre

#define PASPARTOUR 64     // Nombre de pas par tour du moteur
#define RAPPORTVITESSE 50 // Rapport de vitesse du moteur

#define ANGLE_OPEN_SERVO 170
#define ANGLE_CLOSE_SERVO 40
#define LIMIT_SWITCH_PIN 7
#define START_SWITCH_PIN 6

#define OFFSET_PENDULE 5
/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;               // objet arduinoX
MegaServo servo_;           // objet servomoteur
VexQuadEncoder vexEncoder_; // objet encodeur vex
IMU9DOF imu_;               // objet imu
PIDCustom pid_1(0);                  // objet PID pour le moteur
PIDCustom pid_2(sizeof(float) * 3);                  // objet PID pour le pendule

enum etats
{
  StartSequence,
  ReculLimitSwitch,
  AvancerObjet,
  PrendreObjet,
  AtteindreHauteur,
  TraverserObstacle,
  StabiliserObjet,
  LacherObjet,
  Retour,
  TunePid
}; // machine a etats

etats etat = ReculLimitSwitch;

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_; // chronometre d'envoie de messages
SoftTimer timerPulse_;   // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0; // temps dun pulse en ms
float pulsePWM_ = 0;     // Amplitude de la tension au moteur [-1,1]

float Axyz[3]; // tableau pour accelerometre
float Gxyz[3]; // tableau pour giroscope
float Mxyz[3]; // tableau pour magnetometre



bool first_scan = true;
bool pidFini = false;

double lastTimePendule = millis();
double lastValuePot = 0;
double vitessePendule = 0;
double anglePendule = 0;

double penduleGoal = 0.15;
double distanceGoal = 0.1;
bool PIDGoalReached = 0;
int compteurBalance = 0;
int balancement = 0;

double compteurTotalPulse = 0;

double lastTime100ms = 0;
// bool doOnce = 1;
/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg();
void readMsg();
void serialEvent();

// Fonctions pour le PID
double PIDmeasurement_lineaire();
double PIDmeasurement_pendule();
void PIDcommand_motor(double cmd);
void PIDcommand_pendule(double cmd);
void PIDgoalReached_motor();
void PIDgoalReached_pendule();

// Fonctions
bool reculLimitSwitch(void);
bool avancer(double distance);
void setPIDslow(void);
void setPIDHigh(void);
void disableMotorPID(void);
void balancerPendule(void);

/*---------------------------- fonctions "Main" -----------------------------*/

void setup()
{
  Serial.begin(BAUD);     // initialisation de la communication serielle

  AX_.init();             // initialisation de la carte ArduinoX
  imu_.init();            // initialisation de la centrale inertielle
  vexEncoder_.init(2, 3); // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(
      vexEncoder_.getPinInt(), []
      { vexEncoder_.isr(); },
      FALLING);

  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);

  etat = StartSequence;

  servo_.attach(8);
  servo_.write(ANGLE_CLOSE_SERVO);

  // PID
// Initialisation du PID
    // Attache des fonctions de retour
    pid_1.setMeasurementFunc(PIDmeasurement_lineaire);
    pid_1.setCommandFunc(PIDcommand_motor);
    pid_1.setAtGoalFunc(PIDgoalReached_motor);
    pid_1.setEpsilon(0.03);
    pid_1.setGains(5,0.01,0.01);
    pid_1.setPeriod(100);
    pid_1.enable();
    // pid_1.setGoal(0.1);
    // pid_1.setIntegralLim(1);

    // Attache des fonctions de retour
    pid_2.setMeasurementFunc(PIDmeasurement_pendule);
    pid_2.setCommandFunc(PIDcommand_pendule);
    pid_2.setAtGoalFunc(PIDgoalReached_pendule);
    pid_2.setEpsilon(0.01);
    pid_2.setPeriod(100);
    //pid_2.setIntegralLim(1);
    // AX_.setMotorPWM(0,0.4);
    pinMode(START_SWITCH_PIN,INPUT); // Peut-etre pas necessaire ?
    pinMode(LIMIT_SWITCH_PIN,INPUT);
    
}

/* Boucle principale (infinie)*/
void loop()
{
  
  // pid_1.setGoal(penduleGoal/2*sin( millis() * 1300) + penduleGoal / 2  );
  if (shouldRead_)
  {
    readMsg();
  }
  if (shouldSend_)
  {
    sendMsg();
  }
  if (shouldPulse_)
  {
    startPulse();
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  // mise à jour du PID
  
  pid_1.run();
 
  pid_2.disable();
  
  double timeNow = millis();
  if(timeNow - lastTime100ms >= 100)
  {
    lastTime100ms = timeNow;
    switch (etat)
    {
      case StartSequence:
        if(digitalRead(START_SWITCH_PIN))
        {
          etat = ReculLimitSwitch;
          servo_.write(ANGLE_OPEN_SERVO); // just in case
        }
        break;
      case ReculLimitSwitch:
        if(!reculLimitSwitch())
        {
          etat = AvancerObjet;
        }
        break;
      case AvancerObjet:
        if(!avancer(0.1))
          etat = PrendreObjet;
        break;
      case PrendreObjet:
        servo_.write(ANGLE_CLOSE_SERVO);
        etat = AtteindreHauteur;
        delay(3000);
        break;

      case AtteindreHauteur:
        if(balancement == 0)
        {
          Serial.println("Atteindre hauteur balancement 0");
          disableMotorPID();
          pid_1.setGoal(0.1);
          pid_1.enable();
          balancement = 1;
        }
        else
        {
            setPIDHigh();
            Serial.println("Goal achieved");
            //PIDGoalReached = 0;
        }
        /* code */
        break;

      case TraverserObstacle:
        balancement = 0;
        /* code */
        break;

      case StabiliserObjet:
        /* code */
      // pid_1.disable();
      // pid_2.setGoal(0);
        break;

      case LacherObjet:
        /* code */
        servo_.write(ANGLE_OPEN_SERVO);
        break;

      case Retour:
        /* code */
        break;
      case TunePid:
        if (first_scan)
        {
          
          first_scan = false;
        }
      // pid_1.run();
        break;
    }
  }
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent() { shouldRead_ = true; }

void timerCallback() { shouldSend_ = true; }

void startPulse()
{
  /* Demarrage d'un pulse */
  Serial.println("Fonction Start Pulse");
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse()
{
  /* Rappel du chronometre */
  AX_.setMotorPWM(0, 0);
  AX_.setMotorPWM(1, 0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg()
{
  //Serial.println("Send MSG");
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message
  PIDmeasurement_pendule();
  doc["time"] = millis();
  doc["potVex"] = map(analogRead(POTPIN), 96,934,-95,85)+OFFSET_PENDULE;
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_1.getGoal();
  doc["measurements"] = PIDmeasurement_lineaire();
  // env
  // doc["voltage"] = AX_.getVoltage();
  // doc["current"] = AX_.getCurrent();
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  // doc["accelX"] = imu_.getAccelX();
  // doc["accelY"] = imu_.getAccelY();
  // doc["accelZ"] = imu_.getAccelZ();
  // doc["gyroX"] = imu_.getGyroX();
  // doc["gyroY"] = imu_.getGyroY();
  // doc["gyroZ"] = imu_.getGyroZ();
  // doc["isGoal"] = pid_1.isAtGoal();
  // doc["actualTime"] = pid_1.getActualDt();

  doc["VitPend"] = vitessePendule;

  doc["Kp"] = pid_1.getKp();
  doc["Ki"] = pid_1.getKi();
  doc["Kd"] = pid_1.getKd();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg()
{
  Serial.println("Read MSG");
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error)
  {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Analyse des éléments du message message
  parse_msg = doc["pulsePWM"];
  if (!parse_msg.isNull())
  {
    pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if (!parse_msg.isNull())
  {
    pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if (!parse_msg.isNull())
  {
    shouldPulse_ = doc["pulse"];
  }
  parse_msg = doc["setGoal"];
  if (!parse_msg.isNull())
  {
    pid_1.disable();
    if (doc["setGoal"][0] != pid_1.getKp())
    {
      pid_1.setKp(doc["setGoal"][0]);
    }
    if (doc["setGoal"][1] != pid_1.getKi())
    {
      pid_1.setKi(doc["setGoal"][1]);
    }
    if (doc["setGoal"][2] != pid_1.getKd())
    {
      pid_1.setKd(doc["setGoal"][2]);
    }
    pid_1.setEpsilon(doc["setGoal"][3]);
    penduleGoal = doc["setGoal"][4];
    // ajouter hauteurObstacle
    pid_1.enable();
  }
}

// Fonctions pour le PID
double PIDmeasurement_lineaire()
{
  return -1*(0.14*32*PI*AX_.readEncoder(0))/(3200.0*24.0); // encoder 0
}

double PIDmeasurement_pendule() // Trouver vitesse du pendule.
{
  double timeNow = millis();

  double deltaTime = timeNow - lastTimePendule;
  anglePendule = map(analogRead(POTPIN), 96,934,-95,85)+OFFSET_PENDULE;
  double deltaValuePot = anglePendule - lastValuePot;
  vitessePendule = (deltaValuePot*1000) / deltaTime;
  lastValuePot = anglePendule;
  return vitessePendule;
}


void PIDcommand_motor(double cmd)
{
  if(cmd > 1)
  {
    cmd = 1;
  }
  else if(cmd < -1)
  {
    cmd = -1;
  }
  //Serial.print("FUCK");
 // Serial.println(cmd);
  AX_.setMotorPWM(0, -cmd);
}

void PIDcommand_pendule(double cmd)
{
  
}

void PIDgoalReached_motor()
{
  PIDGoalReached = 1;
  // disableMotorPID();
  if(etat == AtteindreHauteur && balancement == 1)
  {
    // PIDGoalReached = 0;
    balancerPendule();
  }
    

}

void PIDgoalReached_pendule()
{
  pidFini = true;
}

bool reculLimitSwitch()
{
  // Set slow pid
  static bool setupRecul = 0;
  if(setupRecul == 0)
  {
    setPIDslow();
    pid_1.setGoal(-0.5);
    pid_1.enable();
    setupRecul = 1;
  }
  // reculer
  if(digitalRead(LIMIT_SWITCH_PIN))
  {
    disableMotorPID();
    PIDGoalReached = 0;
    setupRecul = 0;
    // reset compte de pulse
    return 0;
  }
  return 1;
  // if()
  // limit switch
  // return
}

bool avancer(double distance)
{
  static bool setupAvancer = 0;
  if(setupAvancer == 0)
  {
    PIDGoalReached = 0;
    setPIDslow();
    pid_1.setGoal(distance);
    pid_1.enable();
    setupAvancer = 1;
  }
  if(PIDGoalReached == 1)
  {
    Serial.println("Avancer fini");
    setupAvancer = 0;
    PIDGoalReached = 0;
    disableMotorPID();
    return 0;
  }
  // Serial.println("avance fokk");
  return 1;
}

void setPIDslow(void)
{
  pid_1.setGains(1,0.02,0.01); // surtout kp plus bas.
  pid_1.setEpsilon(0.03);
}

void setPIDHigh(void)
{
  pid_1.setGains(5,0.02,0.01); // surtout kp plus bas.
  pid_1.setEpsilon(0.03);
}

void disableMotorPID()
{
  AX_.setMotorPWM(0,0);
  pid_1.setGoal(0);
  pid_1.disable();
  PIDGoalReached = 0;
}

void balancerPendule()
{
  Serial.println("Balancer pendule");
  if(vitessePendule > 0)
  {
    pid_1.setGoal(0);
  }
  else
  {
    pid_1.setGoal(distanceGoal);
  }
  compteurBalance ++;
  pid_1.enable();
  if(compteurBalance >= 15)
  {
    etat = PrendreObjet;
    pid_1.disable();
    AX_.setMotorPWM(0, 0);
  }
}