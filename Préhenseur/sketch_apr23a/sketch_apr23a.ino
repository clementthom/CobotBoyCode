#include <AutoPID.h>


//paramètres PID
#define KP 1000.0
#define KI 0.0
#define KD 0.08

//constantes circuit
#define RESISTANCE_MESURE_COURANT 10

//paramètres contrôle PWM shield
#define PWM_OUTPUT_MIN 0.0
#define PWM_OUTPUT_MAX 255.0


// ======================================================
// PREHENSEUR AVEC 2 CAPTEURS COURANT (A4 / A5)
// ======================================================



// -------------------- BROCHES --------------------
const int PIN_INA = 6;
const int PIN_INB = 7;
const int PIN_PWM = 3;

const int CURRENT_OPEN_PIN  = A5; // ouverture
const int CURRENT_CLOSE_PIN = A4; // fermeture

// -------------------- PARAMETRES --------------------

// PWM (0 → 1023 sur Arduino classique)
double gripperPWM;

// Seuils courant (à ajuster) 
const float thresholdOpen  = 1.600; //en mA, quand le lasso est ouvert à fond
const float thresholdClose = 2.600; //quand l'objet est serré --> couple max pour i = 260 mA
const float thresholdNominal = 1.500; //tension de consigne correspondant au courant à vide en fonctionnement nominal


// Timing
int dt = 10;
unsigned long timeoutMs = 1000;

// Pauses
unsigned long pauseOpenMs  = 1000;
unsigned long pauseCloseMs = 1000;

// -------------------- PID -------------------

//parametres

double voltageOpen; //PID input is in volts, not current :)
double voltageClose;
double measuredTension; //is tensionOpen or tensionClose depending on the rotation direction


double consigne; /*setPoint dans la doc de la bibliothèque AUTOPID --> change selon le fonctionnement (thresholdOpen
 si butée ouvert, thresholdClose si butée fermé,  thresholdNominal sinon) --> en Volts, pas Ampères*/


AutoPID pidPrehension(&measuredTension, &consigne, &gripperPWM, PWM_OUTPUT_MIN, PWM_OUTPUT_MAX, KP, KI, KD);



// ======================================================
// SETUP
// ======================================================
void setup() {

  pinMode(PIN_INA, OUTPUT);
  pinMode(PIN_INB, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);

  Serial.begin(9600);

  stopGripper();
  pidPrehension.setTimeStep(10);

  Serial.println("Demarrage prehenseur");
}

// ======================================================
// BAS NIVEAU
// ======================================================
void stopGripper() {
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_INA, LOW);
  digitalWrite(PIN_INB, LOW);
}

void ouvrirGripper(float gripperPWM) {
  digitalWrite(PIN_INA, HIGH);
  digitalWrite(PIN_INB, LOW);
  analogWrite(PIN_PWM, (int)gripperPWM); //analogWrite can only write integers
}

void fermerGripper(float currentGripper) {
  digitalWrite(PIN_INA, LOW);
  digitalWrite(PIN_INB, HIGH);
  analogWrite(PIN_PWM, (int)gripperPWM);
}

// ======================================================
// DEPLACEMENT AVEC DETECTION
// ======================================================
void moveUntilLimit(bool opening) {

  unsigned long t0 = millis();
  int detectCount = 0;

    //mesure courant moteur
    double voltageOpen  = ((double)analogRead(CURRENT_OPEN_PIN)*5) / 1023; //courant mesuré quand le préhenseur s'ouvre
    double voltageClose = ((double)analogRead(CURRENT_CLOSE_PIN)*5) / 1023; //courant mesuré quand le préhenseur se ferme


    // Affichage brut
    Serial.print("A4(open): ");
    Serial.print(voltageOpen/RESISTANCE_MESURE_COURANT);
    Serial.print("   A5(close): ");
    Serial.println(voltageClose/RESISTANCE_MESURE_COURANT);

    // Sélection du bon capteur
    measuredTension = opening ? voltageOpen : voltageClose;//setpoint a sa valeur attribuée
        

    //ecriture dans servo
    if (opening) {
      Serial.println("OUVERTURE");
      if(measuredTension < thresholdNominal*1.2) {
        consigne = thresholdNominal;
      }
      else {
        consigne = thresholdOpen;
      }
      //lancement du PID    
      pidPrehension.run();
      ouvrirGripper(gripperPWM);
    } 
    else {
      Serial.println("FERMETURE");
      if(measuredTension < thresholdNominal*1.2) {
        consigne = thresholdNominal;
      }
      else {
        consigne = thresholdClose;
      }
      //lancement du PID    
      pidPrehension.run();
      fermerGripper(gripperPWM);
    }
  }

  Serial.println("TIMEOUT -> STOP");
  stopGripper();
}


// ======================================================
// CYCLE COMPLET
// ======================================================
void cycleGripper() {

  // OUVERTURE
  moveUntilLimit(true);
  delay(pauseOpenMs);

  // FERMETURE
  moveUntilLimit(false);
  delay(pauseCloseMs);
}

// ======================================================
// LOOP
// ======================================================
void loop() {

  cycleGripper();

  delay(4000);
}

