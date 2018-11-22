#include <SoftwareSerial.h>
SoftwareSerial BTserial(6, 7);

#include <AccelStepper.h>
#define HALFSTEP 8

// Motor1 pin definitions
#define motor1Pin1  2     // IN1 on the ULN2003 driver 1
#define motor1Pin2  3     // IN2 on the ULN2003 driver 1
#define motor1Pin3  4     // IN3 on the ULN2003 driver 1
#define motor1Pin4  5     // IN4 on the ULN2003 driver 1

// Motor2 pin definitions
#define motor2Pin1  9     // IN1 on the ULN2003 driver 1
#define motor2Pin2  10     // IN2 on the ULN2003 driver 1
#define motor2Pin3  11     // IN3 on the ULN2003 driver 1
#define motor2Pin4  12     // IN4 on the ULN2003 driver 1

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper1(HALFSTEP, motor1Pin1, motor1Pin3, motor1Pin2, motor1Pin4);
AccelStepper stepper2(HALFSTEP, motor2Pin1, motor2Pin3, motor2Pin2, motor2Pin4);

const float Pi = 3.14159;

const int numChars = 512;       // Für Auslesen der Antwort vom Tachy
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;      // Wenn neue Daten vom Tachy anliegen

int delaytime = 1000;         // Dauer eines Zylkus in ms

long previousMillis = 0; 
long currentMillis = 0;  

double QN;         // QN
double LN;         // Längsneigung

double QN_old;      
double LN_old;    

double deltaQN = 0;
double deltaLN = 1;

double deltaQN_Old;
double deltaLN_old;

double tachyHz = 0;
double dStepTachy = 40;

double endposition= 400;
double savePositionTachy;
boolean newSteps;
boolean rotDirectionTachy = true;  // true -> Uhrzeigersinn, false = CCW

int tarPos1 = 5000;
int tarPos2 = 5000;

double SCHWELLWERT_QN = 0.001; // 3 mgon      Funktioniert auch für 1 mgon
double SCHWELLWERT_LN = 0.001;  

boolean face = true; // Lage des Tachys. true: Lage I - false: Lage II;

long stepperPos1 = 0;
long stepperPos2 = 0;

double actualHz = 0; 

boolean detaillierteAusgabe = false; // Werte beim Horizontieren Anzeigen -> true

long startTime;
long endTime;

//==========================================================================
// Setup 
//==========================================================================


void setup() {

  
  Serial.begin(9600);
  BTserial.begin(38400);

  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(100.0);
  stepper1.setSpeed(200);


  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(100.0);
  stepper2.setSpeed(200);

  Serial.println("Arduino bereit...");
  
  findTachyHz();    // Bestimmt HZ für Justiervorgang und Lage
  //  tachyHz = 30;
  // face = true;

}


//==========================================================================
// Loop
//==========================================================================

void loop() {

 get_LN_QN_at_position();
 
 serialPrintLNandQN();    // Ausgabe

    if((fabs(QN) > SCHWELLWERT_QN)||(fabs(LN) > SCHWELLWERT_LN)){
      startTime = millis();
      Serial.println("Neigung zu groß, horizontiere...");
    //  changeFace();
  //    delay(1000);
      tachyHorizontieren();
    //  changeFace();
  //   delay(1000);
    //  tachyHorizontieren();
      endTime = millis();

      Serial.println("=============================");
      Serial.println("Tachymeter horizontiert");
      Serial.print("Dauer:\t");
      Serial.print((endTime - startTime)/1000);
      Serial.println(" s");
      Serial.print("LN:\t");
      Serial.print(LN);
      Serial.println(" gon");
      Serial.print("QN:\t");
      Serial.print(QN);
      Serial.println(" gon");
      Serial.println("=============================");
    }
    
  delay(10000);     // Kontrolliere alle 10 Sekunden ob Neigung zu groß

}


//==========================================================================
// Methoden
//==========================================================================


void sendCommand(String stringData) { 
  // Befehl über BT an Tachymeter schicken.
  // Befehl kann ein typischer GeoCOM Befehl sein.
  
  stringData = '\n' + stringData + '\r' + '\n'; // Adding CR/LF    CR LF Ascii: \r\n
  
  for (int i = 0; i < stringData.length(); i++)
  {
    BTserial.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
  }
}

//_________________________________________________________________________

void recWithEndMarker(){
  // Antwort Tachymeter lesen. 
  // Achtung, läuft schnell und immer in Main Loop ab.
  // Nötig für die Methode: Globale Variable "newData"
  
  static int ndx = 0; 
  char endMarker = '\n';
  char rc;
  
  
  while(BTserial.available()> 0 && newData == false){
        
    rc = BTserial.read();

    if(rc != endMarker){
      receivedChars[ndx] = rc;
      ndx++;
      if(ndx >= numChars){
        ndx = numChars - 1;
      }
    }else{
      receivedChars[ndx] = endMarker;  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}


//_________________________________________________________________________

void parseData() {
  /* Datenstring wird zerschnitten, Auslesen der wichtigen Variablen 
  HZ, Querneigung und Längsneigung. Variablen werden direkt in die 
  Globalen Variablen gespeichert*/
  
  // split the data into its parts
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(receivedChars,",");      // get the first part - the string
  strtokIndx = strtok(NULL, ",");
  
  strtokIndx = strtok(NULL, ",");     // Hz lesen und speichern
  actualHz = atof(strtokIndx);
  actualHz = actualHz * 200/Pi;
  
  strtokIndx = strtok(NULL, ",");
  strtokIndx = strtok(NULL, ",");
  strtokIndx = strtok(NULL, ",");
  strtokIndx = strtok(NULL, ",");
  
  strtokIndx = strtok(NULL, ",");  
  QN = atof(strtokIndx);  // QN lesen und Speichern  
  QN = QN * 200 / Pi;
  
  strtokIndx = strtok(NULL, ",");  // Längsneigung lesen und Speichern
  LN = atof(strtokIndx);    
  LN = LN * 200 / Pi;

  if((QN == 0) && (LN == 0)){
    Serial.println("ERROR - NEIGUNG ZU HOCH!!!");

  }
  
  strtokIndx = strtok(NULL, ",");
  strtokIndx = strtok(NULL, ",");
  strtokIndx = strtok(NULL, ",");
}

//_________________________________________________________________________


void get_LN_QN(){
  /* Auslesen von Längs und Querneigung in einer Schleife. Achtung, die Schleife
  muss sehr schnell durchlaufen. Funktion ist nötig, wenn Motoren parallel 
  gesteuert werden*/
  
  currentMillis = millis();
  
  if(currentMillis - previousMillis > delaytime){    // Befehl schicken, jede Sekunde 
    sendCommand("%R1Q,2003:0");
    previousMillis = currentMillis;
  }

  recWithEndMarker();

}


//_________________________________________________________________________


void get_LN_QN_at_position(){
  /* Funktion zum Auslesen der Längs und Querneigung an einer festen Position
  Motoren können nicht parallel bewegt werden. */
  
  sendCommand("%R1Q,2003:0"); // Befehl Senden

  while(newData == false){
    recWithEndMarker();       // Lesen bis Antwort komplett
  }

  parseData();                // Parsen -> LN und QN
  
  newData = false;            // Für nächsten Durchlauf 
  
}


//_________________________________________________________________________


void gotToNewPosition(int angleHZgon){
  /* Tachymeter auf neue Position fahren
  Eingabe der neuen Position in Gon*/
  
  double angleHZrad = angleHZgon * Pi / 200;
  double angleVrad = 100 * Pi / 200;

  String angleHZstring = String(angleHZrad,6);
  String angleVstring = String(angleVrad,6);

  String command = "%R1Q,9027:" + angleHZstring + "," + angleVstring + ",0,0,0";

  sendCommand(command);

    while(newData == false){
    recWithEndMarker();       // Lesen bis Antwort komplett
  }
  
  newData = false;            // Für nächsten Durchlauf 
}

//_________________________________________________________________________

void stepperMove(){
  /* Motoren bewegen, bewegt beid eMotoren gleichzeitig zu tarPos1 und tarPos2
  Wird verwendet um Hz Startrichtung zu finden. */
  
  stepper1.move(tarPos1);
  stepper2.move(-tarPos2);

  while(true){
    bool stepper1Running = stepper1.run();
    bool stepper2Running = stepper2.run();

    if((stepper1Running == false) && (stepper2Running == false)){
      break;
    }
  }

  tarPos1 = -tarPos1;
  tarPos2 = -tarPos2;
}

//_________________________________________________________________________

void stepperMoveToPositions(long moveToPos1, long moveToPos2){
  /* Schrittmotoren auf Positionen fahren. Positionen sind relativ zur Start-
   *  Position.
   */
  stepper1.move(moveToPos1);
  stepper2.move(moveToPos2);

  while(true){
    bool stepper1Running = stepper1.run();
    bool stepper2Running = stepper2.run();

    if((stepper1Running == false) && (stepper2Running == false)){
      break;
    }
  }
}


//_________________________________________________________________________


void get_deltaQN_deltaLN(){
  /*
   * deltaQN und deltaLN bestimmen. Funktion dient zur Suche der Hz Justierrichtung
   */
   
  get_LN_QN_at_position();
  
  LN_old = LN;
  QN_old = QN;

  stepperMove();

  get_LN_QN_at_position();

  deltaQN = fabs(LN_old - LN);
  deltaLN = fabs(QN_old - QN);
  
}


//_________________________________________________________________________


void turnMotorsOff(){
  /*
   * Motoren ausschalten. Alle Pins Low.
   */
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor1Pin3, LOW);
    digitalWrite(motor1Pin4, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    digitalWrite(motor2Pin3, LOW);
    digitalWrite(motor2Pin4, LOW);
}


//_________________________________________________________________________


void findTachyHz(){
  /*
   * Bestimmung des TachyHz, der für die Justierung des Dreifuß optimal ist.
   */
    long time1 = millis();

      while(true){          // Abbruchkriterium
        
        gotToNewPosition(tachyHz);      // Tachy auf neue Hz Position fahren
        delay(1000);
    
        deltaLN_old = deltaLN;          // Alte dLN abspeichern
        
        get_deltaQN_deltaLN();          // Neue dLN berechnen
        
        Serial.print("HZ, DeltaQN, DeltaLN: ");
        Serial.print(tachyHz);
        Serial.print("   ");
        Serial.print(deltaQN,5);
        Serial.print("   ");
        Serial.println(deltaLN,5);
        
        if((deltaLN > deltaLN_old) && (newSteps == false)){
          dStepTachy = dStepTachy/3*2;                // Schrittweite neu = 2/3 Schrittweite alt
          rotDirectionTachy =! rotDirectionTachy;   // Drehrichtung umkehren
          Serial.println("Richtungswechsel, Schrittweite halbiert");
          newSteps = true;
        }else{
          newSteps = false;
        }    
      
        turnMotorsOff();
    
        if(deltaLN < 0.0003){             // Abbruchskriterium, bevor Winkel hochgezählt werden
          break;
        }   
        
        if(rotDirectionTachy){
          tachyHz = tachyHz + dStepTachy;   // Bei Uhrzeigersinn
        }else{
          tachyHz = tachyHz - dStepTachy;   // Bei gegen Uhrzeigersinn
        }
        
        if(tachyHz > 400){              // Abfrage ob eingegebener Winkel im Arbeitsbereich
          tachyHz = tachyHz - 400; 
        }else if(tachyHz < 0){
          tachyHz = tachyHz + 400;
        }  
       
      }

  Serial.println("Bestimme Lage...");
  getFace();

  long time2 = millis();
  Serial.println("==========================================");
  Serial.println("Justierposition Tachy gefunden und gesetzt:");
  Serial.print("Hz-Winkel [gon]:\t \t");
  Serial.println(tachyHz);
  Serial.print("deltaLN [gon]:\t \t \t");
  Serial.println(deltaLN,5);
  Serial.print("Tachymeter Lage:\t \t");
  if(face){
    Serial.println("Lage I");
  }else{
    Serial.println("Lage II");
  }
  
  Serial.print("Dauer des Justiervorgangs [s]:\t");
  Serial.println((time2-time1)/1000);
  Serial.println("==========================================");
}


//_________________________________________________________________________


void getFace(){
  /*
   * Bestimmung der aktuellen Lage des Tachymeters.
   */
   
  get_LN_QN_at_position();
  
  QN_old = QN; 
  LN_old = LN;

  stepperMoveToPositions(5000, -5000);

  get_LN_QN_at_position();

  deltaLN = LN_old-LN;

  if(deltaLN < 0){
    face = true;    // Lage I
  }else{
    face = false;   // Lage II
  }
  
  stepperMoveToPositions(-5000, 5000);
  turnMotorsOff();
  
}


//_________________________________________________________________________

void tachyHorizontieren(){
  /*
   * Tachymeter horizontieren. Hauptfunktion
   */
  Serial.println("Horizontiere...");
  
  get_LN_QN_at_position(); // Für Eingangswerte 2. Lage


// Wenn man das Tachy zum horizontieren auf die optimale Position fahren möchte...   
//  gotToNewPosition(tachyHz);
//  delay(1000);
  
    while(true){
      
    get_LN_QN();    
  
    if(newData == true){
      LN_old = LN;
      QN_old = QN;
  
      parseData();

          // Transformation der QN und LN wenn nicht die optimale Pos. angefahren wurde.
          // QN / LN entsprechen hier den gemessenen Werten.
          double LN_gem = LN;  // gerade gemessene Werte
          double QN_gem = QN;  // gerade gemessene Werte
          double phi = (tachyHz - actualHz)/180*Pi; // Verdrehung in rad
      
          LN = QN_gem*sin(phi) + LN_gem*cos(phi);
          QN = QN_gem*cos(phi) - LN_gem*sin(phi);

      if(detaillierteAusgabe){                  // Detaillierte Ausgabe wenn gewünscht.
        Serial.println("=================");
        Serial.println("QN_gem \t \t LN_gem");
        Serial.print(QN_gem,5);
        Serial.print("\t \t ");
        Serial.println(LN_gem,5);
        Serial.println("QN \t \t LN");
        Serial.print(QN,5);
        Serial.print("\t \t");
        Serial.println(LN,5);
      }
      
      newData = false;      
     }


    // Setzen der Geschwindigkeiten: Langsamer wenn feinere Bewegung.
     if((fabs(QN) < 0.005) && fabs(LN) < 0.005){
        stepper1.setMaxSpeed(300.0);
        stepper2.setMaxSpeed(300.0);
     }else if((fabs(QN) < 0.01) && fabs(LN) < 0.01){
        stepper1.setMaxSpeed(500.0);
        stepper2.setMaxSpeed(500.0);
     }else{
        stepper1.setMaxSpeed(1000.0);
        stepper2.setMaxSpeed(1000.0);
     }
  
    // Immer das größere einstellen.
      if(fabs(QN) > fabs(LN)){
        // Stelle QN ein (passt von Vorzeichen her)
        if(QN < 0){
          stepperPos1 = -10000;   // Nach links
          stepperPos2 = -10000;
        }else if(QN > 0){
          stepperPos1 = 10000;
          stepperPos2 = 10000;
        }
      }else if (fabs(LN) > fabs(QN)){
        // Stelle LN ein
        if(LN < 0){
          stepperPos1 = 10000;
          stepperPos2 = -10000;
        }else if(LN > 0){
          stepperPos1 = -10000;
          stepperPos2 = 10000;
        }      
      }

      // Wenn Lage II
      if(face == false){              // In Lage II: Genau anderes herum.
        stepperPos1 = -stepperPos1;
        stepperPos2 = -stepperPos2;
      }
      
    // Bewegen bzw. Stopkriterium. Ausgabe der Werte Seriell.  
    if ((fabs(QN) >= SCHWELLWERT_QN) || (fabs(LN) >= SCHWELLWERT_LN)){      
        stepper1.move(stepperPos1);
        stepper2.move(stepperPos2);
        
          stepper1.run();  
          stepper2.run();    
      }else{
          turnMotorsOff();
          break;
      } 
    }
  }

//_________________________________________________________________________


void changeFace(){
  /*
   * Wechseln der Tachymeterlage
   */
  sendCommand("%R1Q,9028:0,0,0"); // Nochmal genauer anschauen 
  while(newData == false){
    recWithEndMarker();       // Lesen bis Antwort komplett
  }
  Serial.print("Lage gewechselt, neue Lage: ");
  face =! face;
  if(face){
    Serial.println("Lage I");
  }else{
    Serial.println("Lage II");
  }

  newData = false;
}

//_________________________________________________________________________

void serialPrintLNandQN(){
  Serial.print("LN:\t");
  Serial.print(LN,5);
  Serial.print("\tQN:\t");
  Serial.println(QN,5);   
}
  


