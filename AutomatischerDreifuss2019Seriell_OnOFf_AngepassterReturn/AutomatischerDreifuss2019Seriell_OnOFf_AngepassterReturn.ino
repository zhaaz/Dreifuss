#include <SoftwareSerial.h>
#include <AccelStepper.h>

SoftwareSerial serialTachy(4,3);      //RX TX 

#define HALFSTEP 8

#define MOTOR1_PIN1  5 
#define MOTOR1_PIN2  6 
#define MOTOR1_PIN3  7
#define MOTOR1_PIN4  8

#define MOTOR2_PIN1  9 
#define MOTOR2_PIN2  10 
#define MOTOR2_PIN3  11
#define MOTOR2_PIN4  12

AccelStepper stepper1(HALFSTEP, MOTOR1_PIN1, MOTOR1_PIN3, MOTOR1_PIN2, MOTOR1_PIN4);
AccelStepper stepper2(HALFSTEP, MOTOR2_PIN1, MOTOR2_PIN3, MOTOR2_PIN2, MOTOR2_PIN4);

 const int ledPIN = 13; 

const int buttonPin = 2; 
int buttonState = 0; 

int lastButtonState = 0;

long TIMEOUT = 300;   // TIMEOUT in Sekunden (Horizontieren)

float HORIZONTAL_ANGLE = 0;                 // Wird über Tastendruck überschrieben und gilt dann als Konstante
float VERTICAL_ANGLE = 0; 


double THRESHOLD_CROSSINCLIN = 0.001; // 3 mgon      Funktioniert auch für 1 mgon
double THRESHOLD_LENGTHINCLIN = 0.001;


const int numChars = 256;       // Für Auslesen der Antwort vom Tachy
char receivedChars[numChars];
char receivedCharsYKabel[numChars];
bool newData = false;

// Parameter für die Horizontierung

float QN = 0; 
float LN = 0; 
float actualHz = 0;


long stepperPos1 = 0;
long stepperPos2 = 0;

char command;
char commandType;
float valueIn;

boolean face = true; // Lage 1 = true, Lage 2 = false

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPIN, OUTPUT);
  
  Serial.begin(19200);
  delay(2000);
  Serial.println("Arduino bereit...");

  
  serialTachy.begin(19200);
  
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(200.0);
  stepper1.setSpeed(1000);
  
  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(200.0);
  stepper2.setSpeed(1000);

  delay(5000);

  // Setzen der Auslesen und Speichern der aktuellen Orientierung, Rückmeldung über Seriell und LED    
  saveActualTachyPosition();
  
}

void loop() {


  //============= Serial Code ============
  // Alles was über das Y-Kabel kommt
  //======================================#

  recSerialWithEndMarker();     // Schauen, ob etwas über die serielle Schnittstelle (Y-Kabel) kommt...
  
  if (newData == true) {
    parseYKabelData();

    switch (commandType){
      case '%':
        // % Kennzeichnet standard Geocom Befehl - wird an Tachymeter durchgereicht, anschließend Antwort zurück an Y-Kabel        
        sendTachyCommand(receivedCharsYKabel);    // Empfangenen Befehl weiterreichen
        receiveTachyReturn();                     // Antwort auf Arduino empfangen
        Serial.println(receivedChars);            // Antwort an Y-Kabel weiterreichen
        break;

      case '$':
        // $ Kennzeichnet Dreifuss Befehl
        // Auslesen des eigentlichen Befehls:
        
        switch (command){
            case 'I':
              // Auslesen der aktuellen Längs und Querneigung (entspricht %R1Q,2003:0)       
              getCrossAndLengthInclination();
              printLNQN();
              break; 
               
            case 'F':
              // Wechsel der Lage
              Serial.println("Change Face");
              changeFace();
              break; 

            case 'T':
              // Setzen Timeout
              TIMEOUT = valueIn;
              if(TIMEOUT > 300){
                TIMEOUT = 300;
              }
              Serial.print("$R:T,0,");
              Serial.println(TIMEOUT);
              
              break;
              
            case 'S':
              // Setzen der Schwellwerte der Horizontierung 
              
              THRESHOLD_CROSSINCLIN = valueIn;
              THRESHOLD_LENGTHINCLIN = valueIn;
              Serial.print("$R:S,0,");
              Serial.println(THRESHOLD_CROSSINCLIN,4);
              break;
              
            case 'H':            
              // Horizontieren             
              makePositioning(HORIZONTAL_ANGLE, VERTICAL_ANGLE);
                          
              receiveTachyReturn();                     // Antwort auf Arduino empfangen
              face = true;      // Nach Anfahren der Horizontierpositon: Lage 1
              
              int returnState = tachyHorizontieren();
              Serial.print("$R:H,");
              Serial.println(returnState);
                      
              break;
               
            default:
              Serial.println("default");
              break; 
        }

      default:
        // nichts
        break;
    }   
    newData = false;
  }  
}

void runMotors(long stepsStepper1, long stepsStepper2){
  stepper1.move(stepsStepper1);
  stepper2.move(stepsStepper2);
  while(true){
      bool stepper1Running = stepper1.run();
      bool stepper2Running = stepper2.run();
      if(stepper2Running == false && stepper1Running == false){
        break;
      }
    }
}

void changeFace(){
  // Funktion um Lage zu wechseLN.
  serialTachy.write("%R1Q,9028:\r\n");  // change Face
  receiveTachyReturn();  
  face  = !face;
}

void sendTachyCommand(char tachyCommand[256]){
  // beliebigen Command an Tachymeter
  serialTachy.write(tachyCommand);    
}

void receiveTachyReturn(){
  // Empfängt Daten vom Tachymeter und leitet diese auf die Serielle Schnittstelle weiter
  
  int ndx = 0;
   
  while(true){
    
    if(serialTachy.available() > 0){
      char c = serialTachy.read();
      receivedChars[ndx] = c;
      ndx++;
      if(c == '\n')        
        break;
    }
  }
  receivedChars[ndx] = '\0';
}

void recSerialWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
   
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedCharsYKabel[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedCharsYKabel[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void getCrossAndLengthInclination(){
  clearSoftwareSerialBuffer();
  serialTachy.write("%R1Q,2003:0\r\n");    // Abfrage QN LN und Hz
  receiveTachyReturn();
  
  // Parse for Hz
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(receivedChars,",");      // get the first part - the string
  
  strtokIndx = strtok(NULL, ",");     // Return erste Null
  int a = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");     // Return zweite Null
  int b = atof(strtokIndx); 
  
  strtokIndx = strtok(NULL, ",");     // HZ
  actualHz = atof(strtokIndx);
  
  strtokIndx = strtok(NULL, ",");     // V
  float vertical = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");     // AngleAccuracy
  float angleAccuracy = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");     // Angle Time
  long angleTime = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");     // Cross Inclination (QN)
  QN = atof(strtokIndx);
  
  strtokIndx = strtok(NULL, ",");     // Length Inclination (LN)
  LN = atof(strtokIndx);
  

  strtokIndx = strtok(NULL, ",");     // Accuracy Inclin     
  float inclinAccuracy = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");     // Inclin Time     
  long inclinTime = atof(strtokIndx);
 
  strtokIndx = strtok(NULL, ",");     // Face Def     
  long faceDef = atof(strtokIndx);

}

void saveActualTachyPosition(){
  serialTachy.write("%R1Q,2107:0\r\n");    // Abfrage QN LN und Hz
  receiveTachyReturn();

  // Parse for Hz
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(receivedChars,",");      // get the first part - the string
  
  strtokIndx = strtok(NULL, ",");     // Return erste Null
  int a = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");     // Return zweite Null
  int b = atof(strtokIndx); 
  
  strtokIndx = strtok(NULL, ",");     // HZ
  HORIZONTAL_ANGLE = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");     // V
  VERTICAL_ANGLE = atof(strtokIndx);

  if(HORIZONTAL_ANGLE != 0 && VERTICAL_ANGLE != 0){ 
    digitalWrite(ledPIN, HIGH); 
//    Serial.print("Hz [gon]: ");
//    Serial.println(HORIZONTAL_ANGLE*200/PI,4);
//    Serial.print(" V [gon]: ");
//    Serial.println(VERTICAL_ANGLE*200/PI,4);
  }else{
    // Serial.println("ERROR: Verbindung zum Tachymeter nicht hergestellt...");
  }
 
 
}

void makePositioning(float Hz, float V){
  // Tachymeter auf Position fahren
  String command =  "%R1Q,9027:" + String(Hz,4) + "," + String(V,4) + ",0,0,0" + '\r' + '\n' + '\0';  
  char makePosCommand[256];
  command.toCharArray(makePosCommand, 256);
  // Serial.println("Bewege Tachy auf Justierposition...");
  sendTachyCommand(makePosCommand);
}

int tachyHorizontieren(){

  long currentMillis = 0;
  long previousMillis = 0;
  int delaytime = 5000;     // Zeit zwischen Auslesen der Tachyneigungen

  long startTimeHor = millis();
   
  int errorState = 0;

  while(true){

    currentMillis = millis();
    if(currentMillis - previousMillis > delaytime){   // Nur wenn Zeitintervall überschritten
      getCrossAndLengthInclination();                 // QN, LN, actualHz werden von Tachy gelesen
      previousMillis = currentMillis;
      QN = QN * 200 / PI;   // Umwandlung in gon für weiteres Programm
      LN = LN * 200 / PI;
    }
    
    // Setzen der Geschwindigkeiten: Langsamer wenn feinere Bewegung.
     if((fabs(QN) < 0.005) && fabs(LN) < 0.005){
        stepper1.setMaxSpeed(300.0);
        stepper2.setMaxSpeed(300.0);
        delaytime = 2000; 
     }else if((fabs(QN) < 0.01) && fabs(LN) < 0.01){
        stepper1.setMaxSpeed(500.0);
        stepper2.setMaxSpeed(500.0);
        delaytime = 2000;
     }else{
        stepper1.setMaxSpeed(1000.0);
        stepper2.setMaxSpeed(1000.0);
        delaytime = 5000; 
     }
    
    // Immer das größere einstellen.
      if(fabs(QN) > fabs(LN)){
        // Stelle QN ein (passt von Vorzeichen her)
        if(QN < 0){
          stepperPos1 = 10000;
          stepperPos2 = 10000;
        }else if(QN > 0){
          stepperPos1 = -10000;
          stepperPos2 = -10000;
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
    if ((fabs(QN) >= THRESHOLD_CROSSINCLIN) || (fabs(LN) >= THRESHOLD_LENGTHINCLIN)){      
        stepper1.move(stepperPos1);
        stepper2.move(stepperPos2);
        
          stepper1.run();  
          stepper2.run();    
      }else{
          turnMotorsOff();
          break;
      } 

      if((millis() - startTimeHor)/1000 > TIMEOUT){
        // Zeitüberschreitung
        errorState = 1;
        break;
      }
    }

   int passedTime = (millis() - startTimeHor)/1000;
   
   return errorState;
}


void turnMotorsOff(){
  /*
   * Motoren ausschalten. Alle Pins Low.
   */
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR1_PIN3, LOW);
    digitalWrite(MOTOR1_PIN4, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
    digitalWrite(MOTOR2_PIN3, LOW);
    digitalWrite(MOTOR2_PIN4, LOW);
}

void clearSoftwareSerialBuffer(){
  while(serialTachy.available() > 0){
    serialTachy.read();
  }
}

void parseYKabelData(){
  
  char * strtokIndx;

  commandType = receivedCharsYKabel[0];

  if(commandType == '$')
  {
    command = receivedCharsYKabel[1];
    strtokIndx = strtok(receivedCharsYKabel, ":"); // Hier: Command Indx
  
    strtokIndx = strtok(NULL, ":");
    valueIn = atof(strtokIndx);

  }
}

void printLNQN(){
  if(LN == 0 && QN == 0){
    Serial.println("ERROR: Neigung zu groß!");
  }else{

    Serial.print("Längsneigung: ");
    Serial.println(LN*200/PI,4);
    
    Serial.print("Querneigung: ");
    Serial.println(QN*200/PI,4);
  }
}
