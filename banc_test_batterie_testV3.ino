/* 
* Banc de charge/décharge de batteries NiMh
* version  pour test
*
*
* Jean ROGUIN, Maxime KELLER, Emmanuel GALEN  2017/01
*/
// Bibliothèque gerant la date et l'heure
#include "TimeLib.h"
// Bibliothèque carte SD
#include "SD.h"

// Initialisation des paramètres

boolean test = true;            // pour info de debogage-----------------------------test!
boolean finecrit = false;       // pour n'envoyer qu'une fois le message de fin de test
boolean condfinchr;             // indicateur de fin de charge rapide
const int nbcycles = 30;        // nombre de cycles charge/décharge successifs--------------------------test!
const int nbrapid = 10;        // nombre de cycles rapides avant de basculer sur des lents
const int nblent = 5;          // nombre de cycles lents intercalés entre rapides
const float Ufinchrap = 1600;  // tension max (en mV) pour la charge rapide
const float Udebrap = 800;     // tension min pour le début de la charge rapide
const float Ufindcharap = 900;   // seuil tension fin décharge rapide
const float Ufindchalen = 1000;  // seuil tension fin décharge lente
const float templim = 55;      // température max seuil de déclenchement protection
long tempmaxchar = 46800000;     // temps max de charge 13 hours that should not be exceeded
long tempsrepos = 3;      // temps d'attente en secondes apres charge ou décharge rapide---------test!
unsigned long tempomes = 100;        // temps en ms entre deux mesures-----------test!
unsigned long tdeb; 		// pour gestion de l'attente entre deux mesures
boolean finattente = true;			// gestion attente entre deux mesures
float refanalog = 2.56;   // si référence interne Mega
int fintest = 13;    // pin pour allumage led de fin de test
boolean testfin = false;   // indicateur de fin du test de tous les accus ou fin par protection
File fichdonn;      // pointeur du fichier de résultats
String nomfich;     // Nom du fichier incrémenté tous les jours
int jourprec;     // numéro du jour dans le mois
String donnees;    // donnees à enregistrer sur la carte SD
float resistance = 0.75;          // measured resistance of the power resistor
float resistanceBatWire = 0.00;   // measured resistance of wires between FET and battery

// Gestion de la date et heure
const char TIME_HEADER = 'T';   // Header tag for serial time sync message
int heure,minut,seconde,jour,mois,annee;

// Initialisation des variables dependant du nombre d'accus en test

const int nbaccu = 4;         // nombre d'accus en test
double tempacc[nbaccu];       // tableau des températures
unsigned long tempsatt[nbaccu]; // tableau des durées d'attente 
int nbcyacc[] = {0,0,0,0};        // tableau du nombre total de cycles par accu
int etataccu[] = {1,1,1,1};       // etats des accus (0: fin de cycles, 1: charge rapide, 2: attente, 3: décharge rapide, 4: attente, 5: charge lente, 6: décharge lente)
boolean finaccu[nbaccu] = {false,false,false,false};    // indicateur de fin de cyclage
float Uaccu[nbaccu];            // tableau des tensions aux bornes des accus
float Iaccu[nbaccu];            // tableau des courants dans les accus

// declaration des pins utilisees et initialisation
int analogPinVgnd      = 0;
float voltageProbeVgnd   = 0;
int analogPinVacc        = 1;
float voltageProbeVacc     = 0;
int analogPinVr      = 2;
float voltageProbeVr   = 0;
int analogPinVctn      = 3;
float voltageProbeVctn   = 0;
// à vérifier
float voltageDifference = 0;     // difference in voltage between analogPinOne and analogPinTwo
float batteryVoltage    = 0;     // calculated voltage of battery
float current           = 0;     // calculated current through the load (in mA)
int pwmcour[] = {0,1,2,3};        // pins pwm commande courant
int enableDechCh = 26;         // Output MOS command: "1"=Discharge "0"=Charge
int resetLatch   = 28;               // reset à "1" des latch de commande (mode charge des 11 accus)
int holdWrite = 27;
int bit_0 = 22;				// bit 2^^00
int bit_1 = 23;				// bit 2^^01
int bit_2 = 24;				// bit 2^^02
int bit_3 = 25;				// bit 2^^03

// Fonction setup de démarrage

void setup()
  {
  pinMode(fintest,OUTPUT);    // pour allumage d'une led en fin de test
  Serial.begin(9600);
  while (!Serial)
  {
    ; // attente de l'ouverture du moniteur série pour commencer 
  }
  Serial.println ("Initialisation Serie") ;
// Améliore la précision de la mesure en réduisant la plage de mesure sur les entrées analogiques
//  analogReference(INTERNAL); // Pour Arduino UNO
// analogReference(INTERNAL1V1); // Pour Arduino Mega2560
// initialisation carte SD
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) 
  {
   Serial.println("initialization failed!");
  // return; //--------------------------------------test!
  }

  Serial.println("initialization done.");
// Mise à l'heure et à la date et démarrage  
  Serial.println("Temps sous la forme: Thr,min,jour,mois,annee ?, exemple: T20,10,23,11,2016");
  seconde =0;
  while (Serial.available() == 0){
    }
  processSyncMessage();
//  jourprec = day(); // pour test sur changement de jour
  jourprec = minute();  // test avec changement toutes les minutes------------------test!
  nomfich = initficres(test);
  tdeb = millis();
  setupPWM16();
  // declaration des pins pour le multiplexage
	pinMode(bit_A, OUTPUT);
	pinMode(bit_B, OUTPUT);
	pinMode(bit_C, OUTPUT);
	pinMode(bit_D, OUTPUT);
// declaration des pins logiques charge/decharge	
	for (int iaccu=0; iaccu< nbaccu ;iaccu++)
	{
	pinMode(enChDech[iaccu],OUTPUT);
	}
  }
  
// Boucle   loop
void loop()
{
if (!testfin)  // si fin de tous les cycles sur tous les accus on boucle sur rien!!

  {
    if (minute() != jourprec) { // -----------------------------test!
      nomfich = initficres(test); // si changement de jour ouverture d'un nouveau fichier resultats
      jourprec = minute();  }
  // boucle sur les accus en test
    for (int iaccu=0; iaccu< nbaccu ;iaccu++)
    {
  // si atteinte du nombre max de cycles
    if (nbcyacc[iaccu] == nbcycles)
    {
    finaccu[iaccu] = true;
    etataccu[iaccu] = 0;
    if (test) {Serial.println("fin de cycles accus" + String(iaccu));}
    }
	// envoi code accu au multiplexeur
	afficher(iaccu);
	
    // lecture temperature accu
    int rctn = analogRead(enttemp[iaccu]);
    tempacc[iaccu] = temperature(rctn);
    // protection si accu trop chaud
    if (tempacc[iaccu] > templim)
    {
    donnees = String(iaccu)+','+ String(tempacc[iaccu]) + "arret protection temperature";
    Serial.println(donnees);
    testfin = true;
    }
   
    // mesure tension aux bornes del'accu 
    int Upos = analogRead(enttenspos[iaccu]);
    int Uneg = analogRead(enttensneg[iaccu]);
    Uaccu[iaccu] = (Upos-Uneg)/1023*refanalog;        // tensionaccu;
    Iaccu[iaccu] = 1.1;    //------------------------test!
    // gestion des différents états
    // fonction du nombre de cycles atteints
    if ((etataccu[iaccu]==6)&&(nbcyacc[iaccu]%(nbrapid + nblent) == 0))
    {
    etataccu[iaccu] = 1;
    }
    else
    {
    if ((etataccu[iaccu]==4)&&(nbcyacc[iaccu]%(nbrapid + nblent) == nbrapid))
      {
      etataccu[iaccu] = 5;
      }   
    }
    long dureeatt;
    int casaccu = etataccu[iaccu];
    switch (casaccu)
    {
    case 1:
    // charge rapide
    // lecture tension bornes accu
    // si tension inferieure au seuil charge lente initiale
    // lecture courant charge
    chargerapide();
    condfinchr = true; // ----------------------test!
    if (condfinchr)
      {
      donnees = "fin de charge rapide"; //----------------------test!
      ecritmesure(test,nomfich,donnees);        //---------------------test!
      etataccu[iaccu] = etataccu[iaccu] +1;
      tempsatt[iaccu] = now();
      }
    break;
    case 2:
    // attente
    // fin d'attente au bout de tempsrepos
    dureeatt = now() - tempsatt[iaccu];
    if (dureeatt > tempsrepos)
      {
      etataccu[iaccu] = etataccu[iaccu] +1;
      }
    break;
    case 3:
    // décharge rapide
    // décharge jusqu'à atteinte du seuil en tension basse
    dechargerapide();
      if (Uaccu[iaccu] < Ufindcharap) 
      {
      donnees = "fin de décharge rapide";
      ecritmesure(test,nomfich,donnees);
      etataccu[iaccu] = etataccu[iaccu] +1;
      tempsatt[iaccu] = now();
            }
    break;
    case 4:
    // attente
    // fin d'attente au bout de tempsrepos
    dureeatt = now() - tempsatt[iaccu];
    if (dureeatt > tempsrepos)
      {
      etataccu[iaccu] = 1;
      nbcyacc[iaccu] = nbcyacc[iaccu] + 1; // fin d'un cycle
      }
    break; 
    case 5:
    // charge lente
    donnees = "charge lente";  //-------------test!
    ecritmesure(test,nomfich,donnees);  //------------------test!
    etataccu[iaccu] = etataccu[iaccu] +1;
    break;
    case 6:
    // décharge lente
    // fin de décharge lors de l'atteinte seuil tension, on recommence le cycle
    if (Uaccu[iaccu] < Ufindchalen) 
      {
      donnees = "fin décharge lente"; //--------------test!
      ecritmesure(test,nomfich,donnees);  //-------------test!
      etataccu[iaccu] = 5;
      nbcyacc[iaccu] = nbcyacc[iaccu] + 1; // fin d'un cycle
       }
    break;
    default:
    // si erreur sur l'état ou fin de cycles accu
    Serial.println("erreur etat accu:"+String(iaccu)+','+ String(etataccu[iaccu]));
    }
    if (!testfin) {
      if (iaccu == 0)
      {
      testfin = finaccu[0];
      }
      else
      {
      testfin = testfin && finaccu[iaccu];
      }
    }
    if (test)
      {
        donnees = String(iaccu) + ','+ String(etataccu[iaccu])+ ','+ String(nbcyacc[iaccu]);
        Serial.println(donnees); 
      }
    }
  // ecriture mesure sur carte SD
    
		if (finattente)	{
		for (int ibouc=0; ibouc< nbaccu; ibouc++)
			{
				if ( ibouc == 0) 
				{ donnees = String(0)+','+ String(etataccu[0])+','+String(nbcyacc[0])+','+String(tempacc[0])+','+String(Uaccu[0])+','+String(Iaccu[0]);}
				else
				{donnees = donnees + ", " + String(ibouc) + ','+ String(etataccu[ibouc])+ ','+ String(nbcyacc[ibouc])+','+String(tempacc[ibouc])+','+String(Uaccu[ibouc])+','+String(Iaccu[ibouc]);}
			}
			donnees = " données " + donnees;
			if (test) {Serial.println(donnees);} 
			ecritmesure(test,nomfich,donnees);
			finattente = false;
			tdeb = millis();
//			delay ( tempomes );
		}
		else {
			finattente = attente(tdeb,tempomes);
		}
  }
  // Fin du test lorsque tous les accus ont terminé leur cycle
  // allumage led de fin
   else
  {
	  digitalWrite(fintest, HIGH);
	  if (!finecrit) {Serial.println("fin de tous les cycles");}
	  finecrit = true;
  }
// fin boucle loop
}
  
// Fonctions appelées par le programme principal

//Mesure temperature par CTN  
double temperature(int rctn)
{
	const double BALANCE_RESISTOR   = 11820.0; // resistance du pont diviseur
	const double MAX_ADC            = 1023.0;
	const double BETA               = 4700.0; // parametre de temperature de la CTN
	const double ROOM_TEMP          = 298.00;   // temperature de reference la CTN (25°C)
	const double RESISTOR_ROOM_TEMP = 100000.0;  // valeur de la CTN à la temperature de reference

	double rThermistor;                // Holds thermistor resistance value
	double tKelvin     = 0;            // Holds calculated temperature
	double tCelsius    = 0;            // Hold temperature in celsius

	rThermistor = BALANCE_RESISTOR * ( (MAX_ADC / rctn) - 1);

	tKelvin = (BETA * ROOM_TEMP) / 
				(BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));

	tCelsius = tKelvin - 273.00;  // convert kelvin to celsius 
	return tCelsius;    // Return the temperature in Celsius
}
 
// charge rapide
void chargerapide()
{
 donnees = "début charge rapide"; //-------------------test!
 ecritmesure(test,nomfich,donnees);  //---------------test!
}

// decharge rapide
void dechargerapide()  
{
 donnees = "debut décharge rapide"; //----------------test!
 ecritmesure(test,nomfich,donnees);  // test-------------------!
}

// initialisation fichier de résultats (1 par jour)
String initficres(boolean test)
{
  String nomfich;
  if (test) {Serial.println("ouverture nouveau fichier");}
  nomfich = String(year()).substring(2);
  // nomfich = nomfich + month() + day() + hour() +".txt";
  nomfich = nomfich + month() + day() + minute() +".txt"; //-----------test!
  if (test) {Serial.println(nomfich);}
  fichdonn = SD.open(nomfich, FILE_WRITE);
  // if the file opened okay, write to it:
  if (fichdonn)   {
    if (test) {Serial.print("Test écriture fichier ");}
    fichdonn.println("Début d'enregistrement");
    // close the file:
    fichdonn.close();
    if (test) {Serial.println("done.");}
            }
  else 
        {
    // if the file didn't open, print an error:
    Serial.println("error opening fichier données");
        }
 return nomfich;
 }

// ecriture tagee par le temps de donnees
 void ecritmesure(boolean test, String nomfich, String donnees)
   {
    if (test) {Serial.println("ecriture donnees sur carte SD");}
    String text;
      fichdonn = SD.open(nomfich, FILE_WRITE);
      text = String(hour()) +','+ String(minute())+','+ String(second())+", ";
    //  fichdonn.print(text); 
      text = text + donnees;
      fichdonn.println(text);
      fichdonn.close();
    
   }
   
// attente d'une duree "duree"   apres le temps "tdeb", evite l'utilisation de delay
 boolean attente(unsigned long tdeb, unsigned long duree) {
  boolean finduree = false;
  if ((millis()-tdeb) >= duree)
    { finduree = true; }
  return finduree;
}

// Initialisation de l'heure et de la date
void processSyncMessage() {
String donneesprnt;
if(Serial.find(TIME_HEADER)) {
	  heure = Serial.parseInt();
	  minut = Serial.parseInt();
	  jour = Serial.parseInt();
	  mois = Serial.parseInt();
	  annee = Serial.parseInt();
	  setTime(heure,minut,seconde,jour,mois,annee); // Sync Arduino clock to the time received on the serial port
	}
//  verification de l'entree;
donneesprnt = String(heure)+","+String(minut)+","+String(seconde)+","+String(jour)+","+String(mois)+","+String(annee);
Serial.println(donneesprnt);
/*Serial.print(heure);
Serial.print(',');
Serial.print(minut);
Serial.print(',');
Serial.print(seconde);
Serial.print(',');
Serial.print(jour);
Serial.print(',');
Serial.print(mois);
Serial.print(',');
Serial.println(annee);
*/
}

//fonction pour sortie BCD 
void afficher(int chiffre)
{
// mise à zero initiale
digitalWrite(bit_3, LOW);
digitalWrite(bit_2, LOW);
digitalWrite(bit_1, LOW);
digitalWrite(bit_0, LOW);
// il faut avoir déclaré dans le programme principal les pins de sortie
// bit_0 à bit_3    
    //On allume les bits nécessaires
    if(chiffre >= 8)
    {
        digitalWrite(bit_3, HIGH);
        chiffre = chiffre - 8;
    }
    if(chiffre >= 4)
    {
        digitalWrite(bit_2, HIGH);
        chiffre = chiffre - 4;
    }
    if(chiffre >= 2)
    {
        digitalWrite(bit_1, HIGH);
        chiffre = chiffre - 2;
    }
    if(chiffre >= 1)
    {
        digitalWrite(bit_0, HIGH);
        chiffre = chiffre - 1;
    }
}
     

void setupPWM16() 
{
/* FROM: http://arduino.stackexchange.com/questions/12718/increase-pwm-bit-resolution */

	pinMode(11,OUTPUT);
	pinMode(12,OUTPUT);
	pinMode(13,OUTPUT);
	TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
        | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
    TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS10);                    /* prescaler 1 */
    ICR1 = 0xffff;                      /* TOP counter value 13bits (1fff) (freeing OCR1A*/
	pinMode(5,OUTPUT);
	pinMode(2,OUTPUT);
	pinMode(3,OUTPUT);
    TCCR3A = _BV(COM3A1) | _BV(COM3B1)  /* non-inverting PWM */
        | _BV(WGM31);                   /* mode 14: fast PWM, TOP=ICR1 */
    TCCR3B = _BV(WGM33) | _BV(WGM32)
        | _BV(CS30);                    /* prescaler 1 */
    ICR3 = 0xffff;                      /* TOP counter value 13bits (1fff) (freeing OCR3A*/
	pinMode(6,OUTPUT);
	pinMode(7,OUTPUT);
	pinMode(8,OUTPUT);
    TCCR4A = _BV(COM4A1) | _BV(COM4B1)  /* non-inverting PWM */
        | _BV(WGM41);                   /* mode 14: fast PWM, TOP=ICR1 */
    TCCR4B = _BV(WGM43) | _BV(WGM42)
        | _BV(CS40);                    /* prescaler 1 */
    ICR4 = 0xffff;                      /* TOP counter value 13bits (1fff) (freeing OCR4A*/
	pinMode(46,OUTPUT);
	pinMode(45,OUTPUT);
	pinMode(44,OUTPUT);
    TCCR5A = _BV(COM5A1) | _BV(COM5B1)  /* non-inverting PWM */
        | _BV(WGM51);                   /* mode 14: fast PWM, TOP=ICR1 */
    TCCR5B = _BV(WGM53) | _BV(WGM52)
        | _BV(CS50);                    /* prescaler 1 */
    ICR5 = 0xffff;                      /* TOP counter value 13bits (1fff) (freeing OCR5A*/
	
}


/* 16-bit version of analogWrite() */
void analogWrite16(uint8_t pin, uint16_t val)
{
/* FROM: http://arduino.stackexchange.com/questions/12718/increase-pwm-bit-resolution */

    switch (pin) {
        case 11: OCR1A = val; break;
        case 12: OCR1B = val; break;
        case 13: OCR1C = val; break;
        case 5: OCR3A = val; break;
        case 2: OCR3B = val; break;
        case 3: OCR3C = val; break;
        case 6: OCR4A = val; break;
        case 7: OCR4B = val; break;
        case 8: OCR4C = val; break;
        case 46: OCR5A = val; break;
        case 45: OCR5B = val; break;
        case 44: OCR5C = val; break;
    
    }
}

void MesureADC()
/*
fonction de mesure des tensions sur les 3 pins tensions
les valeurs sont lissées par plusieurs mesures successives moyennées
*/  
{
  unsigned int jitter = 0;
//  unsigned int totalJitter = 0;
  float valueProbeVgnd     = 0;
  float valueProbeVacc       = 0;
  float valueProbeVr     = 0;
  float valueProbeVctn     = 0;
  unsigned int i;
  unsigned int ADCSMOOTH = 1024;
  for(i=1;i<=ADCSMOOTH;i++)
  {
    valueProbeVgnd += analogRead(analogPinVgnd);
    valueProbeVr += analogRead(analogPinVr);    // read the input value at probe one
    valueProbeVacc += analogRead(analogPinVacc);    // read the input value at probe two
    valueProbeVctn += analogRead(analogPinVctn);
    jitter = random(1,19);  // nombre aleatoire entre 1 et 18
    delayMicroseconds(jitter);
//    totalJitter += jitter;  // devrait etre egal à ADCSMOOTH*moyenne(1,18)
  }
  
  valueProbeVgnd /= ADCSMOOTH;
  valueProbeVr /= ADCSMOOTH;
  valueProbeVacc /= ADCSMOOTH; 
  valueProbeVctn /= ADCSMOOTH;
      
  voltageProbeVgnd = valueProbeVgnd * AREF / 1023.0;
  voltageProbeVr = valueProbeVr * AREF / 1023.0 ;     //calculate voltage at probe one in milliVolts
  voltageProbeVacc = valueProbeVacc * AREF / 1023.0 ;     //calculate voltage at probe two in milliVolts
  voltageProbeVctn = valueProbeVctn * AREF / 1023.0 ; 
//  return totalJitter;
}
