//Librairies utilisées et broche analogique
#include <avr/interrupt.h>
#include <math.h>
#define actuateur 5  // Broche de sortie
 
//Varaibles de l'échantillonnage
volatile uint8_t currentChannel = 0;
volatile uint16_t adcValues[3];
volatile uint8_t interruptCount = 0;
bool initialised = false;
int compt = 0;
double T_baseline = 25;
/*Fonctions utilitaires :filtrage numérique,  Décalage vers la droite ; Produit scalaire dot */
//-----------------------------------------------------------------------------------------//
double output0 = 0;
double output1 = 0;
double output2 = 0;
//filtre de premier ordre avec fréquence de coupure de 0.5 Hz
double filter(double input, double& output) {
  output = 0.917915 * input + 0.082085 * output;
  return output;
}
 
 //Fonction de rotation d'un tableau vers la droite d'une case pour les équations récurrentes
 void rotate(double* array, int size) {
  if (size <= 1) return;
  for (int i = size - 1; i > 0; i--) {
    array[i] = array[i - 1];
  }
}
//Fonction de calcul de produits scalaires
double dot(const double* A, const double* B, int taille) {
  double somme = 0.000;
  for (int i = 0; i < taille; i++) {
    somme += A[i] * B[i];
  }
  return somme;
}
//Fonction qui permet qui permet de découper une chaine de caractère pour
// en ressortir les valeurs communiquuées par l'interface
void split(String input, double* doubleT, int maxValues) {
  int index = 0;
  int pos = 0;
  int lastPos = 0;
  // Trouver la position de la première virgule
  pos = input.indexOf(',');
  // Si aucune virgule n'est trouvée, convertir toute la chaîne en double
  if (pos == -1) {
    doubleT[0] = input.toFloat();  // Convertir la chaîne en double
    return;                        // Sortir de la fonction
  }
  // Si des virgules sont trouvées, diviser la chaîne
  while (pos != -1 && index < maxValues - 1) {
    // Extraire la sous-chaîne entre la dernière position et la position actuelle
    String substring = input.substring(lastPos, pos);
    // Convertir la sous-chaîne en double et la stocker dans le tableau
    doubleT[index++] = substring.toFloat();
    // Mettre à jour la dernière position
    lastPos = pos + 1;
    // Trouver la position de la prochaine virgule
    pos = input.indexOf(',', lastPos);
  }
  // Ajouter la dernière sous-chaîne après la dernière virgule (si on a encore de la place)
  if (index < maxValues) {
    String substring = input.substring(lastPos);
    doubleT[index] = substring.toFloat();
  }
}
 
/*Variables globales*/
//-----------------------------------------------------------------------------------------//
double f = 0.4;      // double de la fréquence d'échantillonage
double R25 = 10000;  // Résistance à 25 dégré Celsius
//Coefficients de SteinHart-Hart
double A = 0.00335401643468053;
double B = 0.000256523550896126;
double C = 0.00000260597012072052;
double D = 0.000000063292612648746;
//Banque d'offsets et de gains de l'électronique
double Rfixe[3] = { 9948, 9950, 9900};
double gain[3] = { 2.40735, 3.13733, 4.64232};
double offset[3] = { 1.52, 1.74, 1.98};
 
/*Régulateur*/
//-----------------------------------------------------------------------------------------//
double uop = 0;
double error[2] = {0, 0};  // Tableau des erreurs (Consigne - Valeur mesurée)
double u = 0;      // Commande (sortie du régulateur, entrée du variateur)
double y[2] = { 0, 0 };      //
 
// Coefficients du régulateur
double Kc = 0.155;  //Coefficient proportionnel par défaut
// Coefficient du PI
double coeff_u = 0.028983448207563;           // numérateur
double coeff_y[2] = { 0, 0.971016551792437 };  //Dénominateur
 
//Action dérivative et filtre (DF)
double coeff_ym[2] = { 1.67107357564755, -1.01332596656011 };  // Numérateur
double coeff_ym_a[2] = { 0, 0.342252390912559};               // Dénominateur
 
//Variables d'entrée et de sortie du Derivative Kick
double T3_m[2] = {0, 0};  // Avant, T3 mesurée / estimé
double T3[2] = { 0, 0 };    // Après , T3 servant à calculer l'erreur
/*Estimation de T3*/
//-----------------------------------------------------------------------------------------//
//Coefficients pour estimer T3 à partir de T2
double num3_2[2] = {0, 0.189};  //numérateur
double den3_2 = 0.7934 ;  // Dénominateur
//Variables d'entrée et de sortie de l'estimation de T3 à partir de T2
double T2[2] = {25, 25};
double T3_2 = 0;
/* Variables d'asservissement reçue de l'interface*/
//-----------------------------------------------------------------------------------------//
double consigne = 25.0;
bool asservissementActif = false;
double Consigne = 25.0;
double tension_commande = 0;
 
 
/*Gestion du flux de données sur le port série*/
//-----------------------------------------------------------------------------------------//
void handleSerial() {
  String input = Serial.readString();  // Lire la chaîne envoyée par MATLAB
  input.trim();                        // Enlever les espaces en début et en fin de la chaîne
  if (input.startsWith("START")) {
    startAsservissement();
  }
  if (input.startsWith("STOP")) {
    stopAsservissement();
  }
 
  if (input.startsWith("PIDF")) {
    // Enlever le préfixe "PIDF-"
    input = input.substring(5);  // La partie après "PIDF"
 
    // Extraire les valeurs séparées par des barres '|'
    Kc = input.substring(0, input.indexOf('|')).toFloat();
    input = input.substring(input.indexOf('|') + 1);
 
    String AA = input.substring(0, input.indexOf('|'));
    split(AA, coeff_ym, 2);
    input = input.substring(input.indexOf('|') + 1);
 
    String BB = input.substring(0, input.indexOf('|'));
    split(BB, coeff_ym_a, 2);
    input = input.substring(input.indexOf('|') + 1);
 
    coeff_u = input.substring(0, input.indexOf('|')).toFloat();
    input = input.substring(input.indexOf('|') + 1);
 
    String DD = input.substring(0, input.indexOf('|'));
    split(DD, coeff_y, 2);
    input = input.substring(input.indexOf('|') + 1);
    Consigne = input.toFloat();
  } else {
  }
}
 
/*Arrêt de l'asservissement et début de l'asservissement*/
//-----------------------------------------------------------------------------------------//
 
void stopAsservissement() {
  asservissementActif = false;
  OCR3A = 8000;
}
void startAsservissement() {
  asservissementActif = true;
}
 
/*Fonction pour calculer la température*/
//-----------------------------------------------------------------------------------------//
 
double calculerTemperature(double lect, double gain, double offset, double Rfixe) {
  double lecture_analogique = (lect / 1023.0) * 5.0;
  lecture_analogique = (lecture_analogique / gain) + offset;
  double Rntc = Rfixe / (5 / lecture_analogique - 1.0);
  double lnR = log(Rntc / R25);
  double T = 1.0 / (A + B * lnR + C * lnR * lnR + D * lnR * lnR * lnR); //Équation de Steinhar-Hart
  return T - 273.15;
}
 
/* Fonction qui estime T3 en fonction de T1 et T2*/
//-----------------------------------------------------------------------------------------//
double estimer_T3(double temperature_0_mes, double temperature_1_mes) {
 
  rotate(T2, 2);
  T2[0] = temperature_1_mes ;
  T3_2 = dot(T2,num3_2, 2) + T3_2*den3_2;
  return T3_2;
}
 
/*Fonction pour calculer la commande */
//-----------------------------------------------------------------------------------------//
int calculerCommande(double temperature_cible) {
  rotate(error, 2);
  error[0] = (temperature_cible - T_baseline) - T3[0];
  rotate(y, 2);
  y[0] = coeff_u * (u - uop) + dot(coeff_y, y, 2);
  // Calcul de la commande avec anti-windup
  double u_id = Kc * error[0] + y[0] + uop;
  // Application de la saturation
  u = constrain(u_id, -1, 1);
  int tension_commande = round(((u + 1) / 2) * 16000);
  return tension_commande;
}
 
/*Fonction pour initialiser les compteurs */
//-----------------------------------------------------------------------------------------//
void initTimers() {
  //Compteur pour l'échantillonnage (impossible de faire 0.2Hz)
  //On fait 0.4Hz mais on échantillonne après 2 interruptions
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);
  OCR1A = (16000000 / (1024 * f)) - 1;
  TIMSK1 |= (1 << OCIE1A);
 // COmpteur pour le PWM à 1kHz
  TCCR3A = (1 << COM3A1) | (1 << WGM31);
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS30);
  ICR3 = 16000;
  OCR3A = 8000; // Rapport cyclique initial de 50% (milieu) => (-1 0 1)
}
 
/*Routine d'interruption pour l'échantillonnage */
//-----------------------------------------------------------------------------------------//
ISR(TIMER1_COMPA_vect) {
  interruptCount++;
  if (interruptCount == 2) {
    interruptCount = 0;
    currentChannel = 0;
    ADMUX = (ADMUX & 0xF8) | currentChannel;
    ADCSRA |= (1 << ADSC);
  }
}
 
/*Routine d'interruption après la demande conversion par l'ADC*/
//-----------------------------------------------------------------------------------------//
ISR(ADC_vect) {
 
  adcValues[currentChannel] = ADC;
  currentChannel++;
 
  if (currentChannel < 3) {
    ADMUX = (ADMUX & 0xF8) | currentChannel;
    ADCSRA |= (1 << ADSC);
 
  } else {
    // En unités abitraires
    double temperature_0_mes = calculerTemperature(adcValues[0], gain[0], offset[0], Rfixe[0])- T_baseline;
    double temperature_1_mes = calculerTemperature(adcValues[1], gain[1], offset[1], Rfixe[1]) - T_baseline;
    double temperature_2_mes = calculerTemperature(adcValues[2], gain[2], offset[2], Rfixe[2]) - T_baseline;
 
    /*Initialisation des variables pour l'estimation*/
    //-----------------------------------------------------------------------------------------//
    if (initialised == false) {
      initialised = true;
      output0 = temperature_0_mes;
      output1 = temperature_0_mes;
      output2 = temperature_0_mes;
      for(int i = 0; i<2; i++) T2[i] = temperature_1_mes; // T2 pour l'estimation
      T3_2 = temperature_1_mes; //T3_2 pour l'estimation
    }
    // Filtrage des données
    temperature_0_mes = filter(temperature_0_mes, output0);
    temperature_1_mes = filter(temperature_1_mes, output1);
    temperature_2_mes = filter(temperature_2_mes, output2);
    // Derivative Kick
    //-----------------------------------------------------------------------------------------//
    rotate(T3_m, 2);
    T3_m[0] = estimer_T3(temperature_0_mes, temperature_1_mes);
    rotate(T3, 2);
    T3[0] = dot(coeff_ym, T3_m, 2) + dot(T3, coeff_ym_a, 2); //En unités arbitraires
 
    //-----------------------------------------------------------------------------------------//
    if (asservissementActif) {
      compt++;
      if(compt <=1){
      T3_2 = temperature_1_mes;
      }
     OCR3A = calculerCommande(Consigne);
      }
    //-----------------------------------------------------------------------------------------//
    //Envoi des données à l'application interface
      Serial.print("Data");
      Serial.print(temperature_0_mes + T_baseline);
      Serial.print(',');
      Serial.print(temperature_1_mes + T_baseline);
      Serial.print(',');
      Serial.print(temperature_2_mes + T_baseline);
      Serial.print(',');
      Serial.print(T3_2 + T_baseline);
      Serial.print(',');
      Serial.println(u);
    }
}
 
 
/*Setup (première fonction exécutée dès l'alimentation) et Loop ( Fonction exéctée en boucle) */
//-----------------------------------------------------------------------------------------//
void setup() {
  Serial.begin(115200);
  pinMode(actuateur, OUTPUT);
  ADMUX = (1 << REFS0) | currentChannel;
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);
  ADCSRB = 0;
  initTimers();
  sei();
}
// Gestion du flux d'entrée sur le port série Arduino
void loop() {
  if (Serial.available()) {
    handleSerial();
  }
}