// ************************************************************************
// ************************************************************************
// Asservissement position panneau solaire
// 
// Programme Principal
// 
// Auteur: Noël UTTER
// Date de la version: 06/03/2022
// 
// Principe: Asservissement en azimut et en élévation
// - Un moteur sur chaque axe avec des fins de course début et fin directement sur le moteur (non gérés dans le programme)
// - Un encodeur sur chaque axe pour mesurer la position relative en degrés par rapport à la position initiale
// - Les mesures sont comparées à un tableau de consigne variable selon le mois et l'heure
// - En élévation, si la mesure est différente de la consigne, le moteur est activé dans un sens ou dans l'autre
// - En azimut, si la mesure est inférieure de la consigne, le moteur est activé dans le sens horaire
// - Initialisation de la position quotidiennement à heure fixe
// - Surveillance du vent avec un anémomètre à impulsions et mise en position sécurisée du panneau solaire pendant une heure si la vitesse dépasse un seuil pendant une durée déterminée
//
// ************************************************************************
// ************************************************************************

#include <DS3231.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>              // Watchdog


// Entrées
#define IN_AZIMUT_A         2        // Encodeur Azimut A
#define IN_AZIMUT_B         4        // Encodeur Azimut B
#define IN_ELEVATION_A      5        // Encodeur Elévation A
#define IN_ELEVATION_B      3        // Encodeur Elévation B
#define IN_SWITCHLCD        10       // Switch éclairage LCD
#define IN_ANEMOMETRE       11       // Signal impulsions anémomètre

// Sorties
#define OUT_A_M             6        // Sortie moteur azimut moins
#define OUT_A_P             7        // Sortie moteur azimut plus
#define OUT_E_M             8        // Sortie moteur elevation moins
#define OUT_E_P             9        // Sortie moteur elevation plus
#define OUT_LED_VENT        13       // Sortie led vent au delà du seuil

// Paramétrage des valeurs physiques du système
#define DEGRES_PAR_PAS      36        // Nombre de degres par impulsion d'encodeur (*10)
#define MIN_AZIMUT          800       // Valeur de l'azimut en butée minimum (*10)
#define MAX_AZIMUT          2744      // Valeur de l'azimut la plus proche de la butée max au modulo des degres par pas (*10)
#define MIN_ELEVATION       200       // Valeur de l'elevation en butée minimum (*10)
#define MAX_ELEVATION       560       // Valeur de l'elevation la plus proche de la butee max au modulo des degres par pas (*10)

// Valeurs pour la séquence d'initialisation
#define HEURE_INIT          7        // Heure quotidienne d'initialisation
#define DELAI_INIT          120000   // Nombre de milisecondes en déplacement - lors de l'initialisation (au moins égal à la durée nécessaire pour aller d'une extrémité à l'autre)

// Valeurs pour l'anémomètre
#define SEUIL_SECURITE      200      // Nb impulsions par seconde au delà duquel le panneau doit se mettre en sécurité (20 impulsions = 1,75 m/s)
#define DUREE_DEPASSE_MIN   5        // Nombre de secondes pendant lesquels le seuil doit être dépassé pour activer la mise en sécurité
#define DELAI_SECURITE_INI  3600     // Nombre de secondes après mise en sécurité et avant rétablissement

// Autres valeurs
#define DELAI_MVTA          4000     // Nombre de milisecondes de pause à respecter entre deux déplacements en Azimut
#define DELAI_MVTE          1000     // Nombre de milisecondes de pause à respecter entre deux déplacement en Elevation
#define NB_LECTURES         20       // Nombre de lectures successives à faire pour définir l'état des entrées des encodeurs
#define NB_LECTURES_A       3        // Nombre de lectures successives à faire pour définir l'état de l'entrées anémomètre
#define SEUIL_VALIDITE      17       // Nombre de lectures identiques pour qu'on considère que la valeur est stable
#define DELAI_LECTURES      0        // Nombre de milisecondes entre chaque lecture successive
#define MOTEUR_STOP         0        // Valeur pour indiquer qu'un moteur est à l'arrêt
#define MOTEUR_PLUS         255      // Valeur pour indiquer qu'un moteur tourne dans le sens +
#define MOTEUR_MOINS        128      // Valeur pour indiquer qu'un moteur tourne dans le sens -
#define MILLIS_LCD          20000    // Nb de millisecondes où l'écran reste allumé après avoir été allumé

// Variables
bool refreshAzimut;                  // Flag pour indiquer si on doit rafraichir l'affichage de l'azimut
bool refreshElevation;               // Flag pour indiquer si on doit rafraichir l'affichage du elevation
bool etatLCD;                        // Etat de la l'éclairage du LCD

byte moteurAzimut;                   // Etat du moteur azimut
byte moteurElevation;                // Etat du moteur elevation
int valAzimut;                       // Valeur courante de l'encodeur azimut
int valElevation;                    // Valeur courante de l'encodeur elevation
int consigneAzimut;                  // Consigne Azimut
int consigneElevation;               // Consigne Elevation
unsigned long allumageLCD;           // Moment d'allumage du LCD
unsigned long lastMvtA;              // Horodatage dernier mouvement en azimut
unsigned long lastMvtE;              // Horodatage dernier mouvement en élévation
unsigned int nbImpulsionsAnemometre; // Nb Impulsions envoyées par l'anémomètre
unsigned int nbImpulsionsAnemometreAff; // Nb Impulsions envoyées par l'anémomètre pour affichage
unsigned int dureeDepassement;       // Nb de secondes depuis lesquelles le vent a dépassé le seuil
unsigned long delaiSecurite;         // Nb de secondes pendant lesquelles le panneau doit encore rester en sécurité
bool valAzimutA;                     // Etat de l'entrée encodeur azimut A
bool valElevationA;                  // Etat de l'entrée encodeur élévation A
bool posAnemometre;                  // Etat de l'entrée anémomètre

String commande;
Time t;                              // Date et heure courante
DS3231 rtc(SDA, SCL);                // Horloge DS3231 sur I2C
LiquidCrystal_I2C lcd(0x27,20,4);    // Afficheur I2C de 4 lignes sur 20 colonnes, adresse 0x27


// ------------------------------------------------------------
// Tableaux de consignes: 16 valeurs par mois
// 7h,8h,9h,10h,11h,12h,13h,14h,15h,16h,17h,18h,19h,20h,21h,22h

// Consignes en azimut
int tabAzimut[192] = {
800,800,800,1304,1412,1556,1700,1844,1988,2132,2276,2384,2492,2636,2744,2744,
800,800,1124,1232,1376,1520,1664,1844,2024,2168,2312,2420,2564,2672,2744,2744,
800,944,1052,1196,1340,1484,1664,1880,2060,2240,2384,2528,2636,2744,2744,2744,
800,872,980,1124,1268,1448,1700,1916,2168,2348,2492,2600,2744,2744,2744,2744,
800,836,944,1052,1196,1412,1664,1952,2240,2420,2564,2672,2744,2744,2744,2744,
800,800,908,1016,1160,1376,1628,1952,2240,2420,2564,2708,2744,2744,2744,2744,
800,800,908,1052,1196,1376,1628,1916,2204,2384,2528,2672,2744,2744,2744,2744,
800,836,980,1124,1268,1448,1664,1916,2132,2312,2492,2600,2708,2744,2744,2744,
800,980,1088,1196,1376,1520,1736,1916,2132,2276,2420,2564,2672,2744,2744,2744,
800,1052,1160,1304,1448,1592,1772,1916,2096,2240,2384,2492,2600,2744,2744,2744,
800,800,1232,1340,1484,1628,1772,1916,2060,2204,2312,2456,2564,2672,2744,2744,
800,800,800,1340,1448,1592,1736,1880,2024,2168,2276,2384,2528,2636,2744,2744
};

// Consignes en élévation
int tabElevation[192] = {
200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,
200,200,200,200,200,272,308,308,272,236,200,200,200,200,200,200,
200,200,200,236,308,380,416,416,380,344,236,200,200,200,200,200,
200,200,236,344,416,488,524,524,488,416,344,236,200,200,200,200,
200,200,308,416,488,560,560,560,560,488,380,308,200,200,200,200,
200,236,308,416,524,560,560,560,560,524,416,308,236,200,200,200,
200,200,272,380,488,560,560,560,560,488,416,308,200,200,200,200,
200,200,236,344,416,488,524,524,488,416,344,236,200,200,200,200,
200,200,200,272,344,380,416,416,380,308,236,200,200,200,200,200,
200,200,200,200,236,272,308,308,272,200,200,200,200,200,200,200,
200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,
200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200
};


// ------------------------------------------------------------


// --------------
// Initialisation
// --------------
void setup()
{
  // Initialisation des sorties
  digitalWrite(OUT_A_M, HIGH);
  digitalWrite(OUT_A_P, HIGH);
  digitalWrite(OUT_E_M, HIGH);
  digitalWrite(OUT_E_P, HIGH);
  digitalWrite(OUT_LED_VENT, HIGH);
  pinMode(OUT_A_M, OUTPUT);
  pinMode(OUT_A_P, OUTPUT);
  pinMode(OUT_E_M, OUTPUT);
  pinMode(OUT_E_P, OUTPUT);
  pinMode(OUT_LED_VENT, OUTPUT);
  // Initialisation entrées
  pinMode(IN_AZIMUT_A, INPUT);
  pinMode(IN_AZIMUT_B, INPUT);
  pinMode(IN_ELEVATION_A, INPUT);
  pinMode(IN_ELEVATION_B, INPUT);
  pinMode(IN_SWITCHLCD, INPUT_PULLUP);
  pinMode(IN_ANEMOMETRE, INPUT_PULLUP);
  // Initialisation valeurs
  consigneAzimut = MIN_AZIMUT;
  consigneElevation = MIN_ELEVATION;
  valAzimut = MIN_AZIMUT;
  valElevation = MIN_ELEVATION;
  refreshAzimut = true;
  refreshElevation = true;
  etatLCD = false;
  posAnemometre = digitalRead(IN_ANEMOMETRE);
  nbImpulsionsAnemometre = 0;
  dureeDepassement = 0;
  delaiSecurite = 0;
  // Initialisation RTC
  rtc.begin();
  // Initialisation comm série
  commande = "";
  Serial.begin(19200);
  // Initialisation LCD
  lcd.init();
  // Initialation de la position
  initPosition();
  // Activation du watchdog 8 secondes
  wdt_enable(WDTO_8S);
}

// ----------------
// Boucle programme
// ----------------
void loop()
{
  byte m, h, a;
  // Lecture des encodeurs
  lectureAzimut();
  lectureElevation();
  // Lecture de l'anémomètre;
  lectureAnemometre();
  // Refresh des valeurs Azimut et Elevation à l'écran
  if (refreshAzimut)
    afficheAzimut();
  if (refreshElevation)
    afficheElevation();
  if (lit_heure())            // Lecture de l'horloge (La fonction retourne vrai si la seconde a changé)
  {
    traiteAnemometre();
    if ((t.hour == HEURE_INIT) && (t.min == 0) && (t.sec == 0))
    {
      if (delaiSecurite == 0)  // Seulement si on n'est pas en mise en sécurité
      {
        // Initialisation physique tous les jours à heure fixe
        // Reset de l'arduino en utilisant le watchdog et une boucle infinie
        wdt_disable();
        wdt_enable(WDTO_15MS);
        for(;;);
      }
    }
    else
    {
      afficheHeure();        // Mise à jour de l'affichage de l'heure
      // Recherche de la consigne de position dans le tableau
      if (t.date > 6)
        m = t.mon;           // Entre le 6 et la fin du mois, on prend la valeur du mois courant (cause: table faite au 21 de chaque mois)
      else
      {                      // Avant le 7, on prend le mois précédent
        if (t.mon > 1)
          m = t.mon - 1;     // A partir de fevrier, on prend le mois précédent
        else
          m = 12;            // Si on est en janvier, on prend décembre
      }
      if (t.min < 30) 
        h = t.hour;          // Avant la demie, on prend l'heure courante
      else
      {
        if (t.hour < 23)
          h = t.hour + 1;   // A partir de la demie, on prend l'heure suivante
        else
          h = 0;            // A 23h, on prend 0h
      }
      if ((h>=7) && (h<23))
      {
        // Entre 7h et 22h: Consigne dans le tableau
        consigneAzimut = tabAzimut[(m - 1) * 16 + h-7];           // L'indice du tableau commence à 0 -> m-1 / on a 16h pour chaque mois -> *16 / la 1ère valeur est pour 7h -> h-7
        consigneElevation = tabElevation[(m - 1) * 16 + h-7];
      }
      else
      {
        // De 23h à 6h: On ne change pas la consigne
      }

      if (delaiSecurite > 0)
      {
        // On est en situation de mise en sécurité
        consigneElevation = MAX_ELEVATION;
        delaiSecurite--;
      }
    }
    // Mouvement en azimut
      if ((valAzimut < consigneAzimut) && (moteurAzimut != MOTEUR_PLUS))         // S'il faut se déplacer en A+ et si le moteur n'est pas déjà en train de tourner dans ce sens
        moteurAzimutPlus();
      else if ((valAzimut >= consigneAzimut)  && (moteurAzimut != MOTEUR_STOP))   // Si la valeur est supérieure ou égale la consigne et si le moteur n'est pas déjà à l'arrêt
        moteurAzimutStop();

    // Mouvement en elevation
      if ((valElevation < consigneElevation) && (moteurElevation != MOTEUR_PLUS))          // S'il faut se déplacer en E+ et si le moteur n'est pas déjà en train de tourner dans ce sens
        moteurElevationPlus();
      else if ((valElevation > consigneElevation) && (moteurElevation != MOTEUR_MOINS))    // S'il faut se déplacer en E- et si le moteur n'est pas déjà en train de tourner dans ce sens
        moteurElevationMoins();
      else if ((valElevation == consigneElevation) && (moteurElevation != MOTEUR_STOP))    // Si la valeur égale la consigne et si le moteur n'est pas déjà à l'arrêt
        moteurElevationStop();
  }

  // Port série
  while (Serial.available())
  {
    a = Serial.read();
    if (a == 13)
    {
      // CR -> Fin de commande
      traiteCommande();
      commande = "";
    }
    else if (a != 10) // On ne tient pas compte du LF
      commande = commande + char(a);
  }

  // Gestion de l'allumage de l'écran
  if (digitalRead(IN_SWITCHLCD) == LOW)
    allumeLCD();        // Le switch d'allumage est pressé
  else
    checkAllumageLCD();

  // Reset du watchdog
  wdt_reset();
}

// -----------------------
// Refresh affichage heure
// -----------------------
void afficheHeure()
{
  lcd.setCursor(0,0);
  if (delaiSecurite == 0)
  {
    // Affichage normal
    lcd.print(nbEnChaine(t.date, 2,0));
    lcd.print("/");
    lcd.print(nbEnChaine(t.mon, 2,0));
//    lcd.print("/");
//    lcd.print(nbEnChaine(t.year, 4,0));
    lcd.print("  ");
    lcd.print(nbEnChaine(t.hour, 2,0));
    lcd.print(":");
    lcd.print(nbEnChaine(t.min, 2,0));
    lcd.print(":");
    lcd.print(nbEnChaine(t.sec,2,0));
    lcd.print("  ");
    lcd.print(nbEnChaine(nbImpulsionsAnemometreAff,3,0));
  }
  else
  {
    // Affichage sécurité vent
    lcd.print("** SECU VENT " + nbEnChaine(delaiSecurite,4,0) + " **");
  }
}

// ------------------------
// Refresh affichage azimut
// ------------------------
void afficheAzimut()
{
  lcd.setCursor(0,1);
  lcd.print(F("VA:   "));
  lcd.setCursor(3,1);
  lcd.print(nbEnChaine(valAzimut,3,1));
  lcd.setCursor(0,2);
  lcd.print(F("CA:   "));
  lcd.setCursor(3,2);
  lcd.print(nbEnChaine(consigneAzimut,3,1));
  refreshAzimut = false;
  allumeLCD();
}

// ---------------------------
// Refresh affichage elevation
// ---------------------------
void afficheElevation()
{
  lcd.setCursor(12,1);
  lcd.print(F("VE:   "));
  lcd.setCursor(15,1);
  lcd.print(nbEnChaine(valElevation,3,1));
  lcd.setCursor(12,2);
  lcd.print(F("CE:   "));
  lcd.setCursor(15,2);
  lcd.print(nbEnChaine(consigneElevation,3,1));
  refreshElevation = false;
  allumeLCD();
}

// --------------------------------------------------
// Lecture heure et retourne vrai si l'heure a changé
// --------------------------------------------------
bool lit_heure()
{
  Time new_t = rtc.getTime();
  if (t.sec != new_t.sec)
  {
    t = new_t;
    return true;
  }
  else
    return false;
}

// -----------------------
// Lecture encodeur Azimut
// -----------------------
void lectureAzimut()
{
  byte VALHigh = 0;
  byte VALLow = 0;
  bool newVAL;
  for (byte i=0; i<NB_LECTURES; i++)                        // On lit l'entrée A plusieurs fois
  {
    if (digitalRead(IN_AZIMUT_A))
      VALHigh++;
    else
      VALLow++;
    delay(DELAI_LECTURES);
  }
  newVAL = VALHigh > VALLow;                                // On retient la valeur qui apparaît le plus souvent
  if ((newVAL && VALHigh > SEUIL_VALIDITE) || (!newVAL && VALLow > SEUIL_VALIDITE)) // Si on dépasse bien le seuil de validité, valeur stable
  {
    if (newVAL != valAzimutA)
    {
      if (digitalRead(IN_AZIMUT_B) == newVAL)                 // Si les 2 entrées sont au même niveau, on va dans le sens +
      {
        if (valAzimut + DEGRES_PAR_PAS <= MAX_AZIMUT)         // Contrôle qu'on ne dépasse pas la position max
          valAzimut = valAzimut + DEGRES_PAR_PAS;
      }
      else                                                    // Sinon on va dans le sens -
      {
        if (valAzimut - DEGRES_PAR_PAS >= MIN_AZIMUT)         // Contrôle qu'on ne dépasse pas la position min
          valAzimut = valAzimut - DEGRES_PAR_PAS;
      }
      refreshAzimut = true;                                   // Refresh de l'affichage de l'azimut
      trace("Valeur Azimut " + String(valAzimut));
      valAzimutA = newVAL;
    }
  }
  else
    trace("Parasites Azimut");
}

// --------------------------
// Lecture encodeur elevation
// --------------------------
void lectureElevation()
{
  byte VALHigh = 0;
  byte VALLow = 0;
  bool newVAL;
  for (byte i=0; i<NB_LECTURES; i++)                        // On lit l'entrée A plusieurs fois
  {
    if (digitalRead(IN_ELEVATION_A))
      VALHigh++;
    else
      VALLow++;
    delay(DELAI_LECTURES);
  }
  newVAL = VALHigh > VALLow;                                // On retient la valeur qui apparaît le plus souvent
  if ((newVAL && VALHigh > SEUIL_VALIDITE) || (!newVAL && VALLow > SEUIL_VALIDITE)) // Si on dépasse bien le seuil de validité, valeur stable
  {
    if (newVAL != valElevationA)
    {
      if (digitalRead(IN_ELEVATION_B) == newVAL)              // Si les 2 entrées sont au même niveau, on va dans le sens +
      {
        if (valElevation + DEGRES_PAR_PAS <= MAX_ELEVATION)   // Contrôle qu'on ne dépasse pas la position max
          valElevation = valElevation + DEGRES_PAR_PAS;
      }
      else                                                    // Sinon on va dans le sens -
      {
        if (valElevation - DEGRES_PAR_PAS >= MIN_ELEVATION)   // Contrôle qu'on ne dépasse pas la position min
          valElevation = valElevation - DEGRES_PAR_PAS;
      }
      refreshElevation = true;                                // Refresh de l'affichage de l'élevation
      trace("Valeur Elevation " + String(valElevation));
      valElevationA = newVAL;
    }
  }
  else
    trace("Parasites Elevation");
}

// ------------------
// Lecture anémomètre
// ------------------
void lectureAnemometre()
{
  byte VALHigh = 0;
  byte VALLow = 0;
  bool newVAL;
  for (byte i=0; i<NB_LECTURES_A; i++)                        // On lit l'entrée plusieurs fois
  {
    if (digitalRead(IN_ANEMOMETRE))
      VALHigh++;
    else
      VALLow++;
    delay(DELAI_LECTURES);
  }
  newVAL = VALHigh > VALLow;                                // On retient la valeur qui apparaît le plus souvent
  if (newVAL != posAnemometre)
  {
    nbImpulsionsAnemometre++;
    posAnemometre = newVAL;
  }
}

// -------------------------------
// Rotation du moteur Azimut Moins
// -------------------------------
void moteurAzimutMoins()
{
  if ((millis() - lastMvtA) >= DELAI_MVTA)
  {
    allumeLCD();
    trace(F("MVT A -"));
    moteurAzimut = MOTEUR_MOINS;
    lastMvtA = millis();
    digitalWrite(OUT_A_M, LOW);
    digitalWrite(OUT_A_P, HIGH);
    lcd.setCursor(0,3);
    lcd.print(F("<<<-"));
    afficheAzimut();
  }
}

// -------------------------------
// Rotation du moteur Azimut Plus
// -------------------------------
void moteurAzimutPlus()
{
  if ((millis() - lastMvtA) >= DELAI_MVTA)
  {
    allumeLCD();
    trace(F("MVT A +"));
    moteurAzimut = MOTEUR_PLUS;
    lastMvtA = millis();
    digitalWrite(OUT_A_M, HIGH);
    digitalWrite(OUT_A_P, LOW);
    lcd.setCursor(0,3);
    lcd.print(F("->>>"));
    afficheAzimut();
  }
}

// -------------------------------
// Arret rotation du moteur Azimut
// -------------------------------
void moteurAzimutStop()
{
  allumeLCD();
  trace(F("MVT A STOP"));
  moteurAzimut = MOTEUR_STOP;
  lastMvtA = millis();
  digitalWrite(OUT_A_M, HIGH);
  digitalWrite(OUT_A_P, HIGH);
  lcd.setCursor(0,3);
  lcd.print(F("STOP"));
  afficheAzimut();
}

// ----------------------------------
// Rotation du moteur Elevation Moins
// ----------------------------------
void moteurElevationMoins()
{
  if ((millis() - lastMvtE) >= DELAI_MVTE)
  {
    allumeLCD();
    trace(F("MVT E -"));
    moteurElevation = MOTEUR_MOINS;
    lastMvtE = millis();
    digitalWrite(OUT_E_M, LOW);
    digitalWrite(OUT_E_P, HIGH);
    lcd.setCursor(12,3);
    lcd.print(F("<<<-"));
    afficheElevation();
  }
}

// ---------------------------------
// Rotation du moteur Elevation Plus
// ---------------------------------
void moteurElevationPlus()
{
  if ((millis() - lastMvtE) >= DELAI_MVTE)
  {
    allumeLCD();
    trace(F("MVT E +"));
    moteurElevation = MOTEUR_PLUS;
    lastMvtE = millis();
    digitalWrite(OUT_E_M, HIGH);
    digitalWrite(OUT_E_P, LOW);
    lcd.setCursor(12,3);
    lcd.print(F("->>>"));
    afficheElevation();
  }
}

// ----------------------------------
// Arret rotation du moteur Elevation
// ----------------------------------
void moteurElevationStop()
{
  allumeLCD();
  trace(F("MVT E STOP"));
  moteurElevation = MOTEUR_STOP;
  lastMvtE = millis();
  digitalWrite(OUT_E_M, HIGH);
  digitalWrite(OUT_E_P, HIGH);
  lcd.setCursor(12,3);
  lcd.print(F("STOP"));
  afficheElevation();
}

// -----------------------------
// Initialisation de la position
// -----------------------------
void initPosition()
{
  // Désactivation du watchdog
  wdt_disable();
  allumeLCD();
  trace(F("** INITIALISATION **"));
  // Réinitialistion des consignes
  consigneAzimut = MIN_AZIMUT;
  consigneElevation = MIN_ELEVATION;
  // 1ère lecture de l'heure
  lit_heure();
  // Affichage
  lcd.setCursor(0,0);
  lcd.print(F("** INITIALISATION **"));
  // Arrêt des moteurs
  moteurAzimutStop();
  moteurElevationStop();
  // Mouvement en moins pendant au moins la durée complète pour aller d'une extrémité à l'autre
  trace("-- RETOUR " + String(DELAI_INIT) + "ms --");
  while (moteurAzimut == MOTEUR_STOP)
    moteurAzimutMoins();
  while (moteurElevation == MOTEUR_STOP)
    moteurElevationMoins();
  delay(DELAI_INIT);
  lit_heure();
  // Arrêt des moteurs
  moteurAzimutStop();
  moteurElevationStop();
  delay(4000);
  lcd.setCursor(0,0);
  lcd.print(F("                    "));
  valAzimut = MIN_AZIMUT;
  valElevation = MIN_ELEVATION;
  valAzimutA = digitalRead(IN_AZIMUT_A);
  valElevationA = digitalRead(IN_ELEVATION_A);
  refreshAzimut = true;
  refreshElevation = true;
  lit_heure();
  trace(F("** FIN INITIALISATION **"));
}

// ------------------------------
// Traitement des commandes série
// ------------------------------
void traiteCommande()
{
  allumeLCD();
  if ((commande.substring(0,1) == "h") and (commande.length() == 3) and (isDigit(commande[1])) and (isDigit(commande[2])) and (commande.substring(1,3).toInt() >= 0) and (commande.substring(1,3).toInt() < 24))
  {
    // Réglage de l'heure
    Serial.println("Heure: " + commande.substring(1,3));
    rtc.setTime(commande.substring(1,3).toInt(), t.min, 0);
    refreshAzimut = true;
    refreshElevation = true;
  }
  else if ((commande.substring(0,1) == "m") and (commande.length() == 3) and (isDigit(commande[1])) and (isDigit(commande[2])) and (commande.substring(1,3).toInt() >= 0) and (commande.substring(1,3).toInt() < 60))
  {
    // Réglage des minutes
    Serial.println("Minute: " + commande.substring(1,3));
    rtc.setTime(t.hour, commande.substring(1,3).toInt(), 0);
    refreshAzimut = true;
    refreshElevation = true;
  }
  else if ((commande.substring(0,1) == "j") and (commande.length() == 3) and (isDigit(commande[1])) and (isDigit(commande[2])) and (commande.substring(1,3).toInt() > 0) and (commande.substring(1,3).toInt() < 32))
  {
    // Réglage des jours
    Serial.println("Jour: " + commande.substring(1,3));
    rtc.setDate(commande.substring(1,3).toInt(), t.mon, t.year);
    refreshAzimut = true;
    refreshElevation = true;
  }
  else if ((commande.substring(0,1) == "o") and (commande.length() == 3) and (isDigit(commande[1])) and (isDigit(commande[2])) and (commande.substring(1,3).toInt() > 0) and (commande.substring(1,3).toInt() < 13))
  {
    // Réglage du mois
    Serial.println("Mois: " + commande.substring(1,3));
    rtc.setDate(t.date, commande.substring(1,3).toInt(), t.year);
    refreshAzimut = true;
    refreshElevation = true;
  }
  else if ((commande.substring(0,1) == "a") and (commande.length() == 5) and (isDigit(commande[1])) and (isDigit(commande[2])) and (isDigit(commande[3])) and (isDigit(commande[4])) and (commande.substring(1,5).toInt() > 0))
  {
    // Réglage de l'année
    Serial.println("Année: " + commande.substring(1,5));
    rtc.setDate(t.date, t.mon, commande.substring(1,5).toInt());
    refreshAzimut = true;
    refreshElevation = true;
  }
  else if (commande.substring(0,1) == "v")
  {
    // Affichage des valeurs courantes
    trace("");
    Serial.println("Azimut:    " + String(valAzimut) + " (consigne: " + String(consigneAzimut) + ")");
    Serial.println("Elévation: " + String(valElevation) + " (consigne: " + String(consigneElevation) + ")");
  }
  else if (commande.substring(0,1) == "i")
  {
    // Initialisation position
    initPosition();
  }
  else
  {
    // Commande inconnue
    Serial.println(F("******** Commande incorrecte ********"));
    Serial.println(F("  hHH    réglage de l'heure HH"));
    Serial.println(F("  mMM    réglage des minutes MM"));
    Serial.println(F("  jJJ    réglage du jour JJ"));
    Serial.println(F("  oOO    réglage du mois OO"));
    Serial.println(F("  aAAAA  réglage de l'année AAAA"));
    Serial.println(F("  v      affichage valeurs courantes"));
    Serial.println(F("  i      force initialisation"));
    Serial.println(F("*************************************"));
  }
}

// ----------------------------------------
// Traitement de la lecture de l'anémomètre
// ----------------------------------------
void traiteAnemometre()
{
  if (nbImpulsionsAnemometre > SEUIL_SECURITE)
  {
    // Vitesse du vent supérieure au seuil de sécurité
    digitalWrite(OUT_LED_VENT, LOW);
    dureeDepassement++;
    if ((dureeDepassement > DUREE_DEPASSE_MIN) && (delaiSecurite == 0))
    {
      // le vent a dépassé le seuil pendant la durée mini et on n'est pas déjà en sécurité -> Mise en sécurité
      delaiSecurite = DELAI_SECURITE_INI;
    }
  }
  else
  {
    // Vitesse du vent inférieure au seuil de sécurité
    digitalWrite(OUT_LED_VENT, HIGH);
    dureeDepassement = 0;
  }
  trace("Nb Imp Anémo: " + nbEnChaine(nbImpulsionsAnemometre,4,0));
  nbImpulsionsAnemometreAff = nbImpulsionsAnemometre;
  nbImpulsionsAnemometre = 0;    // Réinitialisation du nombre d'impulsions lues
}


// --------------------------------------------------------------
// Mise en forme d'un nombre entier en chaine de longueur définie
// --------------------------------------------------------------
String nbEnChaine(unsigned int nb, byte ent, byte dec)
{
  String retour = String(nb, DEC);
  while (retour.length() < ent+dec)
    retour = "0" + retour;
  if (dec > 0)
   retour = retour.substring(0, ent) + "." + retour.substring(ent, ent+dec);
  return retour;
}

// -------------------------
// Fonction de traçage série
// -------------------------
void trace(String s)
{
  Serial.println(nbEnChaine(t.date, 2,0) + "/" + nbEnChaine(t.mon, 2,0) + "/" + nbEnChaine(t.year, 4,0) + " " + nbEnChaine(t.hour, 2,0) + ":" + nbEnChaine(t.min, 2,0) + ":" + nbEnChaine(t.sec,2,0) + "  " + s);
}

// ---------------
// Allumage du LCD
// ---------------
void allumeLCD()
{
  if (!etatLCD)
  {
    lcd.backlight();
    etatLCD = true;
  }
  allumageLCD = millis();
}

// --------------------------------
// Extinction du LCD après le délai
// --------------------------------
void checkAllumageLCD()
{
  if ((etatLCD) && (millis() > allumageLCD + MILLIS_LCD))  
  {
    lcd.noBacklight();
    etatLCD = false;
  }
}
