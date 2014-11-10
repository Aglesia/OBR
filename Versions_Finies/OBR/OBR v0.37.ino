#include <EEPROM.h>
// Type de carte
#define _MEGA 1
#define _NANO 2
#define _UNO 3
#define _MICRO 4
// Maitre/Esclave
#define MAITRE 3
#define ESCLAVE1 4
#define ESCLAVE2 5



// MODIFIABLE !!!
#define NOM_DU_ROBOT "Demineur2560"
#define VERSION 0.37
#define TYPE_DE_CARTE _MEGA
#define MAITRE_ESCLAVE MAITRE
#define DEBUG 0 // Uniquement pour le déboguage !!! Mettre 0 pour une utilisation finale !!!



// Constantes
// Session
#define CONFIGURATION 0
#define PILOTAGE 1
// Connexions serie
#define PC 1
#define TELECOMMANDE 2
#define CARTE_MAITRE 3 // MAITRE
#define EXTENSION1 4 // ESCLAVE1
#define EXTENSION2 5 // ESCLAVE2
// Type d'E/S
#define VIDE 0
#define ENTREE 1
#define SORTIE 2
#define SERVO 3
#define MOTEUR 4
// Type de tableau RAM
#define PIN_ANALOG 1
#define PIN_ES 2
#define PIN_PWM 3
#define MIN_ANALOG 4
#define MAX_ANALOG 5
#define MIN_PWM 6
#define MAX_PWM 7
#define TYPE_ES 8

// Définition des données selon la carte
#if (TYPE_DE_CARTE==_NANO)
#define NB_PIN_IO 14
#define NB_PIN_PWM 6
#define NB_PIN_ANALOG 8
#define NB_PORT_SERIE 1
#define PWM0 3
#define PWM1 5
#define PWM2 6
#define PWM3 9
#define PWM4 10
#define PWM5 11
#define PWM6 0
#define PWM7 0
#define PWM8 0
#define PWM9 0
#define PWM10 0
#define PWM11 0
#define PWM12 0
#define PWM13 0
#define PWM14 0
#elif (TYPE_DE_CARTE==_MICRO)
#define NB_PIN_IO 20
#define NB_PIN_PWM 7
#define NB_PIN_ANALOG 12
#define NB_PORT_SERIE 1
#define PWM0 3
#define PWM1 5
#define PWM2 6
#define PWM3 9
#define PWM4 10
#define PWM5 11
#define PWM6 13
#define PWM7 0
#define PWM8 0
#define PWM9 0
#define PWM10 0
#define PWM11 0
#define PWM12 0
#define PWM13 0
#define PWM14 0
#elif (TYPE_DE_CARTE==_UNO)
#define NB_PIN_IO 14
#define NB_PIN_PWM 6
#define NB_PIN_ANALOG 6
#define NB_PORT_SERIE 1
#define PWM0 3
#define PWM1 5
#define PWM2 6
#define PWM3 9
#define PWM4 10
#define PWM5 11
#define PWM6 0
#define PWM7 0
#define PWM8 0
#define PWM9 0
#define PWM10 0
#define PWM11 0
#define PWM12 0
#define PWM13 0
#define PWM14 0
#elif (TYPE_DE_CARTE==_MEGA)
#define NB_PIN_IO 54
#define NB_PIN_PWM 15
#define NB_PIN_ANALOG 16
#define NB_PORT_SERIE 4
#define PWM0 2
#define PWM1 3
#define PWM2 4
#define PWM3 5
#define PWM4 6
#define PWM5 7
#define PWM6 8
#define PWM7 9
#define PWM8 10
#define PWM9 11
#define PWM10 12
#define PWM11 13
#define PWM12 44
#define PWM13 45
#define PWM14 46
#else
#error : La carte n'est pas encore supportee
#endif

// Variables globales
// Ports séries
byte Ordinateur = 0; // N° du port série sur lequel est connecté le PC (à partir du port 1)
byte Telecommande = 0;
byte Extension1 = 0;
byte Extension2 = 0;
byte Carte_Maitre = 0;
byte NoCarte = 0; // Différent si esclave d'une extension
unsigned long DerniereActuConnexions = 0; // Temps depuis la derniere actualisation des connexions
// Reception d'une commande
String Commande = ""; // Dernière commande reçue 
String param1 = ""; // Paramètres de la commande
String param2 = "";
String param3 = "";
boolean stringComplete = false; // Indique si une commande est arrivée
byte NbParam = 0; // Nombre de paramètres dans la commande
byte NoEnvoyeur = 0; // Adresse de l'envoyeur du dernier message
byte NoDestinataire = 0; // Adresse du destinataire du dernier message
byte NbActualisation = -1; // limite les envoie de demande de connexion
// Chien de garde sur les ports série
boolean watchdogActif = 1;
byte DernierTempsPC = 0; // Temps en 1/10 de seconde depuis la dernière réception
byte DernierTempsTelecommande = 0;
byte DernierTempsMaitre = 0;
byte DernierTempsEsclave1 = 0;
byte DernierTempsEsclave2 = 0;
unsigned long DernierTempsWatchdog = 0; // Temps en ms de la dernière actualisation du chien de garde
// Administration
byte Connecte = 0; // Indique si on est connecté en administrateur (accès à certaines commandes)
unsigned long Password = 2560; // Mot de passe pour se connecter en administrateur
byte Mode = CONFIGURATION; // définit le mode en cours (configuration ou pilotage)
// Batterie
byte PIN_BATTERIE = 0; // PIN sur lequel est connecté la batterie
byte PIN_BATTERIE_FAIBLE = 0; // PIN sur lequel est connecté la LED de batterie faible
byte PIN_BATTERIE_PLEINE = 0; // Pareil, pour la batterie pleine
int NiveauBatterieActuel = 0; // Indique le niveau actuel des batteries, en pourcentage
int NiveauBatterieMin = 0; // Le niveau correspondant à 0%
int NiveauBatterieMax = 1024; // Le niveau correspondant à 100%
// Gestion d'erreur
unsigned long DerniereErreur = 0;
int NombreErreur = 0;
// Tableaux contenant les valeurs des moteurs
byte PWM[NB_PIN_PWM]; // Valeur en pourcentage d'une sortie PWM (Servo ou moteur)
byte ANALOG[NB_PIN_ANALOG]; // Valeur en pourcentage d'une entrée analogique
boolean ES[NB_PIN_IO]; // Valeur booléenne d'une entrée/sortie (1 si active)
// Tableaux contenant les différentes données d'E/S
byte Type_ES[NB_PIN_IO]; // Les différents type d'entrée/sortie
byte Min_PWM[NB_PIN_PWM]; // Valeur Min d'un moteur/servo
byte Max_PWM[NB_PIN_PWM]; // Valeur Max
byte Min_Analog[NB_PIN_ANALOG]; // Valeur Min d'un capteur
byte Max_Analog[NB_PIN_ANALOG]; // Valeur Max d'un capteur


// Fonctions d'initialisation
// Fonction principale d'initialisation
void setup()
{
  // Initialisation du port série
  Serial.begin(57600);
  #if NB_PORT_SERIE==4
    Serial1.begin(57600);
    Serial2.begin(57600);
    Serial3.begin(57600);
  #endif

  // Initialisation des chaines de commande ans la RAM
  Commande.reserve(128);
  param1.reserve(32);
  param2.reserve(32);
  param3.reserve(32);
  
  // Initialisation de l'EEPROM
  PIN_BATTERIE = EEPROM.read(10);
  PIN_BATTERIE_FAIBLE = EEPROM.read(11);
  PIN_BATTERIE_PLEINE = EEPROM.read(12);
  NiveauBatterieMin = EEPROM.read(13)*4;
  NiveauBatterieMax = EEPROM.read(14)*4;
  
  // Lecture du mot de passe
  Password = LireMotDePasse();
  if(Password==0)
    Password=2560;
  
  // Arrêt de la LED 13
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  // Possibilité à l'utilisateur de réinitialiser le mot de passe
  pinMode(13, INPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  if(digitalRead(13))
    RAZ_MotDePasse();

  digitalWrite(12, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  // Initialisation des E/S
}

// Fonction de lecture d'une case Entrée/Sortie selon l'EEPROM
void Lire_Donnee_EEPROM_ES(byte NoES, boolean Analog)
{
  // On lit le numéro demandée, et on extrait les 7 cases de l'EEPROM

  // Selon le type d'E/S, on remplie les tableaux en RAM

}

// Fonction d'écriture d'une case Entrée/Sortie en EEPROM
void Ecrire_Donnee_EEPROM_ES(byte NoES, boolean Analog)
{
  // On lit les données des tableaux, en RAM

  // On inscrit les données dans l'EEPROM

}

// Fonctions principales
// Boucle principale
void loop()
{
  // On vérifie les connexions
  Verification_Connexion();

  // On vérifie l'arrivée d'une commande
  Verification_Commande();
  
  // On met à jour le niveau de batterie
  MajBatterie();
  
  // On met à jour tous les capteurs
  
  // On met à jour les états des sorties
  
  // On fait une pause de 5 ms pour éviter de trop consommer et de trop chauffer
  delay(5);
}

// Fonction de verrouillage du système
void BloquageSysteme()
{
  // On éteint tous les moteurs


  // On bloque l'arduino, obligeant le RESET physique
  while(1)
  {
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(500);
  }
}

// Fonction de mise à jour du niveau de batterie (en %)
void MajBatterie()
{
  // On définit le pourcentage actuel des batteries
  int NiveauBatterie = analogRead(PIN_BATTERIE);
  NiveauBatterieActuel = map(NiveauBatterie, NiveauBatterieMin, NiveauBatterieMax, 0, 100);
  
  // On indique que la batterie est faible, si elle est en dessous de 15%
  if(NiveauBatterieActuel < 15)
    digitalWrite(PIN_BATTERIE_FAIBLE, HIGH);
  else
    digitalWrite(PIN_BATTERIE_FAIBLE, LOW);
  
  // On indique que la batterie est pleine si elle est au dessus de 95%
  if(NiveauBatterieActuel > 95)
    digitalWrite(PIN_BATTERIE_PLEINE, HIGH);
  else
    digitalWrite(PIN_BATTERIE_PLEINE, LOW);
}

// Fonctions de controle des données de RAM
// Fonction de lecture du N° de PIN PWM
byte No_PIN_PWM(byte NoPWM)
{
  // Selon les #define, on retourne le numéro du PIN correspondant à la sortie PWM demandée
  if(NoPWM>NB_PIN_PWM)
    return 0;

  switch(NoPWM)
  {
    case 0:
      return PWM0;
    case 1:
      return PWM1;
    case 2:
      return PWM2;
    case 3:
      return PWM3;
    case 4:
      return PWM4;
    case 5:
      return PWM5;
    case 6:
      return PWM6;
    case 7:
      return PWM7;
    case 8:
      return PWM8;
    case 9:
      return PWM9;
    case 10:
      return PWM10;
    case 11:
      return PWM11;
    case 12:
      return PWM12;
    case 13:
      return PWM13;
    case 14:
      return PWM14;
    default:
      return 0;
  }
}

// Fonction d'écriture en RAM
void Ecriture_RAM(byte Type, byte NumeroCase, byte Valeur)
{
  // On détermine le type de tableau que nous devons lire

  // On détermine les limites (de case et de taille valeur) du tableau

  // On vérifie qu'on ne dépasse pas les limites

  // On inscrit la valeur dans le tableau

}

// Fonction de lecture en RAM
byte Lecture_RAM(byte Type, byte NumeroCase)
{
  // On détermine le type de tableau que nous devons lire

  // On retourne la valeur de la case demandée

}



// Fonctions de controle du robot




// Fonctions de gestion du mot de passe
// Fonction de reinitialisation du mot de passe
void RAZ_MotDePasse()
{
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
  // On réinititalise le mot de passe à 2560
  Password = 2560;
  // On écrit dans l'EEPROM
  EcrireMotDePasse(2560);
  
  // On bloque l'arduino
  BloquageSysteme();
}

// Fonction de lecture du mot de passe depuis l'EEPROM
unsigned long LireMotDePasse()
{
  unsigned long Pass = EEPROM.read(6);
  Pass = (Pass*256)+EEPROM.read(7);
  Pass = (Pass*256)+EEPROM.read(8);

  return Pass;
}

// Fonction d'écriture du mot de passe sur l'EEPROM
void EcrireMotDePasse(unsigned long Password)
{
  if(Connecte)
  {
    digitalWrite(13, LOW);
    
    unsigned char Pass1 = Password/65536L;
    unsigned char Pass2 = (Password-(Pass1*65536L))/256;
    unsigned char Pass3 = Password-(Pass2*256)-(Pass1*65536L);
    
    EEPROM.write(6, Pass1);
    EEPROM.write(7, Pass2);
    EEPROM.write(8, Pass3);
  
    digitalWrite(13, HIGH);
  }
}

// Fonctions de gestion des commandes
// Fonction de récupération de la commande PC ou Telecommande
void Verification_Commande()
{
  // On récupère la commande du PC
  RecupMessage(NoPortSerie(PC));
  // S'il y a un message, on vérifie l'adresse
  if(stringComplete)
      ExecuterCommande();

  // On récupère la commande de la télécommande
  RecupMessage(NoPortSerie(TELECOMMANDE));
  // S'il y a un message, on vérifie l'adresse
  if(stringComplete)
      ExecuterCommande();

  // On vérifie la présence de messages venant de l'extension 1 ou du maitre
  if(MAITRE_ESCLAVE == MAITRE)
    RecupMessage(NoPortSerie(EXTENSION1));
  else
    RecupMessage(NoPortSerie(CARTE_MAITRE));
  if(stringComplete)
      ExecuterCommande();

  // On vérifie la presence de messages venant de l'extension 2 ou de l'extension 1
  if(MAITRE_ESCLAVE == MAITRE || MAITRE_ESCLAVE == ESCLAVE1)
    RecupMessage(NoPortSerie(EXTENSION2));
  else
    RecupMessage(NoPortSerie(EXTENSION1));
  if(stringComplete)
      ExecuterCommande();
}

// Fonction de récupération du port série
void RecupMessage(byte NoPort)
{
  // On vérifie que le port série existe sur cette carte
  if(NoPort<=0 || NoPort>NB_PORT_SERIE)
   return;

  EffacerCommande();

  // On récupère les données jusqu'à ce qu'on ai le '\n' ou jusqu'à ce que le temps soit écoulé
  // S'il n'y a déjà pas de donnée au départ, on sort tout de suite. Sinon, on lance le compte à rebourd à 20ms (une commande met au max 17.78 ms à arriver)
  if(NoPort==1 && !Serial.available())
    return;
  #if NB_PORT_SERIE>1
    if(NoPort==2 && !Serial1.available())
      return;
    if(NoPort==3 && !Serial2.available())
      return;
    if(NoPort==4 && !Serial3.available())
      return;
  #endif

  // On lance le compte à rebourd
  unsigned long Temps = millis()+50;
  while(millis()<Temps)
  {
    char inChar = 255;
    // On récupère le caractère
    if(NoPort==1 && Serial.available())
      inChar = (char)Serial.read();
    #if NB_PORT_SERIE>1
      if(NoPort==2 && Serial1.available())
        inChar = (char)Serial1.read();
      if(NoPort==3 && Serial2.available())
        inChar = (char)Serial2.read();
      if(NoPort==4 && Serial3.available())
        inChar = (char)Serial3.read();
    #endif

    // S'il y a un caractère à récupérer
    if(inChar != 255 && inChar != 'ÿ' && inChar != EOF)
    {      
      // Si c'est la fin de la commande, on quitte
      if (inChar == '\n')
      {
        // Si c'est en mode de déboguage, on envoie la chaine récupérée, sur le port USB
        #if (DEBUG)
          Serial.print(NoPort);
          Serial.print(":\\ De : ");
          Serial.print(NoEnvoyeur);
          Serial.print(" Vers : ");
          Serial.print(NoDestinataire);
          Serial.print(" :: ");
          Serial.print(Commande);
          Serial.print(' ');
          Serial.print(param1);
          Serial.print(' ');
          Serial.print(param2);
          Serial.print(' ');
          Serial.println(param3);
        #endif

        // Si le watchdog est activé, on remet le compteur à 0
        if(watchdogActif)
        {
          switch(NoEnvoyeur)
          {
            case PC:
            DernierTempsPC = 0;
            break;

            case TELECOMMANDE:
            DernierTempsTelecommande = 0;
            break;

            case ESCLAVE1:
            DernierTempsEsclave1 = 0;
            break;

            case ESCLAVE2:
            DernierTempsEsclave2 = 0;
            break;

            case MAITRE:
            DernierTempsMaitre = 0;
            break;

            default:
            break;
          }
        }

        // Si c'est un simple message de présence (watchdog), on annule le message
        if(Commande==String("="))
        {
          EffacerCommande();
          return;
        }

        // Si le message est à relayer, on le transmet, et on efface la chaine
        if(NoDestinataire != MAITRE_ESCLAVE && NoPortSerie(NoEnvoyeur) != 0)
        {
          // On reconstitue la chaine
          Commande = '['+String(NoEnvoyeur)+'|'+String(NoDestinataire)+']'+String(Commande);
          if(NbParam>0)
            Commande = Commande + '-' + String(param1);
          if(NbParam>1)
            Commande = Commande + '-' + String(param2);
          if(NbParam>2)
            Commande = Commande + '-' + String(param3);

          // On l'envoie
          TransfererMessage(Commande);

          EffacerCommande();
          return;
        }

        // On indique qu'une chaine est disponible
        stringComplete = true;
        return;
      }

      // Si c'est le premier '[', on indique que le caractère suivant est le N° de l'envoyeur
      if(inChar == '[' && NoEnvoyeur == 0)
        NbParam = 10;

      else if(inChar == '|' && NoDestinataire == 0)
        NbParam = 11;

      else if(inChar == ']')
        NbParam = 0;

      // Si c'est un tiret, on change le nb de parametres
      else if(inChar == '-' && NbParam<10)
        NbParam++;

      else
      switch(NbParam)
      {
        case 0:
        Commande += inChar;
        break;
        
        case 1:
        param1 += inChar;
        break;
        
        case 2:
        param2 += inChar;
        break;
        
        case 3:
        param3 += inChar;
        break;

        case 10:
        NoEnvoyeur = String(inChar).toInt();
        break;

        case 11:
        NoDestinataire = String(inChar).toInt();
        break;
        
        default:
        inChar = 255;
      }
    }
  }
}

// Fonction d'envoie d'une chaine de caractère via le port série
void EnvoieMessage(byte Type, String Message)
{
  // On ajoute les adresses
  Message = '['+String(MAITRE_ESCLAVE)+'|'+String(Type)+']'+Message;

  // On envoie le message
  TransfererMessage(Message);
}

// Fonction d'envoie d'un message, ayant déjà une adresse
void TransfererMessage(String Message)
{
  // On détermine le N° du serial sur lequel envoyer
  byte Type = 0;
  Type = Message.substring(3, 4).toInt();
  byte Port = NoPortSerie(Type);
  if(Port>NB_PORT_SERIE || Port<=0)
    return;

  #if (DEBUG)
    Serial.print(Port);
    Serial.print(":\\ ");
    Serial.println(Message);
  #endif

  // On envoie sur le bon port série
  switch(Port)
  {
    case 1:
    Serial.println(Message);
    break;

    case 2:
    Serial1.println(Message);
    break;

    case 3:
    Serial2.println(Message);
    break;

    case 4:
    Serial3.println(Message);
    break;

    default:
    break;
  }
}

// Fonction de détermination des emplacements réseaux
void Verification_Connexion()
{
  // Si ça fait moins de deux secondes qu'on a actualisé, on retourne
  if(DerniereActuConnexions+500 > millis())
    return;

  DerniereActuConnexions = millis();

  // On vérifie que les ports ouverts sont toujours en activité (watchdog)
  WatchDog_Serie();

  // On vérifie que les ports fermés ne sont pas en attente d'ouverture
  if(Ordinateur != 1 && Telecommande != 1 && Extension1 != 1 && Extension2 != 1 && Carte_Maitre != 1)
    Demande_Connexion(1);

  if(Ordinateur != 2 && Telecommande != 2 && Extension1 != 2 && Extension2 != 2 && Carte_Maitre != 2)
    Demande_Connexion(2);

  if(Ordinateur != 3 && Telecommande != 3 && Extension1 != 3 && Extension2 != 3 && Carte_Maitre != 3)
    Demande_Connexion(3);

  if(Ordinateur != 4 && Telecommande != 4 && Extension1 != 4 && Extension2 != 4 && Carte_Maitre != 4)
    Demande_Connexion(4);

  // On vérifie les connexions sur les cartes d'extension
  if(Extension1 && !Ordinateur)
  {
    EnvoieMessage(EXTENSION1, "OBR_CONNEXION_PC");
    RecupMessage(NoPortSerie(EXTENSION1));
    if(stringComplete)
      Ordinateur = Commande.toInt();
    EffacerCommande();
  }

  if(Extension1 && !Telecommande)
  {
    EnvoieMessage(EXTENSION1, "OBR_CONNEXION_TELECOMMANDE");
    RecupMessage(NoPortSerie(EXTENSION1));
    if(stringComplete)
      Telecommande = Commande.toInt();
    EffacerCommande();
  }

  if(Extension1 && !Extension2)
  {
    EnvoieMessage(EXTENSION1, "OBR_CONNEXION_EXTENSION2");
    RecupMessage(NoPortSerie(EXTENSION1));
    if(stringComplete)
      Extension2 = Commande.toInt();
    EffacerCommande();
  }

  if(Extension2 && !Ordinateur)
  {
    EnvoieMessage(EXTENSION2, "OBR_CONNEXION_PC");
    RecupMessage(NoPortSerie(EXTENSION2));
    if(stringComplete)
      Ordinateur = Commande.toInt();
    EffacerCommande();
  }

  if(Extension2 && !Telecommande)
  {
    EnvoieMessage(EXTENSION2, "OBR_CONNEXION_TELECOMMANDE");
    RecupMessage(NoPortSerie(EXTENSION2));
    if(stringComplete)
      Telecommande = Commande.toInt();
    EffacerCommande();
  }

  if(Extension2 && !Extension1)
  {
    EnvoieMessage(EXTENSION2, "OBR_CONNEXION_EXTENSION1");
    RecupMessage(NoPortSerie(EXTENSION2));
    if(stringComplete)
      Extension1 = Commande.toInt();
    EffacerCommande();
  }

  // Si on est une carte d'extension, on envoie un message de connexion sur tous les ports libres
  if(MAITRE_ESCLAVE == ESCLAVE1 || MAITRE_ESCLAVE == ESCLAVE2)
  {
    NbActualisation++;
    if(NbActualisation < 4)
      return;

    NbActualisation = 0;
    if(Ordinateur != 1 && Telecommande != 1 && Extension1 != 1 && Extension2 != 1 && Carte_Maitre != 1)
      Envoie_Connexion(1);

    if(Ordinateur != 2 && Telecommande != 2 && Extension1 != 2 && Extension2 != 2 && Carte_Maitre != 2)
      Envoie_Connexion(2);

    if(Ordinateur != 3 && Telecommande != 3 && Extension1 != 3 && Extension2 != 3 && Carte_Maitre != 3)
      Envoie_Connexion(3);

    if(Ordinateur != 4 && Telecommande != 4 && Extension1 != 4 && Extension2 != 4 && Carte_Maitre != 4)
      Envoie_Connexion(4);
  }
}

// Chien de garde pour la déconnexion automatique des ports séries
void WatchDog_Serie()
{
  // Si ça fait plus de 2 secondes qu'on a pas envoyé un signal au chien de garde des autres cartes, on l'envoie
  switch(MAITRE_ESCLAVE) // On utilise la variable inutilisée pour stocker le temps
  {
    case MAITRE:
    {
      if(DernierTempsMaitre < 20)
        break;
      DernierTempsMaitre = 0;
      EnvoieMessage(EXTENSION1, "=");
      EnvoieMessage(EXTENSION2, "=");
    }
    break;

    case ESCLAVE1:
    {
      if(DernierTempsEsclave1 < 20)
        break;
      DernierTempsEsclave1 = 0;
      EnvoieMessage(MAITRE, "=");
      EnvoieMessage(EXTENSION2, "=");
    }
    break;

    case ESCLAVE2:
    {
      if(DernierTempsEsclave2 < 20)
        break;
      DernierTempsEsclave2 = 0;
      EnvoieMessage(EXTENSION1, "=");
      EnvoieMessage(MAITRE, "=");
    }
    break;

    default:
    break;
  }

  // On met à jour la durée de tous les ports
  byte DifferenceTempsActu = (millis()-DernierTempsWatchdog)/100;
  DernierTempsWatchdog = millis();

  DernierTempsPC += DifferenceTempsActu;
  DernierTempsTelecommande += DifferenceTempsActu;
  DernierTempsEsclave1 += DifferenceTempsActu;
  DernierTempsEsclave2 += DifferenceTempsActu;
  DernierTempsMaitre += DifferenceTempsActu;

  // On vérifie que le chien de garde est activé
  if(!watchdogActif)
    return;

  // On vérifie que ça fait moins de 5 secondes pour tout le monde
  if(DernierTempsPC > 50)
  {
    // On déconnecte l'admin, et le PC
    Connecte = 0;
    digitalWrite(13, LOW);
    Ordinateur = 0;
  }

  if(DernierTempsTelecommande > 50)
    Telecommande = 0;

  if(DernierTempsMaitre > 50)
    Carte_Maitre = 0;

  if(DernierTempsEsclave1 > 50)
    Extension1 = 0;

  if(DernierTempsEsclave2 > 50)
    Extension2 = 0;
}

// On envoie une demande de connexion
void Envoie_Connexion(byte NoPort)
{
  // On récupère la réponse de la demande précédente
  RecupMessage(NoPort);

   // On vérifie que c'est bien l'accusé de réception
  if(Commande==String("OK"))
  {
    // On détermine à qui on est connecté
    switch(NoEnvoyeur)
    {
      case PC:
      if(!Ordinateur)
       Ordinateur = NoPort;
      break;

      case TELECOMMANDE:
      if(!Telecommande)
        Telecommande = NoPort;
      break;

      case MAITRE:
      if(!Carte_Maitre)
        Carte_Maitre = NoPort;
      break;

      case EXTENSION1:
      if(!Extension1)
        Extension1 = NoPort;
      break;

      case EXTENSION2:
      if(!Extension2)
        Extension2 = NoPort;
      break;

      default:
      break;
    }
  }

  // Sinon, cest peut-être une demande de connexion
  else if(Commande==String("OBR_CONNECTER_PC") && !Ordinateur)
  {
    Ordinateur = NoPort;
    EnvoieMessage(PC, "OK");
    DernierTempsPC = 0;
  }

  else if(Commande==String("OBR_CONNECTER_TELECOMMANDE") && !Telecommande)
  {
    Telecommande = NoPort;
    EnvoieMessage(TELECOMMANDE, "OK");
    DernierTempsTelecommande = 0;
  }

  else if(Commande==String("OBR_CONNECTER_EXTENSION1") && Extension1)
  {
    Extension1 = NoPort;
    EnvoieMessage(EXTENSION1, "OK");
    DernierTempsEsclave1 = 0;
  }

  else if(Commande==String("OBR_CONNECTER_EXTENSION2") && Extension2)
  {
    Extension2 = NoPort;
    EnvoieMessage(EXTENSION2, "OK");
    DernierTempsEsclave2 = 0;
  }

  // Sinon, on envoie une nouvelle demande
  else 
  {
    if(NoPort==1)
      if(MAITRE_ESCLAVE==ESCLAVE1)
        Serial.println("OBR_CONNECTER_EXTENSION1");
      else
        Serial.println("OBR_CONNECTER_EXTENSION2");

    #if (NB_PORT_SERIE>1)
      if(NoPort==2)
        if(MAITRE_ESCLAVE==ESCLAVE1)
          Serial1.println("OBR_CONNECTER_EXTENSION1");
        else
          Serial1.println("OBR_CONNECTER_EXTENSION2");

      if(NoPort==3)
        if(MAITRE_ESCLAVE==ESCLAVE1)
          Serial2.println("OBR_CONNECTER_EXTENSION1");
        else
          Serial2.println("OBR_CONNECTER_EXTENSION2");

      if(NoPort==4)
        if(MAITRE_ESCLAVE==ESCLAVE1)
          Serial3.println("OBR_CONNECTER_EXTENSION1");
        else
          Serial3.println("OBR_CONNECTER_EXTENSION2");
    #endif
  }
}

// Fonction de vérification d'une nouvelle connexion au port série
void Demande_Connexion(byte NoPort)
{
  // On regarde si un message est en attente
  RecupMessage(NoPort);

  // On regarde qui c'est, et on lui confirme la connexion si possible
  if(stringComplete)
  {
    // On regarde qui c'est, on remplie la variable, et on envoie une confirmation
    if(Commande==String("OBR_CONNECTER_PC") && !Ordinateur)
    {
      Ordinateur = NoPort;
      EnvoieMessage(PC, "OK");
      DernierTempsPC = 0;
    }

    else if(Commande==String("OBR_CONNECTER_TELECOMMANDE") && !Telecommande)
    {
      Telecommande = NoPort;
      EnvoieMessage(TELECOMMANDE, "OK");
      DernierTempsTelecommande = 0;
    }

    else if(Commande==String("OBR_CONNECTER_EXTENSION1") && Extension1)
    {
      Extension1 = NoPort;
      EnvoieMessage(EXTENSION1, "OK");
      DernierTempsEsclave1 = 0;
    }

    else if(Commande==String("OBR_CONNECTER_EXTENSION2") && Extension2)
    {
      Extension2 = NoPort;
      EnvoieMessage(EXTENSION2, "OK");
      DernierTempsEsclave2 = 0;
    }
  }
  EffacerCommande();
}

// Fonction d'indication du numéro du port sur lequel est connecté tel truc (N° de port à partir de 1)
byte NoPortSerie(byte Objet)
{
  // On regarde où l'objet demandé est connecté, on retourne 0 sinon
  switch(Objet)
  {
    case PC:
      return Ordinateur;

    case TELECOMMANDE:
      return Telecommande;

    case MAITRE:
      return Carte_Maitre;

    case EXTENSION1:
      return Extension1;

    case EXTENSION2:
      return Extension2;

    default:
      return 0;
  }
}

// Fonction d'execution d'une commande de configuration
void ExecuterCommande() /// A RECODER : TOUTES LES COMMANDES DOIVENT PRENDRE EN COMPTE L'ENVOYEUR, ET L'ADMINISTRATEUR !!! A REFAIRE !!!
{
  // On demande si l'OBR est présent
  if(Commande==String("OBR_PRESENT"))
  EnvoieMessage(PC, "VRAIE");
  
  // On demande si l'OBR est prêt
  if(Commande==String("OBR_PRET"))
    if(Connecte)
      EnvoieMessage(PC, "ADMIN");
     else
      EnvoieMessage(PC, "VRAIE");
  
  // On demande la vesrion de l'OBR
  if(Commande==String("OBR_VERSION"))
    EnvoieMessage(PC, String(VERSION));
  
  // On demande le nom du robot
  if(Commande==String("OBR_NOM") && Mode==CONFIGURATION)
    EnvoieMessage(PC, NOM_DU_ROBOT);

  // On demande sur quel port est connecté le PC
  if(Commande==String("OBR_CONNEXION_PC") && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    EnvoieMessage(NoEnvoyeur, String(Ordinateur));

  // On demande sur quel port est connecté
  if(Commande==String("OBR_CONNEXION_TELECOMMANDE") && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    EnvoieMessage(NoEnvoyeur, String(Telecommande));

  // On demande sur quel port est connecté
  if(Commande==String("OBR_CONNEXION_EXTENSION1") && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    EnvoieMessage(NoEnvoyeur, String(Extension1));

  // On demande sur quel port est connecté
  if(Commande==String("OBR_CONNEXION_EXTENSION2") && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    EnvoieMessage(NoEnvoyeur, String(Extension2));

  // On demande à se connecter en administrateur
  if(Commande==String("OBR_CONNECTER_ADMIN") && Mode==CONFIGURATION)
  {
    if(param1==String(Password))
    {
      Connecte = 1;
      digitalWrite(13, HIGH);
      EnvoieMessage(PC, "OK");
    }
    else
      EnvoieMessage(PC, "MAUVAIS_MOT_DE_PASSE");
  }
  
  // On demande à lire un octet de l'EEPROM
  if(Commande==String("OBR_LIRE_EEPROM") && Mode==CONFIGURATION)
  {
    if(param1.toInt()>15 && param1.toInt()<4096 && Connecte)
    {
      digitalWrite(13, LOW);
      EnvoieMessage(PC, String(EEPROM.read(param1.toInt())));
      delay(50);
      digitalWrite(13, HIGH);
  }
    else
      EnvoieMessage(PC, "NON_AUTORISE");
  }
  
  // On demande à écrire un octet dans l'EEPROM
  if(Commande==String("OBR_ECRIRE_EEPROM") && Mode==CONFIGURATION)
  {
    if(param1.toInt()>50 && param1.toInt()<4096 && Connecte)
    {
      digitalWrite(13, LOW);
      EEPROM.write(param1.toInt(), param2.toInt());
      EnvoieMessage(PC, String(EEPROM.read(param1.toInt())));
      delay(50);
      digitalWrite(13, HIGH);
    }
    else
      EnvoieMessage(PC, "NON_AUTORISE");
  }
  
  // On demande à se retirer les droits administrateurs
  if(Commande==String("OBR_DECONNECTER_ADMIN") && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      Connecte=0;
      EnvoieMessage(PC, "ADMINISTRATEUR_DECONNECTE");
    }
    else
      EnvoieMessage(PC, "ADMINISTRATEUR_NON_CONNECTE");
    digitalWrite(13, LOW);
  }
  
  // On demande à changer de mot de passe administrateur
  if(Commande==String("OBR_CHANGER_MOT_DE_PASSE") && Mode==CONFIGURATION)
  {
    if(Connecte && param1.toInt()<16777216 && param1.toInt()>0)
    {        
      // On écrit dans l'EEPROM
      EcrireMotDePasse(param1.toInt());
      Password = LireMotDePasse();
      
      EnvoieMessage(PC, String(Password));
    }
    else
      EnvoieMessage(PC, "NON_AUTORISE");
  }
  
  // On demande la dernière erreur qu'il y a eu
  if(Commande==String("OBR_DERNIERE_ERREUR") && Mode==CONFIGURATION)
  {
    if(!DerniereErreur)
      EnvoieMessage(PC, "AUCUNE_ERREUR");
      
    else
    {
      EnvoieMessage(PC, String(DerniereErreur));
      DerniereErreur = 0;
    }
  }

  // On demande à passer en mode de pilotage
  if(Commande==String("OBR_PILOTER"))
  {
    Mode = PILOTAGE;
    EnvoieMessage(PC, "PILOTAGE");
  }

  // On demande à passer en mode de configuration
  if(Commande==String("OBR_CONFIGURER"))
  {
    Mode = CONFIGURATION;
    EnvoieMessage(PC, "CONFIGURATION");
  }

  if(Commande==String("OBR_DECONNEXION"))
  {
    switch(NoEnvoyeur)
    {
      case PC:
      Ordinateur = 0;
      Connecte = 0;
      digitalWrite(13, LOW);
      break;

      case TELECOMMANDE:
      Telecommande = 0;
      break;

      case MAITRE:
      Carte_Maitre = 0;
      break;

      case EXTENSION1:
      Extension1 = 0;
      break;

      case EXTENSION2:
      Extension2 = 0;
      break;
    }
  }

  if(Commande==String("OBR_DECONNEXION_AUTO") && (NoEnvoyeur == PC || NoEnvoyeur == TELECOMMANDE))
  {
    watchdogActif = param1.toInt();
    if(watchdogActif)
      EnvoieMessage(NoEnvoyeur, "WATCHDOG_ACTIF");
    else
      EnvoieMessage(NoEnvoyeur, "WATCHDOG_INACTIF");
  }

  if(Commande==String("OBR_DECONNECTER"))
  {
    switch(NoEnvoyeur)
    {
      case PC:
      Connecte = 0;
      digitalWrite(13, LOW);
      Ordinateur = 0;
      break;

      case TELECOMMANDE:
      Telecommande = 0;
      break;

      case EXTENSION1:
      Extension1 = 0;
      break;

      case EXTENSION2:
      Extension2 = 0;
      break;

      case MAITRE:
      Carte_Maitre = 0;
      break;

      default:
      break;
    }
  }

  // On connecte la batterie
  if(Commande==String("OBR_CONNECTER_BATTERIE") && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      PIN_BATTERIE = param1.toInt();
      NiveauBatterieMin = param2.toInt()/4;
      NiveauBatterieMax = param3.toInt()/4;
      
      if(NiveauBatterieMin>255)
      {
        NiveauBatterieMin = 255;
        DerniereErreur = 110032;
      }
      
      if(NiveauBatterieMax>255)
      {
        NiveauBatterieMax = 255;
        DerniereErreur = 110033;
      }
      
      if(NiveauBatterieMin>NiveauBatterieMax)
      {
        int Temp = NiveauBatterieMin;
        NiveauBatterieMin = NiveauBatterieMax;
        NiveauBatterieMax = Temp;
      }
      
      EEPROM.write(10, PIN_BATTERIE);
      EEPROM.write(13, NiveauBatterieMin);
      EEPROM.write(14, NiveauBatterieMax);
      
      NiveauBatterieMin=NiveauBatterieMin*4;
      NiveauBatterieMax=NiveauBatterieMax*4;
      
      EnvoieMessage(PC, "OK");
    }
    else
      EnvoieMessage(PC, "NON_AUTORISE");
  }
  
  // On connecte les LED d'indicateur de batterie
  if(Commande==String("OBR_CONNECTER_LED_BATTERIE") && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt() == param2.toInt())
      {
        DerniereErreur = 110022;
        EnvoieMessage(PC, "ERREUR");
      }
      else
      {
        PIN_BATTERIE_FAIBLE = param1.toInt();
        PIN_BATTERIE_PLEINE = param2.toInt();
        EEPROM.write(11, PIN_BATTERIE_FAIBLE);
        EEPROM.write(12, PIN_BATTERIE_PLEINE);
        EnvoieMessage(PC, "OK");
      }
    }
    else
      EnvoieMessage(PC, "NON_AUTORISE");
  }

  // On demande le niveau de la batterie
  if(Commande==String("OBR_NIVEAU_BATTERIE"))
  {
    EnvoieMessage(PC, String(NiveauBatterieActuel));
  }
  
  // On récupère la valeur brut du capteur
  if(Commande==String("OBR_VALEUR_BRUT_CAPTEUR") && Mode==CONFIGURATION)
  {
    EnvoieMessage(PC, String(analogRead(param1.toInt())));
  }
  
  // On récupère la valeur brut du moteur
  if(Commande==String("OBR_VALEUR_BRUT_MOTEUR") && Mode==CONFIGURATION)
  {
    
  }
  
  // On récupère la valeur brut du servomoteur
  if(Commande==String("OBR_VALEUR_BRUT_SERVO") && Mode==CONFIGURATION)
  {
    
  }

  // 
  if(Commande==String("OBR_CONNECTER_MOTEUR") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DECONNECTER_MOTEUR") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_CONNECTER_CAPTEUR") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DECONNECTER_CAPTEUR") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_CONNECTER_SERVO") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DECONNECTER_SERVO") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_CONNECTER_ENTREE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DECONNECTER_ENTREE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_CONNECTER_SORTIE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DECONNECTER_SORTIE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_LIER_MOTEUR_CAPTEUR") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_LIER_SORTIE_CAPTEUR") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_LIER_SORTIE_ENTREE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_LIER_MOTEUR_ENTREE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_LIER_MOTEUR_SORTIE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DELIER_MOTEUR_CAPTEUR") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DELIER_SORTIE_CAPTEUR") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DELIER_SORTIE_ENTREE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DELIER_MOTEUR_ENTREE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_DELIER_MOTEUR_SORTIE") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_CONFIG_MOTEUR_MIN_MAX") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_CONFIG_CAPTEUR_MIN_MAX") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_CONFIG_SERVO_MIN_MAX") && Mode==CONFIGURATION)
  {


  }

  // 
  if(Commande==String("OBR_VITESSE_MOTEUR") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_ARRETER_MOTEUR") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_BLOQUER_MOTEUR") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_DEBLOQUER_MOTEUR") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_BLOQUER_SORTIE") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_DEBLOQUER_SORTIE") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_ALLUMER_SORTIE") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_ARRETER_SORTIE") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_STATUS_ENTREE") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_STATUS_SORTIE") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_STATUS_MOTEUR") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_STATUS_CAPTEUR") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_STATUS_SERVO") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_MOTEUR_BLOQUE") && Mode==PILOTAGE)
  {


  }

  // 
  if(Commande==String("OBR_SORTIE_BLOQUEE") && Mode==PILOTAGE)
  {


  }

  // Effacement de la chaine
  EffacerCommande();
}

void EffacerCommande()
{
  stringComplete = false;
  NbParam = 0;
  Commande = "";
  param1 = "";
  param2 = "";
  param3 = "";
  NoEnvoyeur = 0;
  NoDestinataire = 0;
}