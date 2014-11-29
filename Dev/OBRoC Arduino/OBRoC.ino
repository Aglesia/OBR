/**

Copyright (C) 2014 Fragni Dorian

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

//////////////////////////////////////////// A FAIRE
// - Changement des noms de toutes les fonctions du noyau et sécurisation de leur utilisation
//////////////////////////////////////////// A FAIRE

#include <EEPROM.h>
// Type de carte
#define _MEGA 1
#define _NANO 2
#define _UNO 3
#define _MICRO 4



// MODIFIABLE !!!
#define NOM_DU_ROBOT "Demineur2560" // Nom du robot
#define TYPE_DE_CARTE _MEGA // Définit le type de carte, et donc le nombre d'E/S, d'EEPROM...
#define DEBUG 0 // Uniquement pour le déboguage !!! Mettre 0 pour une utilisation finale !!!
#define ACTIVER_WATCHDOG false // Activer le watchdog sur le port série (déconnecte automatiquement au bout de 5 secondes d'inactivité)


// Constantes
// Version du micrologiciel
#define VERSION 0.5
// Connexions serie
#define ROBOT 1
#define TELECOMMANDE_SECONDAIRE 2
#define PC 3
// Type d'E/S
#define VIDE 0
#define ENTREE 1
#define SORTIE 2
// Type de tableau RAM
#define MIN_ANALOG 1
#define MAX_ANALOG 2
#define TYPE_ES 3
#define VALEUR_ACTU_ANALOG 4
#define VALEUR_ACTU_ES 5

// Définition des données selon la carte
#if (TYPE_DE_CARTE==_NANO)
#define NB_PIN_IO 14
#define NB_PIN_ANALOG 8
#define NB_PORT_SERIE 1
#elif (TYPE_DE_CARTE==_MICRO)
#define NB_PIN_IO 20
#define NB_PIN_ANALOG 12
#define NB_PORT_SERIE 1
#elif (TYPE_DE_CARTE==_UNO)
#define NB_PIN_IO 14
#define NB_PIN_ANALOG 6
#define NB_PORT_SERIE 1
#elif (TYPE_DE_CARTE==_MEGA)
#define NB_PIN_IO 54
#define NB_PIN_ANALOG 16
#define NB_PORT_SERIE 4
#else
#error : La carte n'est pas encore supportee
#endif

// Variables globales
// Ports séries
byte Robot = 0; // N° du port série sur lequel est connecté la télécommande (à partir du port 1)
byte Telecommande_Secondaire = 0;
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
byte DernierTempsRobot = 0; // Temps en 1/10 de seconde depuis la dernière réception
byte DernierTempsTelecommande_Secondaire = 0;
unsigned long DernierTempsWatchdog = 0; // Temps en ms de la dernière actualisation du chien de garde
// Administration
byte Connecte = 0; // Indique si on est connecté en administrateur (accès à certaines commandes)
unsigned long Password = 2560; // Mot de passe pour se connecter en administrateur
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
// Tableaux contenant les différentes données d'E/S
byte Type_ES[NB_PIN_IO]; // Les différents type d'entrée/sortie
byte Min_Analog[NB_PIN_ANALOG]; // Valeur Min d'un capteur
byte Max_Analog[NB_PIN_ANALOG]; // Valeur Max d'un capteur
byte Valeur_Actu_Analog[NB_PIN_ANALOG]; // Valeur actuelle du PIN analog
byte Valeur_Actu_IO[NB_PIN_IO]; // Valeur actuelle du PIN I/O



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// MODIFIABLE PAR L'UTILISATEUR AVANT COMPILATION //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Fonction d'initialisation
void user_setup()
{

}

// Boucle principale
void user_loop()
{

}

// Fonctions personelles










////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// NON MODIFIABLE PAR L'UTILISATEUR (FONCTIONS SYSTEMES) ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Fonction d'envoie des commandes au robot
byte Envoie_Commande_Robot(byte NoCommande, byte Param1, byte Param2, byte Param3)
{
	// Selon le numéro de commande demandé, on vérifie les paramètres
	switch(NoCommande)
	{
		case VITESSE_MOTEUR:
		Commande="OBR_VITESSE_MOTEUR";
		break;

		case ARRETER_MOTEUR;
		Commande="OBR_ARRETER_MOTEUR";
		break;

		case POSITION_SERVO:
		Commande="OBR_POSITION_SERVO";
		break;

		case ALLUMER_SORTIE:
		Commande="OBR_ALLUMER_SORTIE";
		break;

		case ARRETER_SORTIE:
		Commande="OBR_ARRETER_SORTIE";
		break;

		case OBR_VALEUR_BRUT_CAPTEUR:
		Commande="OBR_VALEUR_BRUT_CAPTEUR";
		break;

		case OBR_VALEUR_ES:
		Commande="OBR_VALEUR_BRUT_ES";
		break;

		default:
		return;
	}

	// On ajoute les paramètres
	Commande = "[2|3]OBR_"+Commande+'-'+Param1+'-'+Param2+'-'+Param3+'\n';

	// On envoie la commande, et attendons la réponse
	EnvoieMessage(ROBOT, Commande);
	EffacerCommande();
	delay(5);
	RecupMessage(ROBOT);

	// Si la réponse est un nombre entre 0 et 255, on le retourne
	if(Commande.toInt()<255 && Commande.toInt()>0)
		return Commande.toInt();
	// Sinon, on retourne 0
	else
		return 0;
}

// Fonction d'actualisation des E/S en fonction de l'état du robot
void MAJ_ES_ROBOT()
{
	// Pour chaque sortie, on regarde s'il est lié à un PIN du robot
	for(i=0; i<NB_PIN_IO; i++)
	{
		if(Lecture_RAM(TYPE_ES, i)==SORTIE)
		{
			// S'il est lié à un PIN du ROBOT, on actualise la valeur

			// On met à jour sa valeur_actu

		}
	}

	// Pour chaque entrée, on regarde s'il est lié à un PIN du robot
	for(i=0; i<NB_PIN_IO; i++)
	{
		if(Lecture_RAM(TYPE_ES, i)==ENTREE)
		{
			// S'il est lié à un PIN du ROBOT, on actualise la valeur

			// On envoie la commande au Robot si besoin

		}
	}

	// Pour chaque entrée Analogique, on regarde s'il est lié à un PIN du robot
	for(i=0; i<NB_PIN_ANALOG; i++)
	{
		// S'il est lié à un PIN du ROBOT, on actualise la valeur

		// On envoie la commande en conséquence

	}

}

// Fonction d'execution d'une commande
void ExecuterCommande()
{
  // On demande si l'OBR est présent
  if(Commande=="OBR_PRESENT")
  EnvoieMessage(NoEnvoyeur, "VRAIE");
  
  // On demande si l'OBR est prêt
  if(Commande=="OBR_PRET")
    if(Connecte)
      EnvoieMessage(NoEnvoyeur, "ADMIN");
     else
      EnvoieMessage(NoEnvoyeur, "VRAIE");
  
  // On demande la vesrion de l'OBR
  if(Commande=="OBR_VERSION")
    EnvoieMessage(NoEnvoyeur, String(VERSION));
  
  // On demande le nom du robot
  if(Commande=="OBR_NOM" && Mode==CONFIGURATION)
    EnvoieMessage(NoEnvoyeur, NOM_DU_ROBOT);

  // On demande sur quel port est connecté le PC
  if(Commande=="OBR_CONNEXION_PC" && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    EnvoieMessage(NoEnvoyeur, String(Ordinateur));

  if(Commande=="OBR_CONNEXION_ROBOT")
  	EnvoieMessage(NoEnvoyeur, String(Robot));

  // On demande à se connecter en administrateur
  if(Commande=="OBR_CONNECTER_ADMIN" && Mode==CONFIGURATION)
  {
	if(param1==String(Password))
	{
	  Connecte = 1;
	  EnvoieMessage(NoEnvoyeur, "OK");
	}
	else
	  Erreur(130030);
  }

  // On demande à se retirer les droits administrateurs
  if(Commande=="OBR_DECONNECTER_ADMIN" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      Connecte=0;
      EnvoieMessage(NoEnvoyeur, "ADMINISTRATEUR_DECONNECTE");
    }
    else
      Erreur(130010);
  }

  // On demande à faire un soft-reboot
  if(Commande=="OBR_REBOOT" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      Connecte=0;
      EnvoieMessage(NoEnvoyeur, "REDEMARRAGE...");
      boot();
    }
    else
      Erreur(130010);
  }
  
  // On demande à lire un octet de l'EEPROM
  if(Commande=="OBR_LIRE_EEPROM" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt()>15 && param1.toInt()<4096)
        EnvoieMessage(NoEnvoyeur, String(EEPROM.read(param1.toInt())));
      else
        Erreur(130020);
    }
    else
      Erreur(130010);
  }
  
  // On demande à écrire un octet dans l'EEPROM
  if(Commande=="OBR_ECRIRE_EEPROM" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt()>50 && param1.toInt()<4096)
      {
        EEPROM.write(param1.toInt(), param2.toInt());
        EnvoieMessage(NoEnvoyeur, String(EEPROM.read(param1.toInt())));
      }
      else
        Erreur(130020);
    }
    else
      Erreur(130010);
  }
  
  // On demande à changer de mot de passe administrateur
  if(Commande=="OBR_CHANGER_MOT_DE_PASSE" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt()<16777216)
      {
        // On écrit dans l'EEPROM
        EcrireMotDePasse(param1.toInt());
        Password = LireMotDePasse();
        
        EnvoieMessage(NoEnvoyeur, String(Password));
      }
      else
        Erreur(110031);
    }
    else
      Erreur(130010);
  }
  
  // On demande la dernière erreur qu'il y a eu
  if(Commande=="OBR_DERNIERE_ERREUR" && Mode==CONFIGURATION)
  {
    if(!DerniereErreur)
      EnvoieMessage(NoEnvoyeur, "AUCUNE_ERREUR");
      
    else
    {
      EnvoieMessage(NoEnvoyeur, String(DerniereErreur));
      DerniereErreur = 0;
    }
  }

  if(Commande=="OBR_DECONNEXION" || Commande=="OBR_DECONNECTER")
  {
    switch(NoEnvoyeur)
    {
      case PC:
      Ordinateur = 0;
      Connecte = 0;
      break;

      case TELECOMMANDE_SECONDAIRE:
      Telecommande_Secondaire = 0;
      break;

      case ROBOT:
      Robot = 0;
      break;
    }
  }

  if(Commande=="OBR_DECONNEXION_AUTO" && (NoEnvoyeur == PC || NoEnvoyeur == TELECOMMANDE))
  {
    watchdogActif = param1.toInt();
    if(watchdogActif)
      EnvoieMessage(NoEnvoyeur, "WATCHDOG_ACTIF");
    else
      EnvoieMessage(NoEnvoyeur, "WATCHDOG_INACTIF");
  }

  // On connecte la batterie
  if(Commande=="OBR_CONNECTER_BATTERIE" && Mode==CONFIGURATION)
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
      
      EnvoieMessage(NoEnvoyeur, "OK");
    }
    else
      Erreur(130010);
  }
  
  // On connecte les LED d'indicateur de batterie
  if(Commande=="OBR_CONNECTER_LED_BATTERIE" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt() == param2.toInt())
      {
        Erreur(110022);
      }
      else
      {
        PIN_BATTERIE_FAIBLE = param1.toInt();
        PIN_BATTERIE_PLEINE = param2.toInt();
        EEPROM.write(11, PIN_BATTERIE_FAIBLE);
        EEPROM.write(12, PIN_BATTERIE_PLEINE);
        EnvoieMessage(NoEnvoyeur, "OK");
      }
    }
    else
      Erreur(130010);
  }

  // On demande le niveau de la batterie
  if(Commande=="OBR_NIVEAU_BATTERIE")
  {
    // Retourne le niveau actuel des batteries
    EnvoieMessage(NoEnvoyeur, String(NiveauBatterieActuel));
  }
  
  // On récupère la valeur brut du capteur
  if(Commande=="OBR_VALEUR_BRUT_CAPTEUR")
  {
    if(param1.toInt() < NB_PIN_ANALOG)
      EnvoieMessage(NoEnvoyeur, String(Lecture_RAM(VALEUR_ACTU_ANALOG, param1.toInt())));
    else
      Erreur(110031);
  }

  // On récupère la valeur brut d'une entrée/Sortie
  if(Commande=="OBR_VALEUR_BRUT_ES")
  {
    if(param1.toInt()<NB_PIN_IO)
      EnvoieMessage(NoEnvoyeur, String(Lecture_RAM(VALEUR_ACTU_ES, param1.toInt())));
    else
      Erreur(110031);
  }

  // Définit les limites d'un capteur
  if(Commande=="OBR_CONNECTER_CAPTEUR" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      // On vérifie les arguments
      if(param1.toInt()>=NB_PIN_ANALOG || param2.toInt()>255 || param3.toInt()>255 || param2.toInt()>param3.toInt())
        Erreur(110030);
      else
      {
        // On configure les MIN et MAX des capteurs
        Ecriture_RAM(MIN_ANALOG, param1.toInt(), param2.toInt());
        Ecriture_RAM(MAX_ANALOG, param1.toInt(), param3.toInt());

        // On enregistre les paramètres
        Ecrire_Donnee_EEPROM_ES(param1.toInt(), 1);

        // On envoie une confirmation
        EnvoieMessage(NoEnvoyeur, "OK");
      }
    }
    else
      Erreur(130010);
  }

  // Vide les limites d'un capteur
  if(Commande=="OBR_DECONNECTER_CAPTEUR" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      // On vérifie les arguments
      if(param1.toInt()>=NB_PIN_ANALOG)
        Erreur(110031);
      else
      {
        // On configure les MIN et MAX des capteurs
        Ecriture_RAM(MIN_ANALOG, param1.toInt(), 0);
        Ecriture_RAM(MAX_ANALOG, param1.toInt(), 0);

        // On enregistre les paramètres
        Ecrire_Donnee_EEPROM_ES(param1.toInt(), 1);

        // On envoie une confirmation
        EnvoieMessage(NoEnvoyeur, "OK");
      }
    }
    else
      Erreur(130010);
  }

  // Configure un PIN comme une entrée
  if(Commande=="OBR_CONNECTER_ENTREE" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt()<NB_PIN_IO)
      {
        // On indique au système que c'est une entrée
        ModeES(param1.toInt(), ENTREE);

        // On met la valeur à 0
        Ecriture_RAM(VALEUR_ACTU_ES, param1.toInt(), 0);

        // On enregistre les paramètres
        Ecrire_Donnee_EEPROM_ES(param1.toInt(), 0);

        // On envoie une confirmation
        EnvoieMessage(NoEnvoyeur, "OK");
      }
      else
        Erreur(110031);
    }
    else
      Erreur(130010);
  }

  // déconnecte une entrée
  if(Commande=="OBR_DECONNECTER_ENTREE" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt()<NB_PIN_IO)
      {
        if(Lecture_RAM(TYPE_ES, param1.toInt())==ENTREE)
        {
          // On indique un PIN vide
          ModeES(param1.toInt(), VIDE);

          // On enregistre le tout en EEPROM
          Ecrire_Donnee_EEPROM_ES(param1.toInt(), 0);

          // On envoie une confirmation
          EnvoieMessage(NoEnvoyeur, "OK");
        }
        else
          Erreur(130020);
      }
      else
        Erreur(110031);
    }
    else
      Erreur(130010);
  }

  // Connecte une sortie sur un PIN
  if(Commande=="OBR_CONNECTER_SORTIE" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt()<NB_PIN_IO)
      {
        // On indique au système que c'est une sortie
        ModeES(param1.toInt(), SORTIE);

        // On met la valeur à 0
        Ecriture_RAM(VALEUR_ACTU_ES, param1.toInt(), 0);

        // On enregistre les paramètres
        Ecrire_Donnee_EEPROM_ES(param1.toInt(), 0);

        // On envoie une confirmation
        EnvoieMessage(NoEnvoyeur, "OK");
      }
      else
        Erreur(110031);
    }
    else
      Erreur(130010);
  }

  // Déconnecte une sortie
  if(Commande=="OBR_DECONNECTER_SORTIE" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      if(param1.toInt()<NB_PIN_IO)
      {
        if(Lecture_RAM(TYPE_ES, param1.toInt())==SORTIE)
        {
          // On indique un PIN vide
          ModeES(param1.toInt(), VIDE);

          // On enregistre le tout en EEPROM
          Ecrire_Donnee_EEPROM_ES(param1.toInt(), 0);

          // On envoie une confirmation
          EnvoieMessage(NoEnvoyeur, "OK");
        }
        else
          Erreur(130020);
      }
      else
        Erreur(110031);
    }
    else
      Erreur(130010);
  }

  // 
  if(Commande=="OBR_CONFIG_CAPTEUR_MIN_MAX" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      // On vérifie les paramètres
      if(param1.toInt()>NB_PIN_ANALOG)
        Erreur(110031);
      else if(param2.toInt()>255)
        Erreur(110032);
      else if(param3.toInt()>255)
        Erreur(110033);
      else if(param2.toInt()>param3.toInt())
        Erreur(110023);

      // On inscrit les valeurs
      else
      {
        // on change les paramètres
        Ecriture_RAM(MIN_ANALOG, param1.toInt(), param2.toInt());
        Ecriture_RAM(MAX_ANALOG, param1.toInt(), param3.toInt());

        // on les enregistre
        Ecrire_Donnee_EEPROM_ES(param1.toInt(), 1);

        // on envoie une confirmation
        EnvoieMessage(NoEnvoyeur, "OK");
      }
    }
    else
      Erreur(130010);
  }

  // Enregistrer les paramètres
  if(Commande=="OBR_ENREGISTRER_PARAMETRES" && Mode==CONFIGURATION && Connecte)
  {
    EnvoieMessage(NoEnvoyeur, "OK");

    // Tant qu'on a des E/S à enregistrer, on les enregistre
    byte i = 0;
    for(i=0; i<NB_PIN_IO; i++)
    {
      Ecrire_Donnee_EEPROM_ES(i, 0);
    }
    for(i=0; i<NB_PIN_ANALOG; i++)
    {
      Ecrire_Donnee_EEPROM_ES(i, 1);
    }
  }

  // On lie une sortie avec un état d'un PIN du robot. Param3 = Type de liaison
  if(commande=="OBR_LIER_SORTIE_ES")
  {

  }

  // On lie une entree avec un état d'un PIN du robot (allumer/éteindre)
  if(commande=="OBR_LIER_ENTREE_ES")
  {
  	
  }

  // On lie une entrée analogique avec un état d'un PIN du robot (de 0 à 255)
  if(commande=="OBR_LIER_ANALOG_PWM")
  {
  	
  }

  // Effacement de la chaine
  EffacerCommande();
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////                                           NOYAU DU SYSTEME                                          //////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



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
  // Possibilité à l'utilisateur de réinitialiser le mot de passe
  ModeES(13, SYSTEME);
  digitalWrite(13, LOW);
  pinMode(13, INPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  if(digitalRead(13))
    RAZ_MotDePasse();

  digitalWrite(12, LOW);
  ModeES(12, VIDE);

  // Le reste de l'initialisation se fait dans une fonction externe, qui peut être appelée pour faire un soft-reboot.
  boot();
}

// Fonction de démarrage, peut être utilisée pour faire un soft-reboot
void boot()
{
  // On vide la RAM
  // Ports séries
  Ordinateur = 0; // N° du port série sur lequel est connecté le PC (à partir du port 1)
  Telecommande_Secondaire = 0;
  Robot=0;
  DerniereActuConnexions = 0; // Temps depuis la derniere actualisation des connexions
  // Reception d'une commande
  String Commande = ""; // Dernière commande reçue 
  String param1 = ""; // Paramètres de la commande
  String param2 = "";
  String param3 = "";
  boolean stringComplete = false; // Indique si une commande est arrivée
  NbParam = 0; // Nombre de paramètres dans la commande
  NoEnvoyeur = 0; // Adresse de l'envoyeur du dernier message
  NoDestinataire = 0; // Adresse du destinataire du dernier message
  NbActualisation = -1; // limite les envoie de demande de connexion
  // Chien de garde sur les ports série
  boolean watchdogActif = ACTIVER_WATCHDOG;
  DernierTempsPC = 0; // Temps en 1/10 de seconde depuis la dernière réception
  DernierTempsTelecommande = 0;
  DernierTempsRobot = 0;
  DernierTempsWatchdog = 0; // Temps en ms de la dernière actualisation du chien de garde
  // Administration
  Connecte = 0; // Indique si on est connecté en administrateur (accès à certaines commandes)
  // Batterie
  PIN_BATTERIE = 0; // PIN sur lequel est connecté la batterie
  PIN_BATTERIE_FAIBLE = 0; // PIN sur lequel est connecté la LED de batterie faible
  PIN_BATTERIE_PLEINE = 0; // Pareil, pour la batterie pleine
  NiveauBatterieActuel = 0; // Indique le niveau actuel des batteries, en pourcentage
  NiveauBatterieMin = 0; // Le niveau correspondant à 0%
  NiveauBatterieMax = 1024; // Le niveau correspondant à 100%
  // Gestion d'erreur
  DerniereErreur = 0;
  NombreErreur = 0;

  // On initialise les tableaux
  Initialisation_Tableaux();

  // Arrêt de la LED 13
  ModeES(13, SYSTEME);
  Ecriture_RAM(VALEUR_ACTU_ES, 13, 0);

  // Initialisation des E/S
  byte i = 0;
  for(i=0; i<NB_PIN_ANALOG; i++)
  {
    Lire_Donnee_EEPROM_ES(i, 1);
  }
  for(i=0; i<NB_PIN_IO; i++)
  {
    Lire_Donnee_EEPROM_ES(i, 0);
  }

  // On autorise l'utilisateur à exécuter son propre code
  user_setup();
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
  
  // On met à jour les états des entrées/sorties et capteurs
  MAJ_RAM();

  // On autorise l'utilisateur à exécuter son propre code
  user_loop();
  
  // On fait une pause de 1 ms pour éviter de trop consommer et de trop chauffer
  delay(1);
}

// Fonction de gestion des erreurs
void Erreur(unsigned long NoErreur)
{
  DerniereErreur = NoErreur;
  EnvoieMessage(NoEnvoyeur, "ERREUR_"+String(NoErreur));
}

// Fonction qui retourne la dernière erreur
unsigned int LireErreur()
{
  // On retourne la dernière erreur
  return DerniereErreur;
}

// Fonction de lecture d'une case Entrée/Sortie selon l'EEPROM (Analog=0 si E/S, 1 si ANALOG)
void Lire_Donnee_EEPROM_ES(byte NoES, boolean Analog) // Travaille directement sur la RAM, sans passer par les fonctions noyau
{
  // On lit le numéro demandée
  if(Analog && NoES>=NB_PIN_ANALOG)
    return;
  if(!Analog && NoES>=NB_PIN_IO)
    return;

  // On regarde le type d'entrée/sortie (ou ANALOG)
  byte TypeES = 0;
  if(Analog)
  {
    Min_Analog[NoES] = EEPROM.read(100+(2*NoES)); // Apres les 100 premieres case, toutes les 2 cases
    Max_Analog[NoES] = EEPROM.read(101+(2*NoES));
    return;
  }

  // On est avec une entrée/sortie, on récupère le type d'entrée/sortie
  TypeES = EEPROM.read(100 + (2*NB_PIN_ANALOG) + (NoES*6)); // Après les 100 premières case, et les cases des capteurs, toutes les 1 cases

  // Selon le type d'E/S, on récupère les différentes valeurs qu'il faut et on les enregistre en RAM
  Type_ES[NoES] = TypeES;

  // Si le PIN est configuré (n'est pas vide), on charge les paramètres
  if(TypeES==VIDE)
    return;

  // On charge les N° des commandes en lien avec l'E/S
}

// Fonction d'écriture d'une case Entrée/Sortie en EEPROM (Analog=0 si E/S, 1 si ANALOG)
void Ecrire_Donnee_EEPROM_ES(byte NoES, boolean Analog) // Travaille directement sur la RAM, sans passer par les fonctions noyau
{
  // On lit le numéro demandée
  if(Analog && NoES>=NB_PIN_ANALOG)
    return;
  if(!Analog && NoES>=NB_PIN_IO)
    return;

  // On regarde le type d'entrée/sortie (ou ANALOG)
  byte TypeES = 0;
  if(Analog)
  {
    EEPROM.write(100+(2*NoES), Lecture_RAM(MIN_ANALOG, NoES));
    EEPROM.write(101+(2*NoES), Lecture_RAM(MAX_ANALOG, NoES));
    return;
  }

  // On est avec une entrée/sortie, on récupère le type d'entrée/sortie
  TypeES = Type_ES[NoES];

  // Selon le type d'E/S, on récupère les différentes valeurs qu'il faut et on les enregistre en EEPROM
  EEPROM.write(100 + (2*NB_PIN_ANALOG) + (NoES*6), TypeES); // Après les 100 premières case, et les cases des capteurs, toutes les 6 cases

  // On enregistre l'action à effectuer (chaque commande a un n° unique)

}

// Fonction de verrouillage du système
void BloquageSysteme()
{
  // On bloque l'arduino, obligeant le RESET physique
  while(1)
  {
    if(Type_ES[13]==SYSTEME)
    {
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(13, HIGH);
      delay(500);
    }
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
// Fonction d'initialisation des tableaux
void Initialisation_Tableaux()
{
  byte i = 0;
  // Type_ES = 0
  for(i=0; i<NB_PIN_IO; i++)
    Type_ES[i]=0; // Tout à 0

  for(i=0; i<NB_PIN_ANALOG; i++)
    Min_Analog[i]=0; // Tout à 0

  for(i=0; i<NB_PIN_ANALOG; i++)
    Max_Analog[i]=255; // Tout à 255

  for(i=0; i<NB_PIN_ANALOG; i++)
    Valeur_Actu_Analog[i]=0; // Tout à 0

  for(i=0; i<NB_PIN_IO; i++)
    Valeur_Actu_IO[i]=0; // Tout à 0
}

// Fonction de mise à jour en RAM
void MAJ_RAM()
{
  // On met à jour le tableau des capteurs
  byte i = 0;
  for(i=0; i<NB_PIN_ANALOG; i++)
  {
    Ecriture_RAM(VALEUR_ACTU_ANALOG, i, analogRead(i));
  }

  // On met à jour le tableau des Entrées
  for(i=0; i<NB_PIN_IO; i++)
  {
    if(Type_ES[i]==ENTREE)
      Ecriture_RAM(VALEUR_ACTU_ES, i, digitalRead(i));
  }

  // On appel la fonction de mise à jour de l'état des sorties
  MAJ_ES();
}

// Fonction d'écriture en RAM
void Ecriture_RAM(byte Type, byte NumeroCase, byte Valeur)
{
  // On détermine le type de tableau que nous devons lire
  // On détermine les limites (de case et de taille valeur) du tableau
  // On inscrit la valeur dans le tableau
  if(Valeur > 255)
    return;

  switch(Type)
  {
    case VALEUR_ACTU_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      Valeur_Actu_Analog[NumeroCase] = Valeur;
    break;

    case VALEUR_ACTU_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Valeur_Actu_IO[NumeroCase] = Valeur;
    break;

    case MIN_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      Min_Analog[NumeroCase] = Valeur;
    break;

    case MAX_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      Max_Analog[NumeroCase] = Valeur;
    break;

    case TYPE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Type_ES[NumeroCase] = Valeur;
    break;

    default:
    break;
  }
}

// Fonction de lecture en RAM
byte Lecture_RAM(byte Type, byte NumeroCase)
{
  // On détermine le type de tableau que nous devons lire et on retourne la veleur de la case demandée
  switch(Type)
  {
    case VALEUR_ACTU_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return Valeur_Actu_Analog[NumeroCase];
    break;

    case VALEUR_ACTU_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Valeur_Actu_IO[NumeroCase];
    break;

    case MIN_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return Min_Analog[NumeroCase];
    break;

    case MAX_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return Max_Analog[NumeroCase];
    break;

    case TYPE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Type_ES[NumeroCase];
    break;

    default:
    return 0;
    break;

    return 0;
  }
}

// Fonctions de controle du robot
// Fonction de mise à jour des sorties, selon la RAM
void MAJ_ES()
{
  // On lit le tableau des sorties, on regarde si on doit (et peut) activer une sortie, et on l'active
  for(i=0; i<NB_PIN_IO; i++)
  {
    // On regarde si c'est une sortie non PWM
    if(Lecture_RAM(TYPE_ES, i)==SORTIE)
      digitalWrite(i, Lecture_RAM(VALEUR_ACTU_ES, i));

    // Si c'est un PIN vide, on le met à 0
    if(Lecture_RAM(TYPE_ES, i)==VIDE)
      digitalWrite(i, 0);
  }

  // Gestion différente pour le PIN 13, s'il est utilisé par le système
  if(Lecture_RAM(TYPE_ES, 13)==VIDE)
  {
    if(Connecte)
      digitalWrite(13, HIGH);
    else
      digitalWrite(13, LOW);
  }
}

// Fonction d'écriture sur les ports digitals
void WriteES(byte PIN, boolean Etat)
{
  if(PIN<0 || PIN>=NB_PIN_IO)
    return;

  // On vérifie que c'est une sortie
  if(Type_ES[PIN]!=SORTIE)
    return;

  // On écrit la donnée en RAM
  Ecriture_RAM(VALEUR_ACTU_ES, PIN, Etat);
}

// Fonction de changement de mode d'une E/S
void ModeES(byte PIN, byte Mode)
{
  // On vérifie que le PIN existe
  if(PIN<0 || PIN>=NB_PIN_IO)
    return;

  // On vérifie que le mode existe
  if(Mode<0 || Mode>2)
    return;

  // On met à jour les variables
  Ecriture_RAM(TYPE_ES, PIN, Mode);
  Ecriture_RAM(VALEUR_ACTU_ES, PIN, 0);

  // On met à jour le type de sortie
  switch(Mode)
  {
    case SORTIE:
    case SYSTEME:
    pinMode(PIN, OUTPUT);
    break;

    default:
    pinMode(PIN, INPUT);
    break;
  }
}

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
  RecupMessage(NoPortSerie(TELECOMMANDE_SECONDAIRE));
  // S'il y a un message, on vérifie l'adresse
  if(stringComplete)
      ExecuterCommande();

  // On vérifie la présence de messages venant de l'extension 1 ou du maitre
  RecupMessage(NoPortSerie(ROBOT));
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

            case TELECOMMANDE_SECONDAIRE:
            DernierTempsTelecommande = 0;
            break;

            case ROBOT:
            DernierTempsMaitre = 0;
            break;

            default:
            break;
          }
        }

        // Si c'est un simple message de présence (watchdog), on annule le message
        if(Commande=="=")
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
  Message = '[2'+'|'+String(Type)+']'+Message;

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

    #if(NB_PORT_SERIE>1)
    case 2:
    Serial1.println(Message);
    break;
    #endif

    #if(NB_PORT_SERIE>1)
    case 3:
    Serial2.println(Message);
    break;
    #endif

    #if(NB_PORT_SERIE>1)
    case 4:
    Serial3.println(Message);
    break;
    #endif

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
  if(Ordinateur != 1 && Telecommande_Secondaire != 1 && Robot != 1)
    Demande_Connexion(1);

  if(Ordinateur != 2 && Telecommande_Secondaire != 2 && Robot != 2)
    Demande_Connexion(2);

  if(Ordinateur != 3 && Telecommande_Secondaire != 3 && Robot != 3)
    Demande_Connexion(3);

  if(Ordinateur != 4 && Telecommande_Secondaire != 4 && Robot != 4)
    Demande_Connexion(4);
}

// Chien de garde pour la déconnexion automatique des ports séries
void WatchDog_Serie()
{
  // On met à jour la durée de tous les ports
  byte DifferenceTempsActu = (millis()-DernierTempsWatchdog)/100;
  DernierTempsWatchdog = millis();

  DernierTempsPC += DifferenceTempsActu;
  DernierTempsTelecommande += DifferenceTempsActu;

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
    Telecommande_Secondaire = 0;

  if(DernierTempsRobot > 50)
    Robot = 0;
}

// On envoie une demande de connexion
void Envoie_Connexion(byte NoPort)
{
  // On récupère la réponse de la demande précédente
  RecupMessage(NoPort);

   // On vérifie que c'est bien l'accusé de réception
  if(Commande=="OK")
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

      case ROBOT:
      if(!Robot)
        Robot = NoPort;
      break;

      default:
      break;
    }
  }

  // Sinon, cest peut-être une demande de connexion
  else if(Commande=="OBR_CONNECTER_PC" && !Ordinateur)
  {
    Ordinateur = NoPort;
    EnvoieMessage(PC, "OK");
    DernierTempsPC = 0;
  }

  else if(Commande=="OBR_CONNECTER_TELECOMMANDE" && !Telecommande)
  {
    Telecommande_Secondaire = NoPort;
    EnvoieMessage(TELECOMMANDE_SECONDAIRE, "OK");
    DernierTempsTelecommande = 0;
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
    if(Commande=="OBR_CONNECTER_PC" && !Ordinateur)
    {
      Ordinateur = NoPort;
      EnvoieMessage(PC, "OK");
      DernierTempsPC = 0;
    }

    else if(Commande=="OBR_CONNECTER_TELECOMMANDE" && !Telecommande)
    {
      Telecommande_Secondaire = NoPort;
      EnvoieMessage(TELECOMMANDE_SECONDAIRE, "OK");
      DernierTempsTelecommande = 0;
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
    case ROBOT:
      return Robot;

    case TELECOMMANDE_SECONDAIRE:
      return Telecommande_Secondaire;

    case PC:
      return Ordinateur;

    default:
      return 0;
  }
}

// Fonction d'effacement de commande en file d'attente
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