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
// - Remplissage des commandes (la plupart sont vides)
// - Gestion des centaines dans les numéros de PIN (pour différencier les cartes), le PIN 255 pointe vers rien !
// - Extinction de toutes les sorties en mode dégradé (fonction d'arrêt d'urgence)
// - Prise en compte des limites pour les liens
//////////////////////////////////////////// A FAIRE

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
#define NOM_DU_ROBOT "Demineur2560" // Nom du robot
#define TYPE_DE_CARTE _MEGA // Définit le type de carte, et donc le nombre d'E/S, d'EEPROM...
#define MAITRE_ESCLAVE MAITRE // Définit si la carte sera le maitre, ou un des deux esclaves
#define DEBUG 0 // Uniquement pour le déboguage !!! Mettre 0 pour une utilisation finale !!!



// Constantes
// Version du micrologiciel
#define VERSION 0.5
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
#define SYSTEME 5
// Type de tableau RAM
#define MIN_ANALOG 1
#define MAX_ANALOG 2
#define MIN_PWM 3
#define MAX_PWM 4
#define TYPE_ES 5
#define LIAISON_ANALOG 6
#define LIAISON_1 7
#define LIAISON_2 8
#define LIAISON_3 9
#define TYPE_LIAISON_ANALOG 10
#define TYPE_LIAISON_1 11
#define TYPE_LIAISON_2 12
#define TYPE_LIAISON_3 13
#define VALEUR_ACTU_ANALOG 14
#define VALEUR_ACTU_ES 15
#define VALEUR_ACTU_PWM 16
#define VITESSE_TRANSITION_PWM 17
#define VALEUR_VOULUE_PWM 18
#define VALEUR_VOULUE_ES 19

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
// Tableaux contenant les différentes données d'E/S
byte Type_ES[NB_PIN_IO]; // Les différents type d'entrée/sortie
byte Min_PWM[NB_PIN_PWM]; // Valeur Min d'un moteur/servo
byte Max_PWM[NB_PIN_PWM]; // Valeur Max
byte Min_Analog[NB_PIN_ANALOG]; // Valeur Min d'un capteur
byte Max_Analog[NB_PIN_ANALOG]; // Valeur Max d'un capteur
byte Liaison_Analog[NB_PIN_ANALOG]; // Liaison E/S -> Analog
byte Liaison_1[NB_PIN_IO]; // Première liaison E/S -> E/S
byte Liaison_2[NB_PIN_IO]; // Deuxième
byte Liaison_3[NB_PIN_IO]; // Troisième
byte Type_Liaisons[NB_PIN_IO]; // 4x2 bits pour déterminer le type de liaison (0 = entre MIN et MAX, 1 = en dehors de MIN et MAX, 3 = >= MIN, 4 = <= MAX)
byte Valeur_Actu_Analog[NB_PIN_ANALOG]; // Valeur actuelle du PIN analog
byte Valeur_Actu_IO[NB_PIN_IO]; // Valeur actuelle du PIN I/O
byte Vitesse_Transition_PWM[NB_PIN_PWM]; // Vitesse de transition des valeurs PWM
byte Valeur_Voulue_IO[NB_PIN_IO]; // Valeur/vitesse voulue (pas forcément celle qu'on a en sortie)
unsigned long DernierTempsTransitionPWM = 0;




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

  // On demande sur quel port est connecté la télécommande
  if(Commande=="OBR_CONNEXION_TELECOMMANDE" && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    EnvoieMessage(NoEnvoyeur, String(Telecommande));

  // On demande sur quel port est connecté l'extension 1
  if(Commande=="OBR_CONNEXION_EXTENSION1" && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    EnvoieMessage(NoEnvoyeur, String(Extension1));

  // On demande sur quel port est connecté l'extension 2
  if(Commande=="OBR_CONNEXION_EXTENSION2" && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    EnvoieMessage(NoEnvoyeur, String(Extension2));

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

  // On demande à passer en mode de pilotage
  if(Commande=="OBR_PILOTER")
  {
    Mode = PILOTAGE;
    EnvoieMessage(NoEnvoyeur, "PILOTAGE");
  }

  // On demande à passer en mode de configuration
  if(Commande=="OBR_CONFIGURER")
  {
    Mode = CONFIGURATION;
    EnvoieMessage(NoEnvoyeur, "CONFIGURATION");
  }

  if(Commande=="OBR_DECONNEXION")
  {
    switch(NoEnvoyeur)
    {
      case PC:
      Ordinateur = 0;
      Connecte = 0;
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

  if(Commande=="OBR_DECONNEXION_AUTO" && (NoEnvoyeur == PC || NoEnvoyeur == TELECOMMANDE))
  {
    watchdogActif = param1.toInt();
    if(watchdogActif)
      EnvoieMessage(NoEnvoyeur, "WATCHDOG_ACTIF");
    else
      EnvoieMessage(NoEnvoyeur, "WATCHDOG_INACTIF");
  }

  if(Commande=="OBR_DECONNECTER")
  {
    switch(NoEnvoyeur)
    {
      case PC:
      Connecte = 0;
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
  
  // On récupère la valeur brut du servomoteur ou du moteur
  if(Commande=="OBR_VALEUR_BRUT_SERVO" || Commande=="OBR_VALEUR_BRUT_MOTEUR")
  {
    if(param1.toInt()<NB_PIN_PWM)
      EnvoieMessage(NoEnvoyeur, String(Lecture_RAM(VALEUR_ACTU_PWM, param1.toInt())));
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

  // On configure un moteur
  if(Commande=="OBR_CONNECTER_MOTEUR" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      // On vérifie le Numéro du PIN et les MIN et MAX
      if(param1.toInt()>=NB_PIN_PWM || param2.toInt()>255 || param3.toInt()>255 || param2.toInt()>param3.toInt())
        Erreur(110030);
      else
      {
        // On configure le PIN en MOTEUR
        ModeES(No_PIN_PWM(param1.toInt()), MOTEUR);

        // On indique ses limites
        Ecriture_RAM(MIN_PWM, param1.toInt(), param2.toInt());
        Ecriture_RAM(MAX_PWM, param1.toInt(), param3.toInt());

        // On enregistre en EEPROM
        Ecrire_Donnee_EEPROM_ES(No_PIN_PWM(param1.toInt()), 0);

        // On envoie une confirmation
        EnvoieMessage(NoEnvoyeur, "OK");
      }
    }
    else
      Erreur(130010);
  }

  // Déconnecte un moteur
  if(Commande=="OBR_DECONNECTER_MOTEUR" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      // On vérifie le numéro du PIN
      if(param1.toInt()>=NB_PIN_PWM || Lecture_RAM(TYPE_ES, No_PIN_PWM(param1.toInt()))!=MOTEUR)
        Erreur(130020);
      else
      {
        // On indique un PIN vide et on vide les MIN et MAX
        ModeES(No_PIN_PWM(param1.toInt()), VIDE);
        Ecriture_RAM(MIN_PWM, param1.toInt(), 0);
        Ecriture_RAM(MAX_PWM, param1.toInt(), 0);

        // On enregistre les paramètres
        Ecrire_Donnee_EEPROM_ES(No_PIN_PWM(param1.toInt()), 0);

        // On envoie une confirmation
        EnvoieMessage(NoEnvoyeur, "OK");
      }
    }
    else
      Erreur(130010);
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

  // Connecte un servomoteur
  if(Commande=="OBR_CONNECTER_SERVO" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      // On vérifie le Numéro du PIN et les MIN et MAX
      if(param1.toInt()>=NB_PIN_PWM || param2.toInt()>255 || param3.toInt()>255 || param2.toInt()>param3.toInt())
        Erreur(110030);
      else
      {
        // On configure le PIN en SERVO
        ModeES(No_PIN_PWM(param1.toInt()), SERVO);

        // On indique ses limites
        Ecriture_RAM(MIN_PWM, param1.toInt(), param2.toInt());
        Ecriture_RAM(MAX_PWM, param1.toInt(), param3.toInt());

        // On enregistre les paramètres
        Ecrire_Donnee_EEPROM_ES(No_PIN_PWM(param1.toInt()), 0);

        // On envoie une confirmation
        EnvoieMessage(NoEnvoyeur, "OK");
      }
    }
    else
      Erreur(130010);
  }

  // Déconnecte un Servomoteur
  if(Commande=="OBR_DECONNECTER_SERVO" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {
      // On vérifie le numéro du PIN
      if(param1.toInt()>=NB_PIN_PWM || Lecture_RAM(TYPE_ES, No_PIN_PWM(param1.toInt()))!=SERVO)
        Erreur(130020);
      else
      {
        // On indique un PIN vide et on vide les MIN et MAX
        ModeES(No_PIN_PWM(param1.toInt()), VIDE);
        Ecriture_RAM(MIN_PWM, param1.toInt(), 0);
        Ecriture_RAM(MAX_PWM, param1.toInt(), 0);

        // On enregistre les paramètres
        Ecrire_Donnee_EEPROM_ES(No_PIN_PWM(param1.toInt()), 0);

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

          // On enregistre les paramètres
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

          // On enregistre les paramètres
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

  // Lier une E/S avec un capteur. params : 1=N°E/S 2=N°Capteur 3=TypeLien
  if(Commande=="OBR_LIER_CAPTEUR" && Mode==CONFIGURATION)
  {

  }

  // Lier une E/S avec une autre E/S. params : 1=N°E/S 2=N°Lien 3=TypeLien
  if(Commande=="OBR_LIER_ES" && Mode==CONFIGURATION)
  {

  }

  // délier une E/S de tout. param : 1=N°E/S
  if(Commande=="OBR_DELIER" && Mode==CONFIGURATION)
  {

  }

  // 
  if(Commande=="OBR_CONFIG_MOTEUR_MIN_MAX" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {



      
      EnvoieMessage(NoEnvoyeur, "OK");
    }
    else
      Erreur(130010);
  }

  // 
  if(Commande=="OBR_CONFIG_CAPTEUR_MIN_MAX" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {



      
      EnvoieMessage(NoEnvoyeur, "OK");
    }
    else
      Erreur(130010);
  }

  // 
  if(Commande=="OBR_CONFIG_SERVO_MIN_MAX" && Mode==CONFIGURATION)
  {
    if(Connecte)
    {



      
      EnvoieMessage(NoEnvoyeur, "OK");
    }
    else
      Erreur(130010);
  }

  // Définit la vitesse de transition des valeurs d'une sortie PWM (250=0s, 1=5s, 0=interdit; map()=> 0=0s, 250=2,5s)
  if(Commande=="OBR_CONFIG_PWM_TRANSITION" && Mode==CONFIGURATION)
  {
    // On vérifie que c'est bien un moteur ou un capteur
    if(Connecte)
    {
      if(param1.toInt()<NB_PIN_PWM)
      {
        if(Lecture_RAM(TYPE_ES, No_PIN_PWM(param1.toInt()))==MOTEUR || Lecture_RAM(TYPE_ES, No_PIN_PWM(param1.toInt()))==SERVO)
        {
          // On vérifie la vitesse de transmission (entre 0 et 255)
          if(param2.toInt()>250)
            Erreur(110032);
          else
          {
            // On adapte la nouvelle valeur
            byte Vitesse = 251 / param2.toInt();
            // On enregistre la nouvelle valeur
            Ecriture_RAM(VITESSE_TRANSITION_PWM, param1.toInt(), Vitesse);
            // On enregistre les paramètres
            Ecrire_Donnee_EEPROM_ES(No_PIN_PWM(param1.toInt()), 0);
            EnvoieMessage(NoEnvoyeur, "OK");
          }
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

  // On change la vitesse du moteur
  if(Commande=="OBR_VITESSE_MOTEUR" && (Mode==PILOTAGE || DEBUG))
  {
    // On vérifie que la sortie est un moteur
    if(Lecture_RAM(TYPE_ES, No_PIN_PWM(param1.toInt()))!=MOTEUR || param1.toInt()>NB_PIN_PWM)
      Erreur(110021);
    else
    {
      // On met à jour la vitesse en tenant compte des limites
      if(param2.toInt()<Lecture_RAM(MIN_PWM, param1.toInt()))
      {
        Ecriture_RAM(VALEUR_VOULUE_PWM, param1.toInt(), Lecture_RAM(MIN_PWM, param1.toInt()));
        EnvoieMessage(NoEnvoyeur, "MIN");
      }
      else if(param2.toInt()>Lecture_RAM(MAX_PWM, param1.toInt()))
      {
        Ecriture_RAM(VALEUR_VOULUE_PWM, param1.toInt(), Lecture_RAM(MAX_PWM, param1.toInt()));
        EnvoieMessage(NoEnvoyeur, "MAX");
      }
      else
      {
        Ecriture_RAM(VALEUR_VOULUE_PWM, param1.toInt(), param2.toInt());
        EnvoieMessage(NoEnvoyeur, "OK");
      }
    }
  }

  // On arrête un moteur
  if(Commande=="OBR_ARRETER_MOTEUR" && (Mode==PILOTAGE || DEBUG))
  {
    // On vérifie que la sortie est un moteur
    if(Lecture_RAM(TYPE_ES, No_PIN_PWM(param1.toInt()))!=MOTEUR || param1.toInt()>NB_PIN_PWM)
      Erreur(110021);
    else
    {
      Ecriture_RAM(VALEUR_VOULUE_PWM, param1.toInt(), 0);
      Ecriture_RAM(VALEUR_ACTU_PWM, param1.toInt(), 0);
      EnvoieMessage(NoEnvoyeur, "OK");
    }
  }

  // On change la position du servomoteur
  if(Commande=="OBR_POSITION_SERVO" && (Mode==PILOTAGE || DEBUG))
  {
    // On vérifie que la sortie est un servomoteur
    if(Lecture_RAM(TYPE_ES, No_PIN_PWM(param1.toInt()))!=SERVO || param1.toInt()>NB_PIN_PWM)
      Erreur(110021);
    else
    {
      // On met à jour la vitesse en tenant compte des limites
      if(param2.toInt()<Lecture_RAM(MIN_PWM, param1.toInt()))
        Ecriture_RAM(VALEUR_VOULUE_PWM, param1.toInt(), Lecture_RAM(MIN_PWM, param1.toInt()));
      else if(param2.toInt()>Lecture_RAM(MAX_PWM, param1.toInt()))
        Ecriture_RAM(VALEUR_VOULUE_PWM, param1.toInt(), Lecture_RAM(MAX_PWM, param1.toInt()));
      else
        Ecriture_RAM(VALEUR_VOULUE_PWM, param1.toInt(), param2.toInt());
      EnvoieMessage(NoEnvoyeur, "OK");
    }
  }

  // 
  if(Commande=="OBR_ALLUMER_SORTIE" && (Mode==PILOTAGE || DEBUG))
  {
    // On vérifie que c'est bien une sortie digitale
    if(Lecture_RAM(TYPE_ES, param1.toInt())!=SORTIE || param1.toInt()>NB_PIN_IO)
      Erreur(110021);
    else
    {
      Ecriture_RAM(VALEUR_VOULUE_ES, param1.toInt(), 1);
      EnvoieMessage(NoEnvoyeur, "OK");
    }
  }

  // 
  if(Commande=="OBR_ARRETER_SORTIE" && (Mode==PILOTAGE || DEBUG))
  {
    // On vérifie que c'est bien une sortie digitale
    if(Lecture_RAM(TYPE_ES, param1.toInt())!=SORTIE || param1.toInt()>NB_PIN_IO)
      Erreur(110021);
    else
    {
      Ecriture_RAM(VALEUR_VOULUE_ES, param1.toInt(), 0);
      EnvoieMessage(NoEnvoyeur, "OK");
    }
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
  Telecommande = 0;
  Extension1 = 0;
  Extension2 = 0;
  Carte_Maitre = 0;
  NoCarte = 0; // Différent si esclave d'une extension
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
  boolean watchdogActif = 1;
  DernierTempsPC = 0; // Temps en 1/10 de seconde depuis la dernière réception
  DernierTempsTelecommande = 0;
  DernierTempsMaitre = 0;
  DernierTempsEsclave1 = 0;
  DernierTempsEsclave2 = 0;
  DernierTempsWatchdog = 0; // Temps en ms de la dernière actualisation du chien de garde
  // Administration
  Connecte = 0; // Indique si on est connecté en administrateur (accès à certaines commandes)
  Mode = CONFIGURATION; // définit le mode en cours (configuration ou pilotage)
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
  Ecriture_RAM(VALEUR_VOULUE_ES, 13, 0);

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

  if(Lecture_RAM(TYPE_ES, 13)==VIDE)
    ModeES(13, SYSTEME);

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

  // Si le PIN est configuré (n'est pas vide ou SYSTEME), on charge les paramètres
  if(TypeES==VIDE || TypeES==SYSTEME)
    return;

  // Liaison Analog
  Liaison_Analog[NoES] = EEPROM.read(101 + (2*NB_PIN_ANALOG) + (NoES*6));

  // Liaison 1
  Liaison_1[NoES] = EEPROM.read(102 + (2*NB_PIN_ANALOG) + (NoES*6));

  // Liaison 2
  Liaison_2[NoES] = EEPROM.read(103 + (2*NB_PIN_ANALOG) + (NoES*6));

  // Liaison 3
  Liaison_3[NoES] = EEPROM.read(104 + (2*NB_PIN_ANALOG) + (NoES*6));

  // Types Liaisons
  Type_Liaisons[NoES] = EEPROM.read(105 + (2*NB_PIN_ANALOG) + (NoES*6));

  // Si c'est du PWM, il y a des cases en plus
  if(TypeES==SERVO || TypeES==MOTEUR)
  {
    // On indique le N°PWM et plus le N°E/S
    NoES = NoES_PWM(NoES);

    // Min PWM
    Min_PWM[NoES] = EEPROM.read(100 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));

    // Max PWM
    Max_PWM[NoES] = EEPROM.read(101 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));

    // Transition PWM
    Vitesse_Transition_PWM[NoES] = EEPROM.read(102 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));
  }
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
  EEPROM.write(100 + (2*NB_PIN_ANALOG) + (NoES*6), TypeES); // Après les 100 premières case, et les cases des capteurs, toutes les 1 cases

  // Liaison Analog
  EEPROM.write(101 + (2*NB_PIN_ANALOG) + (NoES*6), Liaison_Analog[NoES]);

  // Liaison 1
  EEPROM.write(102 + (2*NB_PIN_ANALOG) + (NoES*6), Liaison_1[NoES]);

  // Liaison 2
  EEPROM.write(103 + (2*NB_PIN_ANALOG) + (NoES*6), Liaison_2[NoES]);

  // Liaison 3
  EEPROM.write(104 + (2*NB_PIN_ANALOG) + (NoES*6), Liaison_3[NoES]);

  // Types Liaisons
  EEPROM.write(105 + (2*NB_PIN_ANALOG) + (NoES*6), Type_Liaisons[NoES]);

  // Si c'est du PWM, il y a des cases en plus
  if(TypeES==SERVO || TypeES==MOTEUR)
  {
    // On indique le N°PWM et plus le N°E/S
    NoES = NoES_PWM(NoES);
    
    // Min PWM
    EEPROM.write(100 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Min_PWM[NoES]);

    // Max PWM
    EEPROM.write(101 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Max_PWM[NoES]);

    // Transition PWM
    EEPROM.write(102 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Vitesse_Transition_PWM[NoES]);
  }
}

// Fonction de verrouillage du système
void BloquageSysteme()
{
  // On éteint tous les moteurs


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

  for(i=0; i<NB_PIN_PWM; i++)
    Min_PWM[i]=0; // Tout à 0

  for(i=0; i<NB_PIN_PWM; i++)
    Max_PWM[i]=255; // Tout à 255

  for(i=0; i<NB_PIN_ANALOG; i++)
    Min_Analog[i]=0; // Tout à 0

  for(i=0; i<NB_PIN_ANALOG; i++)
    Max_Analog[i]=255; // Tout à 255

  for(i=0; i<NB_PIN_ANALOG; i++)
    Liaison_Analog[i]=255; // Tout à 255

  for(i=0; i<NB_PIN_IO; i++)
    Liaison_1[i]=255; // Tout à 255

  for(i=0; i<NB_PIN_IO; i++)
    Liaison_2[i]=255; // Tout à 255

  for(i=0; i<NB_PIN_IO; i++)
    Liaison_3[i]=255; // Tout à 255

  for(i=0; i<NB_PIN_IO; i++)
    Type_Liaisons[i]=0; // Tout à 0

  for(i=0; i<NB_PIN_ANALOG; i++)
    Valeur_Actu_Analog[i]=0; // Tout à 0

  for(i=0; i<NB_PIN_IO; i++)
    Valeur_Actu_IO[i]=0; // Tout à 0

  for(i=0; i<NB_PIN_PWM; i++)
    Vitesse_Transition_PWM[i]=255; // Tout à 255

  for(i=0; i<NB_PIN_IO; i++)
    Valeur_Voulue_IO[i]=0; // Tout à 0
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

// Fonction qui retourne le no PWM en fonction de l'E/S
byte NoES_PWM(byte NoES)
{
  byte i = 0;
  for(i=0; i<NB_PIN_PWM; i++)
  {
    if(No_PIN_PWM(i)==NoES)
      return i;
  }
  return 255;
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
    case VALEUR_ACTU_PWM: // On met à jour le numéro de case selon le PWM demandé
    NumeroCase = No_PIN_PWM(NumeroCase);
    if(Lecture_RAM(TYPE_ES, NumeroCase)!=MOTEUR && Lecture_RAM(TYPE_ES, NumeroCase)!=SERVO)
      break;

    case VALEUR_ACTU_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Valeur_Actu_IO[NumeroCase] = Valeur;
    break;

    if(NumeroCase != 0) // Inscrit la valeur dans la case IO
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

    case MIN_PWM:
    if(NumeroCase<NB_PIN_PWM && NumeroCase>=0)
      Min_PWM[NumeroCase] = Valeur;
    break;

    case MAX_PWM:
    if(NumeroCase<NB_PIN_PWM && NumeroCase>=0)
      Max_PWM[NumeroCase] = Valeur;
    break;

    case TYPE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Type_ES[NumeroCase] = Valeur;
    break;

    case LIAISON_ANALOG: // Numéro du PIN qui est lié
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Liaison_Analog[NumeroCase] = Valeur;
    break;

    case LIAISON_1:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Liaison_1[NumeroCase] = Valeur;
    break;

    case LIAISON_2:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Liaison_2[NumeroCase] = Valeur;
    break;

    case LIAISON_3:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Liaison_3[NumeroCase] = Valeur;
    break;

    case TYPE_LIAISON_ANALOG: // Type de liage
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0 && Valeur <= 4)
      Type_Liaisons[NumeroCase]  = (Type_Liaisons[NumeroCase] && 0b00111111) + Valeur*64; // On prend les 2 premiers octets
    break;

    case TYPE_LIAISON_1:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0 && Valeur <= 4)
      Type_Liaisons[NumeroCase]  = (Type_Liaisons[NumeroCase] && 0b11001111) + Valeur*16; // On prend les 2 octets suivants
    break;

    case TYPE_LIAISON_2:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0 && Valeur <= 4)
      Type_Liaisons[NumeroCase]  = (Type_Liaisons[NumeroCase] && 0b11110011) + Valeur*8; // On prend les 2 octets suivants
    break;

    case TYPE_LIAISON_3:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0 && Valeur <= 4)
      Type_Liaisons[NumeroCase]  = (Type_Liaisons[NumeroCase] && 0b11111100) + Valeur; // On prend les 2 derniers octets
    break;

    case VITESSE_TRANSITION_PWM: // Vitesse à laquelle le mouvement se fait sur le port PWM
    if(NumeroCase<NB_PIN_PWM && NumeroCase>=0)
      Vitesse_Transition_PWM[NumeroCase] = Valeur;
    break;

    case VALEUR_VOULUE_PWM:
    NumeroCase = No_PIN_PWM(NumeroCase);
    if(Lecture_RAM(TYPE_ES, NumeroCase)!=MOTEUR && Lecture_RAM(TYPE_ES, NumeroCase)!=SERVO)
      break;
    
    case VALEUR_VOULUE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      Valeur_Voulue_IO[NumeroCase] = Valeur;
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

    case VALEUR_ACTU_PWM: // On met à jour le numéro de case selon le PWM demandé
    NumeroCase = No_PIN_PWM(NumeroCase);

    if(NumeroCase != 0) // Retourne la veleur de la case PWM
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

    case MIN_PWM:
    if(NumeroCase<NB_PIN_PWM && NumeroCase>=0)
      return Min_PWM[NumeroCase];
    break;

    case MAX_PWM:
    if(NumeroCase<NB_PIN_PWM && NumeroCase>=0)
      return Max_PWM[NumeroCase];
    break;

    case TYPE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Type_ES[NumeroCase];
    break;

    case LIAISON_ANALOG: // Numéro du PIN qui est lié
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Liaison_Analog[NumeroCase];
    break;

    case LIAISON_1:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Liaison_1[NumeroCase];
    break;

    case LIAISON_2:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Liaison_2[NumeroCase];
    break;

    case LIAISON_3:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Liaison_3[NumeroCase];
    break;

    case TYPE_LIAISON_ANALOG: // Type de liage
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return (Type_Liaisons[NumeroCase] && 0b11000000)/64; // On prend les 2 premiers octets
    break;

    case TYPE_LIAISON_1:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return (Type_Liaisons[NumeroCase] && 0b00110000)/16; // On prend les 2 octets suivants
    break;

    case TYPE_LIAISON_2:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return (Type_Liaisons[NumeroCase] && 0b00001100)/4; // On prend les 2 octets suivants
    break;

    case TYPE_LIAISON_3:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Type_Liaisons[NumeroCase] && 0b00000011; // On prend les 2 derniers octets
    break;

    case VITESSE_TRANSITION_PWM: // Vitesse à laquelle le mouvement se fait sur le port PWM
    if(NumeroCase<NB_PIN_PWM && NumeroCase>=0)
      return Vitesse_Transition_PWM[NumeroCase];
    break;

    case VALEUR_VOULUE_PWM:
    NumeroCase = No_PIN_PWM(NumeroCase);

    case VALEUR_VOULUE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Valeur_Voulue_IO[NumeroCase];
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
  // On lit le tableau des PWM, on regarde si on doit (et peut) activer une sortie, et on l'active
  byte i = 0;
  for(i=0; i<NB_PIN_PWM; i++)
  {
    if(Lecture_RAM(TYPE_ES, No_PIN_PWM(i))==SERVO || Lecture_RAM(TYPE_ES, No_PIN_PWM(i))==MOTEUR)
      analogWrite(No_PIN_PWM(i), Autoriser_Sortie(No_PIN_PWM(i)));
  }

  // On lit le tableau des sorties, on regarde si on doit (et peut) activer une sortie, et on l'active
  for(i=0; i<NB_PIN_IO; i++)
  {
    // On regarde si c'est une sortie non PWM
    if(Lecture_RAM(TYPE_ES, i)==SORTIE)
      digitalWrite(i, Autoriser_Sortie(i));

    // Si c'est un PIN vide, on le met à 0
    if(Lecture_RAM(TYPE_ES, i)==VIDE)
      digitalWrite(i, 0);
  }

  // Gestion différente pour le PIN 13, s'il est utilisé par le système
  if(Lecture_RAM(TYPE_ES, 13)==SYSTEME || Lecture_RAM(TYPE_ES, 13)==VIDE)
  {
    if(Connecte)
      digitalWrite(13, HIGH);
    else
      digitalWrite(13, LOW);
  }
}

// Fonction qui retourne la valeur à écrire en sortie physique (sur les PINs)
byte Autoriser_Sortie(byte NumeroSortie)
{
  byte Valeur = 0;
  
  // On regarde quel type de sortie c'est
  byte Type = Lecture_RAM(TYPE_ES, NumeroSortie);
  if(Type != SORTIE && Type != MOTEUR && Type != SERVO)
    return 0;

  // Si c'est un servomoteur, on fait en sorte qu'il ne bouge pas
  if(Type == SERVO)
    Valeur = Lecture_RAM(VALEUR_ACTU_ES, NumeroSortie);

  // Si aucune télécommande/PC n'est connecté, on coupe les moteurs et sorties
  if(!Ordinateur && !Telecommande)
    return Valeur;

  // On regarde si les liaisons l'autorisent
  if(!Liaison_OK(NumeroSortie))
    return Valeur;

  // On regarde si c'est un PWM, et s'il est bien dans ses limites (MIN et MAX)
  if(Type==SERVO || Type==MOTEUR)
  {
    // On regarde la vitesse de transition
    byte VitesseVoulue = Lecture_RAM(VALEUR_VOULUE_ES, NumeroSortie);
    byte VitesseActu = Lecture_RAM(VALEUR_ACTU_ES, NumeroSortie);
    byte VitesseTransition = Lecture_RAM(VITESSE_TRANSITION_PWM, NoES_PWM(NumeroSortie));

    // Si ça fait plus de 10 ms qu'on a pas auctualisé, on actualise
    if(DernierTempsTransitionPWM<millis()-10)
    {
      DernierTempsTransitionPWM+=10;
      // Si on doit accélérer
      if(VitesseActu<VitesseVoulue)
        if(VitesseActu<VitesseVoulue-VitesseTransition)
          VitesseActu += VitesseTransition;
        else
          VitesseActu = VitesseVoulue;

      // Si on doit freiner
      else if(VitesseActu>VitesseVoulue)
        if(VitesseActu>VitesseVoulue+VitesseTransition)
          VitesseActu -= VitesseTransition;
        else
          VitesseActu = VitesseVoulue;
    }

    // On met à jour la nouvelle vitesse/valeur
    Ecriture_RAM(VALEUR_ACTU_ES, NumeroSortie, VitesseActu);

    // On regarde les limites
    byte MIN = Lecture_RAM(MIN_PWM, NoES_PWM(NumeroSortie));
    byte MAX = Lecture_RAM(MAX_PWM, NoES_PWM(NumeroSortie));
    if(Lecture_RAM(VALEUR_ACTU_ES, NumeroSortie)<MIN && Lecture_RAM(VALEUR_ACTU_ES, NumeroSortie)!=0) // On met au minimum sauf s'il est carément arrêté
    {
      Valeur = MIN;
      Ecriture_RAM(VALEUR_ACTU_ES, NumeroSortie, Valeur);
      return Valeur;
    }
    if(Lecture_RAM(VALEUR_ACTU_ES, NumeroSortie)>MAX) // On met au maximum s'il dépasse
    {
      Valeur = MAX;
      Ecriture_RAM(VALEUR_ACTU_ES, NumeroSortie, Valeur);
      return Valeur;
    }
    return VitesseActu;
  }

  // On regarde la valeur demandée en RAM si c'est un ToutOuRien
  Valeur = Lecture_RAM(VALEUR_VOULUE_ES, NumeroSortie);
  Ecriture_RAM(VALEUR_ACTU_ES, NumeroSortie, Valeur);

  // On retourne la valeur
  return Valeur;
}

// retourne 1 si toutes les liaisons sont OK
boolean Liaison_OK(byte NumeroSortie)
{
  byte NoLiaison = 0;
  for(NoLiaison=LIAISON_ANALOG; NoLiaison<LIAISON_3; NoLiaison++)
  {
    // On récupère le numéro du lien
    byte NoLien = Lecture_RAM(NoLiaison, NumeroSortie);

    // S'il y a un lien
    if(NoLien != 255)
    {
      // On récupère le type de liaison
      byte Type_Liaison = Lecture_RAM(NoLiaison+4, NumeroSortie);

      // On récupère le MIN et MAX
      byte MIN = 0;
      byte MAX = 0;
      if(NoLiaison == LIAISON_ANALOG)
      {
        MIN = Lecture_RAM(MIN_ANALOG, NoLien);
        MAX = Lecture_RAM(MAX_ANALOG, NoLien);
      }
      else
      {
        MIN = 0;
        MAX = 0;
      }

      // On récupère la valeur actuelle du capteur ou E/S
      byte ValeurActu = 0;
      if(NoLiaison == LIAISON_ANALOG)
        ValeurActu = Lecture_RAM(VALEUR_ACTU_ANALOG, NoLien);
      else
        ValeurActu = Lecture_RAM(VALEUR_ACTU_ES, NoLien);

      // Selon le type de liaison, on regarde si c'est OK
      switch(Type_Liaison)
      {
        case 0: // Entre MIN et MAX (calcul inverse)
        if(ValeurActu<MIN || ValeurActu>MAX)
          return 0;
        break;

        case 1: // En dehors de MIN et MAX
        if(ValeurActu>MIN && ValeurActu<MAX)
          return 0;
        break;

        case 2: // >=MIN
        if(ValeurActu<MIN)
          return 0;
        break;

        case 3: // <=MAX
        if(ValeurActu>MAX)
          return 0;
        break;

        default:
        return 0;
        break;
      }
    }
  }
  // Si on arrive là, c'est que tout est OK
  return 1;
}

// Fonction d'écriture sur les ports digitals
void WriteES(byte PIN, boolean Etat)
{
  if(PIN<0 || PIN>=NB_PIN_IO)
    return;

  // On vérifie que c'est une sortie
  if(Type_ES[PIN]!=SORTIE && Type_ES[PIN]!=SYSTEME)
    return;

  // On écrit la donnée en RAM
  Ecriture_RAM(VALEUR_VOULUE_ES, PIN, Etat);
}

// Fonction d'écriture sur les ports PWM
void WritePWM(byte PWM, byte Valeur)
{
  if(PWM<0 || PWM>=NB_PIN_PWM)
    return;

  // On vérifie que c'est une sortie
  if(Type_ES[PWM]!=MOTEUR && Type_ES[PWM]!=SERVO)
    return;
  
  // On écrit la donnée en RAM
  Ecriture_RAM(VALEUR_VOULUE_PWM, PWM, Valeur);
}

// Fonction de changement de mode d'une E/S
void ModeES(byte PIN, byte Mode)
{
  // On vérifie que le PIN existe
  if(PIN<0 || PIN>=NB_PIN_IO)
    return;

  // On vérifie que le mode existe
  if(Mode<0 || Mode>5)
    return;

  // On met à jour les variables
  Ecriture_RAM(TYPE_ES, PIN, Mode);
  Ecriture_RAM(VALEUR_ACTU_ES, PIN, 0);
  Ecriture_RAM(VALEUR_VOULUE_ES, PIN, 0);

  // On met à jour le type de sortie
  switch(Mode)
  {
    case SORTIE:
    case MOTEUR:
    case SERVO:
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
  else if(Commande=="OBR_CONNECTER_PC" && !Ordinateur)
  {
    Ordinateur = NoPort;
    EnvoieMessage(PC, "OK");
    DernierTempsPC = 0;
  }

  else if(Commande=="OBR_CONNECTER_TELECOMMANDE" && !Telecommande)
  {
    Telecommande = NoPort;
    EnvoieMessage(TELECOMMANDE, "OK");
    DernierTempsTelecommande = 0;
  }

  else if(Commande=="OBR_CONNECTER_EXTENSION1" && Extension1)
  {
    Extension1 = NoPort;
    EnvoieMessage(EXTENSION1, "OK");
    DernierTempsEsclave1 = 0;
  }

  else if(Commande=="OBR_CONNECTER_EXTENSION2" && Extension2)
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
    if(Commande=="OBR_CONNECTER_PC" && !Ordinateur)
    {
      Ordinateur = NoPort;
      EnvoieMessage(PC, "OK");
      DernierTempsPC = 0;
    }

    else if(Commande=="OBR_CONNECTER_TELECOMMANDE" && !Telecommande)
    {
      Telecommande = NoPort;
      EnvoieMessage(TELECOMMANDE, "OK");
      DernierTempsTelecommande = 0;
    }

    else if(Commande=="OBR_CONNECTER_EXTENSION1" && Extension1)
    {
      Extension1 = NoPort;
      EnvoieMessage(EXTENSION1, "OK");
      DernierTempsEsclave1 = 0;
    }

    else if(Commande=="OBR_CONNECTER_EXTENSION2" && Extension2)
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