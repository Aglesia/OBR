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
// - Gestion des Servomoteurs
// - Passage de 57600 à 155200 bauds à la volée (envoie d'une commande pour passer à 115200, passage à 57600 à la déconnexion)
// - Optimiser la transmission des messages (toutes les commandes toutes les 5 secondes, et envoie des commandes qui changent entre les 5 secondes)
// - BUG : Mot de passe ne reste pas !!!
//////////////////////////////////////////// A FAIRE

#include <EEPROM.h>
// Type de carte
#define _MEGA 1
#define _NANO 2
#define _UNO 3
#define _MICRO 4



// MODIFIABLE !!!
#define NOM_DU_ROBOT "Demineur2560" // Nom du robot
#define TYPE_DE_CARTE _UNO // Définit le type de carte, et donc le nombre d'E/S, d'EEPROM...
#define DEBUG 0 // Uniquement pour le déboguage !!! Mettre 0 pour une utilisation finale !!!



// Constantes
// Version du micrologiciel
#define VERSION "0.8"
// Session
#define CONFIGURATION 0
#define PILOTAGE 1
// Connexions serie
#define PC 1
// Type de message
#define COMMANDE 1
#define REPONSE 2
#define WATCHDOG 0
// Type d'E/S
#define VIDE 0
#define ENTREE 1
#define SORTIE 2
#define SYSTEME 3
#define SERVO 4
// Type de tableau RAM
#define MIN_ANALOG 1 // Minimum d'une entrée analog
#define MAX_ANALOG 2 // Maximum d'une entrée analog
#define TYPE_ES 3 // Type E/S d'une E/S de la télécommande
#define VALEUR_ACTU_ANALOG 4 // Valeur actuelle d'une entrée analogique
#define VALEUR_ACTU_ES 5 // Valeur actuelle d'une E/S
#define VALEUR_A_ZERO 6 // Valeur à 0 d'une E/S
#define VALEUR_A_ZERO_ANALOG 7 // Valeur au point 0 d'une entrée analogique
#define COMMANDE_ASSOCIEE 8 // Commande associée à une entrée (numérique ou analogique)
#define NO_PIN_ROBOT 9 // No du PIN associé sur le robot
#define MIN_ES 10 // Valeur minimum d'une sortie (savoir si on doit l'allumer)
#define MAX_ES 11 // Valeur maximum d'une sortie (savoir si on doit l'allumer)
#define TYPE_LIAISON_ES 12 // Type de liaison entre une valeur_brut du robot et une sortie sur la télécommande
#define VALEUR_PARAM2 13 // Valeur à envoyer en paramètre 2 si c'est une entrée TOR qui envoie une valeur non booléenne
#define PIN_SENS_MOTEUR 14 // No du PIN qui gère le sens d'un moteur, sur le robot (255 = aucun)
#define VALEUR_A_ZERO_PIN_SENS 15 // Valeur à 0 du PIN qui gère le sens d'un moteur sur le robot
#define TYPE_ANALOG 16 // Méthode de getion utilisée pour le calcul d'une entrée analog
#define MIN_ROBOT_MOTEUR_1 17 // Vitesse minimum du moteur 1 associé à une entrée analog
#define MAX_ROBOT_MOTEUR_1 18 // Vitesse maximum du moteur 1 associé à une entrée analog
#define MIN_ROBOT_MOTEUR_2 19 // Vitesse minimum du moteur 2 associé à une entrée analog
#define MAX_ROBOT_MOTEUR_2 20 // Vitesse maximum du moteur 2 associé à une entrée analog
#define COMMANDE_ASSOCIEE_2 21 // Deuxième commande associée pour une entrée analogique
#define NO_PIN_ROBOT_2 22 // Numéro du PIN sur lequel est connecté le deuxième moteur

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
byte Ordinateur = 0; // N° du port série sur lequel est connecté le PC (à partir du port 1)
byte Robot = 0;
byte No_Carte_Robot = 0; // No de la carte connectée (Maitre, esclave1 ou esclave2)
byte Maitre = 0; // Indique si on peut communiquer avec le maitre
byte Extension1 = 0;
byte Extension2 = 0;
unsigned long DerniereActuConnexions = 0; // Temps depuis la derniere actualisation des connexions
// Reception d'une commande
String Commande = ""; // Dernière commande reçue 
String param1 = ""; // Paramètres de la commande
String param2 = "";
String param3 = "";
String Reponse = "";
String ChaineReponse = ""; // Chaine contenant la réponse (32 char MAX)
boolean ReponseRecue = 1; // Indique si on peut envoyer une nouvelle commande ou si on attend encore une réponse
boolean stringComplete = false; // Indique si une commande est arrivée
byte NbParam = 0; // Nombre de paramètres dans la commande
byte NoEnvoyeur = 0; // Adresse de l'envoyeur du dernier message
byte NoDestinataire = 0; // Adresse du destinataire du dernier message
byte ID = 0; // ID du message (commande, réponse...)
byte NbActualisation = -1; // limite les envoie de demande de connexion
unsigned long Dernier_Temps_Envoie_Commande = 0; // Définit s'il faut remettre la variable ReponseRecue à 1
// Chien de garde sur les ports série
boolean watchdogActif = 0;
byte DernierTempsPC = 0; // Temps en 1/10 de seconde depuis la dernière réception
byte DernierTempsRobot = 0;
byte DernierTempsRobot_Envoie = 0;
unsigned long DernierTempsWatchdog = 0; // Temps en ms de la dernière actualisation du chien de garde
byte dernierCycleConnexions = 0; // NB cycle depuis la dernière actualisation des connexions sur les autres cartes
// Administration
byte Connecte = 0; // Indique si on est connecté en administrateur (accès à certaines commandes)
unsigned long Password = 2560; // Mot de passe pour se connecter en administrateur
byte Mode = PILOTAGE; // définit le mode en cours (configuration ou pilotage)
// Batterie
byte PIN_BATTERIE = 0; // PIN sur lequel est connecté la batterie
byte PIN_BATTERIE_FAIBLE = 0; // PIN sur lequel est connecté la LED de batterie faible
byte PIN_BATTERIE_PLEINE = 0; // Pareil, pour la batterie pleine
int NiveauBatterieActuel = 0; // Indique le niveau actuel des batteries, en pourcentage
int NiveauBatterieMin = 0; // Le niveau correspondant à 0%
int NiveauBatterieMax = 1024; // Le niveau correspondant à 100%
// Gestion d'Ecrire_Erreur
unsigned long DerniereErreur = 0;
int NombreErreur = 0;
// Tableaux contenant les différentes données d'E/S
byte Valeur_Actu_Analog[NB_PIN_ANALOG]; // Valeur actuelle du PIN analog
byte Valeur_Actu_IO[NB_PIN_IO]; // Valeur actuelle du PIN I/O

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// MODIFIABLE PAR L'UTILISATEUR AVANT COMPILATION //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Variables


// Fonction d'initialisation
void user_setup()
{
  pinMode(8, OUTPUT);
}

// Boucle principale
void user_loop()
{
  if(Robot)
    digitalWrite(8, HIGH);
  else
    digitalWrite(8, LOW);
}

// Fonctions personelles










////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// NON MODIFIABLE PAR L'UTILISATEUR (FONCTIONS SYSTEMES) ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Fonction d'execution d'une commande
void Executer_Commande_Recue()
{
  if(!stringComplete)
    return;

  Reponse="Erreur_100010";
  
  // Si l'envoyeur est une carte du robot
  if(NoEnvoyeur>2)
    Reponse="";

  // On demande si l'OBR est présent
  if(Commande==F("OBR_PRESENT"))
    Reponse=F("VRAIE");
  
  // On demande si l'OBR est prêt
  if(Commande==F("OBR_PRET"))
    if(Connecte)
      Reponse="ADMIN";
     else
      Reponse="VRAIE";
  
  // On demande la vesrion de l'OBR
  if(Commande==F("OBR_VERSION"))
    Reponse=String(VERSION);
  
  // On demande le nom du robot
  if(Commande==F("OBR_NOM"))
    Reponse=NOM_DU_ROBOT;

  // On demande de lui envoyer les différentes connexions internes
  if(Commande==F("OBR_CONNEXIONS_ROBOT"))
  {
    Reponse="OBR_ROBOT_CONNEXIONS-";
    if(Maitre && Maitre!=SYS_No_Port_Serie(NoEnvoyeur)+10)
      Reponse=Reponse+"1-";
    else
      Reponse=Reponse+"0-";
    if(Extension1 && Extension1!=SYS_No_Port_Serie(NoEnvoyeur)+10)
      Reponse=Reponse+"1-";
    else
      Reponse=Reponse+"0-";
    if(Extension1 && Extension1!=SYS_No_Port_Serie(NoEnvoyeur)+10)
      Reponse=Reponse+"1";
    else
      Reponse=Reponse+"0";

    Envoyer_Message(NoEnvoyeur, COMMANDE, Reponse);
    SYS_Effacer_Commande();
    return;
  }

  // On demande de lui envoyer les connexions PC et Télécommande
  if(Commande==F("OBR_CONNEXIONS_EXTERNE"))
  {
    Reponse="OBR_EXTERNE_CONNEXIONS-";
    if(Ordinateur && Ordinateur!=SYS_No_Port_Serie(NoEnvoyeur)+10)
      Reponse=Reponse+"1-";
    else
      Reponse=Reponse+"0-";
      
    // On dit que le robot est connecté
    Reponse=Reponse+"1";

    Envoyer_Message(NoEnvoyeur, COMMANDE, Reponse);
    SYS_Effacer_Commande();
    return;
  }

  // On nous envoie les différentes connexions interne
  if(Commande==F("OBR_ROBOT_CONNEXIONS"))
  {
    // S'il est vide, on le remplie
    if(!Maitre && param1.toInt())
      Maitre = SYS_No_Port_Serie(NoEnvoyeur)+10;
    if(!Extension1 && param2.toInt())
      Extension1 = SYS_No_Port_Serie(NoEnvoyeur)+10;
    if(!Extension2 && param3.toInt())
      Extension2 = SYS_No_Port_Serie(NoEnvoyeur)+10;

    // S'il est remplie mais que le NoEnvoyeur nous dit qu'il n'est plus connecté, on le vide
    if(Maitre==SYS_No_Port_Serie(NoEnvoyeur)+10 && !param1.toInt())
      Maitre = 0;
    if(Extension1==SYS_No_Port_Serie(NoEnvoyeur)+10 && !param2.toInt())
      Extension1 = 0;
    if(Extension2==SYS_No_Port_Serie(NoEnvoyeur)+10 && !param3.toInt())
      Extension2 = 0;

    SYS_Effacer_Commande();
    return;
  }

  // On nous envoie les différentes connexions externes
  if(Commande==F("OBR_EXTERNE_CONNEXIONS"))
  {
    // S'il est vide, on le remplie
    if(!Ordinateur && param1.toInt())
      Ordinateur = SYS_No_Port_Serie(NoEnvoyeur)+10;

    // S'il est remplie mais plus connecté sur la carte externe, on le vide
    if(Ordinateur==SYS_No_Port_Serie(NoEnvoyeur)+10 && !param1.toInt())
      Ordinateur = 0;

    SYS_Effacer_Commande();
    return;
  }

  // On demande à passer en mode de pilotage
  if(Commande==F("OBR_PILOTER"))
  {
    Mode = PILOTAGE;
    Reponse="PILOTAGE";

    // On déconnecte l'admin
    Connecte = 0;
  }

  // On demande à passer en mode de configuration
  if(Commande==F("OBR_CONFIGURER"))
  {
    Mode = CONFIGURATION;
    Reponse="CONFIGURATION";
  }

  // On indique qu'on se déconnecte physiquement
  if(Commande==F("OBR_DECONNECTER"))
  {
    switch(NoEnvoyeur)
    {
      case PC:
      Connecte = 0;
      Ordinateur = 0;
      break;
      
      case 3:
      case 4:
      case 5:
      Robot = 0;
      break;

      default:
      break;
    }
  }

  // On demande le niveau de la batterie
  if(Commande==F("OBR_NIVEAU_BATTERIE"))
  {
    // Retourne le niveau actuel des batteries
    Reponse=String(NiveauBatterieActuel);
  }

  if(Mode==CONFIGURATION)
  {
    // On demande à se connecter en administrateur
    if(Commande==F("OBR_CONNECTER_ADMIN"))
    {
      if(param1.toInt()==Password)
      {
        Connecte = 1;
        Reponse="OK";
      }
      else
        Ecrire_Erreur(130030);
    }

    // On demande à se retirer les droits administrateurs
    if(Commande==F("OBR_DECONNECTER_ADMIN"))
    {
      if(Connecte)
      {
        Connecte=0;
        Reponse="ADMINISTRATEUR_DECONNECTE";
      }
      else
        Ecrire_Erreur(130010);
    }

    // On demande à faire un soft-reboot
    if(Commande==F("OBR_REBOOT"))
    {
      if(Connecte)
      {
        Connecte=0;
        Envoyer_Message(NoEnvoyeur, REPONSE, "REDEMARRAGE...");
        boot();
        return;
      }
      else
        Ecrire_Erreur(130010);
    }
    
    // On demande à lire un octet de l'EEPROM
    if(Commande==F("OBR_LIRE_EEPROM"))
    {
      if(Connecte)
      {
        if(param1.toInt()>15 && param1.toInt()<4096)
          Reponse=String(EEPROM.read(param1.toInt()));
        else
          Ecrire_Erreur(130020);
      }
      else
        Ecrire_Erreur(130010);
    }
    
    // On demande à écrire un octet dans l'EEPROM
    if(Commande==F("OBR_ECRIRE_EEPROM"))
    {
      if(Connecte)
      {
        if(param1.toInt()>50 && param1.toInt()<4096)
        {
          EEPROM.write(param1.toInt(), param2.toInt());
          Reponse=String(EEPROM.read(param1.toInt()));
        }
        else
          Ecrire_Erreur(130020);
      }
      else
        Ecrire_Erreur(130010);
    }
    
    // On demande à changer de mot de passe administrateur
    if(Commande==F("OBR_CHANGER_MOT_DE_PASSE"))
    {
      if(Connecte)
      {
        if(param1.toInt()<16777216)
        {
          // On écrit dans l'EEPROM
          SYS_Ecrire_Mot_De_Passe(param1.toInt());
          Password = SYS_Lire_Mot_De_Passe();
          
          Reponse=String(Password);
        }
        else
          Ecrire_Erreur(110031);
      }
      else
        Ecrire_Erreur(130010);
    }
    
    // On demande la dernière Ecrire_Erreur qu'il y a eu
    if(Commande==F("OBR_DERNIERE_ERREUR"))
    {
      if(!DerniereErreur)
        Reponse="AUCUNE_Erreur";
        
      else
      {
        Reponse=String(DerniereErreur);
        DerniereErreur = 0;
      }
    }

    // On connecte la batterie
    if(Commande==F("OBR_CONNECTER_BATTERIE"))
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
        
        Reponse="OK";
      }
      else
        Ecrire_Erreur(130010);
    }
    
    // On connecte les LED d'indicateur de batterie
    if(Commande==F("OBR_CONNECTER_LED_BATTERIE"))
    {
      if(Connecte)
      {
        if(param1.toInt() == param2.toInt())
        {
          Ecrire_Erreur(110022);
        }
        else
        {
          PIN_BATTERIE_FAIBLE = param1.toInt();
          PIN_BATTERIE_PLEINE = param2.toInt();
          EEPROM.write(11, PIN_BATTERIE_FAIBLE);
          EEPROM.write(12, PIN_BATTERIE_PLEINE);
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Définit les limites d'un capteur
    if(Commande==F("OBR_CONNECTER_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les arguments
        if(param1.toInt()>=NB_PIN_ANALOG || param2.toInt()>255 || param3.toInt()>255 || param2.toInt()>param3.toInt())
          Ecrire_Erreur(110030);
        else
        {
          // On configure les MIN et MAX des capteurs
          Ecrire_RAM(MIN_ANALOG, param1.toInt(), param2.toInt());
          Ecrire_RAM(MAX_ANALOG, param1.toInt(), param3.toInt());

          // On envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Vide les limites d'un capteur
    if(Commande==F("OBR_DECONNECTER_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les arguments
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else
        {
          // On configure les MIN et MAX des capteurs
          Ecrire_RAM(MIN_ANALOG, param1.toInt(), 0);
          Ecrire_RAM(MAX_ANALOG, param1.toInt(), 0);

          // On envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Configure un PIN comme une entrée
    if(Commande==F("OBR_CONNECTER_ENTREE"))
    {
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          // On indique au système que c'est une entrée
          Changer_Mode_PIN_ES(param1.toInt(), ENTREE);

          // On met la valeur à 0
          if(param2.toInt())
            Ecrire_RAM(VALEUR_A_ZERO, param1.toInt(), 1);
          else
            Ecrire_RAM(VALEUR_A_ZERO, param1.toInt(), 0);

          Ecrire_RAM(VALEUR_ACTU_ES, param1.toInt(), 0);

          // On envoie une confirmation
          Reponse="OK";
        }
        else
          Ecrire_Erreur(110031);
      }
      else
        Ecrire_Erreur(130010);
    }

    // déconnecte une entrée
    if(Commande==F("OBR_DECONNECTER_ENTREE"))
    {
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          if(Lire_RAM(TYPE_ES, param1.toInt())==ENTREE)
          {
            // On indique un PIN vide
            Changer_Mode_PIN_ES(param1.toInt(), VIDE);

            // Si c'est le PIN 13, on le rend au système
            if(param1.toInt()==13)
              Changer_Mode_PIN_ES(13, SYSTEME);

            // On envoie une confirmation
            Reponse="OK";
          }
          else
            Ecrire_Erreur(130020);
        }
        else
          Ecrire_Erreur(110031);
      }
      else
        Ecrire_Erreur(130010);
    }

    // Connecte une sortie sur un PIN
    if(Commande==F("OBR_CONNECTER_SORTIE"))
    {
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          // On indique au système que c'est une sortie
          Changer_Mode_PIN_ES(param1.toInt(), SORTIE);

          // On met la valeur à 0
          if(param2.toInt())
            Ecrire_RAM(VALEUR_A_ZERO, param1.toInt(), 1);
          else
            Ecrire_RAM(VALEUR_A_ZERO, param1.toInt(), 0);


          Ecrire_RAM(VALEUR_ACTU_ES, param1.toInt(), 0);

          // On envoie une confirmation
          Reponse="OK";
        }
        else
          Ecrire_Erreur(110031);
      }
      else
        Ecrire_Erreur(130010);
    }

    // Déconnecte une sortie
    if(Commande==F("OBR_DECONNECTER_SORTIE"))
    {
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          if(Lire_RAM(TYPE_ES, param1.toInt())==SORTIE)
          {
            // On indique un PIN vide
            Changer_Mode_PIN_ES(param1.toInt(), VIDE);

            // Si c'est le PIN 13, on le rend au système
            if(param1.toInt()==13)
              Changer_Mode_PIN_ES(13, SYSTEME);

            // On envoie une confirmation
            Reponse="OK";
          }
          else
            Ecrire_Erreur(130020);
        }
        else
          Ecrire_Erreur(110031);
      }
      else
        Ecrire_Erreur(130010);
    }

    // On configure le MIn et MAX d'un capteur
    if(Commande==F("OBR_CONFIG_CAPTEUR_MIN_MAX"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>255)
          Ecrire_Erreur(110032);
        else if(param3.toInt()>255)
          Ecrire_Erreur(110033);
        else if(param2.toInt()>param3.toInt())
          Ecrire_Erreur(110023);

        // On inscrit les valeurs
        else
        {
          // on change les paramètres
          Ecrire_RAM(MIN_ANALOG, param1.toInt(), param2.toInt());
          Ecrire_RAM(MAX_ANALOG, param1.toInt(), param3.toInt());

          // on envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit la commande d'une entrée
    if(Commande==F("OBR_COMMANDE_ES"))
    {
      if(Connecte)
      {
        // On vérifie que c'est bien une entrée/sortie
        if(Lire_RAM(TYPE_ES, param1.toInt())!=ENTREE && Lire_RAM(TYPE_ES, param1.toInt())!=SORTIE && Lire_RAM(TYPE_ES, param1.toInt())!=SERVO)
          Ecrire_Erreur(110021);
        else if(param1.toInt()>=NB_PIN_IO)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>5)
          Ecrire_Erreur(110022);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(COMMANDE_ASSOCIEE, param1.toInt(), param2.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit les paramètres d'une entrée
    if(Commande==F("OBR_PARAMETRES_ENTREE"))
    {
      if(Connecte)
      {
        // On vérifie que c'est bien une entrée
        if(Lire_RAM(TYPE_ES, param1.toInt())!=ENTREE)
          Ecrire_Erreur(110021);
        else if(param1.toInt()>=NB_PIN_IO)
          Ecrire_Erreur(110031);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(NO_PIN_ROBOT, param1.toInt(), param2.toInt());
          Ecrire_RAM(VALEUR_PARAM2, param1.toInt(), param3.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit la commande d'une entrée analog
    if(Commande==F("OBR_COMMANDE_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>5)
          Ecrire_Erreur(110022);
        else if(param3.toInt()>5)
          Ecrire_Erreur(110023);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(COMMANDE_ASSOCIEE, 100+param1.toInt(), param2.toInt());
          Ecrire_RAM(COMMANDE_ASSOCIEE_2, param1.toInt(), param3.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit le type de l'entrée analogique
    if(Commande==F("OBR_TYPE_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>5)
          Ecrire_Erreur(110022);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(TYPE_ANALOG, param1.toInt(), param2.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit sur quel PIN du robot l'analog va travailler
    if(Commande==F("OBR_PIN_ROBOT_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>255)
          Ecrire_Erreur(110022);
        else if(param3.toInt()>255)
          Ecrire_Erreur(110023);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(NO_PIN_ROBOT, 100+param1.toInt(), param2.toInt());
          Ecrire_RAM(NO_PIN_ROBOT_2, param1.toInt(), param3.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit la valeur à 0 de l'analog
    if(Commande==F("OBR_VALEUR_A_ZERO_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>255)
          Ecrire_Erreur(110022);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(VALEUR_A_ZERO_ANALOG, param1.toInt(), param2.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit le PIN sur le robot qui gère le sens de rotation d'un moteur
    if(Commande==F("OBR_PIN_SENS_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>255)
          Ecrire_Erreur(110022);
        else if(param3.toInt()>1)
          Ecrire_Erreur(110023);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(PIN_SENS_MOTEUR, param1.toInt(), param2.toInt());
          Ecrire_RAM(VALEUR_A_ZERO_PIN_SENS, param1.toInt(), param3.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit les MIN et MAX du moteur 1 sur le robot
    if(Commande==F("OBR_MIN_MAX_1_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>255)
          Ecrire_Erreur(110022);
        else if(param3.toInt()>255 || param2.toInt()>param3.toInt())
          Ecrire_Erreur(110023);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(MIN_ROBOT_MOTEUR_1, param1.toInt(), param2.toInt());
          Ecrire_RAM(MAX_ROBOT_MOTEUR_1, param1.toInt(), param3.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit les MIN et MAX du moteur 2 sur le robot
    if(Commande==F("OBR_MIN_MAX_2_ANALOG"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>255)
          Ecrire_Erreur(110022);
        else if(param3.toInt()>255 || param2.toInt()>param3.toInt())
          Ecrire_Erreur(110023);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(MIN_ROBOT_MOTEUR_2, param1.toInt(), param2.toInt());
          Ecrire_RAM(MAX_ROBOT_MOTEUR_2, param1.toInt(), param3.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit les MIN et MAX d'une sortie de la télécommande
    if(Commande==F("OBR_MIN_MAX_SORTIE"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_IO)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>255)
          Ecrire_Erreur(110022);
        else if(param3.toInt()>255 || param2.toInt()>param3.toInt())
          Ecrire_Erreur(110023);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(MIN_ES, param1.toInt(), param2.toInt());
          Ecrire_RAM(MAX_ES, param1.toInt(), param3.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // On définit le type de lien d'une sortie de la télécommande
    if(Commande==F("OBR_TYPE_LIEN_SORTIE"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()>=NB_PIN_ANALOG)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>5)
          Ecrire_Erreur(110022);
        else
        {
          // On enregistre les paramètres
          Ecrire_RAM(TYPE_LIAISON_ES, param1.toInt(), param2.toInt());

          // On envoie une confirmation
          Reponse = "OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    if(Commande==F("OBR_RESET_CONFIGURATION"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(param1.toInt()!=Password)
          Ecrire_Erreur(130030);
        else
        {
          // On envoie une confiramtion
          Envoyer_Message(NoEnvoyeur, REPONSE, "OK");
          SYS_Effacer_Commande();

          // On efface tous les paramètres
          SYS_Remise_A_Zero_Systeme();
          return;
        }
      }
      else
        Ecrire_Erreur(130010);
    }
  }

  if(Mode==PILOTAGE || DEBUG)
  {
    // On récupère la valeur brut du capteur
    if(Commande==F("OBR_VALEUR_BRUT_CAPTEUR"))
    {
      if(param1.toInt() < NB_PIN_ANALOG)
        Reponse=String(Lire_RAM(VALEUR_ACTU_ANALOG, param1.toInt()));
      else
        Ecrire_Erreur(110031);
    }

    // On récupère la valeur brut d'une entrée/Sortie
    if(Commande==F("OBR_VALEUR_BRUT_ES"))
    {
      if(param1.toInt()<NB_PIN_IO)
        Reponse=String(Lire_RAM(VALEUR_ACTU_ES, param1.toInt()));
      else
        Ecrire_Erreur(110031);
    }

    // 
    if(Commande==F("OBR_ALLUMER_SORTIE"))
    {
      // On vérifie que c'est bien une sortie digitale
      if(Lire_RAM(TYPE_ES, param1.toInt())!=SORTIE || param1.toInt()>NB_PIN_IO)
        Ecrire_Erreur(110021);
      else
      {
        Ecrire_RAM(VALEUR_ACTU_ES, param1.toInt(), 1);
        Reponse="OK";
      }
    }

    // 
    if(Commande==F("OBR_ARRETER_SORTIE"))
    {
      // On vérifie que c'est bien une sortie digitale
      if(Lire_RAM(TYPE_ES, param1.toInt())!=SORTIE || param1.toInt()>NB_PIN_IO)
        Ecrire_Erreur(110021);
      else
      {
        Ecrire_RAM(VALEUR_ACTU_ES, param1.toInt(), 0);
        Reponse="OK";
      }
    }
  }

  // On envoie la réponse
  if(Reponse!="")
    Envoyer_Message(NoEnvoyeur, REPONSE, Reponse);

  // Effacement de la chaine
  SYS_Effacer_Commande();
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
  ChaineReponse.reserve(32);
  
  // Initialisation de l'EEPROM
  PIN_BATTERIE = EEPROM.read(10);
  PIN_BATTERIE_FAIBLE = EEPROM.read(11);
  PIN_BATTERIE_PLEINE = EEPROM.read(12);
  NiveauBatterieMin = EEPROM.read(13)*4;
  NiveauBatterieMax = EEPROM.read(14)*4;
  
  // Lecture du mot de passe
  //Password = SYS_Lire_Mot_De_Passe();
  if(Password==0)
    Password=2560;
  // Possibilité à l'utilisateur de réinitialiser le mot de passe
  Changer_Mode_PIN_ES(13, SYSTEME);
  digitalWrite(13, LOW);
  pinMode(13, INPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  if(digitalRead(13))
    SYS_Remise_A_Zero_Systeme();

  digitalWrite(12, LOW);
  Changer_Mode_PIN_ES(12, VIDE);

  // Le reste de l'initialisation se fait dans une fonction externe, qui peut être appelée pour faire un soft-reboot.
  boot();
}

// Fonction de démarrage, peut être utilisée pour faire un soft-reboot
void boot()
{
  // On vide la RAM
  // Ports séries
  Ordinateur = 0; // N° du port série sur lequel est connecté le PC (à partir du port 1)
  Robot = 0;
  No_Carte_Robot = 0; // Différent si esclave d'une extension
  DerniereActuConnexions = 0; // Temps depuis la derniere actualisation des connexions
  Maitre = 0;
  Extension1 = 0;
  Extension2 = 0;
  // Reception d'une commande
  Commande = ""; // Dernière commande reçue 
  param1 = ""; // Paramètres de la commande
  param2 = "";
  param3 = "";
  ChaineReponse = "";
  stringComplete = false; // Indique si une commande est arrivée
  NbParam = 0; // Nombre de paramètres dans la commande
  NoEnvoyeur = 0; // Adresse de l'envoyeur du dernier message
  NoDestinataire = 0; // Adresse du destinataire du dernier message
  NbActualisation = -1; // limite les envoie de demande de connexion
  ReponseRecue = 1; // Indique si on peut envoyer une nouvelle commande ou si on attend encore une réponse
  // Chien de garde sur les ports série
  watchdogActif = 1;
  DernierTempsPC = 0; // Temps en 1/10 de seconde depuis la dernière réception
  DernierTempsRobot = 0;
  DernierTempsWatchdog = 0; // Temps en ms de la dernière actualisation du chien de garde
  // Administration
  Connecte = 0; // Indique si on est connecté en administrateur (accès à certaines commandes)
  Mode = PILOTAGE; // définit le mode en cours (configuration ou pilotage)
  // Batterie
  PIN_BATTERIE = 0; // PIN sur lequel est connecté la batterie
  PIN_BATTERIE_FAIBLE = 0; // PIN sur lequel est connecté la LED de batterie faible
  PIN_BATTERIE_PLEINE = 0; // Pareil, pour la batterie pleine
  NiveauBatterieActuel = 0; // Indique le niveau actuel des batteries, en pourcentage
  NiveauBatterieMin = 0; // Le niveau correspondant à 0%
  NiveauBatterieMax = 1024; // Le niveau correspondant à 100%
  // Gestion d'Ecrire_Erreur
  DerniereErreur = 0;
  NombreErreur = 0;

  // On initialise les tableaux
  SYS_Initialiser_Tableaux_RAM();

  // Arrêt de la LED 13
  Changer_Mode_PIN_ES(13, SYSTEME);
  Ecrire_RAM(VALEUR_ACTU_ES, 13, 0);

  if(Lire_RAM(TYPE_ES, 13)==VIDE)
    Changer_Mode_PIN_ES(13, SYSTEME);

  // On autorise l'utilisateur à exécuter son propre code
  user_setup();
}

// Fonctions principales
// Boucle principale
void loop()
{
  // On vérifie les connexions
  SYS_Verification_Connexions();

  // On vérifie l'arrivée d'une commande
  Verifier_Reception_Commande();
  
  // On met à jour le niveau de batterie
  SYS_Mise_A_Jour_Batterie();
  
  // On met à jour les états des entrées/sorties et capteurs
  SYS_Mise_A_Jour_RAM();

  // On autorise l'utilisateur à exécuter son propre code
  user_loop();
}

// Fonction de gestion des Ecrire_Erreurs
void Ecrire_Erreur(unsigned long NoEcrire_Erreur)
{
  DerniereErreur = NoEcrire_Erreur;
  Reponse="Erreur_"+String(NoEcrire_Erreur);
}

// Fonction qui retourne la dernière Erreur
unsigned long Lire_DerniereErreur()
{
  // On retourne la dernière Ecrire_Erreur
  return DerniereErreur;
}

// Fonction de mise à jour du niveau de batterie (en %)
void SYS_Mise_A_Jour_Batterie()
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
void SYS_Initialiser_Tableaux_RAM()
{
  byte i = 0;
  // Valeur actu IO
  for(i=0; i<NB_PIN_IO; i++)
  {
    Valeur_Actu_IO[i]=0; // Tout à 0
    switch(Lire_RAM(TYPE_ES, i))
    {
      case SORTIE:
      case SERVO:
      pinMode(i, OUTPUT);
      break;

      case ENTREE:
      case SYSTEME:
      pinMode(i, INPUT);
      break;

      default:
      break;
    }
  }

  // Valeur actu Analog
  for(i=0; i<NB_PIN_ANALOG; i++)
    Valeur_Actu_Analog[i]=0; // Tout à 0
}

// Fonction de mise à jour en RAM
void SYS_Mise_A_Jour_RAM()
{
  // On met à jour le tableau des capteurs et E/S
  SYS_Verification_Changement_Etat_PIN();

  // On appel la fonction de mise à jour de l'état des sorties
  SYS_Mise_A_Jour_Des_ES();
}

// Fonction d'écriture en RAM
void Ecrire_RAM(byte Type, byte NumeroCase, byte Valeur)
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
      EEPROM.write((100+(3*NB_PIN_ANALOG)+(7*NB_PIN_IO)+(NumeroCase*11)), Valeur);
    break;

    case MAX_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((101+(3*NB_PIN_ANALOG)+(7*NB_PIN_IO)+(NumeroCase*11)), Valeur);
    break;

    case TYPE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      EEPROM.write((100+(NumeroCase*3)), Valeur);
    break;

    case VALEUR_A_ZERO:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      EEPROM.write((100+(3*(NB_PIN_ANALOG+NB_PIN_IO))+(NumeroCase*4)), Valeur);
    break;

    case VALEUR_A_ZERO_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((102+(3*NB_PIN_ANALOG+7*NB_PIN_IO)+(NumeroCase*11)), Valeur);
    break;
    
    case COMMANDE_ASSOCIEE:
    if((NumeroCase<NB_PIN_IO && NumeroCase>=0) || (NumeroCase-100<NB_PIN_ANALOG && NumeroCase>=100))
      if(NumeroCase<100)
        EEPROM.write((101+(NumeroCase*3)), Valeur);
      else
        EEPROM.write((101+(3*NB_PIN_IO)+((NumeroCase-100)*3)), Valeur);
    break;
    
    case NO_PIN_ROBOT:
    if((NumeroCase<NB_PIN_IO && NumeroCase>=0) || (NumeroCase-100<NB_PIN_ANALOG && NumeroCase>=100))
      if(NumeroCase<100)
        EEPROM.write((102+(NumeroCase*3)), Valeur);
      else
        EEPROM.write((102+(3*NB_PIN_IO)+((NumeroCase-100)*3)), Valeur);
    break;
    
    case MIN_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      EEPROM.write((101+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(NumeroCase*4)), Valeur);
    break;
    
    case MAX_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      EEPROM.write((102+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(NumeroCase*4)), Valeur);
    break;
    
    case TYPE_LIAISON_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      EEPROM.write((103+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(NumeroCase*4)), Valeur);
    break;
    
    case VALEUR_PARAM2:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      EEPROM.write((101+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(NumeroCase*4)), Valeur);
    break;
    
    case PIN_SENS_MOTEUR:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((103+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11)), Valeur);
    break;
    
    case VALEUR_A_ZERO_PIN_SENS:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((106+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11)), Valeur);
    break;
    
    case TYPE_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((100+(3*NB_PIN_IO)+(NumeroCase*3)), Valeur);
    break;
    
    case MIN_ROBOT_MOTEUR_1:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((107+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11)), Valeur);
    break;
    
    case MAX_ROBOT_MOTEUR_1:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((108+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11)), Valeur);
    break;
    
    case MIN_ROBOT_MOTEUR_2:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((109+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11)), Valeur);
    break;
    
    case MAX_ROBOT_MOTEUR_2:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((110+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11)), Valeur);
    break;
    
    case COMMANDE_ASSOCIEE_2:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((104+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11)), Valeur);
    break;
    
    case NO_PIN_ROBOT_2:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      EEPROM.write((105+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11)), Valeur);
    break;

    default:
    break;
  }
}

// Fonction de lecture en RAM
byte Lire_RAM(byte Type, byte NumeroCase)
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
      return EEPROM.read(100+(3*NB_PIN_ANALOG)+(7*NB_PIN_IO)+(NumeroCase*11));
    break;

    case MAX_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(101+(3*NB_PIN_ANALOG)+(7*NB_PIN_IO)+(NumeroCase*11));
    break;

    case TYPE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
       return EEPROM.read(100+(NumeroCase*3));
    break;

    case VALEUR_A_ZERO:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return EEPROM.read(100+(3*(NB_PIN_ANALOG+NB_PIN_IO))+(NumeroCase*4));
    break;

    case VALEUR_A_ZERO_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(102+(3*NB_PIN_ANALOG)+(7*NB_PIN_IO)+(NumeroCase*11));
    break;
    
    case COMMANDE_ASSOCIEE:
    if((NumeroCase<NB_PIN_IO && NumeroCase>=0) || (NumeroCase-100<NB_PIN_ANALOG && NumeroCase>=100))
      if(NumeroCase<100)
        return EEPROM.read(101+(NumeroCase*3));
      else
        return EEPROM.read(101+(3*NB_PIN_IO)+((NumeroCase-100)*3));
    break;
    
    case NO_PIN_ROBOT:
    if((NumeroCase<NB_PIN_IO && NumeroCase>=0) || (NumeroCase-100<NB_PIN_ANALOG && NumeroCase>=100))
      if(NumeroCase<100)
        return EEPROM.read(102+(NumeroCase*3));
      else
        return EEPROM.read(102+(3*NB_PIN_IO)+((NumeroCase-100)*3));
    break;
    
    case MIN_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return EEPROM.read(101+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(NumeroCase*4));
    break;
    
    case MAX_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return EEPROM.read(102+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(NumeroCase*4));
    break;
    
    case TYPE_LIAISON_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return EEPROM.read(103+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(NumeroCase*4));
    break;
    
    case VALEUR_PARAM2:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return EEPROM.read(101+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(NumeroCase*4));
    break;
    
    case PIN_SENS_MOTEUR:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(103+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11));
    break;
    
    case VALEUR_A_ZERO_PIN_SENS:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(106+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11));
    break;
    
    case TYPE_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(100+(3*NB_PIN_IO)+(NumeroCase*3));
    break;
    
    case MIN_ROBOT_MOTEUR_1:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(107+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11));
    break;
    
    case MAX_ROBOT_MOTEUR_1:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(108+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11));
    break;
    
    case MIN_ROBOT_MOTEUR_2:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(109+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11));
    break;
    
    case MAX_ROBOT_MOTEUR_2:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(110+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11));
    break;
    
    case COMMANDE_ASSOCIEE_2:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(104+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11));
    break;
    
    case NO_PIN_ROBOT_2:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return EEPROM.read(105+(7*NB_PIN_IO+3*NB_PIN_ANALOG)+(NumeroCase*11));
    break;

    default:
    return 0;
    break;
  }
  return 0;
}

// Fonctions de controle du robot
// Fonction d'envoie d'une commande au robot
byte Envoyer_Commande(byte No_Commande, byte No_PIN, byte Param2)
{
  unsigned long TimeReponse = 0;

  #if(DEBUG)
    Serial.print("Commande envoyee : ");
    Serial.println(No_Commande);
  #endif

  switch(No_Commande)
  {
    case 1: // Allumer_Sortie
    Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_ALLUMER_SORTIE-"+String(No_PIN));
    TimeReponse = millis();
    ReponseRecue = 0;
    while(TimeReponse > millis()-25 && !ReponseRecue)
      SYS_Recuperer_Message(Robot);
    ReponseRecue = 1;
    break;

    case 2: // Arreter_sortie
    Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_ARRETER_SORTIE-"+String(No_PIN));
    TimeReponse = millis();
    ReponseRecue = 0;
    while(TimeReponse > millis()-25 && !ReponseRecue)
      SYS_Recuperer_Message(Robot);
    ReponseRecue = 1;
    break;

    case 3: // Vitesse_Moteur
    Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_VITESSE_MOTEUR-"+String(No_PIN)+'-'+String(Param2));
    TimeReponse = millis();
    ReponseRecue = 0;
    while(TimeReponse > millis()-25 && !ReponseRecue)
      SYS_Recuperer_Message(Robot);
    ReponseRecue = 1;
    break;

    case 4: // Arreter_moteur
    Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_ARRETER_MOTEUR-"+String(No_PIN));
    TimeReponse = millis();
    ReponseRecue = 0;
    while(TimeReponse > millis()-25 && !ReponseRecue)
      SYS_Recuperer_Message(Robot);
    ReponseRecue = 1;
    break;

    case 5: // Position_Servo
    Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_POSITION_SERVO-"+String(No_PIN)+'-'+String(Param2));
    TimeReponse = millis();
    ReponseRecue = 0;
    while(TimeReponse > millis()-25 && !ReponseRecue)
      SYS_Recuperer_Message(Robot);
    ReponseRecue = 1;
    break;

    case 6: // Valeur_Analog
    Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_VALEUR_BRUT_CAPTEUR-"+String(No_PIN));
    // On récupère la valeur
    TimeReponse = millis();
    ReponseRecue = 0;
    while(TimeReponse > millis()-25 && !ReponseRecue)
      SYS_Recuperer_Message(Robot);
    ReponseRecue = 1;
    // On retourne la valeur
    return ChaineReponse.toInt();

    case 7: // Valeur_E/S
    Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_VALEUR_BRUT_ES-"+String(No_PIN));
    // On récupère la valeur
    TimeReponse = millis();
    ReponseRecue = 0;
    while(TimeReponse > millis()-25 && !ReponseRecue)
      SYS_Recuperer_Message(Robot);
    ReponseRecue = 1;
    // On retourne la valeur
    return ChaineReponse.toInt();

    default:
    break;
  }
  return 0;
}

// Fonction qui envoie des commandes selon les changements d'états des entrées (IO et Analog)
void SYS_Verification_Changement_Etat_PIN()
{
  byte i=0;

  // On vérifie les PIN Analog
  for(i=0; i<NB_PIN_ANALOG; i++)
  {
    byte Valeur = Lire_RAM(VALEUR_ACTU_ANALOG, i);

    // Si la valeur a changée, on envoie la commande vers le robot
    if(analogRead(i)/4 < Valeur-3 || analogRead(i)/4 > Valeur+3)
    {
      Ecrire_RAM(VALEUR_ACTU_ANALOG, i, analogRead(i)/4);
      SYS_Traitement_Envoie_Commande_PIN(i, 1);
    }
  }

  // On vérifie les entrées E/S
  for(i=0; i<NB_PIN_IO; i++)
  {
    byte Valeur = Lire_RAM(VALEUR_ACTU_ES, i);

    if(Lire_RAM(VALEUR_A_ZERO, i))
      Valeur = !Valeur;

    // Si la valeur a changée, on envoie la commande vers le robot
    if(Lire_RAM(TYPE_ES, i)==ENTREE)
      if(digitalRead(i) != Valeur)
      {
        if(Lire_RAM(VALEUR_A_ZERO, i) == digitalRead(i))
          Ecrire_RAM(VALEUR_ACTU_ES, i, 0);
        else
          Ecrire_RAM(VALEUR_ACTU_ES, i, 1);

        #if(DEBUG)
          Serial.print("Changement état PIN ");
          Serial.println(i);
        #endif

        SYS_Traitement_Envoie_Commande_PIN(i, 0);
      }
    if(Lire_RAM(TYPE_ES, i)==SORTIE || Lire_RAM(TYPE_ES, i)==SERVO)
    {
      SYS_Mise_A_Jour_Sorties(i);
    }
  }
}

// Fonction de traitement des commandes à envoyer
void SYS_Traitement_Envoie_Commande_PIN(byte PIN, boolean Analog)
{
  byte Commande = Lire_RAM(COMMANDE_ASSOCIEE, (Analog*100)+PIN);

  // On vérifie que la commande est bien une commande de controle
  if(Commande > 5 || Commande == 0)
    return;

  #if(DEBUG)
    Serial.print("Commande associee : ");
    Serial.println(Commande);
  #endif

  // Si c'est une commande analog, on regarde le type de commande à envoyer, et on fait les calculs
  if(Analog)
  {
    boolean Sens = 0;
    byte Valeur_A_Envoyer = SYS_Calcul_Commande_Analog(PIN, &Sens);

    // On envoie la (les) commande(s)
    if(Lire_RAM(TYPE_ANALOG, PIN)==2 || Lire_RAM(TYPE_ANALOG, PIN)==3)
    {
      if(Sens!=Lire_RAM(VALEUR_A_ZERO_PIN_SENS, PIN))
      {
        if(Lire_RAM(PIN_SENS_MOTEUR, PIN)!=255)
        {
          Envoyer_Commande(1, Lire_RAM(PIN_SENS_MOTEUR, PIN), 0);
          delay(10);
        }
        Envoyer_Commande(Commande, Lire_RAM(NO_PIN_ROBOT_2, PIN), Valeur_A_Envoyer);
      }
      else
      {
        if(Lire_RAM(PIN_SENS_MOTEUR, PIN)!=255)
        {
          Envoyer_Commande(2, Lire_RAM(PIN_SENS_MOTEUR, PIN), 0);
          delay(10);
        }
        Envoyer_Commande(Commande, Lire_RAM(NO_PIN_ROBOT, 100+PIN), Valeur_A_Envoyer);
      }
    }
    else
    {
      Envoyer_Commande(Commande, Lire_RAM(NO_PIN_ROBOT, 100+PIN), Valeur_A_Envoyer);
    }
  }

  // On envoie la commande
  else
    // Si c'est un appuie, on envoie la commande
    if(Lire_RAM(VALEUR_ACTU_ES, PIN))
      Envoyer_Commande(Commande, Lire_RAM(NO_PIN_ROBOT, PIN), Lire_RAM(VALEUR_PARAM2, PIN));
    // Sinon, on envoie la commande inverse
    else if(Commande<4 && Commande>0)
      Envoyer_Commande(Commande+1, Lire_RAM(NO_PIN_ROBOT, PIN), Lire_RAM(VALEUR_PARAM2, PIN));
}

// Commande qui retourne la vitesse du moteur selon l'analog, et inscrit son sens dans un booléen
byte SYS_Calcul_Commande_Analog(byte PIN_Analog, boolean* Sens)
{
  // On vérifie que le PIN analog est connecté à qque chose
  if(Lire_RAM(TYPE_ANALOG, PIN_Analog) == 0)
    return 0;

  // On récupère les données
  byte Valeur_Analog = Lire_RAM(VALEUR_ACTU_ANALOG, PIN_Analog);
  byte Min_Analog = Lire_RAM(MIN_ANALOG, PIN_Analog);
  byte Max_Analog = Lire_RAM(MAX_ANALOG, PIN_Analog);
  byte Valeur_A_Zero_Analog = Lire_RAM(VALEUR_A_ZERO_ANALOG, PIN_Analog);
  byte Min_Moteur1 = Lire_RAM(MIN_ROBOT_MOTEUR_1, PIN_Analog);
  byte Max_Moteur1 = Lire_RAM(MAX_ROBOT_MOTEUR_1, PIN_Analog);
  byte Min_Moteur2 = Lire_RAM(MIN_ROBOT_MOTEUR_2, PIN_Analog);
  byte Max_Moteur2 = Lire_RAM(MAX_ROBOT_MOTEUR_2, PIN_Analog);

  // Selon le type de lien, on fait différents calculs
  switch(Lire_RAM(TYPE_ANALOG, PIN_Analog))
  {
    case 1: //Sens 0, De Min à Max
    *Sens = 0;
    Valeur_Analog = map(Valeur_Analog, Min_Analog, Max_Analog, Min_Moteur1, Max_Moteur1);
    break;

    case 2: // Min->0, Sens = 0; 0->Max, Sens = 1;
    if(Valeur_Analog<=Valeur_A_Zero_Analog)
    {
      *Sens = 0;
      Valeur_Analog = map(Valeur_Analog, Valeur_A_Zero_Analog, Min_Analog, Min_Moteur1, Max_Moteur1);
    }
    else
    {
      *Sens = 1;
      Valeur_Analog = map(Valeur_Analog, Valeur_A_Zero_Analog, Max_Analog, Min_Moteur2, Max_Moteur2);
    }
    break;

    case 3: // Min->0, Sens = 1; 0->Max, Sens = 0;
    if(Valeur_Analog<=Valeur_A_Zero_Analog)
    {
      *Sens = 1;
      Valeur_Analog = map(Valeur_Analog, Valeur_A_Zero_Analog, Min_Analog, Min_Moteur2, Max_Moteur2);
    }
    else
    {
      *Sens = 0;
      Valeur_Analog = map(Valeur_Analog, Valeur_A_Zero_Analog, Max_Analog, Min_Moteur1, Max_Moteur1);
    }
    break;

    case 4: // Sens = 0; <MIN ou >MAX, Vitesse=0
    *Sens = 0;
    if(Valeur_Analog<Min_Analog || Valeur_Analog>Max_Analog)
      Valeur_Analog = 0;
    else
      Valeur_Analog = map(Valeur_Analog, Min_Analog, Max_Analog, Min_Moteur1, Max_Moteur1);
    break;

    case 5: // Sens = 0; >MIN ou <MAX, Vitesse=0
    *Sens = 0;
    if(Valeur_Analog<Max_Analog)
      Valeur_Analog = map(Valeur_Analog, Min_Analog, 0, Min_Moteur1, Max_Moteur1);
    else if(Valeur_Analog>Min_Analog)
      Valeur_Analog = map(Valeur_Analog, Max_Analog, 255, Min_Moteur1, Max_Moteur1);
    else
      Valeur_Analog = 0;
    break;

    default:
    *Sens = 0;
    return 0;
    break;
  }

  return Valeur_Analog;
}

// Fonction de traitement des sorties selon les PIN du Robot
void SYS_Mise_A_Jour_Sorties(byte PIN)
{
  byte Commande = 0;

  // On vérifie que la commande est bien une commande d'actualisation
  if(Commande = Lire_RAM(COMMANDE_ASSOCIEE, PIN)<6)
    return;

  // On vérifie que le PIN est bien une sortie / Servo
  if(Lire_RAM(TYPE_ES, PIN)!=SORTIE && Lire_RAM(TYPE_ES, PIN)!=SERVO)
    return;

  // On demande la valeur brut sur le robot
  byte Retour = Envoyer_Commande(Commande, PIN, 0);

  // On met à jour la sortie/le servo selon la réponse
  Ecrire_RAM(VALEUR_ACTU_ES, PIN, Retour);
}

// Fonction de mise à jour des sorties, selon la RAM
void SYS_Mise_A_Jour_Des_ES()
{
  byte i = 0;

  // On lit le tableau des sorties, on regarde si on doit activer une sortie, et on l'active
  for(i=0; i<NB_PIN_IO; i++)
  {
    // On regarde si c'est une sortie ou un servomoteur
    switch(Lire_RAM(TYPE_ES, i))
    {
      case SORTIE:
      digitalWrite(i, Lire_RAM(VALEUR_ACTU_ES, i));
      break;

      case SERVO:
      break;

      // Si c'est un PIN vide, on le met à 0
      case VIDE:
      digitalWrite(i, 0);
      break;

      default:
      break;
    }
  }

  // Gestion différente pour le PIN 13, s'il est utilisé par le système
  if(Lire_RAM(TYPE_ES, 13)==SYSTEME || Lire_RAM(TYPE_ES, 13)==VIDE)
  {
    if(Connecte)
      digitalWrite(13, HIGH);
    else
      digitalWrite(13, LOW);
  }
}

// Fonction de changement de mode d'une E/S
void Changer_Mode_PIN_ES(byte PIN, byte Mode)
{
  // On vérifie que le PIN existe
  if(PIN<0 || PIN>=NB_PIN_IO)
    return;

  // On vérifie que le mode existe
  if(Mode<0 || Mode>4)
    return;

  // On met à jour les variables
  Ecrire_RAM(TYPE_ES, PIN, Mode);
  Ecrire_RAM(VALEUR_ACTU_ES, PIN, 0);

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
void SYS_Remise_A_Zero_Systeme()
{
  digitalWrite(12, LOW);
  pinMode(12, INPUT);
  pinMode(13, OUTPUT);

  // On vide l'EEPROM
  unsigned long i = 0;
  for(i=100; i<=(100+(3*(NB_PIN_IO+NB_PIN_ANALOG))+(4*NB_PIN_IO)+(11*NB_PIN_ANALOG)); i++)
  {
    EEPROM.write(i, 0);
    delay(10);
  }

  // On réinititalise le mot de passe à 2560
  Password = 2560;
  // On écrit dans l'EEPROM
  SYS_Ecrire_Mot_De_Passe(2560);

  // On fait clignoter la LED 3x
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(300);
  digitalWrite(13, HIGH);
  delay(300);
  digitalWrite(13, LOW);
  delay(300);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(300);
}

// Fonction de lecture du mot de passe depuis l'EEPROM
unsigned long SYS_Lire_Mot_De_Passe()
{
  unsigned long Pass = EEPROM.read(6);
  Pass = (Pass*256)+EEPROM.read(7);
  Pass = (Pass*256)+EEPROM.read(8);

  return Pass;
}

// Fonction d'écriture du mot de passe sur l'EEPROM
void SYS_Ecrire_Mot_De_Passe(unsigned long Password)
{
  if(Connecte)
  {
    digitalWrite(13, LOW);
    
    unsigned char Pass1 = (Password/256)/256;
    unsigned char Pass2 = (Password-((Pass1*256)*256))/256;
    unsigned char Pass3 = Password-(Pass2*256)-((Pass1*256)*256);
    
    EEPROM.write(6, Pass1);
    EEPROM.write(7, Pass2);
    EEPROM.write(8, Pass3);
  
    digitalWrite(13, HIGH);
  }
}

// Fonctions de gestion des commandes
// Fonction de récupération de la commande
void Verifier_Reception_Commande()
{
  // On scan tous les ports
  SYS_Recuperer_Message(1);

  #if(NB_PORT_SERIE>1)
    SYS_Recuperer_Message(2);
    SYS_Recuperer_Message(3);
    SYS_Recuperer_Message(4);
  #endif
}

// Fonction de récupération du port série
void SYS_Recuperer_Message(byte NoPort)
{
  // On vérifie que le port série existe sur cette carte
  if(NoPort<=0 || NoPort>NB_PORT_SERIE)
   return;

  SYS_Effacer_Commande();

  // On récupère les données jusqu'à ce qu'on ai le '\n' ou jusqu'à ce que le temps soit écoulé
  // S'il n'y a déjà pas de donnée au départ, on sort tout de suite. Sinon, on lance le compte à rebourd à 50ms (une commande met au max 17.78 ms à arriver)
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
        // Si le watchdog est activé, on remet le compteur à 0
        if(watchdogActif)
        {
          switch(NoEnvoyeur)
          {
            case PC:
            if(Ordinateur<10)
              DernierTempsPC = 0;
            break;

            case 3:
            case 4:
            case 5:
            if(Robot<10)
              DernierTempsRobot = 0;
            break;

            default:
            break;
          }
        }

        // Si c'est un simple message de présence (watchdog), on annule le message
        if(Commande=="=" || Commande=="")
        {
          SYS_Effacer_Commande();
          return;
        }

        // Si c'est en mode de déboguage, on envoie la chaine récupérée, sur le port USB
        #if (DEBUG)
          Serial.print("Recoit <== ");
          Serial.print(NoPort);
          Serial.print(":\\ [");
          Serial.print(NoEnvoyeur);
          Serial.print("=>");
          Serial.print(NoDestinataire);
          Serial.print("|");
          Serial.print(ID);
          Serial.print("]");
          Serial.print(Commande);
          Serial.print(' ');
          Serial.print(param1);
          Serial.print(' ');
          Serial.print(param2);
          Serial.print(' ');
          Serial.print(param3);
          Serial.print("\n");
        #endif

        // Si le message est à relayer, on le transmet, et on efface la chaine
        if(NoDestinataire != 2 && NoEnvoyeur != 0)
        {
          // On reconstitue la chaine
          Commande = '['+String(NoEnvoyeur)+String(NoDestinataire)+String(ID)+']'+String(Commande);
          if(NbParam>0)
            Commande = Commande + '-' + String(param1);
          if(NbParam>1)
            Commande = Commande + '-' + String(param2);
          if(NbParam>2)
            Commande = Commande + '-' + String(param3);

          // On l'envoie
          SYS_Transferer_Message(NoDestinataire, Commande);

          SYS_Effacer_Commande();
          return;
        }

        // On indique qu'une chaine est disponible
        stringComplete = true;

        // Selon le type du message, on appelle la bonne fonction
        switch(ID)
        {
          case 0: // Watchdog
          break;

          case 1: // Commande
          Executer_Commande_Recue();
          break;

          case 2: // Réponse
          ReponseRecue = 1;
          ChaineReponse = Commande;
          break;

          case 3: // Demande de connexion
          SYS_Reception_Demande_Connexion(NoPort);
          break;

          default:
          if(NoEnvoyeur!=0)
            Executer_Commande_Recue();
          break;
        }

        return;
      }

      // Si c'est le premier '[', on indique que le caractère suivant est le N° de l'envoyeur
      if(inChar == '[' && NoEnvoyeur == 0)
        NbParam = 10;

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
        NbParam = 11;
        break;

        case 11:
        NoDestinataire = String(inChar).toInt();
        NbParam = 12;
        break;

        case 12:
        ID = String(inChar).toInt();
        NbParam = 0;
        break;
        
        default:
        inChar = 255;
      }
    }
  }
}

// Fonction d'envoie d'une chaine de caractère via le port série
void Envoyer_Message(byte Destinataire, byte Type, String Message)
{
  // On vérifie la présence d'un message
  if(Message=="")
    return;

  // On ajoute les adresses
  Message = "[2"+String(Destinataire)+String(Type)+"]"+String(Message);

  // On envoie le message
  SYS_Transferer_Message(Destinataire, Message);
}

// Fonction d'envoie d'un message, ayant déjà une adresse
void SYS_Transferer_Message(byte Type, String Message)
{
  // On détermine le N° du serial sur lequel envoyer
  byte Port = SYS_No_Port_Serie(Type);

  if(Port>NB_PORT_SERIE || Port<=0)
    return;

  #if (DEBUG)
    if(Message!="")
    {
      Serial.print("Envoie ==> ");
      Serial.print(Port);
      Serial.print(":\\ ");
      Serial.print(Message);
      Serial.print("\n");
    }
  #endif

  // On envoie sur le bon port série
  switch(Port)
  {
    case 1:
    Serial.print(Message);
    Serial.print("\n");
    break;

    #if(NB_PORT_SERIE>1)
    case 2:
    Serial1.print(Message);
    Serial1.print("\n");
    break;
    #endif

    #if(NB_PORT_SERIE>1)
    case 3:
    Serial2.print(Message);
    Serial2.print("\n");
    break;
    #endif

    #if(NB_PORT_SERIE>1)
    case 4:
    Serial3.print(Message);
    Serial3.print("\n");
    break;
    #endif

    default:
    break;
  }
}

// Fonction de détermination des emplacements réseaux
void SYS_Verification_Connexions()
{
  // Si ça fait moins de deux secondes qu'on a actualisé, on retourne
  if(DerniereActuConnexions+200 > millis())
    return;

  DerniereActuConnexions += 200;

  // On vérifie que les ports ouverts sont toujours en activité (watchdog)
  SYS_WatchDog_Serie();

  // On envoie une demande de connexion
  if(Robot == 0)
    SYS_Demande_Connexion();

  // On demande à chaque carte quelles sont leurs connexions
  switch(dernierCycleConnexions)
  {
    case 1: // Maitre
    if(Robot<10 && Robot>0 && No_Carte_Robot==3)
    {
      Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_CONNEXIONS_ROBOT");
      Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_CONNEXIONS_EXTERNE");
    }
    break;

    case 2: // E1
    if(Robot<10 && Robot>0 && No_Carte_Robot==4)
    {
      Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_CONNEXIONS_ROBOT");
      Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_CONNEXIONS_EXTERNE");
    }
    break;

    case 3: // E2
    if(Robot<10 && Robot>0 && No_Carte_Robot==5)
    {
      Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_CONNEXIONS_ROBOT");
      Envoyer_Message(No_Carte_Robot, COMMANDE, "OBR_CONNEXIONS_EXTERNE");
    }
    break;

    default: // Retour au début
    dernierCycleConnexions = 0;
    break;
  }
  dernierCycleConnexions++;
}

// Chien de garde pour la déconnexion automatique des ports séries
void SYS_WatchDog_Serie()
{
  // Si ça fait plus de 0.3 secondes qu'on a pas envoyé un signal au chien de garde des autres cartes, on l'envoie
  if(DernierTempsRobot_Envoie > 30 && Robot<10)
  {
    DernierTempsRobot_Envoie = 0;
    Envoyer_Message(No_Carte_Robot, WATCHDOG, "=");
  }

  // On met à jour la durée de tous les ports
  byte DifferenceTempsActu = (millis()-DernierTempsWatchdog)/10;
  DernierTempsWatchdog = millis();

  DernierTempsPC += DifferenceTempsActu;
  DernierTempsRobot += DifferenceTempsActu;
  DernierTempsRobot_Envoie += DifferenceTempsActu;

  // On vérifie que le chien de garde est activé
  if(!watchdogActif)
    return;

  // On vérifie que ça fait moins de 1 seconde pour tout le monde
  if(DernierTempsPC > 100 && Ordinateur<10)
  {
    // On déconnecte l'admin, et le PC
    Connecte = 0;
    digitalWrite(13, LOW);

    // On déconnecte toutes les connexions qui passaient par cette carte là
    if(Robot-10==Ordinateur)
      Extension1=0;

    // On déconnecte la carte
    Ordinateur = 0;
  }

  if(DernierTempsRobot > 100 && Robot<10)
  {
    // On déconnecte toutes les connexions qui passaient par cette carte là
    if(Ordinateur-10==Robot)
      Ordinateur=0;

    // On déconnecte la carte
    Robot = 0;
  }
}

// Fonction de vérification d'une nouvelle connexion au port série (envoie un message sur le réseau quand un nouveau périphérique se connecte)
void SYS_Reception_Demande_Connexion(byte NoPort)
{
  // On regarde qui c'est, et on lui confirme la connexion si possible
  if(stringComplete)
  {
    #if(DEBUG)
      if(Commande=="OBR_CONNECTER_PC")
        Serial.print("Demande de connexion du PC\n");

      if(Commande=="OBR_CONNECTER_TELECOMMANDE")
        Serial.print("Demande de connexion de la telecommande (Refusee)\n");

      if(Commande=="OBR_CONNECTER_EXTENSION1")
        Serial.print("Demande de connexion de l'extension1\n");

      if(Commande=="OBR_CONNECTER_EXTENSION2")
        Serial.print("Demande de connexion de l'extension2\n");
    #endif

    // On regarde qui c'est, on remplie la variable, et on envoie une confirmation
    if(Commande=="OBR_CONNECTER_PC" && !Ordinateur)
    {
      Ordinateur = NoPort;
      Envoyer_Message(PC, 3, "OK");
      DernierTempsPC = 0;
    }

    else if(Commande=="OBR_CONNECTER_EXTENSION1" && !Robot)
    {
      Robot = NoPort;
      No_Carte_Robot = 4;
      Envoyer_Message(No_Carte_Robot, 3, "OK");
      DernierTempsRobot = 0;
    }

    else if(Commande=="OBR_CONNECTER_EXTENSION2" && !Robot)
    {
      Robot = NoPort;
      No_Carte_Robot = 5;
      Envoyer_Message(No_Carte_Robot, 3, "OK");
      DernierTempsRobot = 0;
    }

    // On regarde si on a pas reçu un accusé de réception
    else if(Commande=="OK")
    {
      // On détermine à qui on est connecté
      switch(NoEnvoyeur)
      {
        case PC:
        if(!Ordinateur)
         Ordinateur = NoPort;
        break;

        case 3:
        if(!Robot)
          Robot = NoPort;
          No_Carte_Robot = NoEnvoyeur;
        break;

        case 4:
        if(!Robot)
          Robot = NoPort;
          No_Carte_Robot = NoEnvoyeur;
        break;

        case 5:
        if(!Robot)
          Robot = NoPort;
          No_Carte_Robot = NoEnvoyeur;
        break;

        default:
        break;
      }
    }
  }
  SYS_Effacer_Commande();
}

void SYS_Demande_Connexion()
{
  // Si on est pas connecté au maitre, on envoie une demande de connexion
  if(Robot==0)
  {
    if(Ordinateur!=1)
      Serial.print("[003]OBR_CONNECTER_TELECOMMANDE\n");

    #if (NB_PORT_SERIE>1)
    if(Ordinateur!=2)
      Serial1.print("[003]OBR_CONNECTER_TELECOMMANDE\n");

    if(Ordinateur!=3)
      Serial2.print("[003]OBR_CONNECTER_TELECOMMANDE\n");

    if(Ordinateur!=4)
      Serial3.print("[003]OBR_CONNECTER_TELECOMMANDE\n");
    #endif
  }
}

// Fonction d'indication du numéro du port sur lequel est connecté tel truc (N° de port à partir de 1)
byte SYS_No_Port_Serie(byte Objet)
{
  // On regarde où l'objet demandé est connecté, on retourne 0 sinon
  switch(Objet)
  {
    case PC:
    if(Ordinateur<10)
      return Ordinateur;
    else
      return Ordinateur-10;

    case 3:
    case 4:
    case 5:
    if(Robot<10)
      return Robot;
    else
      return Robot-10;

    default:
      return 0;
  }
}

// Fonction d'effacement de commande en file d'attente
void SYS_Effacer_Commande()
{
  stringComplete = false;
  NbParam = 0;
  Commande = "";
  param1 = "";
  param2 = "";
  param3 = "";
  NoEnvoyeur = 0;
  NoDestinataire = 0;
  ID = 0;
}
