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
// - Gestion des centaines dans les numéros de PIN (pour différencier les cartes), le PIN 255 pointe vers rien !
// - Vérifier la gestion de vitesse de transition
// - Centraliser toutes les communications via le maitre
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
#define VERSION 0.7
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
#define VITESSE_TRANSITION_PWM 16
#define VALEUR_VOULUE_ES 17

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
#define PWM6 255
#define PWM7 255
#define PWM8 255
#define PWM9 255
#define PWM10 255
#define PWM11 255
#define PWM12 255
#define PWM13 255
#define PWM14 255
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
#define PWM7 255
#define PWM8 255
#define PWM9 255
#define PWM10 255
#define PWM11 255
#define PWM12 255
#define PWM13 255
#define PWM14 255
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
#define PWM6 255
#define PWM7 255
#define PWM8 255
#define PWM9 255
#define PWM10 255
#define PWM11 255
#define PWM12 255
#define PWM13 255
#define PWM14 255
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
String Reponse = "";
boolean stringComplete = false; // Indique si une commande est arrivée
byte NbParam = 0; // Nombre de paramètres dans la commande
byte NoEnvoyeur = 0; // Adresse de l'envoyeur du dernier message
byte NoDestinataire = 0; // Adresse du destinataire du dernier message
byte NbActualisation = -1; // limite les envoie de demande de connexion
// Chien de garde sur les ports série
boolean watchdogActif = 0;
byte DernierTempsPC = 0; // Temps en 1/10 de seconde depuis la dernière réception
byte DernierTempsTelecommande = 0;
byte DernierTempsMaitre = 0;
byte DernierTempsEsclave1 = 0;
byte DernierTempsEsclave2 = 0;
byte DernierTempsPC_Envoie = 0; // Temps en 1/10 de seconde depuis le dernier envoie
byte DernierTempsTelecommande_Envoie = 0;
byte DernierTempsMaitre_Envoie = 0;
byte DernierTempsEsclave1_Envoie = 0;
byte DernierTempsEsclave2_Envoie = 0;
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
// Gestion d'Ecrire_Erreur
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
void Executer_Commande_Recue()
{
  Reponse="Erreur_100010";

  // On demande si l'OBR est présent
  if(Commande=="OBR_PRESENT")
    Reponse="VRAIE";
  
  // On demande si l'OBR est prêt
  if(Commande=="OBR_PRET")
    if(Connecte)
      Reponse="ADMIN";
     else
      Reponse="VRAIE";
  
  // On demande la vesrion de l'OBR
  if(Commande=="OBR_VERSION")
    Reponse=String(VERSION);
  
  // On demande le nom du robot
  if(Commande=="OBR_NOM")
    Reponse=NOM_DU_ROBOT;

  // On demande sur quel port est connecté le PC
  if(Commande=="OBR_CONNEXION_PC" && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    Reponse=String(Ordinateur);

  // On demande sur quel port est connecté la télécommande
  if(Commande=="OBR_CONNEXION_TELECOMMANDE" && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    Reponse=String(Telecommande);

  // On demande sur quel port est connecté l'extension 1
  if(Commande=="OBR_CONNEXION_EXTENSION1" && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    Reponse=String(Extension1);

  // On demande sur quel port est connecté l'extension 2
  if(Commande=="OBR_CONNEXION_EXTENSION2" && ((NoEnvoyeur==PC && Connecte) || NoEnvoyeur==MAITRE))
    Reponse=String(Extension2);

  // On demande à passer en mode de pilotage
  if(Commande=="OBR_PILOTER")
  {
    Mode = PILOTAGE;
    Reponse="PILOTAGE";
  }

  // On demande à passer en mode de configuration
  if(Commande=="OBR_CONFIGURER")
  {
    Mode = CONFIGURATION;
    Reponse="CONFIGURATION";
  }

  // On indique qu'on se déconnecte physiquement
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

  // On demande le niveau de la batterie
  if(Commande=="OBR_NIVEAU_BATTERIE")
  {
    // Retourne le niveau actuel des batteries
    Reponse=String(NiveauBatterieActuel);
  }

  if(Mode==CONFIGURATION)
  {
    // On demande à se connecter en administrateur
    if(Commande=="OBR_CONNECTER_ADMIN")
    {
      if(param1==String(Password))
      {
        Connecte = 1;
        Reponse="OK";
      }
      else
        Ecrire_Erreur(130030);
    }

    // On demande à se retirer les droits administrateurs
    if(Commande=="OBR_DECONNECTER_ADMIN")
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
    if(Commande=="OBR_REBOOT")
    {
      if(Connecte)
      {
        Connecte=0;
        Envoyer_Message(NoEnvoyeur, "REDEMARRAGE...");
        boot();
        return;
      }
      else
        Ecrire_Erreur(130010);
    }
    
    // On demande à lire un octet de l'EEPROM
    if(Commande=="OBR_LIRE_EEPROM")
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
    if(Commande=="OBR_ECRIRE_EEPROM")
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
    if(Commande=="OBR_CHANGER_MOT_DE_PASSE")
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
    if(Commande=="OBR_DerniereErreur")
    {
      if(!DerniereErreur)
        Reponse="AUCUNE_Erreur";
        
      else
      {
        Reponse=String(DerniereErreur);
        DerniereErreur = 0;
      }
    }

    if(Commande=="OBR_DECONNEXION_AUTO" && (NoEnvoyeur == PC || NoEnvoyeur == TELECOMMANDE))
    {
      if(watchdogActif != param1.toInt())
        // On enregistre en EEPROM
        EEPROM.write(25, param1.toInt());

      watchdogActif = param1.toInt();
      if(watchdogActif)
        Reponse="WATCHDOG_ACTIF";
      else
        Reponse="WATCHDOG_INACTIF";
    }

    // On connecte la batterie
    if(Commande=="OBR_CONNECTER_BATTERIE")
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
    if(Commande=="OBR_CONNECTER_LED_BATTERIE")
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

    // On configure un moteur
    if(Commande=="OBR_CONNECTER_MOTEUR" || Commande=="OBR_CONNECTER_SERVO")
    {
      if(Connecte)
      {
        // On vérifie le Numéro du PIN et les MIN et MAX
        if(No_PWM_PIN(param1.toInt())>NB_PIN_PWM || param2.toInt()>255 || param3.toInt()>255 || param2.toInt()>param3.toInt())
          Ecrire_Erreur(110030);
        else
        {
          if(Commande=="OBR_CONNECTER_MOTEUR")
            // On configure le PIN en MOTEUR
            Changer_Mode_PIN_ES(param1.toInt(), MOTEUR);
          else
            // On configure le PIN en SERVO
            Changer_Mode_PIN_ES(param1.toInt(), SERVO);

          // On indique ses limites
          Ecrire_RAM(MIN_PWM, param1.toInt(), param2.toInt());
          Ecrire_RAM(MAX_PWM, param1.toInt(), param3.toInt());

          // On enregistre en EEPROM
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

          // On envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Déconnecte un moteur
    if(Commande=="OBR_DECONNECTER_MOTEUR" || Commande=="OBR_DECONNECTER_SERVO")
    {
      if(Connecte)
      {
        // On vérifie le numéro du PIN
        if(No_PWM_PIN(param1.toInt())>NB_PIN_PWM || (Lire_RAM(TYPE_ES, param1.toInt())!=MOTEUR && Commande=="OBR_DECONNECTER_MOTEUR") || (Lire_RAM(TYPE_ES, param1.toInt())!=SERVO) && Commande=="OBR_DECONNECTER_SERVO")
          Ecrire_Erreur(130020);
        else
        {
          // On indique un PIN vide et on vide les MIN et MAX
          Changer_Mode_PIN_ES(param1.toInt(), VIDE);
          Ecrire_RAM(MIN_PWM, param1.toInt(), 0);
          Ecrire_RAM(MAX_PWM, param1.toInt(), 0);

          // On supprime les liens
          Ecrire_RAM(TYPE_LIAISON_ANALOG, param1.toInt(), 0);
          Ecrire_RAM(TYPE_LIAISON_1, param1.toInt(), 0);
          Ecrire_RAM(TYPE_LIAISON_2, param1.toInt(), 0);
          Ecrire_RAM(TYPE_LIAISON_3, param1.toInt(), 0);

          Ecrire_RAM(LIAISON_ANALOG, param1.toInt(), 255);
          Ecrire_RAM(LIAISON_1, param1.toInt(), 255);
          Ecrire_RAM(LIAISON_2, param1.toInt(), 255);
          Ecrire_RAM(LIAISON_3, param1.toInt(), 255);

          // On enregistre le tout en EEPROM
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

          // On envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Définit les limites d'un capteur
    if(Commande=="OBR_CONNECTER_CAPTEUR")
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

          // On enregistre les paramètres
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 1);

          // On envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Vide les limites d'un capteur
    if(Commande=="OBR_DECONNECTER_CAPTEUR")
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

          // On enregistre les paramètres
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 1);

          // On envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Configure un PIN comme une entrée
    if(Commande=="OBR_CONNECTER_ENTREE")
    {
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          // On indique au système que c'est une entrée
          Changer_Mode_PIN_ES(param1.toInt(), ENTREE);

          // On met la valeur à 0
          Ecrire_RAM(VALEUR_ACTU_ES, param1.toInt(), 0);

          // On enregistre les paramètres
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

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
    if(Commande=="OBR_DECONNECTER_ENTREE")
    {
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          if(Lire_RAM(TYPE_ES, param1.toInt())==ENTREE)
          {
            // On indique un PIN vide
            Changer_Mode_PIN_ES(param1.toInt(), VIDE);

            // On supprime les liens
            Ecrire_RAM(TYPE_LIAISON_ANALOG, param1.toInt(), 0);
            Ecrire_RAM(TYPE_LIAISON_1, param1.toInt(), 0);
            Ecrire_RAM(TYPE_LIAISON_2, param1.toInt(), 0);
            Ecrire_RAM(TYPE_LIAISON_3, param1.toInt(), 0);

            Ecrire_RAM(LIAISON_ANALOG, param1.toInt(), 255);
            Ecrire_RAM(LIAISON_1, param1.toInt(), 255);
            Ecrire_RAM(LIAISON_2, param1.toInt(), 255);
            Ecrire_RAM(LIAISON_3, param1.toInt(), 255);

            // On enregistre le tout en EEPROM
            SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

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
    if(Commande=="OBR_CONNECTER_SORTIE")
    {
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          // On indique au système que c'est une sortie
          Changer_Mode_PIN_ES(param1.toInt(), SORTIE);

          // On met la valeur à 0
          Ecrire_RAM(VALEUR_ACTU_ES, param1.toInt(), 0);

          // On enregistre les paramètres
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

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
    if(Commande=="OBR_DECONNECTER_SORTIE")
    {
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          if(Lire_RAM(TYPE_ES, param1.toInt())==SORTIE)
          {
            // On indique un PIN vide
            Changer_Mode_PIN_ES(param1.toInt(), VIDE);

            // On supprime les liens
            Ecrire_RAM(TYPE_LIAISON_ANALOG, param1.toInt(), 0);
            Ecrire_RAM(TYPE_LIAISON_1, param1.toInt(), 0);
            Ecrire_RAM(TYPE_LIAISON_2, param1.toInt(), 0);
            Ecrire_RAM(TYPE_LIAISON_3, param1.toInt(), 0);

            Ecrire_RAM(LIAISON_ANALOG, param1.toInt(), 255);
            Ecrire_RAM(LIAISON_1, param1.toInt(), 255);
            Ecrire_RAM(LIAISON_2, param1.toInt(), 255);
            Ecrire_RAM(LIAISON_3, param1.toInt(), 255);

            // On enregistre le tout en EEPROM
            SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

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

    // Lier une E/S avec un capteur. params : 1=N°E/S 2=N°Capteur 3=TypeLien
    if(Commande=="OBR_LIER_CAPTEUR")
    {
      if(Connecte)
      {
        // On vérifie les limites des paramètres
        if(Lire_RAM(TYPE_ES, param1.toInt())==VIDE || Lire_RAM(TYPE_ES, param1.toInt())==SYSTEME)
          Ecrire_Erreur(110021);
        else if(param2.toInt()>NB_PIN_ANALOG)
          Ecrire_Erreur(110032);
        else if(param3.toInt()>3)
          Ecrire_Erreur(110033);
        else
        {
          // On crée le lien
          Ecrire_RAM(LIAISON_ANALOG, param1.toInt(), param2.toInt());

          // On enregistre le type du lien
          Ecrire_RAM(TYPE_LIAISON_ANALOG, param1.toInt(), param3.toInt());

          // On enregistre le tout en EEPROM
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

          // On envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Lier une E/S avec une autre E/S. params : 1=N°E/S 2=N°Lien 3=TypeLien
    if(Commande=="OBR_LIER_ES")
    {
      if(Connecte)
      {
        // On vérifie les limites des paramètres
        if(Lire_RAM(TYPE_ES, param1.toInt())==VIDE || Lire_RAM(TYPE_ES, param1.toInt())==SYSTEME)
          Ecrire_Erreur(110021);
        else if(param2.toInt()>NB_PIN_IO)
          Ecrire_Erreur(110032);
        else if(param3.toInt()>3)
          Ecrire_Erreur(110033);

        // On regarde si le PIN demandé en deuxième paramètre est un PWM ou pas, et on interdit les type 0 et 1 pour les ToutOuRien
        else if(Lire_RAM(TYPE_ES, param2.toInt())==MOTEUR || Lire_RAM(TYPE_ES, param2.toInt())==SERVO)
        {
          // On crée le lien
          if(Lire_RAM(LIAISON_1, param1.toInt())==255)
          {
            Ecrire_RAM(LIAISON_1, param1.toInt(), param2.toInt());
            Ecrire_RAM(TYPE_LIAISON_1, param1.toInt(), param3.toInt());
            
            // On enregistre le tout en EEPROM
            SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

            // On envoie une confirmation
            Reponse="OK";
          }
          else if(Lire_RAM(LIAISON_2, param1.toInt())==255)
          {
            Ecrire_RAM(LIAISON_2, param1.toInt(), param2.toInt());
            Ecrire_RAM(TYPE_LIAISON_2, param1.toInt(), param3.toInt());
            
            // On enregistre le tout en EEPROM
            SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

            // On envoie une confirmation
            Reponse="OK";
          }
          else if(Lire_RAM(LIAISON_3, param1.toInt())==255)
          {
            Ecrire_RAM(LIAISON_3, param1.toInt(), param2.toInt());
            Ecrire_RAM(TYPE_LIAISON_3, param1.toInt(), param3.toInt());

            // On enregistre le tout en EEPROM
            SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

            // On envoie une confirmation
            Reponse="OK";
          }
          else
            // Si tous les tableaux sont pris, on retourne une Ecrire_Erreur
            Ecrire_Erreur(210010);
        }

        // On empèche les types de liaison 0 et 1
        else if(param3.toInt()<2)
          Ecrire_Erreur(110033);
        else
        {
          // On crée le lien
          if(Lire_RAM(LIAISON_1, param1.toInt())==255)
          {
            Ecrire_RAM(LIAISON_1, param1.toInt(), param2.toInt());
            Ecrire_RAM(TYPE_LIAISON_1, param1.toInt(), param3.toInt());
            
            // On enregistre le tout en EEPROM
            SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

            // On envoie une confirmation
            Reponse="OK";
          }
          else if(Lire_RAM(LIAISON_2, param1.toInt())==255)
          {
            Ecrire_RAM(LIAISON_2, param1.toInt(), param2.toInt());
            Ecrire_RAM(TYPE_LIAISON_2, param1.toInt(), param3.toInt());
            
            // On enregistre le tout en EEPROM
            SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

            // On envoie une confirmation
            Reponse="OK";
          }
          else if(Lire_RAM(LIAISON_3, param1.toInt())==255)
          {
            Ecrire_RAM(LIAISON_3, param1.toInt(), param2.toInt());
            Ecrire_RAM(TYPE_LIAISON_3, param1.toInt(), param3.toInt());

            // On enregistre le tout en EEPROM
            SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

            // On envoie une confirmation
            Reponse="OK";
          }
          else
            // Si tous les tableaux sont pris, on retourne une Ecrire_Erreur
            Ecrire_Erreur(210010);
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // délier une E/S de tout. param : 1=N°E/S
    if(Commande=="OBR_DELIER")
    {
      if(Connecte)
      {
        // On verrifie les paramètres
        if(param1.toInt()>NB_PIN_IO)
          Ecrire_Erreur(110031);
        else
        {
          // On supprime les liens
          Ecrire_RAM(TYPE_LIAISON_ANALOG, param1.toInt(), 0);
          Ecrire_RAM(TYPE_LIAISON_1, param1.toInt(), 0);
          Ecrire_RAM(TYPE_LIAISON_2, param1.toInt(), 0);
          Ecrire_RAM(TYPE_LIAISON_3, param1.toInt(), 0);

          Ecrire_RAM(LIAISON_ANALOG, param1.toInt(), 255);
          Ecrire_RAM(LIAISON_1, param1.toInt(), 255);
          Ecrire_RAM(LIAISON_2, param1.toInt(), 255);
          Ecrire_RAM(LIAISON_3, param1.toInt(), 255);

          // On enregistre le tout en EEPROM
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

          // On envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // 
    if(Commande=="OBR_CONFIG_MOTEUR_MIN_MAX" || Commande=="OBR_CONFIG_SERVO_MIN_MAX")
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(No_PWM_PIN(param1.toInt())>NB_PIN_PWM)
          Ecrire_Erreur(110031);
        else if(param2.toInt()>255)
          Ecrire_Erreur(110032);
        else if(param3.toInt()>255)
          Ecrire_Erreur(110033);
        else if(param2.toInt()>param3.toInt())
          Ecrire_Erreur(110023);

        // On vérifie que la sortie PWM est bien configurée en SERVO ou MOTEUR
        else if(Lire_RAM(TYPE_ES, param1.toInt())==SERVO || Lire_RAM(TYPE_ES, param1.toInt())==MOTEUR)
        {
          // on change les paramètres
          Ecrire_RAM(MIN_PWM, param1.toInt(), param2.toInt());
          Ecrire_RAM(MAX_PWM, param1.toInt(), param3.toInt());

          // on les enregistre
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);

          // on envoie une confirmation
          Reponse="OK";
        }

        else
          Ecrire_Erreur(110021);
      }
      else
        Ecrire_Erreur(130010);
    }

    // 
    if(Commande=="OBR_CONFIG_CAPTEUR_MIN_MAX")
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

          // on les enregistre
          SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 1);

          // on envoie une confirmation
          Reponse="OK";
        }
      }
      else
        Ecrire_Erreur(130010);
    }

    // Définit la vitesse de transition des valeurs d'une sortie PWM (250=0s, 1=5s, 0=interdit; map()=> 0=0s, 250=2,5s)
    if(Commande=="OBR_CONFIG_PWM_TRANSITION")
    {
      // On vérifie que c'est bien un moteur ou un capteur
      if(Connecte)
      {
        if(No_PWM_PIN(param1.toInt())<NB_PIN_PWM)
        {
          if(Lire_RAM(TYPE_ES, param1.toInt())==MOTEUR || Lire_RAM(TYPE_ES, param1.toInt())==SERVO)
          {
            // On vérifie la vitesse de transmission (entre 0 et 255)
            if(param2.toInt()>250 || param2.toInt()<1)
              Ecrire_Erreur(110032);
            else
            {
              // On adapte la nouvelle valeur
              byte Vitesse = 250 / param2.toInt();
              // On enregistre la nouvelle valeur
              Ecrire_RAM(VITESSE_TRANSITION_PWM, param1.toInt(), Vitesse);
              // On enregistre les paramètres
              SYS_Ecrire_Donnees_Vers_EEPROM(param1.toInt(), 0);
              Reponse="OK";
            }
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

    // Enregistrer les paramètres
    if(Commande=="OBR_ENREGISTRER_PARAMETRES" && Connecte)
    {
      Reponse="OK";

      // Tant qu'on a des E/S à enregistrer, on les enregistre
      byte i = 0;
      for(i=0; i<NB_PIN_IO; i++)
      {
        SYS_Ecrire_Donnees_Vers_EEPROM(i, 0);
      }
      for(i=0; i<NB_PIN_ANALOG; i++)
      {
        SYS_Ecrire_Donnees_Vers_EEPROM(i, 1);
      }
    }
  }

  if(Mode==PILOTAGE || DEBUG)
  {
    // On récupère la valeur brut du capteur
    if(Commande=="OBR_VALEUR_BRUT_CAPTEUR")
    {
      if(param1.toInt() < NB_PIN_ANALOG)
        Reponse=String(Lire_RAM(VALEUR_ACTU_ANALOG, param1.toInt()));
      else
        Ecrire_Erreur(110031);
    }

    // On récupère la valeur brut d'une entrée/Sortie
    if(Commande=="OBR_VALEUR_BRUT_ES")
    {
      if(param1.toInt()<NB_PIN_IO)
        Reponse=String(Lire_RAM(VALEUR_ACTU_ES, param1.toInt()));
      else
        Ecrire_Erreur(110031);
    }

    // On change la vitesse du moteur
    if(Commande=="OBR_VITESSE_MOTEUR")
    {
      // On vérifie que la sortie est un moteur
      if(Lire_RAM(TYPE_ES, param1.toInt())!=MOTEUR || No_PWM_PIN(param1.toInt())>NB_PIN_PWM)
        Ecrire_Erreur(110021);
      else
      {
        // On met à jour la vitesse en tenant compte des limites
        if(param2.toInt()<Lire_RAM(MIN_PWM, param1.toInt()))
        {
          Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), Lire_RAM(MIN_PWM, param1.toInt()));
          Reponse="MIN";
        }
        else if(param2.toInt()>Lire_RAM(MAX_PWM, param1.toInt()))
        {
          Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), Lire_RAM(MAX_PWM, param1.toInt()));
          Reponse="MAX";
        }
        else
        {
          Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), param2.toInt());
          Reponse="OK";
        }
      }
    }

    // On arrête un moteur
    if(Commande=="OBR_ARRETER_MOTEUR")
    {
      // On vérifie que la sortie est un moteur
      if(Lire_RAM(TYPE_ES, param1.toInt())!=MOTEUR || No_PWM_PIN(param1.toInt())>NB_PIN_PWM)
        Ecrire_Erreur(110021);
      else
      {
        Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), 0);
        Ecrire_RAM(VALEUR_ACTU_ES, param1.toInt(), 0);
        Reponse="OK";
      }
    }

    // On change la position du servomoteur
    if(Commande=="OBR_POSITION_SERVO")
    {
      // On vérifie que la sortie est un servomoteur
      if(Lire_RAM(TYPE_ES, param1.toInt())!=SERVO || No_PWM_PIN(param1.toInt())>NB_PIN_PWM)
        Ecrire_Erreur(110021);
      else
      {
        // On met à jour la vitesse en tenant compte des limites
        if(param2.toInt()<Lire_RAM(MIN_PWM, param1.toInt()))
          Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), Lire_RAM(MIN_PWM, param1.toInt()));
        else if(param2.toInt()>Lire_RAM(MAX_PWM, param1.toInt()))
          Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), Lire_RAM(MAX_PWM, param1.toInt()));
        else
          Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), param2.toInt());
        Reponse="OK";
      }
    }

    // 
    if(Commande=="OBR_ALLUMER_SORTIE")
    {
      // On vérifie que c'est bien une sortie digitale
      if(Lire_RAM(TYPE_ES, param1.toInt())!=SORTIE || param1.toInt()>NB_PIN_IO)
        Ecrire_Erreur(110021);
      else
      {
        Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), 1);
        Reponse="OK";
      }
    }

    // 
    if(Commande=="OBR_ARRETER_SORTIE")
    {
      // On vérifie que c'est bien une sortie digitale
      if(Lire_RAM(TYPE_ES, param1.toInt())!=SORTIE || param1.toInt()>NB_PIN_IO)
        Ecrire_Erreur(110021);
      else
      {
        Ecrire_RAM(VALEUR_VOULUE_ES, param1.toInt(), 0);
        Reponse="OK";
      }
    }
  }

  // On envoie la réponse
  Envoyer_Message(NoEnvoyeur, Reponse);

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
  
  // Initialisation de l'EEPROM
  PIN_BATTERIE = EEPROM.read(10);
  PIN_BATTERIE_FAIBLE = EEPROM.read(11);
  PIN_BATTERIE_PLEINE = EEPROM.read(12);
  NiveauBatterieMin = EEPROM.read(13)*4;
  NiveauBatterieMax = EEPROM.read(14)*4;
  
  // Lecture du mot de passe
  Password = SYS_Lire_Mot_De_Passe();
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
  stringComplete = false; // Indique si une commande est arrivée
  NbParam = 0; // Nombre de paramètres dans la commande
  NoEnvoyeur = 0; // Adresse de l'envoyeur du dernier message
  NoDestinataire = 0; // Adresse du destinataire du dernier message
  NbActualisation = -1; // limite les envoie de demande de connexion
  // Chien de garde sur les ports série
  watchdogActif = EEPROM.read(25);
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
  // Gestion d'Ecrire_Erreur
  DerniereErreur = 0;
  NombreErreur = 0;

  // On initialise les tableaux
  SYS_Initialiser_Tableaux_RAM();

  // Arrêt de la LED 13
  Changer_Mode_PIN_ES(13, SYSTEME);
  Ecrire_RAM(VALEUR_ACTU_ES, 13, 0);
  Ecrire_RAM(VALEUR_VOULUE_ES, 13, 0);

  // Initialisation des E/S
  byte i = 0;
  for(i=0; i<NB_PIN_ANALOG; i++)
  {
    SYS_Lire_Donnees_Depuis_EEPROM(i, 1);
  }
  for(i=0; i<NB_PIN_IO; i++)
  {
    SYS_Lire_Donnees_Depuis_EEPROM(i, 0);
  }

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
  
  // On fait une pause de 1 ms pour éviter de trop consommer et de trop chauffer
  delay(1);
}

// Fonction de gestion des Ecrire_Erreurs
void Ecrire_Erreur(unsigned long NoEcrire_Erreur)
{
  DerniereErreur = NoEcrire_Erreur;
  Reponse="Erreur_"+String(NoEcrire_Erreur);
}

// Fonction qui retourne la dernière Ecrire_Erreur
unsigned long Lire_DerniereErreur()
{
  // On retourne la dernière Ecrire_Erreur
  return DerniereErreur;
}

// Fonction de lecture d'une case Entrée/Sortie selon l'EEPROM (Analog=0 si E/S, 1 si ANALOG)
void SYS_Lire_Donnees_Depuis_EEPROM(byte NoES, boolean Analog) // Travaille directement sur la RAM, sans passer par les fonctions noyau
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
    NoES = No_PWM_PIN(NoES);

    // Min PWM
    Min_PWM[NoES] = EEPROM.read(100 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));

    // Max PWM
    Max_PWM[NoES] = EEPROM.read(101 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));

    // Transition PWM
    Vitesse_Transition_PWM[NoES] = EEPROM.read(102 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));
  }
}

// Fonction d'écriture d'une case Entrée/Sortie en EEPROM (Analog=0 si E/S, 1 si ANALOG)
void SYS_Ecrire_Donnees_Vers_EEPROM(byte NoES, boolean Analog) // Travaille directement sur la RAM, sans passer par les fonctions noyau
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
    EEPROM.write(100+(2*NoES), Lire_RAM(MIN_ANALOG, NoES));
    EEPROM.write(101+(2*NoES), Lire_RAM(MAX_ANALOG, NoES));
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
    NoES = No_PWM_PIN(NoES);
    
    // Min PWM
    EEPROM.write(100 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Min_PWM[NoES]);

    // Max PWM
    EEPROM.write(101 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Max_PWM[NoES]);

    // Transition PWM
    EEPROM.write(102 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Vitesse_Transition_PWM[NoES]);
  }
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
void SYS_Mise_A_Jour_RAM()
{
  // On met à jour le tableau des capteurs
  byte i = 0;
  for(i=0; i<NB_PIN_ANALOG; i++)
  {
    Ecrire_RAM(VALEUR_ACTU_ANALOG, i, analogRead(i)/4);
  }

  // On met à jour le tableau des Entrées
  for(i=0; i<NB_PIN_IO; i++)
  {
    if(Type_ES[i]==ENTREE)
      Ecrire_RAM(VALEUR_ACTU_ES, i, digitalRead(i));
  }

  // On appel la fonction de mise à jour de l'état des sorties
  SYS_Mise_A_Jour_Des_ES();
}

// Fonction de lecture du N° de PIN PWM
byte No_PIN_PWM(byte NoPWM)
{
  // Selon les #define, on retourne le numéro du PIN correspondant à la sortie PWM demandée
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
      return 255;
  }
}

// Fonction qui retourne le no PWM en fonction de l'E/S
byte No_PWM_PIN(byte NoES)
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
      Min_Analog[NumeroCase] = Valeur;
    break;

    case MAX_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      Max_Analog[NumeroCase] = Valeur;
    break;

    case MIN_PWM:
    if(No_PWM_PIN(NumeroCase)<NB_PIN_PWM)
      Min_PWM[No_PWM_PIN(NumeroCase)] = Valeur;
    break;

    case MAX_PWM:
    if(No_PWM_PIN(NumeroCase)<NB_PIN_PWM)
      Max_PWM[No_PWM_PIN(NumeroCase)] = Valeur;
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
      Type_Liaisons[NumeroCase]  = (Type_Liaisons[NumeroCase] & 0b00111111) + Valeur*64; // On prend les 2 premiers octets
    break;

    case TYPE_LIAISON_1:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0 && Valeur <= 4)
      Type_Liaisons[NumeroCase]  = (Type_Liaisons[NumeroCase] & 0b11001111) + Valeur*16; // On prend les 2 octets suivants
    break;

    case TYPE_LIAISON_2:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0 && Valeur <= 4)
      Type_Liaisons[NumeroCase]  = (Type_Liaisons[NumeroCase] & 0b11110011) + Valeur*4; // On prend les 2 octets suivants
    break;

    case TYPE_LIAISON_3:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0 && Valeur <= 4)
      Type_Liaisons[NumeroCase]  = (Type_Liaisons[NumeroCase] & 0b11111100) + Valeur; // On prend les 2 derniers octets
    break;

    case VITESSE_TRANSITION_PWM: // Vitesse à laquelle le mouvement se fait sur le port PWM
    if(No_PWM_PIN(NumeroCase)<NB_PIN_PWM)
      Vitesse_Transition_PWM[No_PWM_PIN(NumeroCase)] = Valeur;
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
      return Min_Analog[NumeroCase];
    break;

    case MAX_ANALOG:
    if(NumeroCase<NB_PIN_ANALOG && NumeroCase>=0)
      return Max_Analog[NumeroCase];
    break;

    case MIN_PWM:
    if(No_PWM_PIN(NumeroCase)<NB_PIN_PWM)
      return Min_PWM[No_PWM_PIN(NumeroCase)];
    break;

    case MAX_PWM:
    if(No_PWM_PIN(NumeroCase)<NB_PIN_PWM)
      return Max_PWM[No_PWM_PIN(NumeroCase)];
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
      return (Type_Liaisons[NumeroCase] & 0b11000000)/64; // On prend les 2 premiers octets
    break;

    case TYPE_LIAISON_1:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return (Type_Liaisons[NumeroCase] & 0b00110000)/16; // On prend les 2 octets suivants
    break;

    case TYPE_LIAISON_2:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return (Type_Liaisons[NumeroCase] & 0b00001100)/4; // On prend les 2 octets suivants
    break;

    case TYPE_LIAISON_3:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Type_Liaisons[NumeroCase] & 0b00000011; // On prend les 2 derniers octets
    break;

    case VITESSE_TRANSITION_PWM: // Vitesse à laquelle le mouvement se fait sur le port PWM
    if(No_PWM_PIN(NumeroCase)<NB_PIN_PWM)
      return Vitesse_Transition_PWM[No_PWM_PIN(NumeroCase)];
    break;

    case VALEUR_VOULUE_ES:
    if(NumeroCase<NB_PIN_IO && NumeroCase>=0)
      return Valeur_Voulue_IO[NumeroCase];
    break;

    default:
    return 0;
    break;
  }
  return 0;
}

// Fonctions de controle du robot
// Fonction de mise à jour des sorties, selon la RAM
void SYS_Mise_A_Jour_Des_ES()
{
  byte i = 0;

  // On lit le tableau des sorties, on regarde si on doit (et peut) activer une sortie, et on l'active
  for(i=0; i<NB_PIN_IO; i++)
  {
    // On regarde si c'est une sortie non PWM
    switch(Lire_RAM(TYPE_ES, i))
    {
      case MOTEUR:
      case SERVO:
      analogWrite(i, SYS_Autoriser_Sortie(i));
      break;

      case SORTIE:
      digitalWrite(i, SYS_Autoriser_Sortie(i));
      break;

      // Si c'est un PIN vide, on le met à 0
      case VIDE:
      digitalWrite(i, 0);
      break;

      default:
      break;
    }
  }

  // Si ça fait plus de 10 millisecondes qu'on a actualisé tous les moteurs, on les réactualise
  if(DernierTempsTransitionPWM<millis()-10)
      DernierTempsTransitionPWM+=10;

  // Gestion différente pour le PIN 13, s'il est utilisé par le système
  if(Lire_RAM(TYPE_ES, 13)==SYSTEME || Lire_RAM(TYPE_ES, 13)==VIDE)
  {
    if(Connecte)
      digitalWrite(13, HIGH);
    else
      digitalWrite(13, LOW);
  }
}

// Fonction qui retourne la valeur à écrire en sortie physique (sur les PINs)
byte SYS_Autoriser_Sortie(byte NumeroSortie)
{
  byte Valeur = 0;
  
  // On regarde quel type de sortie c'est
  byte Type = Lire_RAM(TYPE_ES, NumeroSortie);
  if(Type != SORTIE && Type != MOTEUR && Type != SERVO)
    return 0;

  // Si c'est un servomoteur, on fait en sorte qu'il ne bouge pas
  if(Type == SERVO)
    Valeur = Lire_RAM(VALEUR_ACTU_ES, NumeroSortie);

  // Si aucune télécommande/PC n'est connecté, on coupe les moteurs et sorties
  if(!Ordinateur && !Telecommande)
    return Valeur;

  // On regarde si les liaisons l'autorisent
  if(!SYS_Verification_Des_Liaisons(NumeroSortie))
  {
    // Si c'est un moteur ou une sortie ToutOuRien, on inscrit dans la mémoire que la sortie est à 0.
    if(Lire_RAM(TYPE_ES, NumeroSortie)==SORTIE || Lire_RAM(TYPE_ES, NumeroSortie)==MOTEUR)
      Ecrire_RAM(VALEUR_ACTU_ES, NumeroSortie, 0);

    // On retourne la valeur inchangée
    return Valeur;
  }

  // On regarde si c'est un PWM, et s'il est bien dans ses limites (MIN et MAX)
  if(Type==SERVO || Type==MOTEUR)
  {
    // On regarde la vitesse de transition
    byte VitesseVoulue = Lire_RAM(VALEUR_VOULUE_ES, NumeroSortie);
    byte VitesseActu = Lire_RAM(VALEUR_ACTU_ES, NumeroSortie);
    byte VitesseTransition = Lire_RAM(VITESSE_TRANSITION_PWM, NumeroSortie);

    // Si ça fait plus de 10 ms qu'on a pas actualisé, on actualise
    if(DernierTempsTransitionPWM<millis()-10)
    {
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

    // On regarde les limites
    byte MIN = Lire_RAM(MIN_PWM, NumeroSortie);
    byte MAX = Lire_RAM(MAX_PWM, NumeroSortie);
    if(Lire_RAM(VALEUR_ACTU_ES, NumeroSortie)<MIN && Lire_RAM(VALEUR_ACTU_ES, NumeroSortie)!=0) // On met au minimum sauf s'il est carément arrêté
    {
      Valeur = MIN;
      Ecrire_RAM(VALEUR_ACTU_ES, NumeroSortie, Valeur);
      return Valeur;
    }
    if(Lire_RAM(VALEUR_ACTU_ES, NumeroSortie)>MAX) // On met au maximum s'il dépasse
    {
      Valeur = MAX;
      Ecrire_RAM(VALEUR_ACTU_ES, NumeroSortie, Valeur);
      return Valeur;
    }
    // On met à jour la nouvelle vitesse/valeur
    Ecrire_RAM(VALEUR_ACTU_ES, NumeroSortie, VitesseActu);
    return VitesseActu;
  }

  // On regarde la valeur demandée en RAM si c'est un ToutOuRien
  Valeur = Lire_RAM(VALEUR_VOULUE_ES, NumeroSortie);
  Ecrire_RAM(VALEUR_ACTU_ES, NumeroSortie, Valeur);

  // On retourne la valeur
  return Valeur;
}

// retourne 1 si toutes les liaisons sont OK
boolean SYS_Verification_Des_Liaisons(byte NumeroSortie)
{
  byte NoLiaison = 0;
  for(NoLiaison=LIAISON_ANALOG; NoLiaison<LIAISON_3; NoLiaison++)
  {
    // On récupère le numéro du lien
    byte NoLien = Lire_RAM(NoLiaison, NumeroSortie);

    // S'il y a un lien
    if(NoLien != 255)
    {
      // On récupère le type de liaison
      byte Type_Liaison = Lire_RAM(NoLiaison+4, NumeroSortie);

      // On récupère le MIN et MAX
      byte MIN = 0;
      byte MAX = 0;
      if(NoLiaison == LIAISON_ANALOG)
      {
        MIN = Lire_RAM(MIN_ANALOG, NoLien);
        MAX = Lire_RAM(MAX_ANALOG, NoLien);
      }
      else
      {
        MIN = 0;
        MAX = 0;
      }

      // On récupère la valeur actuelle du capteur ou E/S
      byte ValeurActu = 0;
      if(NoLiaison == LIAISON_ANALOG)
        ValeurActu = Lire_RAM(VALEUR_ACTU_ANALOG, NoLien);
      else
        ValeurActu = Lire_RAM(VALEUR_ACTU_ES, NoLien);

      // Selon le type de liaison, on regarde si c'est OK
      switch(Type_Liaison)
      {
        case 0: // Entre MIN et MAX (calcul inverse)
        if(ValeurActu<=MIN || ValeurActu>=MAX)
          return 0;
        break;

        case 1: // En dehors de MIN et MAX
        if(ValeurActu>=MIN && ValeurActu<=MAX)
          return 0;
        break;

        case 2: // >MIN
        if(ValeurActu<=MIN)
          return 0;
        break;

        case 3: // <MAX
        if(ValeurActu>=MAX)
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
void Ecrire_PIN_ES(byte PIN, boolean Etat)
{
  if(PIN<0 || PIN>=NB_PIN_IO)
    return;

  // On vérifie que c'est une sortie
  if(Type_ES[PIN]==VIDE || Type_ES[PIN]==SYSTEME || Type_ES[PIN]==ENTREE)
    return;

  // On écrit la donnée en RAM
  Ecrire_RAM(VALEUR_VOULUE_ES, PIN, Etat);
}

// Fonction de changement de mode d'une E/S
void Changer_Mode_PIN_ES(byte PIN, byte Mode)
{
  // On vérifie que le PIN existe
  if(PIN<0 || PIN>=NB_PIN_IO)
    return;

  // On vérifie que le mode existe
  if(Mode<0 || Mode>5)
    return;

  // On met à jour les variables
  Ecrire_RAM(TYPE_ES, PIN, Mode);
  Ecrire_RAM(VALEUR_ACTU_ES, PIN, 0);
  Ecrire_RAM(VALEUR_VOULUE_ES, PIN, 0);

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
void SYS_Remise_A_Zero_Systeme()
{
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
  // On réinititalise le mot de passe à 2560
  Password = 2560;
  // On écrit dans l'EEPROM
  SYS_Ecrire_Mot_De_Passe(2560);

  // On initialise les tableaux
  SYS_Initialiser_Tableaux_RAM();

  // On vide l'EEPROM
  unsigned long i = 0;
  for(i=100; i<(100+(2*NB_PIN_ANALOG)+(6*NB_PIN_IO)+(3*NB_PIN_PWM)); i++)
    EEPROM.write(i, 0);

  for(i=0; i<NB_PIN_IO; i++)
    SYS_Ecrire_Donnees_Vers_EEPROM(i, 0);
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
void Verifier_Reception_Commande()
{
  // On récupère la commande du PC
  SYS_Recuperer_Message(SYS_No_Port_Serie(PC));
  // S'il y a un message, on vérifie l'adresse
  if(stringComplete)
      Executer_Commande_Recue();

  // On récupère la commande de la télécommande
  SYS_Recuperer_Message(SYS_No_Port_Serie(TELECOMMANDE));
  // S'il y a un message, on vérifie l'adresse
  if(stringComplete)
      Executer_Commande_Recue();

  // On vérifie la présence de messages venant de l'extension 1 ou du maitre
  if(MAITRE_ESCLAVE == MAITRE)
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION1));
  else
    SYS_Recuperer_Message(SYS_No_Port_Serie(CARTE_MAITRE));
  if(stringComplete)
      Executer_Commande_Recue();

  // On vérifie la presence de messages venant de l'extension 2 ou de l'extension 1
  if(MAITRE_ESCLAVE == MAITRE || MAITRE_ESCLAVE == ESCLAVE1)
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION2));
  else
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION1));
  if(stringComplete)
      Executer_Commande_Recue();
}

// Fonction de récupération du port série
void SYS_Recuperer_Message(byte NoPort)
{
  // On vérifie que le port série existe sur cette carte
  if(NoPort<=0 || NoPort>NB_PORT_SERIE)
   return;

  SYS_Effacer_Commande();

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
          SYS_Effacer_Commande();
          return;
        }

        // Si le message est à relayer, on le transmet, et on efface la chaine
        if(NoDestinataire != MAITRE_ESCLAVE && SYS_No_Port_Serie(NoEnvoyeur) != 0)
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
          SYS_Transferer_Message(Commande);

          SYS_Effacer_Commande();
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
void Envoyer_Message(byte Type, String Message)
{
  // On ajoute les adresses
  Message = '['+String(MAITRE_ESCLAVE)+'|'+String(Type)+']'+Message;

  // On envoie le message
  SYS_Transferer_Message(Message);
}

// Fonction d'envoie d'un message, ayant déjà une adresse
void SYS_Transferer_Message(String Message)
{
  // On détermine le N° du serial sur lequel envoyer
  byte Type = 0;
  Type = Message.substring(3, 4).toInt();
  byte Port = SYS_No_Port_Serie(Type);
  if(Port>NB_PORT_SERIE || Port<=0)
    return;

  #if (DEBUG)
    Serial.print(Port);
    Serial.print(":\\ ");
    Serial.println(Message);
  #endif

  // On indique au chien de garde qu'on a envoyé le message
  switch(Type)
  {
    case MAITRE:
    DernierTempsMaitre_Envoie = 0;
    break;

    case EXTENSION1:
    DernierTempsEsclave1_Envoie = 0;
    break;

    case EXTENSION2:
    DernierTempsEsclave2_Envoie = 0;
    break;

    default:
    break;
  }

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
void SYS_Verification_Connexions()
{
  // Si ça fait moins de deux secondes qu'on a actualisé, on retourne
  if(DerniereActuConnexions+500 > millis())
    return;

  DerniereActuConnexions = millis();

  // On vérifie que les ports ouverts sont toujours en activité (watchdog)
  SYS_WatchDog_Serie();

  // On vérifie que les ports fermés ne sont pas en attente d'ouverture
  if(Ordinateur != 1 && Telecommande != 1 && Extension1 != 1 && Extension2 != 1 && Carte_Maitre != 1)
    SYS_Demande_Connexion(1);

  if(Ordinateur != 2 && Telecommande != 2 && Extension1 != 2 && Extension2 != 2 && Carte_Maitre != 2)
    SYS_Demande_Connexion(2);

  if(Ordinateur != 3 && Telecommande != 3 && Extension1 != 3 && Extension2 != 3 && Carte_Maitre != 3)
    SYS_Demande_Connexion(3);

  if(Ordinateur != 4 && Telecommande != 4 && Extension1 != 4 && Extension2 != 4 && Carte_Maitre != 4)
    SYS_Demande_Connexion(4);

  // On vérifie les connexions sur les cartes d'extension
  if(Extension1 && !Ordinateur)
  {
    Envoyer_Message(EXTENSION1, "OBR_CONNEXION_PC");
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION1));
    if(stringComplete)
      Ordinateur = Commande.toInt();
    SYS_Effacer_Commande();
  }

  if(Extension1 && !Telecommande)
  {
    Envoyer_Message(EXTENSION1, "OBR_CONNEXION_TELECOMMANDE");
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION1));
    if(stringComplete)
      Telecommande = Commande.toInt();
    SYS_Effacer_Commande();
  }

  if(Extension1 && !Extension2)
  {
    Envoyer_Message(EXTENSION1, "OBR_CONNEXION_EXTENSION2");
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION1));
    if(stringComplete)
      Extension2 = Commande.toInt();
    SYS_Effacer_Commande();
  }

  if(Extension2 && !Ordinateur)
  {
    Envoyer_Message(EXTENSION2, "OBR_CONNEXION_PC");
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION2));
    if(stringComplete)
      Ordinateur = Commande.toInt();
    SYS_Effacer_Commande();
  }

  if(Extension2 && !Telecommande)
  {
    Envoyer_Message(EXTENSION2, "OBR_CONNEXION_TELECOMMANDE");
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION2));
    if(stringComplete)
      Telecommande = Commande.toInt();
    SYS_Effacer_Commande();
  }

  if(Extension2 && !Extension1)
  {
    Envoyer_Message(EXTENSION2, "OBR_CONNEXION_EXTENSION1");
    SYS_Recuperer_Message(SYS_No_Port_Serie(EXTENSION2));
    if(stringComplete)
      Extension1 = Commande.toInt();
    SYS_Effacer_Commande();
  }

  // Si on est une carte d'extension, on envoie un message de connexion sur tous les ports libres
  if(MAITRE_ESCLAVE == ESCLAVE1 || MAITRE_ESCLAVE == ESCLAVE2)
  {
    NbActualisation++;
    if(NbActualisation < 4)
      return;

    NbActualisation = 0;
    if(Ordinateur != 1 && Telecommande != 1 && Extension1 != 1 && Extension2 != 1 && Carte_Maitre != 1)
      SYS_Envoie_Demande_Connexion(1);

    if(Ordinateur != 2 && Telecommande != 2 && Extension1 != 2 && Extension2 != 2 && Carte_Maitre != 2)
      SYS_Envoie_Demande_Connexion(2);

    if(Ordinateur != 3 && Telecommande != 3 && Extension1 != 3 && Extension2 != 3 && Carte_Maitre != 3)
      SYS_Envoie_Demande_Connexion(3);

    if(Ordinateur != 4 && Telecommande != 4 && Extension1 != 4 && Extension2 != 4 && Carte_Maitre != 4)
      SYS_Envoie_Demande_Connexion(4);
  }
}

// Chien de garde pour la déconnexion automatique des ports séries
void SYS_WatchDog_Serie()
{
  // Si ça fait plus de 2 secondes qu'on a pas envoyé un signal au chien de garde des autres cartes, on l'envoie
  if(DernierTempsMaitre_Envoie > 20)
  {
    DernierTempsMaitre_Envoie = 0;
    Envoyer_Message(MAITRE, "=");
  }

  if(DernierTempsEsclave1_Envoie > 20)
  {
    DernierTempsEsclave1_Envoie = 0;
    Envoyer_Message(EXTENSION1, "=");
  }

  if(DernierTempsEsclave2_Envoie > 20)
  {
    DernierTempsEsclave2_Envoie = 0;
    Envoyer_Message(EXTENSION2, "=");
  }

  // On met à jour la durée de tous les ports
  byte DifferenceTempsActu = (millis()-DernierTempsWatchdog)/100;
  DernierTempsWatchdog = millis();

  DernierTempsPC += DifferenceTempsActu;
  DernierTempsTelecommande += DifferenceTempsActu;
  DernierTempsEsclave1 += DifferenceTempsActu;
  DernierTempsEsclave2 += DifferenceTempsActu;
  DernierTempsMaitre += DifferenceTempsActu;
  DernierTempsEsclave1_Envoie += DifferenceTempsActu;
  DernierTempsEsclave2_Envoie += DifferenceTempsActu;
  DernierTempsMaitre_Envoie += DifferenceTempsActu;

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
void SYS_Envoie_Demande_Connexion(byte NoPort)
{
  // On récupère la réponse de la demande précédente
  SYS_Recuperer_Message(NoPort);

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
    Envoyer_Message(PC, "OK");
    DernierTempsPC = 0;
  }

  else if(Commande=="OBR_CONNECTER_TELECOMMANDE" && !Telecommande)
  {
    Telecommande = NoPort;
    Envoyer_Message(TELECOMMANDE, "OK");
    DernierTempsTelecommande = 0;
  }

  else if(Commande=="OBR_CONNECTER_EXTENSION1" && Extension1)
  {
    Extension1 = NoPort;
    Envoyer_Message(EXTENSION1, "OK");
    DernierTempsEsclave1 = 0;
  }

  else if(Commande=="OBR_CONNECTER_EXTENSION2" && Extension2)
  {
    Extension2 = NoPort;
    Envoyer_Message(EXTENSION2, "OK");
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
void SYS_Demande_Connexion(byte NoPort)
{
  // On regarde si un message est en attente
  SYS_Recuperer_Message(NoPort);

  // On regarde qui c'est, et on lui confirme la connexion si possible
  if(stringComplete)
  {
    // On regarde qui c'est, on remplie la variable, et on envoie une confirmation
    if(Commande=="OBR_CONNECTER_PC" && !Ordinateur)
    {
      Ordinateur = NoPort;
      Envoyer_Message(PC, "OK");
      DernierTempsPC = 0;
    }

    else if(Commande=="OBR_CONNECTER_TELECOMMANDE" && !Telecommande)
    {
      Telecommande = NoPort;
      Envoyer_Message(TELECOMMANDE, "OK");
      DernierTempsTelecommande = 0;
    }

    else if(Commande=="OBR_CONNECTER_EXTENSION1" && Extension1)
    {
      Extension1 = NoPort;
      Envoyer_Message(EXTENSION1, "OK");
      DernierTempsEsclave1 = 0;
    }

    else if(Commande=="OBR_CONNECTER_EXTENSION2" && Extension2)
    {
      Extension2 = NoPort;
      Envoyer_Message(EXTENSION2, "OK");
      DernierTempsEsclave2 = 0;
    }
  }
  SYS_Effacer_Commande();
}

// Fonction d'indication du numéro du port sur lequel est connecté tel truc (N° de port à partir de 1)
byte SYS_No_Port_Serie(byte Objet)
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
}