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
// - Doubler les liens (recréer 4 octets contenant 1 analog et 3 E/S, et un octet pour les types) (au final : 2 analog + 6 E/S)
// - Passage de 57600 à 155200 bauds
// - Connexion auto en tant qu'admin sur les esclaves, si maitre connecté en admin et maitre en mode de configuration
// - TESTER : SERVO, MOTEUR, SORTIE, ENTREE, Valeur à 0 Sortie, Valeur à 0 Entrée, pilotage via OBRoC
// - Attendre la réponse avant d'envoyer une nouvelle commande (évite les surcharges séries)
//////////////////////////////////////////// A FAIRE

#include <EEPROM.h>
#include <Servo.h>
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
#define DEBUG 1 // Uniquement pour le déboguage !!! Mettre 0 pour une utilisation finale !!!


// Constantes
// Version du micrologiciel
#define VERSION 0.8
// Session
#define CONFIGURATION 0
#define PILOTAGE 1
// Connexions serie
#define PC 1
#define TELECOMMANDE 2
#define CARTE_MAITRE 3 // MAITRE
#define EXTENSION1 4 // ESCLAVE1
#define EXTENSION2 5 // ESCLAVE2
// Type de message
#define COMMANDE 1
#define REPONSE 2
#define WATCHDOG 0
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
#define VALEUR_A_ZERO 18

// Définition des données selon la carte
#if (TYPE_DE_CARTE==_NANO)
#define NB_PIN_IO 14
#define NB_PIN_PWM 6
#define NB_PIN_ANALOG 8
#define NB_PORT_SERIE 1
#define NB_SERVO 12
#define PWM0 3
#define PWM1 5
#define PWM2 6
#define PWM3 11
#define PWM4 255
#define PWM5 255
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
#define NB_SERVO 12
#define PWM0 3
#define PWM1 5
#define PWM2 6
#define PWM3 11
#define PWM4 13
#define PWM5 255
#define PWM6 255
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
#define NB_SERVO 12
#define PWM0 3
#define PWM1 5
#define PWM2 6
#define PWM3 11
#define PWM4 255
#define PWM5 255
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
#define NB_SERVO 48
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
String ChaineReponse = "";
boolean ReponseRecue = 1;
boolean stringComplete = false; // Indique si une commande est arrivée
byte NbParam = 0; // Nombre de paramètres dans la commande
byte NoEnvoyeur = 0; // Adresse de l'envoyeur du dernier message
byte ID = 0; // ID du message (commande, réponse...)
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
byte Type_ES[NB_PIN_IO]; // Les différents type d'entrée/sortie
byte Min_PWM[NB_PIN_IO]; // Valeur Min d'un moteur/servo
byte Max_PWM[NB_PIN_IO]; // Valeur Max
byte Min_Analog[NB_PIN_ANALOG]; // Valeur Min d'un capteur
byte Max_Analog[NB_PIN_ANALOG]; // Valeur Max d'un capteur
byte Liaison_Analog[NB_PIN_ANALOG]; // Liaison E/S -> Analog
byte Liaison_1[NB_PIN_IO]; // Première liaison E/S -> E/S
byte Liaison_2[NB_PIN_IO]; // Deuxième
byte Liaison_3[NB_PIN_IO]; // Troisième
byte Type_Liaisons[NB_PIN_IO]; // 4x2 bits pour déterminer le type de liaison (0 = entre MIN et MAX, 1 = en dehors de MIN et MAX, 3 = >= MIN, 4 = <= MAX)
byte Valeur_Actu_Analog[NB_PIN_ANALOG]; // Valeur actuelle du PIN analog
byte Valeur_Actu_IO[NB_PIN_IO]; // Valeur actuelle du PIN I/O
byte Vitesse_Transition_PWM[NB_PIN_IO]; // Vitesse de transition des valeurs moteurs
byte Valeur_Voulue_IO[NB_PIN_IO]; // Valeur/vitesse voulue (pas forcément celle qu'on a en sortie)
Servo Servomoteur[NB_SERVO]; // Tableau des servomoteurs
byte PIN_Servomoteur[NB_SERVO]; // Tableau des N° des servomoteurs
boolean Valeur_A_Zero[NB_PIN_IO]; // Valeur à 0 des Entrées et des Sorties
unsigned long DernierTempsTransitionPWM = 0;
byte Nb_Servo_Utilises = 0;




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// MODIFIABLE PAR L'UTILISATEUR AVANT COMPILATION //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Fonction d'initialisation
void user_setup()
{
  Changer_Mode_PIN_ES(41, SORTIE);
}

// Boucle principale
void user_loop()
{
  Ecrire_RAM(VALEUR_ACTU_ES, 41, !Lire_RAM(VALEUR_ACTU_ES, 40));
  digitalWrite(41, !Lire_RAM(VALEUR_ACTU_ES, 40));
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
    Reponse="VRAIE";
  
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
    if(Carte_Maitre && Carte_Maitre!=SYS_No_Port_Serie(NoEnvoyeur)+10 || MAITRE_ESCLAVE==MAITRE)
      Reponse=Reponse+"1-";
    else
      Reponse=Reponse+"0-";
    if(Extension1 && Extension1!=SYS_No_Port_Serie(NoEnvoyeur)+10 || MAITRE_ESCLAVE==ESCLAVE1)
      Reponse=Reponse+"1-";
    else
      Reponse=Reponse+"0-";
    if(Extension2 && Extension2!=SYS_No_Port_Serie(NoEnvoyeur)+10 || MAITRE_ESCLAVE==ESCLAVE2)
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
    if(Telecommande && Telecommande!=SYS_No_Port_Serie(NoEnvoyeur)+10)
      Reponse=Reponse+"1";
    else
      Reponse=Reponse+"0";

    Envoyer_Message(NoEnvoyeur, COMMANDE, Reponse);
    SYS_Effacer_Commande();
    return;
  }

  // On nous envoie les différentes connexions interne
  if(Commande==F("OBR_ROBOT_CONNEXIONS"))
  {
    // S'il est vide, on le remplie
    if(!Carte_Maitre && param1.toInt())
      Carte_Maitre = SYS_No_Port_Serie(NoEnvoyeur)+10;
    if(!Extension1 && param2.toInt())
      Extension1 = SYS_No_Port_Serie(NoEnvoyeur)+10;
    if(!Extension2 && param3.toInt())
      Extension2 = SYS_No_Port_Serie(NoEnvoyeur)+10;

    // S'il est remplie mais que le NoEnvoyeur nous dit qu'il n'est plus connecté, on le vide
    if(Carte_Maitre==SYS_No_Port_Serie(NoEnvoyeur)+10 && !param1.toInt())
      Carte_Maitre = 0;
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
    if(!Telecommande && param2.toInt())
      Telecommande = SYS_No_Port_Serie(NoEnvoyeur)+10;

    // S'il est remplie mais plus connecté sur la carte externe, on le vide
    if(Ordinateur==SYS_No_Port_Serie(NoEnvoyeur)+10 && !param1.toInt())
      Ordinateur = 0;
    if(Telecommande==SYS_No_Port_Serie(NoEnvoyeur)+10 && !param2.toInt())
      Telecommande = 0;

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
        // On déconnecte toutes les Entrées/Sorties
        byte i=0;
        for(i=0; i<NB_PIN_IO; i++)
          Changer_Mode_PIN_ES(i, VIDE);

        // On redémarre
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

    // On configure un moteur
    if(Commande==F("OBR_CONNECTER_MOTEUR"))
    {
      if(Connecte)
      {
        // On vérifie le Numéro du PIN et les MIN et MAX
        if(No_PWM_PIN(param1.toInt())>NB_PIN_PWM || param2.toInt()>255 || param3.toInt()>255 || param2.toInt()>param3.toInt() && No_PWM_PIN(param1.toInt())!=255)
          Ecrire_Erreur(110030);
        else
        {
          // On configure le PIN en MOTEUR
          Changer_Mode_PIN_ES(param1.toInt(), MOTEUR);

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
    if(Commande==F("OBR_DECONNECTER_MOTEUR"))
    {
      if(Connecte)
      {
        // On vérifie le numéro du PIN
        if(No_PWM_PIN(param1.toInt())>NB_PIN_PWM)
          Ecrire_Erreur(130020);
        else
        {
          // On indique un PIN vide et on vide les MIN et MAX
          Changer_Mode_PIN_ES(param1.toInt(), VIDE);
          Ecrire_RAM(MIN_PWM, param1.toInt(), 0);
          Ecrire_RAM(MAX_PWM, param1.toInt(), 0);

          // Si c'est le PIN 13, on le rend au système
          if(param1.toInt()==13)
            Changer_Mode_PIN_ES(13, SYSTEME);

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

    // Connecte un servomoteur
    if(Commande==F("OBR_CONNECTER_SERVO"))
    {
      if(Connecte)
      {
        // On vérifie le Numéro du PIN et les MIN et MAX
        if(param1.toInt()>NB_PIN_IO || param2.toInt()>255 || param3.toInt()>255 || param2.toInt()>param3.toInt())
          Ecrire_Erreur(110030);
        else
        {
          // On vérifie qu'il n'y a pas de moteur sur les PIN interdits
          #if(TYPE_DE_CARTE==_MEGA)
            // PIN interdits : 11 et 12 (Seulement s'il y a plus de 12 servos)
            if(Nb_Servo_Utilises>12)
            {
              if(param1.toInt()==11 || param1.toInt()==12)
              {
                Reponse="Erreur_300011";
                Envoyer_Message(NoEnvoyeur, REPONSE, Reponse);
                SYS_Effacer_Commande();
                return;
              }
            }
          #else
            // PIN interdits : 9 et 10
            if(param1.toInt()==9 || param1.toInt()==10)
            {
              Reponse="Erreur_300011";
              Envoyer_Message(NoEnvoyeur, REPONSE, Reponse);
              SYS_Effacer_Commande();
              return;
            }
          #endif

          // On indique que c'est un servomoteur
          byte Retour = Changer_Mode_PIN_ES(param1.toInt(), SERVO);

          // On vérifie qu'il n'y a pas d'erreur, et on la retourne s'il y en a une
          switch(Retour)
          {
            case 0:
            break;

            case 1:
            Reponse="Erreur_300011";
            Envoyer_Message(NoEnvoyeur, REPONSE, Reponse);
            SYS_Effacer_Commande();
            return;

            case 2:
            Reponse="Erreur_300020";
            Envoyer_Message(NoEnvoyeur, REPONSE, Reponse);
            SYS_Effacer_Commande();
            return;

            case 3:
            Reponse="Erreur_300030";
            Envoyer_Message(NoEnvoyeur, REPONSE, Reponse);
            SYS_Effacer_Commande();
            return;

            default:
            Reponse="Erreur_300010";
            Envoyer_Message(NoEnvoyeur, REPONSE, Reponse);
            SYS_Effacer_Commande();
            return;
          }

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

    // On déconnecte un servomoteur
    if(Commande==F("OBR_DECONNECTER_SERVO"))
    {
      if(Connecte)
      {
        // On vérifie le numéro du PIN
        if(param1.toInt()>NB_PIN_IO)
          Ecrire_Erreur(110031);
        else
        {
          // On indique un PIN vide
          Changer_Mode_PIN_ES(param1.toInt(), VIDE);

          // Si c'est le PIN 13, on le rend au système
          if(param1.toInt()==13)
            Changer_Mode_PIN_ES(13, SYSTEME);

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
    if(Commande==F("OBR_CONNECTER_CAPTEUR"))
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
    if(Commande==F("OBR_DECONNECTER_CAPTEUR"))
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
    if(Commande==F("OBR_CONNECTER_ENTREE"))
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
    if(Commande==F("OBR_CONNECTER_SORTIE"))
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
    if(Commande==F("OBR_LIER_CAPTEUR"))
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
    if(Commande==F("OBR_LIER_ES"))
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
    if(Commande==F("OBR_DELIER"))
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
    if(Commande==F("OBR_MOTEUR_MIN_MAX") || Commande==F("OBR_SERVO_MIN_MAX"))
    {
      if(Connecte)
      {
        // On vérifie les paramètres
        if(No_PWM_PIN(param1.toInt())>NB_PIN_IO)
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
    if(Commande==F("OBR_CAPTEUR_MIN_MAX"))
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
    if(Commande==F("OBR_VITESSE_TRANSITION_MOTEUR"))
    {
      // On vérifie que c'est bien un moteur ou un capteur
      if(Connecte)
      {
        if(No_PWM_PIN(param1.toInt())<NB_PIN_PWM)
        {
          if(Lire_RAM(TYPE_ES, param1.toInt())==MOTEUR)
          {
            // On vérifie la vitesse de transmission (entre 0 et 255)
            if(param2.toInt()>250 || param2.toInt()<1)
              Ecrire_Erreur(110032);
            else
            {
              // On adapte la nouvelle valeur
              byte Vitesse = param2.toInt();
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

    // Définit la vitesse de transition des valeurs d'une sortie PWM (250=0s, 1=5s, 0=interdit; map()=> 0=0s, 250=2,5s)
    if(Commande==F("OBR_VITESSE_TRANSITION_SERVO"))
    {
      // On vérifie que c'est bien un moteur ou un capteur
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          if(Lire_RAM(TYPE_ES, param1.toInt())==SERVO)
          {
            // On vérifie la vitesse de transmission (entre 0 et 255)
            if(param2.toInt()>250 || param2.toInt()<1)
              Ecrire_Erreur(110032);
            else
            {
              // On adapte la nouvelle valeur
              byte Vitesse = param2.toInt();
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

    // Définit la valeur à 0 d'une entrée ou d'une sortie
    if(Commande==F("OBR_VALEUR_A_ZERO"))
    {
      // On vérifie que c'est bien un moteur ou un capteur
      if(Connecte)
      {
        if(param1.toInt()<NB_PIN_IO)
        {
          if(param2.toInt()<2)
          {
            if(Lire_RAM(TYPE_ES, param1.toInt())==SORTIE || Lire_RAM(TYPE_ES, param1.toInt())==ENTREE)
            {
              Ecrire_RAM(VALEUR_A_ZERO, param1.toInt(), param2.toInt());
              Reponse = "OK";
            }
            else
              Ecrire_Erreur(130020);
          }
          else
            Ecrire_Erreur(110032);
        }
        else
          Ecrire_Erreur(110031);
      }
      else
        Ecrire_Erreur(130010); 
    }

    // Enregistrer les paramètres
    if(Commande==F("OBR_ENREGISTRER_PARAMETRES") && Connecte)
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

    // On change la vitesse du moteur
    if(Commande==F("OBR_VITESSE_MOTEUR"))
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
    if(Commande==F("OBR_ARRETER_MOTEUR"))
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
    if(Commande==F("OBR_POSITION_SERVO"))
    {
      // On vérifie que la sortie est un servomoteur
      if(Lire_RAM(TYPE_ES, param1.toInt())!=SERVO)
        Ecrire_Erreur(110021);
      else
      {
        // On met à jour la valeur en tenant compte des limites
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

    // 
    if(Commande==F("OBR_ALLUMER_SORTIE"))
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
    if(Commande==F("OBR_ARRETER_SORTIE"))
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
  // Servo
  Nb_Servo_Utilises = 0;

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
  Changer_Mode_PIN_ES(NoES, TypeES);

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
    // Min PWM
    Min_PWM[NoES] = EEPROM.read(100 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));

    // Max PWM
    Max_PWM[NoES] = EEPROM.read(101 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));

    // Transition PWM
    Vitesse_Transition_PWM[NoES] = EEPROM.read(102 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3));
  }

  // Valeur à 0
  if(TypeES==SORTIE || TypeES==ENTREE)
  {
    // Valeur à 0
    Valeur_A_Zero[NoES] = EEPROM.read(100 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (3*NB_PIN_IO) + NoES);
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
    // Min PWM
    EEPROM.write(100 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Min_PWM[NoES]);

    // Max PWM
    EEPROM.write(101 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Max_PWM[NoES]);

    // Transition PWM
    EEPROM.write(102 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (NoES*3), Vitesse_Transition_PWM[NoES]);
  }

  // Valeur à 0
  if(TypeES==SORTIE || TypeES==ENTREE)
  {
    // Valeur à 0
    EEPROM.write(100 + (2*NB_PIN_ANALOG) + (6*NB_PIN_IO) + (3*NB_PIN_IO) + NoES, Valeur_A_Zero[NoES]);
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
  {
    Type_ES[i]=0; // Tout à 0
    Min_PWM[i]=0; // Tout à 0
    Max_PWM[i]=255; // Tout à 255
    Liaison_1[i]=255; // Tout à 255
    Liaison_2[i]=255; // Tout à 255
    Liaison_3[i]=255; // Tout à 255
    Type_Liaisons[i]=0; // Tout à 0
    Valeur_Actu_IO[i]=0; // Tout à 0
    Vitesse_Transition_PWM[i]=255; // Tout à 255
    Valeur_Voulue_IO[i]=0; // Tout à 0
    Valeur_A_Zero[i]=0; // Tout à 0
    PIN_Servomoteur[i]=0; // Tout à 0
  }

  for(i=0; i<NB_PIN_ANALOG; i++)
  {
    Min_Analog[i]=0; // Tout à 0
    Max_Analog[i]=255; // Tout à 255
    Liaison_Analog[i]=255; // Tout à 255
    Valeur_Actu_Analog[i]=0; // Tout à 0
  }
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
    {
      if(!Lire_RAM(VALEUR_A_ZERO, i))
        Ecrire_RAM(VALEUR_ACTU_ES, i, digitalRead(i));
      else
        Ecrire_RAM(VALEUR_ACTU_ES, i, 1-digitalRead(i));
    }
  }

  // On appel la fonction de mise à jour de l'état des sorties
  SYS_Mise_A_Jour_Des_ES();
}

// Fonction de lecture du N° de PIN PWM
byte No_PIN_PWM(byte NoPWM)
{
  // Si c'est un MEGA et qu'il y a plus de 12 servomoteurs, on verrouille les PIN 11 et 12
  if(TYPE_DE_CARTE==_MEGA && Nb_Servo_Utilises>12)
    if(NoPWM==9 || NoPWM==10)
      return 255;

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
  if(Valeur > 255 || NumeroCase >= NB_PIN_IO)
    return;

  switch(Type)
  {
    case VALEUR_A_ZERO:
    Valeur_A_Zero[NumeroCase] = Valeur;
    break;

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
    Min_PWM[NumeroCase] = Valeur;
    break;

    case MAX_PWM:
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
    Vitesse_Transition_PWM[NumeroCase] = Valeur;
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
  // On évite les dépassements de tempon
  if(NumeroCase >= NB_PIN_IO)
    return 0;

  // On détermine le type de tableau que nous devons lire et on retourne la veleur de la case demandée
  switch(Type)
  {
    case VALEUR_A_ZERO:
    return Valeur_A_Zero[NumeroCase];
    break;

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
    return Min_PWM[NumeroCase];
    break;

    case MAX_PWM:
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
    return Vitesse_Transition_PWM[NumeroCase];
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
      analogWrite(i, SYS_Autoriser_Sortie(i));
      break;

      case SERVO:
      Valeur_Servo(i, SYS_Autoriser_Sortie(i));
      // Si le servo vaut 0 et le MIN est plus grand, on met le servo au millieu
      if(Lire_RAM(VALEUR_ACTU_ES, i)==0 && Lire_RAM(MIN_PWM, i)!=0)
        Ecrire_RAM(VALEUR_ACTU_ES, i, ((Lire_RAM(MAX_PWM, i)-Lire_RAM(MIN_PWM, i))/2)+Lire_RAM(MIN_PWM, i));

      // Si le servo sort de ses limites, on les met dans ses limites
      if(Lire_RAM(VALEUR_ACTU_ES, i)>Lire_RAM(MAX_PWM, i))
        Ecrire_RAM(VALEUR_ACTU_ES, i, Lire_RAM(MAX_PWM, i));
      if(Lire_RAM(VALEUR_ACTU_ES, i)<Lire_RAM(MIN_PWM, i))
        Ecrire_RAM(VALEUR_ACTU_ES, i, Lire_RAM(MIN_PWM, i));

      break;

      case SORTIE:
      if(!Lire_RAM(VALEUR_A_ZERO, i))
        digitalWrite(i, SYS_Autoriser_Sortie(i));
      else
        digitalWrite(i, 1-SYS_Autoriser_Sortie(i));
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
    byte VitesseTransition = map(Lire_RAM(VITESSE_TRANSITION_PWM, NumeroSortie), 0, 250, 250, 1);

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
byte Changer_Mode_PIN_ES(byte PIN, byte Mode)
{
  // On vérifie que le PIN existe
  if(PIN<0 || PIN>=NB_PIN_IO)
    return 0;

  // On vérifie que le mode existe
  if(Mode<0 || Mode>5)
    return 0;

  // On vérifie que c'était pas un servo à la base
  if(Lire_RAM(TYPE_ES, PIN)==SERVO)
    Deconnecter_Servo(PIN);

  // On met à jour les variables
  Ecrire_RAM(TYPE_ES, PIN, Mode);
  Ecrire_RAM(VALEUR_ACTU_ES, PIN, 0);
  Ecrire_RAM(VALEUR_VOULUE_ES, PIN, 0);

  // On met à jour le type de sortie
  switch(Mode)
  {
    case SORTIE:
    case MOTEUR:
    case SYSTEME:
    pinMode(PIN, OUTPUT);
    return 0;

    case SERVO:
    return Connecter_Servo(PIN);

    default:
    pinMode(PIN, INPUT);
    return 0;
  }
}

// Connexion d'un servomoteur sur un PIN, retourne 0 si tout est OK, 1 si non autorisé, 2 si tous les Servos sont pris.
byte Connecter_Servo(byte NoPIN)
{
  // Si tous les PIN sont pris
  if(Nb_Servo_Utilises>NB_SERVO)
    return 2;

  // Si le PIN n'est pas autorisé
  if(NoPIN>NB_PIN_IO || ((NoPIN==11 || NoPIN==12) && TYPE_DE_CARTE==_MEGA) || ((NoPIN==9 || NoPIN==10) && TYPE_DE_CARTE!=_MEGA))
    return 1;

  // On vérifie que le PIN n'est pas déjà connecté à un servomoteur
  byte i=0;
  for(i=0; i<NB_SERVO; i++)
  {
    if(PIN_Servomoteur[i]==NoPIN)
      return 1;
  }

  // Si on doit interdire les PIN interdits, on les vides (s'ils sont en PWM)
  #if(TYPE_DE_CARTE==_MEGA)
  if((Lire_RAM(TYPE_ES, 11)==SERVO || Lire_RAM(TYPE_ES, 11)==MOTEUR) && Nb_Servo_Utilises>=11)
    Changer_Mode_PIN_ES(11, VIDE);
  if((Lire_RAM(TYPE_ES, 12)==SERVO || Lire_RAM(TYPE_ES, 12)==MOTEUR) && Nb_Servo_Utilises>=11)
    Changer_Mode_PIN_ES(12, VIDE);
  #else
  if(Lire_RAM(TYPE_ES, 9)==SERVO || Lire_RAM(TYPE_ES, 9)==MOTEUR)
    Changer_Mode_PIN_ES(9, VIDE);
  if(Lire_RAM(TYPE_ES, 10)==SERVO || Lire_RAM(TYPE_ES, 10)==MOTEUR)
    Changer_Mode_PIN_ES(10, VIDE);
  #endif


  // On inscrit le nouveau PIN en tant que Servomoteur
  for(i=0; i<NB_SERVO; i++)
  {
    if(PIN_Servomoteur[i]==0)
    {
      Servomoteur[i].attach(NoPIN);
      PIN_Servomoteur[i] = NoPIN;
      Ecrire_RAM(TYPE_ES, NoPIN, SERVO);
      Nb_Servo_Utilises++;
      return 0;
    }
  }
  return 3;
}

// Deconnexion d'un servo
void Deconnecter_Servo(byte NoPin)
{
  // On vérifie que le PIN est connecté à un servomoteur
  byte i=0;
  byte Servo = 255;
  for(i=0; i<NB_SERVO; i++)
  {
    if(PIN_Servomoteur[i]==NoPin)
      Servo = i;
  }

  // S'il n'y a pas de servomoteur sur ce PIN, on retourne
  if(Servo)
    return;

  // On inscrit le nouveau PIN en tant que VIDE
  Servomoteur[Servo].detach();
  PIN_Servomoteur[Servo] = 0;
  Ecrire_RAM(TYPE_ES, NoPin, VIDE);
  Nb_Servo_Utilises--;
}

// Déplace le servomoteur
void Valeur_Servo(byte NoPIN, byte Valeur)
{
  // On vérifie que le PIN est connecté à un servomoteur
  byte i=0;
  byte Servo = 255;
  for(i=0; i<NB_SERVO; i++)
  {
    if(PIN_Servomoteur[i]==NoPIN)
      Servo = i;
  }

  // S'il n'y a pas de servomoteur sur ce PIN, on retourne
  if(Servo==255)
    return;

  // On indique la nouvelle valeur sur la sortie PWM
  Servomoteur[Servo].write(Valeur);

  // On écrit la nouvelle valeur en RAM
  Ecrire_RAM(VALEUR_ACTU_ES, NoPIN, Servomoteur[Servo].read());
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
        // Si le watchdog est activé, on remet le compteur à 0
        if(watchdogActif)
        {
          switch(NoEnvoyeur)
          {
            case PC:
            if(Ordinateur<10)
              DernierTempsPC = 0;
            break;

            case TELECOMMANDE:
            if(Telecommande<10)
              DernierTempsTelecommande = 0;
            break;

            case ESCLAVE1:
            if(Extension1<10)
              DernierTempsEsclave1 = 0;
            break;

            case ESCLAVE2:
            if(Extension2<10)
              DernierTempsEsclave2 = 0;
            break;

            case MAITRE:
            if(Carte_Maitre<10)
              DernierTempsMaitre = 0;
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
          Serial.print(millis());
          Serial.print(" Recoit <== ");
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
        if(NoDestinataire != MAITRE_ESCLAVE && NoEnvoyeur != 0)
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
  Message = "["+String(MAITRE_ESCLAVE)+String(Destinataire)+String(Type)+"]"+String(Message);

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
      Serial.print(millis());
      Serial.print(" Envoie ==> ");
      Serial.print(Port);
      Serial.print(":\\ ");
      Serial.print(Message);
      Serial.print("\n");
    }
  #endif

  // On indique au chien de garde qu'on a envoyé le message
  switch(Type)
  {
    case MAITRE:
    if(Carte_Maitre<10)
      DernierTempsMaitre_Envoie = 0;
    break;

    case EXTENSION1:
    if(Extension1<10)
      DernierTempsEsclave1_Envoie = 0;
    break;

    case EXTENSION2:
    if(Extension2<10)
      DernierTempsEsclave2_Envoie = 0;
    break;

    default:
    break;
  }

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

  // On vérifie que les ports fermés ne sont pas en attente d'ouverture
  if(Ordinateur != 1 && Telecommande != 1 && Extension1 != 1 && Extension2 != 1 && Carte_Maitre != 1)
    SYS_Demande_Connexion(1);

  if(NB_PORT_SERIE>1)
  {
    if(Ordinateur != 2 && Telecommande != 2 && Extension1 != 2 && Extension2 != 2 && Carte_Maitre != 2)
      SYS_Demande_Connexion(2);

    if(Ordinateur != 3 && Telecommande != 3 && Extension1 != 3 && Extension2 != 3 && Carte_Maitre != 3)
      SYS_Demande_Connexion(3);

    if(Ordinateur != 4 && Telecommande != 4 && Extension1 != 4 && Extension2 != 4 && Carte_Maitre != 4)
      SYS_Demande_Connexion(4);
  }

  // On demande à chaque carte quelles sont leurs connexions
  switch(dernierCycleConnexions)
  {
    case 1: // Maitre
    if(Carte_Maitre<10 && Carte_Maitre>0)
    {
      Envoyer_Message(MAITRE, COMMANDE, "OBR_CONNEXIONS_ROBOT");
      Envoyer_Message(MAITRE, COMMANDE, "OBR_CONNEXIONS_EXTERNE");
    }
    break;

    case 2: // E1
    if(Extension1<10 && Extension1>0)
    {
      Envoyer_Message(EXTENSION1, COMMANDE, "OBR_CONNEXIONS_ROBOT");
      Envoyer_Message(EXTENSION1, COMMANDE, "OBR_CONNEXIONS_EXTERNE");
    }
    break;

    case 3: // E2
    if(Extension2<10 && Extension2>0)
    {
      Envoyer_Message(EXTENSION2, COMMANDE, "OBR_CONNEXIONS_ROBOT");
      Envoyer_Message(EXTENSION2, COMMANDE, "OBR_CONNEXIONS_EXTERNE");
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
  if(DernierTempsMaitre_Envoie > 30 && Carte_Maitre<10)
  {
    DernierTempsMaitre_Envoie = 0;
    Envoyer_Message(MAITRE, 0, "=");
  }

  if(DernierTempsEsclave1_Envoie > 30 && Extension1<10)
  {
    DernierTempsEsclave1_Envoie = 0;
    Envoyer_Message(EXTENSION1, 0, "=");
  }

  if(DernierTempsEsclave2_Envoie > 30 && Extension2<10)
  {
    DernierTempsEsclave2_Envoie = 0;
    Envoyer_Message(EXTENSION2, 0, "=");
  }

  if(DernierTempsTelecommande_Envoie > 30 && Telecommande<10)
  {
    DernierTempsTelecommande_Envoie = 0;
    Envoyer_Message(TELECOMMANDE, 0, "=");
  }

  // On met à jour la durée de tous les ports
  byte DifferenceTempsActu = (millis()-DernierTempsWatchdog)/10;
  DernierTempsWatchdog = millis();

  DernierTempsPC += DifferenceTempsActu;
  DernierTempsTelecommande += DifferenceTempsActu;
  DernierTempsEsclave1 += DifferenceTempsActu;
  DernierTempsEsclave2 += DifferenceTempsActu;
  DernierTempsMaitre += DifferenceTempsActu;
  DernierTempsEsclave1_Envoie += DifferenceTempsActu;
  DernierTempsEsclave2_Envoie += DifferenceTempsActu;
  DernierTempsMaitre_Envoie += DifferenceTempsActu;
  DernierTempsTelecommande_Envoie += DifferenceTempsActu;

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
    if(Extension1-10==Ordinateur)
      Extension1=0;
    if(Extension2-10==Ordinateur)
      Extension2=0;
    if(Carte_Maitre-10==Ordinateur)
      Carte_Maitre=0;
    if(Telecommande-10==Ordinateur)
      Telecommande=0;

    // On déconnecte la carte
    Ordinateur = 0;

    // Si la télécommande n'est pas connectée, on coupe toutes les sorties
    if(!Telecommande)
    {
      byte i=0;
      for(i=0; i<NB_PIN_IO; i++)
        if(Lire_RAM(TYPE_ES, i)==MOTEUR || Lire_RAM(TYPE_ES, i)==SORTIE)
        {
          Ecrire_RAM(VALEUR_VOULUE_ES, i, 0);
          Ecrire_RAM(VALEUR_ACTU_ES, i, 0);
        }
    }
  }

  if(DernierTempsTelecommande > 100 && Telecommande<10)
  {
    // On déconnecte toutes les connexions qui passaient par cette carte là
    if(Extension1-10==Telecommande)
      Extension1=0;
    if(Extension2-10==Telecommande)
      Extension2=0;
    if(Carte_Maitre-10==Telecommande)
      Carte_Maitre=0;
    if(Ordinateur-10==Telecommande)
      Ordinateur=0;

    // On déconnecte la carte
    Telecommande = 0;

    // Si la PC n'est pas connecté, on arrête toutes les sorties
    if(!Ordinateur)
    {
      byte i=0;
      for(i=0; i<NB_PIN_IO; i++)
        if(Lire_RAM(TYPE_ES, i)==MOTEUR || Lire_RAM(TYPE_ES, i)==SORTIE)
        {
          Ecrire_RAM(VALEUR_VOULUE_ES, i, 0);
          Ecrire_RAM(VALEUR_ACTU_ES, i, 0);
        }
    }
  }

  if(DernierTempsMaitre > 100 && Carte_Maitre<10)
  {
    // On déconnecte toutes les connexions qui passaient par cette carte là
    if(Extension1-10==Carte_Maitre)
      Extension1=0;
    if(Extension2-10==Carte_Maitre)
      Extension2=0;
    if(Ordinateur-10==Carte_Maitre)
      Ordinateur=0;
    if(Telecommande-10==Carte_Maitre)
      Telecommande=0;

    // On déconnecte la carte
    Carte_Maitre = 0;
  }

  if(DernierTempsEsclave1 > 100 && Extension1<10)
  {
    // On déconnecte toutes les connexions qui passaient par cette carte là
    if(Ordinateur-10==Extension1)
      Ordinateur=0;
    if(Extension2-10==Extension1)
      Extension2=0;
    if(Carte_Maitre-10==Extension1)
      Carte_Maitre=0;
    if(Telecommande-10==Extension1)
      Telecommande=0;

    // On déconnecte la carte
    Extension1 = 0;
  }

  if(DernierTempsEsclave2 > 100 && Extension2<10)
  {
    // On déconnecte toutes les connexions qui passaient par cette carte là
    if(Ordinateur-10==Extension2)
      Ordinateur=0;
    if(Extension1-10==Extension2)
      Extension1=0;
    if(Carte_Maitre-10==Extension2)
      Carte_Maitre=0;
    if(Telecommande-10==Extension2)
      Telecommande=0;

    // On déconnecte la carte
    Extension2 = 0;
  }
}

// Fonction de vérification d'une nouvelle connexion au port série
void SYS_Reception_Demande_Connexion(byte NoPort)
{
  // On regarde qui c'est, et on lui confirme la connexion si possible
  if(stringComplete)
  {
    #if(DEBUG)
      if(Commande==F("OBR_CONNECTER_PC"))
        Serial.print("Demande de connexion du PC\n");

      if(Commande==F("OBR_CONNECTER_TELECOMMANDE"))
        Serial.print("Demande de connexion de la telecommande\n");

      if(Commande==F("OBR_CONNECTER_EXTENSION1"))
        Serial.print("Demande de connexion de l'extension1\n");

      if(Commande==F("OBR_CONNECTER_EXTENSION2"))
        Serial.print("Demande de connexion de l'extension2\n");
    #endif

    // On regarde qui c'est, on remplie la variable, et on envoie une confirmation
    if(Commande=="OBR_CONNECTER_PC" && !Ordinateur)
    {
      Ordinateur = NoPort;
      Envoyer_Message(PC, 3, "OK");
      DernierTempsPC = 0;
    }

    else if(Commande=="OBR_CONNECTER_TELECOMMANDE" && !Telecommande)
    {
      Telecommande = NoPort;
      Envoyer_Message(TELECOMMANDE, 3, "OK");
      DernierTempsTelecommande = 0;
    }

    else if(Commande=="OBR_CONNECTER_EXTENSION1" && !Extension1)
    {
      Extension1 = NoPort;
      Envoyer_Message(EXTENSION1, 3, "OK");
      DernierTempsEsclave1 = 0;
    }

    else if(Commande=="OBR_CONNECTER_EXTENSION2" && !Extension2)
    {
      Extension2 = NoPort;
      Envoyer_Message(EXTENSION2, 3, "OK");
      DernierTempsEsclave2 = 0;
    }

    // On regarde si on a pas reçu un accusé de réception
    else if(Commande=="OK" && MAITRE_ESCLAVE!=MAITRE)
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
  }
  SYS_Effacer_Commande();
}

void SYS_Demande_Connexion(byte NoPort)
{
  // Si on est un esclave et qu'on est pas connecté au maitre, on envoie une demande de connexion
  if(MAITRE_ESCLAVE!=MAITRE && Carte_Maitre==0)
  {
    if(NoPort==1)
      if(MAITRE_ESCLAVE==ESCLAVE1)
        Serial.print("[003]OBR_CONNECTER_EXTENSION1\n");
      else
        Serial.print("[003]OBR_CONNECTER_EXTENSION2\n");

    #if (NB_PORT_SERIE>1)
      if(NoPort==2)
        if(MAITRE_ESCLAVE==ESCLAVE1)
          Serial1.print("[003]OBR_CONNECTER_EXTENSION1\n");
        else
          Serial1.print("[003]OBR_CONNECTER_EXTENSION2\n");

      if(NoPort==3)
        if(MAITRE_ESCLAVE==ESCLAVE1)
          Serial2.print("[003]OBR_CONNECTER_EXTENSION1\n");
        else
          Serial2.print("[003]OBR_CONNECTER_EXTENSION2\n");

      if(NoPort==4)
        if(MAITRE_ESCLAVE==ESCLAVE1)
          Serial3.print("[003]OBR_CONNECTER_EXTENSION1\n");
        else
          Serial3.print("[003]OBR_CONNECTER_EXTENSION2\n");
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

    case TELECOMMANDE:
    if(Telecommande<10)
      return Telecommande;
    else
      return Telecommande-10;

    case MAITRE:
    if(Carte_Maitre<10)
      return Carte_Maitre;
    else
      return Carte_Maitre-10;

    case EXTENSION1:
    if(Extension1<10)
      return Extension1;
    else
      return Extension1-10;

    case EXTENSION2:
    if(Extension2<10)
      return Extension2;
    else
      return Extension2-10;

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
  ID = 0;
  NoEnvoyeur = 0;
  NoDestinataire = 0;
}