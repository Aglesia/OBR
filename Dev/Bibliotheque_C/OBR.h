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

#ifndef OBR_H_INCLUDED
#define OBR_H_INCLUDED

#ifndef SERIAL_H_INCLUDED
#error "Aucune fonction de lecture du port s�rie n'est disponible !!!"
#error "Veuillez inclure Serial.h"
#endif // SERIAL_H_INCLUDED


/// d�finition d'une structure de commande
typedef struct Commande_OBR
{
    char* Commande[128]; // Derniere commande re�ue
    char* Nom[128]; // Nom du robot
    int PortArduino; // Port s�rie sur lequel est connect� l'arduino
    int NoEnvoyeur; // N� du dernier envoyeur
    int NoDestinataire; // N� du dernier destinataire
    int MAITRE_ESCLAVE; // D�finit sur quel type de carte le PC est branch� (la variable contient l'adresse)
    int TELECOMMANDE_PC; // 1 si c'est le PC, 2 si c'est la t�l�commande
    double OBR_Version; // Version de l'OBR install� sur l'arduino
} Commande_OBR;

#define OBR_VERSION "1.3"
#define OBR_PC 1
#define OBR_TELECOMMANDE 2

/// R�cup�re la commande tap�e en console (inutile pour un programme graphique)
void OBR_Commande_Console(char* Buffer, Commande_OBR *Commande);

/// R�cup�re la derni�re commande re�ue par l'arduino, retourne l'adresse de l'envoyeur
int OBR_Recup_Commande(Commande_OBR *Commande);

/// Envoie une commande � l'arduino
void OBR_Envoie_Commande(char* Message, Commande_OBR *Commande, int NoDestinataire);

/// Demande � l'arduino s'il est pret, retourne 0 si OK
int OBR_Arduino_Pret(Commande_OBR *Commande);

/// Envoie un message de watchdog pour maintenir la connexion active
void OBR_Maintenir_Connexion(Commande_OBR *Commande);

/// Demande � l'arduino une connexion, en se faisant passer pour un PC (configuration) ou une t�l�commande (pilotage), retourne l'adresse de la Commande_OBR si connect�
int OBR_Connecter(int NoPort, int TypePeripherique);

/// Deconnecte l'arduino du PC
void OBR_Deconnecter(Commande_OBR *Commande);

/// Faire une pause x ms
void delay(int millis);

#include "OBR.c"

#endif // OBR_H_INCLUDED
