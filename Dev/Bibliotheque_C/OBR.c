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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#define Debug 0

#ifdef __WIN32__
    #include <windows.h>
    #define Sleep(n) Sleep(n)

#else /*Le reste, UNIX en particulier*/
    #include <unistd.h>
    #define Sleep(n) usleep(n)
#endif

/// Demande � l'arduino une connexion, en pr�cisant si on doit se faire passer pour un PC (Configuration) ou une t�l�commande (Pilotage), retourne l'adresse de la Commande_OBR si connect�
int OBR_Connecter(int NoPort, int TypePeripherique)
{
    if(TypePeripherique!=OBR_PC && TypePeripherique!=OBR_TELECOMMANDE)
        return 0;

    /// On ouvre le port s�rie en 57600 bauds
    int PortOBR = OuvrirCom(NoPort, 5);
    if(!PortOBR)
        return 0;

    /// On affiche que la connexion est faite, si on est en d�bug
    if(Debug)
	printf("Connecte au port %d\n", NoPort);

    /// On cr�e une Commande_OBR
    Commande_OBR *Commande = malloc(sizeof(Commande_OBR));
    if(!Commande)
        return -1;

    Commande->PortArduino = PortOBR;
    Commande->WatchdogLance = 0;

    /// On v�rifie la pr�sence de l'OBR sur l'arduino
    // On vide le buffer d'entr�e
    Recup_Commande(Commande);

    // V�rification de la pr�sence de l'OBR sur l'arduino
    if(TypePeripherique==OBR_PC)
        Envoie_Commande("OBR_CONNECTER_PC", Commande, 0);
    else
        Envoie_Commande("OBR_CONNECTER_TELECOMMANDE", Commande, 0);
    delay(500);
    Recup_Commande(Commande);

    // Si l'OBR n'est pas pr�sent
    if(strcmp(Commande->Commande, "OK"))
    {
        // On lib�re la m�moire de la Commande_OBR
        free(Commande);

        // On ferme le port s�rie
        FermerCom(PortOBR);

        // On retourne 0;
        return 0;
    }

    if(Debug)
        printf("OBR Connecte\n");

    /// On r�cup�re l'adresse
    Commande->MAITRE_ESCLAVE = Commande->NoEnvoyeur;
    Commande->TELECOMMANDE_PC = TypePeripherique;

    /// On r�cup�re sa version
    Commande->MAITRE_ESCLAVE = Commande->NoEnvoyeur;
    Envoie_Commande("OBR_VERSION", Commande, Commande->MAITRE_ESCLAVE);
    Recup_Commande(Commande);
    Commande->OBR_Version = atof(Commande->Commande);
    if(Debug)
        printf("Version : %f\n", Commande->OBR_Version);

    /// On r�cup�re son nom
    Envoie_Commande("OBR_NOM", Commande, Commande->MAITRE_ESCLAVE);
    Recup_Commande(Commande);
    strcpy(Commande->Nom, Commande->Commande);
    if(Debug)
        printf("Nom : %s\n", Commande->Nom);

    /// On initialise le mutex
    Commande->mutex = PTHREAD_MUTEX_INITIALIZER;

    /// Si la version est plus grande que la 0.7, on lance le watchdog
    if(Commande->OBR_Version >= 0.7)
        Activer_Watchdog(Commande);

    /// On retourne l'adresse de la Commande_OBR
    return Commande;
}

void OBR_Deconnecter(Commande_OBR *Commande)
{
    if(Commande->OBR_Version>=0.37)
    {
        Envoie_Commande("OBR_DECONNECTER_ADMIN", Commande, Commande->MAITRE_ESCLAVE);
        Envoie_Commande("OBR_DECONNEXION_AUTO-1", Commande, Commande->MAITRE_ESCLAVE);
        Envoie_Commande("OBR_DECONNECTER", Commande, Commande->MAITRE_ESCLAVE);
    }
    else
        Envoie_Commande("OBR_DECONNECTER", Commande, Commande->MAITRE_ESCLAVE);

    // On d�sactive le Watchdog
    Desactiver_Watchdog(Commande);

    // On supprime le mutex
    pthread_mutex_destroy(Commande->mutex);

    FermerCom(Commande->PortArduino);
    free(Commande);
}

int ArduinoPret(Commande_OBR *Commande)
{
    double InitTime = clock();
    double EndTime = InitTime;

    Envoie_Commande("OBR_PRET", Commande, Commande->MAITRE_ESCLAVE);

    while(InitTime+2000 > EndTime)
    {
        Recup_Commande(Commande);
        if(!strcmp(Commande->Commande, "VRAIE") || !strcmp(Commande->Commande, "ADMIN"))
        {
            if(!strcmp(Commande->Commande, "VRAIE"))
                return 1;
            else
                return 2;
        }
        delay(100);
        EndTime = clock();
    }
    return 0;
}

void Commande_Console(char* Buffer, Commande_OBR *Commande)
{
    int NbCaractere = 0;
    char DernierCaractere = NULL;
    strcpy(Buffer, "");

    /// Tant qu'on a pas de "\n", ou qu'on ne d�passe pas les 128 caract�res, on attend.
    while(DernierCaractere!='\n' && NbCaractere<128)
    {
        /// On r�cup�re les 128 premiers caract�res (ou jusqu'au \n)
        DernierCaractere = getchar();
        if(DernierCaractere!=NULL && DernierCaractere!='\n')
        {
            /// Si c'est une lettre minuscule, on la met en majuscule
            if(DernierCaractere <= 'z' && DernierCaractere >= 'a')
                DernierCaractere-=32;

            Buffer[NbCaractere]=DernierCaractere;
            NbCaractere++;
            Buffer[NbCaractere]='\0';
            DernierCaractere=NULL;
        }
    }
}

int Recup_Commande(Commande_OBR *Commande)
{
    strcpy(Commande->Commande, "");
    char Buffer[130]="";

    /// On attend que le mutex soit libre, et on le prend
    if(Commande->WatchdogLance)
        while(pthread_mutex_lock(&Commande->mutex)!=0);

    /// Tant qu'on a pas de "\n", ou qu'on est pas encore dans les 100ms, on attend.
    unsigned long Delay = clock();
    int ChainePrete = 0;

    /// On r�cup�re caract�re par caract�re, on s'arr�te au '\n'.
    while(!ChainePrete && Delay+200>clock())
    {
        RecevoirCom(Commande->PortArduino, Buffer);

        /// On assemble la chaine
        if(strcmp(Buffer, "") && strlen(Commande->Commande)+strlen(Buffer)<sizeof(Commande->Commande))
            strcat(Commande->Commande, Buffer);
        strcpy(Buffer, "");

        /// On regarde si la ligne est complete
        if(strchr(Commande->Commande, '\n'))
            ChainePrete = 1;
        delay(1);
    }

    /// S'il n'y a rien � r�cup�rer
    if(!ChainePrete)
    {
        /// On lib�re le mutex
        if(Commande->WatchdogLance)
            pthread_mutex_unlock(&Commande->mutex);
        return 1;
    }

    /// On s�parre la commande, des adresses
    strcpy(Buffer, Commande->Commande);
    sscanf(Buffer, "[%d|%d]%s\n", &Commande->NoEnvoyeur, &Commande->NoDestinataire, Commande->Commande);

    /// On laisse le temps au syst�me pour ne pas le surcharger
    delay(5);

    /// on lib�re le mutex
    if(Commande->WatchdogLance)
        pthread_mutex_unlock(&Commande->mutex);

    return 0;
}

void Envoie_Commande(char* Message, Commande_OBR *Commande, int NoDestinataire)
{
    char *Char = NULL, Temp[130]="";

    /// On attend que le mutex soit lib�r� et on le prend
    if(Debug)
        printf("Envoie Message : ");
    if(Commande->WatchdogLance)
        while(pthread_mutex_lock(&Commande->mutex)!=0);
    if(Debug)
        printf("En cours... ");

    /// Si on ne demande pas � quitter
    if(strcmp(Message, "EXIT"))
    {
        /// On remplace tous les " " par des "-"
        do
        {
            Char = strchr(Message, ' ');
            if(Char)
                *Char = '-';
        } while(Char!=NULL);

        /// La commande exacte est du type "OBR_[COMMANDE]-[PARAM1]-[PARAM2]-...".
        /// Ici, on peut �crire "_[COMMANDE]-...", voir juste "[COMMANDE]-...", le programme ajoute le "OBR_" devant.

        /// On regarde si la commande commence par une adresse
        if(!strncmp(Message, "M:", 2)) // Maitre
        {
            strcpy(Temp, Message);
            strcpy(Message, Temp+2);
            NoDestinataire = 3;
        }

        if(!strncmp(Message, "E1:", 3)) // Esclave1
        {
            strcpy(Temp, Message);
            strcpy(Message, Temp+3);
            NoDestinataire = 4;
        }

        if(!strncmp(Message, "E2:", 3)) // Esclave2
        {
            strcpy(Temp, Message);
            strcpy(Message, Temp+3);
            NoDestinataire = 5;
        }

        if(!strncmp(Message, "P:", 2)) // PC
        {
            strcpy(Temp, Message);
            strcpy(Message, Temp+2);
            NoDestinataire = 1;
        }

        if(!strncmp(Message, "T:", 2)) // Telecommande
        {
            strcpy(Temp, Message);
            strcpy(Message, Temp+2);
            NoDestinataire = 2;
        }

        /// On regarde si "_" est pr�sent.
        if(!strncmp(Message, "_", 1))
        {
            /// S'il le faut, on lui ajoute "OBR" devant la Commande->
            strcpy(Temp, "OBR");
            strcat(Temp, Message);
            strcpy(Message, Temp);
            strcpy(Temp, "");
        }
        /// On regarde si "OBR_" est absent.
        if(strncmp(Message, "OBR_", 4))
        {
            /// S'il le faut, on lui ajoute "OBR_" devant la Commande->
            strcpy(Temp, "OBR_");
            strcat(Temp, Message);
            strcpy(Message, Temp);
            strcpy(Temp, "");
        }
        /// On ajoute l'adresse � la commande
        sprintf(Commande->Commande, "[%d|%d]%s\n", Commande->TELECOMMANDE_PC, NoDestinataire, Message);

        /// On envoie la commande
        EnvoyerCom(Commande->Commande, Commande->PortArduino);

        // On met � jour le temps Watchdog
        Commande->Temps_WatchDog = clock();

        /// On laisse le temps au syst�me pour ne pas le surcharger
        delay(5);
    }
    /// On d�bloque le mutex
    if(Commande->WatchdogLance)
        pthread_mutex_unlock(&Commande->mutex);
    if(Debug)
        printf("FAIT !\n");
}

void MaintenirConnexion(Commande_OBR *Commande)
{
    if(Debug)
        printf("Watchdog actif\n");
    Commande->Temps_WatchDog = clock();
    char Message[10] = "";
    /// boucle infinie, arr�t�e lorsqu'on arr�te le WatchDog
    while(Commande->WatchdogLance)
    {
        /// On verrouille le mutex
        if(Debug)
            printf("Lock Mutex...\n");
        if(pthread_mutex_lock(&Commande->mutex)==0)
        {
            if(Debug)
                printf("Mutex locked...\n");
            // Si on doit envoyer un message de watchdog, on l'envoie
            if((clock() - Commande->Temps_WatchDog)>=500)
            {
                // On envoie le message
                sprintf(Message, "[%d|%d]=\n", Commande->TELECOMMANDE_PC, Commande->MAITRE_ESCLAVE);
                EnvoyerCom(Message, Commande->PortArduino);

                // On met � jour le temps Watchdog
                Commande->Temps_WatchDog = clock();
            }

            // On lib�re le mutex
            pthread_mutex_unlock(&Commande->mutex);
            if(Debug)
            printf("Unlock Mutex...\n");

            // On attend une seconde
            delay(250);
        }
    }
    if(Debug)
        printf("WatchDog inactif\n");
    return;
}

int Activer_Watchdog(Commande_OBR *Commande)
{
    // On cr�e le nouveau tread, et on l'enregistre dans la Commande_OBR
    pthread_t thread;
    Commande->WatchdogLance = 1;
    if(!pthread_create(&thread, NULL, MaintenirConnexion, Commande))
        Commande->Tread = thread;
    else
        Commande->WatchdogLance = 0;
}

void Desactiver_Watchdog(Commande_OBR *Commande)
{
    Commande->WatchdogLance = 0;
    pthread_exit(&Commande->Tread);
}

void delay(int millis)
{
    Sleep(millis);
    return;
}
