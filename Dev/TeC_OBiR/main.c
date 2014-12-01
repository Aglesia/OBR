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
#include "../../../Bibliotheques/include/Serial/Serial.h"
#include "../BibliothequeOBR/OBR.h"

/// Liste des fonctions
void AfficherAide();

int main()
{
    Commande_OBR *Commande = 0;
    int NoPort = 0, temp = 0;
    char Buffer[130] = "";

    #ifdef WIN32
        system("Title Tec'OBir");
        system("COLOR 09");
    #endif

    /// Message de bienvenue
    printf("Bienvenue dans TeC'OBiR (Terminal Configuration for Open Bidouille Robot) V1.3\nCe programme est developpe par Dorian Fragni et le FabLab des fabriques du ponan.\nIcone de http://icons8.com.\nLe programme sert a configurer/deboguer un arduino tournant sour Open Bidouille Robot.\n\n");
    printf("Scan des ports en cours...\n");
    Commande = 0;

    while(!Commande)
    {
        for(NoPort=0; NoPort<10; NoPort++)
        {
            Commande = OBR_Connecter(NoPort, OBR_PC);
            if(Commande)
                break;
        }
        delay(200);
    }

    if(Commande)
    {
        printf("\n%s V%f Connecte !\n", Commande->Nom, Commande->OBR_Version);

        /// Si la version est plus grande que la 0.2 mais plus petite que la 0.7
        if(Commande->OBR_Version >= 0.3 && Commande->OBR_Version < 0.7)
        {
            /// On désactive la déconnexion auto
            if(Commande->OBR_Version >= 0.37)
                Envoie_Commande("OBR_DECONNEXION_AUTO-0", Commande, Commande->MAITRE_ESCLAVE);
            Recup_Commande(Commande);

        }

        /// Si la version est plus grande que la 0.7
        else if(Commande->OBR_Version >= 0.7)
        {
            /// On lance le watchdog inclus par obligation dans OBR4

        }

        /// Si la version est plus grande que la 0.7
        if(Commande->OBR_Version >= 0.2 && Commande->OBR_Version <= 0.7)
        {
            /// On s'authentifie si possible
            printf("Connexion en tant qu'admin...\n");
            /// Selon la version, la commande de connexion en admin n'est pas ma même
            if(Commande->OBR_Version<0.37)
                Envoie_Commande("OBR_CONNECTER-2560", Commande, Commande->MAITRE_ESCLAVE);
            else
                Envoie_Commande("OBR_CONNECTER_ADMIN-2560", Commande, Commande->MAITRE_ESCLAVE);
            Recup_Commande(Commande);
            if(!strcmp(Commande->Commande, "OK"))
                printf("Connecte en tant qu'administrateur.\n");
            else
            {
                /// Le mot de passe n'est pas 2560, on demande le mot de passe
                printf("Mot de passe : ");
                scanf("%s", Commande->Commande);
                /// On vide le buffer d'entrée
                while(getchar()!='\n' && getchar()!='EOF');
                if(Commande->OBR_Version<0.37)
                    sprintf(Buffer, "OBR_CONNECTER-%s", Commande->Commande);
                else
                    sprintf(Buffer, "OBR_CONNECTER_ADMIN-%s", Commande->Commande);
                Envoie_Commande(Buffer, Commande, Commande->MAITRE_ESCLAVE);
                Recup_Commande(Commande);
                if(!strcmp(Commande->Commande, "OK"))
                    printf("Connecte en tant qu'administrateur.\n");
                else
                    printf("Le mot de passe n'est pas bon, Connecte en tant que simple utilisateur.\n");
            }
        }

        /// On invite l'utilisateur à entrer ses commandes
        printf("Vous pouvez maintenant entrer vos commandes ;)\n128 caracteres MAX par commande !\n\nHELP=afficher l'aide et la liste des commandes\nVERSION=Afficher la version du programme\nEXIT=fermer la connexion et quitter\n\n");
        strcpy(Buffer, "");

        /// Tant qu'on ne quitte pas
        while(strcmp(Buffer, "EXIT"))
        {
            /// On vide les buffers
            strcpy(Buffer, "");
            strcpy(Commande->Commande, "");

            /// On demande à l'arduino s'il est prêt, il répond 1 si on est utilisateur, 2 si on est administrateur, et rien s'il est déconnecté
            temp = ArduinoPret(Commande);
            if(temp == 1)
                printf("@utilisateur>");
            else if(temp == 2)
                printf("@administrateur>");
            else
                {
                    printf("Impossible de communiquer avec l'arduino, verifiez sa connexion.\nDeconnexion.\n");
                    OBR_Deconnecter(Commande);
                    Commande = 0;
                    return;
                }


            /// On récupère la commande entrée par l'utilisateur
            Commande_Console(Buffer, Commande);

            /// Si on demande l'aide
            if(!strcmp(Buffer, "HELP"))
                AfficherAide();
            /// Si on demande la version de l'OBR installé sur l'arduino
            else if(!strcmp(Buffer, "VERSION"))
                printf("Version d'OBR : %f\n", Commande->OBR_Version);
            else
            {
                /// On envoie la commande
                Envoie_Commande(Buffer, Commande, Commande->MAITRE_ESCLAVE);

                /// On récupère la réponse
                Recup_Commande(Commande);
                /// S'il y a une réponse, on l'écrit à l'écran
                if(strcmp(Commande->Commande, ""))
                {
                    switch(Commande->NoEnvoyeur)
                    {
                    case 1:
                        printf("P:>%s\n", Commande->Commande);
                    break;

                    case 2:
                        printf("T:>%s\n", Commande->Commande);
                    break;

                    case 3:
                        printf("M:>%s\n", Commande->Commande);
                    break;

                    case 4:
                        printf("E1:>%s\n", Commande->Commande);
                    break;

                    case 5:
                        printf("E2:>%s\n", Commande->Commande);
                    break;


                    default:
                        printf("?>%s\n", Commande->Commande);
                    break;
                    }
                }
            }
        }

        /// on se déconnecte et on arrête le programme
        printf("\nFermeture du port, et arret du programme...");
        OBR_Deconnecter(Commande);
        Commande = 0;
    }
    return 0;
}

void AfficherAide()
{
    printf("\n\nAide :\nPour envoyer une commande, tappez la commande certifiee OBR, puis faites 'entree'.\nApres 1/10 de seconde, le programme affiche le retour, et vous rend la main.\n");
    printf("\nListe des commandes de base :\n");
    printf("OBR_PRESENT=Retourne VRAIE si OBR est present\n");
    printf("OBR_VERSION=Retourne la version d'OBR installee\n");
    printf("OBR_PRET=Retourne VRAIE si vous etes un utilisateur, ADMIN si vous etes un administrateur\n");
    printf("Pour toutes les commandes, veuillez vous referer a la documentation.\n\n");
}
