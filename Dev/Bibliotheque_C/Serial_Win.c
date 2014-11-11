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

/// Inclusion des headers
#include <windows.h>
#include <winbase.h>
#include <conio.h>

/// Définition d'une structure dédiée aux ports séries
typedef struct Serial {
    HANDLE g_hCom; // Adresse du fichier com ouvert
    DCB g_DCB; // Configuration du port com ouvert
    int NoCOM; // N° du port COM
} Serial;

// FONCTION : OuvrirCom
int OuvrirCom(int NoPort, int Flags)
{
    // On ouvre le port série
    Serial *SerialPort = malloc(sizeof(Serial));
    SerialPort->NoCOM=NoPort;
    char NomPort[50]="\\\\.\\COM";
    char NoPortTemp[10]="";
    itoa(NoPort, NoPortTemp, 10);
    strcat(NomPort, NoPortTemp);
    SerialPort->g_hCom = CreateFile(NomPort, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_WRITE_THROUGH | FILE_FLAG_NO_BUFFERING, NULL);

    if(SerialPort->g_hCom == INVALID_HANDLE_VALUE || !SerialPort->g_hCom)
    {
        return 0;
    }

    // On déclare les variables
    int BaudRate=9600, ByteSize=8, DTR_Control=0, Parity=0, StopBit=1; // Config de base

    // On vide les tampons d'émission et de réception, mise à 1 DTR
    PurgeComm(SerialPort->g_hCom, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR);

    // On paramètre le port série
    SerialPort->g_DCB.DCBlength = sizeof(DCB);

    //Configuration actuelle
    GetCommState(SerialPort->g_hCom, &SerialPort->g_DCB);

    // Définition de la vitesse
    char Type = Flags/100
    Flags = Flags - (Type*100);
    BaudRate = 9600;
    switch(Flags) // On détermine la vitesse de transmission
    {
        case 0:
        BaudRate = 9600;
        break;

        case 1:
        BaudRate = 14400;
        break;

        case 2:
        BaudRate = 19200;
        break;

        case 3:
        BaudRate = 28800;
        break;

        case 4:
        BaudRate = 38400;
        break;

        case 5:
        BaudRate = 57600;
        break;

        case 6:
        BaudRate = 115200;
        break;

        default:
        BaudRate = 9600;
    }

    //Modification du DCB
    SerialPort->g_DCB.BaudRate = BaudRate;
    SerialPort->g_DCB.ByteSize = ByteSize;

    if(DTR_Control)
        SerialPort->g_DCB.fDtrControl = DTR_CONTROL_ENABLE;
    else
        SerialPort->g_DCB.fDtrControl = DTR_CONTROL_DISABLE;

    // Gestion de la parité parmi paire, impaire et aucune
    if (Parity == 0)
        SerialPort->g_DCB.Parity = NOPARITY;
    if (Parity == 1)
        SerialPort->g_DCB.Parity = EVENPARITY;
    if (Parity == 2)
        SerialPort->g_DCB.Parity = ODDPARITY;

    // Gestion du Stop Bit
    if (StopBit == 1)
        SerialPort->g_DCB.StopBits = ONESTOPBIT;
    if (StopBit == 3)
        SerialPort->g_DCB.StopBits = ONE5STOPBITS;
    if (StopBit == 2)
        SerialPort->g_DCB.StopBits = TWOSTOPBITS;

    //Configuration de la liaison serie
    SetCommState(SerialPort->g_hCom, &SerialPort->g_DCB);

    return SerialPort;
}

//----------------------------------------------------------------------------
// FONCTION : EnvoyerCom
//----------------------------------------------------------------------------

int EnvoyerCom(const char* Message, int SerialPointer)
{
    Serial* SerialPort = SerialPointer;
    if(!SerialPort)
        return -2;

    DWORD NumBytes=0;
    int TailleChaine = strlen(Message) ;

    if(SerialPort->g_hCom == NULL)
        return 1;

    //Emission du message
    if(!WriteFile(SerialPort->g_hCom, Message, TailleChaine, &NumBytes, NULL))
    {
        return -1;
    }

    return 0;
}

//---------------------------------------------------------------------------
// FONCTION : RecevoirCom
//---------------------------------------------------------------------------

int RecevoirCom(int SerialPointer, char* Buffer)
{
    Serial* SerialPort = SerialPointer;
    if(!SerialPort)
        return -2;

   COMSTAT Stat;
   DWORD Errors;
   long nbCarALire=0;
   long NCarLus;

   if(SerialPort->g_hCom==NULL)
        return -3;

    //Pour connaitre le nombre d'octets dans le buffer d'entrée
    ClearCommError(SerialPort->g_hCom, &Errors, &Stat);
    nbCarALire=Stat.cbInQue;

    //On effectue la lecture s'il y a des caractères présents
    if(nbCarALire>0)
    {
        //Reception de la Chaine
        if(!ReadFile(SerialPort->g_hCom, Buffer, nbCarALire, &NCarLus, NULL))
        {
            return 0;
        }

        //Finition de la Chaine
        Buffer[nbCarALire] = '\0';
        return nbCarALire;
    }
    return 0;
}

//-----------------------------------------------------------------------
// FONCTION : FermerCom
//-----------------------------------------------------------------------

int FermerCom(int SerialPointer)
{
    if(!SerialPointer)
        return -2;

    Serial* SerialPort = SerialPointer;

   if(SerialPort->g_hCom==NULL)
        return -1;

    CloseHandle(SerialPort->g_hCom);
    return 0;
}
