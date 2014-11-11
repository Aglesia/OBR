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
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

/// Définition d'une structure dédiée aux ports séries
typedef struct Serial {
    int fd; // Adresse du fichier com ouvert
    int NoCOM; // N° du port COM
} Serial;

// FONCTION : OuvrirCom
int OuvrirCom(int NoPort, int Flags)
{
    // On crée les variables
    char CharPort[50] = "/dev/ttyUSB";
    char Type = Flags/100; /// Si la centaine vaut 1, c'est ttyUSB. Sinon, c'est ttyS.
    if(Type)
        sprintf(CharPort, "/dev/ttyUSB%d", NoPort);
    else
        sprintf(CharPort, "/dev/ttyS%d", NoPort);

    // On ouvre le port série
    Serial *SerialPort = malloc(sizeof(Serial));
    if(!SerialPort)
        return 0;

    SerialPort->NoCOM=NoPort;
    SerialPort->fd = open(CharPort, O_RDWR) ;

    // On récupère les paramètres du port série
    struct termios options;
    tcgetattr(SerialPort->fd, &options);

    // On définit les paramètres du port série
    Flags = Flags - (Type*100);

    // On détermine la vitesse de transmission
    switch(Flags)
    {
        case 1:
        cfsetispeed(&options, B4800);
        cfsetospeed(&options, B4800);
        break;

        case 2:
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        break;

        case 3:
        cfsetispeed(&options, B19200);
        cfsetospeed(&options, B19200);
        break;

        case 4:
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
        break;

        case 5:
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
        break;

        case 6:
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        break;

        case 7:
        cfsetispeed(&options, B230400);
        cfsetospeed(&options, B230400);
        break;

        case 8:
        cfsetispeed(&options, B460800);
        cfsetospeed(&options, B460800);
        break;

        case 9:
        cfsetispeed(&options, B921600);
        cfsetospeed(&options, B921600);
        break;

        default:
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
    }

    // On enregistre les paramètres
    tcsetattr(SerialPort->fd, TCSANOW, &options);


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

    // On détermine la taille du message
    int TailleChaine = strlen(Message) ;

    if(SerialPort->fd == NULL)
        return 1;

    // Emission du message
    tcflush(SerialPort->fd, TCIOFLUSH);
    if(write(SerialPort->fd, Message, TailleChaine) < 0)
    {
        return -1;
    }

    // On attend que le message soit envoyé
    tcdrain(SerialPort->fd);

    return 0;
}

//---------------------------------------------------------------------------
// FONCTION : RecevoirCom
//---------------------------------------------------------------------------

int RecevoirCom(int SerialPointer, char* Buffer)
{
    Serial* SerialPort = SerialPointer;
    if(!SerialPort)
        return 0;

   long NbCarLus;

   if(SerialPort->fd==NULL)
        return 0;

     //Reception de la Chaine
    if((NbCarLus = read(SerialPort->fd, Buffer, sizeof(Buffer))) < 0)
    {
        return 0;
    }

    //Finition de la Chaine
    Buffer[NbCarLus] = '\0';
    return NbCarLus;
}

//-----------------------------------------------------------------------
// FONCTION : FermerCom
//-----------------------------------------------------------------------

int FermerCom(int SerialPointer)
{
    if(!SerialPointer)
        return -2;

    Serial* SerialPort = SerialPointer;

   if(SerialPort->fd==NULL)
        return -1;

    close(SerialPort->fd);
    return 0;
}
