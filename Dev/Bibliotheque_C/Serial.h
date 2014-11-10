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

#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#include "Serial.c"

   int OuvrirCom(int NoPort, int Flags);
   int EnvoyerCom(const char* Message, int SerialPointer);
   int RecevoirCom(int SerialPointer, char* Buffer);
   int FermerCom(int SerialPointer);

#endif // PORTCOM_H_INCLUDED
