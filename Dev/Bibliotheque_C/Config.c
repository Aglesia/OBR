#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/// Liste des fonctions
int CONFIG_ReadInt(const char* FileName, const char* ConfName, int DefaultValue)
{
    int Value = DefaultValue, Tempon = 0, i = 0;

    /// On ouvre le fichier
    FILE* Fichier = NULL;
    if(!(Fichier = fopen(FileName, "r")))
        return Value;

    /// On lit chaque ligne avec un tempon d'1Ko pour connaître la valeur
    char Temp[1024]="", ConfLine[100]="", LastChar = 1;
    while(LastChar != EOF && LastChar != NULL)
    {
        /// On récupère la ligne complète
        memset (Temp, 0, sizeof(Temp));
        i = 0;
        LastChar = 1;
        while(LastChar != EOF && LastChar != '\n' && i < 1023)
        {
            LastChar = fgetc(Fichier);
            Temp[i++] = LastChar;
        }

        /// On extrait les données de la ligne
        sscanf(Temp, "%s = %d", ConfLine, &Tempon);
        if(!strcmp(ConfLine, ConfName))
        {
            Value = Tempon;
            break;
        }
    }

    /// On ferme le fichier
    fclose(Fichier);
    printf(" \n");
    return Value;
}

char CONFIG_ReadChar(const char* FileName, const char* ConfName, char DefaultValue)
{
    /// On ouvre le fichier
    FILE* Fichier = fopen(FileName, "r");

    char Value = DefaultValue, Tempon = 0;
    int i = 0;
    if(!Fichier)
        return Value;

    /// On lit chaque ligne avec un tempon d'1Ko pour connaître la valeur
    char Temp[1024]="", ConfLine[100]="", LastChar = 1;
    while(LastChar != EOF && LastChar != NULL)
    {
        /// On récupère la ligne complète
        memset (Temp, 0, sizeof(Temp));
        i = 0;
        LastChar = 1;
        while(LastChar != EOF && LastChar != '\n' && i < 1023)
        {
            LastChar = fgetc(Fichier);
            Temp[i++] = LastChar;
        }

        /// On extrait les données de la ligne
        sscanf(Temp, "%s = %c", ConfLine, &Tempon);
        if(!strcmp(ConfLine, ConfName))
        {
            Value = Tempon;
            break;
        }
    }

    /// On ferme le fichier
    fclose(Fichier);
    return Value;
}

char* CONFIG_ReadString(const char* FileName, const char* ConfName, const char* DefaultValue)
{
    /// On ouvre le fichier
    FILE* Fichier = fopen(FileName, "r");

    char Value[300] = "", Tempon[300] = "";
    int i = 0;
    strcpy(Value, DefaultValue);

    if(!Fichier)
        return Value;

    /// On lit chaque ligne avec un tempon d'1Ko pour connaître la valeur
    char Temp[1024]="", ConfLine[100]="", LastChar = 1;
    while(LastChar != EOF && LastChar != NULL)
    {
        /// On récupère la ligne complète
        memset (Temp, 0, sizeof(Temp));
        i = 0;
        LastChar = 1;
        while(LastChar != EOF && LastChar != '\n' && i < 1023)
        {
            LastChar = fgetc(Fichier);
            Temp[i++] = LastChar;
        }

        /// On extrait les données de la ligne
        sscanf(Temp, "%s = %s", ConfLine, Tempon);
        if(!strcmp(ConfLine, ConfName))
        {
            strcpy(Value, Tempon);
            break;
        }
    }

    /// On ferme le fichier
    fclose(Fichier);
    return Value;
}

int CONFIG_WriteInt(const char* FileName, const char* ConfName, int Value)
{
    return 42;
}

int CONFIG_WriteChar(const char* FileName, const char* ConfName, char Value)
{
    return 42;
}

int CONFIG_WriteString(const char* FileName, const char* ConfName, const char* Value)
{
    return 42;
}
