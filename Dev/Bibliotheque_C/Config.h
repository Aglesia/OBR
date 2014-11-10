#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/** Lit dans le fichier [FileName] la valeur int de [ConfName], retourne [DefaultValue] si rien n'est trouvé */
int CONFIG_ReadInt(const char* FileName, const char* ConfName, int DefaulValue);

/** Lit dans le fichier [FileName] la valeur char de [ConfName], retourne [DefaultValue] si rien n'est trouvé */
char CONFIG_ReadChar(const char* FileName, const char* ConfName, char DefaulValue);

/** Lit dans le fichier [FileName] le tableau de char de [ConfName], retourne [DefaultValue] si rien n'est trouvé */
char* CONFIG_ReadString(const char* FileName, const char* ConfName, const char* DefaulValue);


/** Ecrit dans le fichier [FileName] la valeur int de [ConfName], retourne 0 si OK */
int CONFIG_WriteInt(const char* FileName, const char* ConfName, int Value);

/** Ecrit dans le fichier [FileName] la valeur char de [ConfName], retourne 0 si OK */
int CONFIG_WriteChar(const char* FileName, const char* ConfName, char Value);

/** Ecrit dans le fichier [FileName] le tableau de char de [ConfName], retourne 0 si OK */
int CONFIG_WriteString(const char* FileName, const char* ConfName, const char* Value);

#include "Config.c"

#endif // CONFIG_H_INCLUDED
