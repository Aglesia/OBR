/// Liste des fonctions systèmes OBR

/// Gestion d'initilaisation
// Fonction de lecture d'une case Entrée/Sortie selon l'EEPROM
void Lire_Donnee_EEPROM_ES(byte NoES, boolean Analog);

// Fonction d'écriture d'une case Entrée/Sortie en EEPROM
void Ecrire_Donnee_EEPROM_ES(byte NoES, boolean Analog);

// Fonction de verrouillage du système
void BloquageSysteme();

// Fonction de mise à jour du niveau de batterie (en %)
void MajBatterie();

/// Fonctions de gestion de la RAM
// Fonction de mise à jour en RAM
void MAJ_RAM();

// Fonction de lecture du N° de PIN PWM
byte No_PIN_PWM(byte NoPWM);

// Fonction d'écriture en RAM
void Ecriture_RAM(byte Type, byte NumeroCase, byte Valeur);

// Fonction de lecture en RAM
byte Lecture_RAM(byte Type, byte NumeroCase);

/// Fonctions de gestion des E/S
// Fonction de mise à jour des sorties, selon la RAM
void MAJ_ES();

// Fonction qui retourne la valeur à écrire en sortie physique (sur les PINs)
byte Autoriser_Sortie(byte NumeroSortie);

// retourne 1 si toutes les liaisons sont OK
boolean Liaison_OK(byte NumeroSortie);

// Fonction d'écriture sur les ports digitals
void WriteES(byte PIN, boolean Etat);

// Fonction d'écriture sur les ports PWM
void WritePWM(byte PWM, byte Valeur);

// Fonction de changement de mode d'une E/S
void ModeES(byte PIN, byte Mode);

/// Fonctions de gestion du mot de passe
// Fonction de reinitialisation du mot de passe
void RAZ_MotDePasse();

// Fonction de lecture du mot de passe depuis l'EEPROM
unsigned long LireMotDePasse();

// Fonction d'écriture du mot de passe sur l'EEPROM
void EcrireMotDePasse(unsigned long Password);

// Fonctions de gestion des commandes
// Fonction de récupération de la commande PC ou Telecommande
void Verification_Commande();

// Fonction de récupération du port série
void RecupMessage(byte NoPort);

// Fonction d'envoie d'une chaine de caractère via le port série
void EnvoieMessage(byte Type, String Message);

// Fonction d'envoie d'un message, ayant déjà une adresse
void TransfererMessage(String Message);

// Fonction de détermination des emplacements réseaux
void Verification_Connexion();

// Chien de garde pour la déconnexion automatique des ports séries
void WatchDog_Serie();

// Fonction d'envoie d'une demande de connexion
void Envoie_Connexion(byte NoPort);

// Fonction de vérification d'une nouvelle connexion au port série
void Demande_Connexion(byte NoPort);

// Fonction d'indication du numéro du port sur lequel est connecté tel truc (N° de port à partir de 1)
byte NoPortSerie(byte Objet);

// Fonction d'execution d'une commande
void ExecuterCommande();

// Fonction d'effacement de commande en file d'attente
void EffacerCommande();