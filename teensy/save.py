import serial

# Spécifiez le port série et la vitesse de communication
port_serie = serial.Serial('/dev/ttyACM0',
                           19200)  # Remplacez 'COM1' par le port série que vous utilisez et 9600 par la vitesse de communication appropriée

# Nom du fichier dans lequel enregistrer les données
nom_fichier = 'data.txt'

# Ouvrir le fichier en mode écriture ('w')
with open(nom_fichier, 'w') as fichier:
    try:
        while True:
            # Lire une ligne depuis le port série
            ligne = port_serie.readline().decode('utf-8').strip()

            print(ligne)

            # Écrire la ligne dans le fichier
            fichier.write(ligne + '\n')



    except KeyboardInterrupt:
        # Fermer le port série et le fichier lorsque l'utilisateur appuie sur Ctrl+C
        port_serie.close()
        print("Arrêt du programme.")
