import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 15})
nom_fichier = 'data.txt'
legend = [r'x', r'y', r'$\theta$', r'$x_{ref}$', r'$y_{ref}$', r'$\omega_{l,ref}$', r'$\omega_{r,ref}$', r'$\omega_{l,filt}$', r'$\omega_{r,filt}$', r'$V_l$', r'$V_r$', r'$I_l$', r'$I_r$', r'$I_{l, mes}$', r'$I_{r, mes}$']

# Lecture des données depuis le fichier
with open(nom_fichier, 'r') as fichier:
    lignes = fichier.readlines()

# Traitement des données
donnees = []
for ligne in lignes:
    valeurs = [float(valeur) for valeur in ligne.strip().split()]
    donnees.append(valeurs)

donnees_transposees = list(map(list, zip(*donnees)))

# Diviser les données en groupes
donnees_groupe1 = donnees_transposees[:5]   # Les 4 premières colonnes
donnees_groupe2 = donnees_transposees[5:9]  # Les 4 suivantes
donnees_groupe3 = donnees_transposees[9:11] # Les 2 suivantes
donnees_groupe4 = donnees_transposees[11:13]  # Les 4 dernières

# Créer les subplots
fig, axs = plt.subplots(4, 1, figsize=(8, 12), sharex=True)

# Premier graphe (4 premières courbes)
for i, valeurs_courbe in enumerate(donnees_groupe1, start=1):
    axs[0].plot(valeurs_courbe, label=legend[i-1])

axs[0].set_title('Position')
axs[0].set_ylabel('m')
axs[0].legend(loc='upper right')

# Deuxième graphe (4 courbes suivantes)
for i, valeurs_courbe in enumerate(donnees_groupe2, start=6):
    axs[1].plot(valeurs_courbe, label=legend[i-1])

axs[1].set_title('Speed')
axs[1].set_ylabel(r'$\frac{rad}{s}$')
axs[1].legend(loc='upper right')

# Troisième graphe (2 courbes suivantes)
for i, valeurs_courbe in enumerate(donnees_groupe3, start=10):
    axs[2].plot(valeurs_courbe, label=legend[i-1])

with open("ua.txt", 'w') as fd:
    for i in range(len(donnees_groupe3[0])):
        fd.write(str(round(i*32/19200, 4)) + '\t' + str(donnees_groupe3[0][i]) + '\t' + str(donnees_groupe3[1][i]) + '\n')

axs[2].set_title('Voltage')
axs[2].set_ylabel('V')
axs[2].legend(loc='upper right')

# Quatrième graphe (4 dernières courbes)
for i, valeurs_courbe in enumerate(donnees_groupe4, start=12):
    axs[3].plot(valeurs_courbe, label=legend[i-1])

axs[3].set_title('Currents')
axs[3].set_xlabel('Samples')
axs[3].set_ylabel('A')
axs[3].legend()

# Ajuster l'espacement entre les subplots
plt.tight_layout()

# Afficher les graphiques
plt.show()
