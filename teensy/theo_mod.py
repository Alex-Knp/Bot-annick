import matplotlib.pyplot as plt
import numpy as np

# Lire le fichier
theo = np.loadtxt('data_1/out.txt')
theo = theo[::5]
mes = np.loadtxt('data_1/data.txt')

# Extraire les colonnes
wltheo = theo[:, 1]  # Deuxième colonne
wrtheo = theo[:, 2]  # Troisième colonne
wlmes = mes[:, 7]  # Deuxième colonne
wrmes = mes[:, 8]  # Troisième colonne

fig, axs = plt.subplots(2, 1, figsize=(8, 12), sharex=True)

# Premier subplot
axs[0].plot(wltheo, label=r'$\omega_l, theo$')
axs[0].plot(wrtheo, label=r'$\omega_r, theo$')
axs[0].set_xlabel('Samples')
axs[0].set_ylabel(r'$\omega$')
axs[0].legend()

axs[1].plot(wlmes, label=r'$\omega_l, mes$')
axs[1].plot(wrmes, label=r'$\omega_r, mes$')
axs[1].set_xlabel('Samples')
axs[1].set_ylabel(r'$\omega$')
axs[1].legend()


# Afficher le graphique
plt.show()
