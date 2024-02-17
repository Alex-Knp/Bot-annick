import threading
import serial
import matplotlib.pyplot as plt
import drawnow
import numpy as np


nb_points = 50
arduino_port = '/dev/ttyACM0'
arduino_baudrate = 4800
legend = [r'$\omega_{l,ref}$', r'$\omega_{r,ref}$', r'$\omega_{l,filt}$', r'$\omega_{r,filt}$', r'$V_l$', r'$V_r$', r'$I_l$', r'$I_r$', r'$I_{l, mes}$', r'$I_{r, mes}$']


# Fonction principale
def read_data(ser):
    # Gestion de l'interruption (fermeture de la fenêtre)
    def handle_close(evt):
        ser.flush()
        ser.close()
        print('Closed Figure!')

    fig = plt.figure()
    fig.canvas.mpl_connect('close_event', handle_close)

    data = []  # Liste des données
    plt.ion()  # Mode interactif
    cnt = 0  # Compteur du nombre de points

    def makeFig():  # Fonction pour tracer le graphique

        plt.subplot(411)
        #plt.ylim(-30, 30)
        plt.title("RT plotter")
        plt.grid(True)
        plt.ylabel("Speed [rad/s]")
        for i in range(4):
            plt.plot(np.array(data).T[i], label=legend[i])
        plt.legend(loc='upper right')

        plt.subplot(412)
        plt.ylabel("Voltage [V]")
        for i in range(4, 6):
            plt.plot(np.array(data).T[i], label=legend[i])
        plt.legend(loc='upper right')

        plt.subplot(413)
        plt.ylabel("Current [A]")
        for i in range(6, 10):
            plt.plot(np.array(data).T[i], label=legend[i])
        plt.legend(loc='upper right')

    while True:
        while ser.inWaiting() == 0:  # Vérification que l'on reçoit quelque chose
            pass

        arduinoString = ser.readline()
        try:
            values_str = arduinoString.split()
            float_values = [float(value) for value in values_str]
            if len(float_values) == 10:
                data.append(float_values)
                drawnow.drawnow(makeFig)  # Rafraichissement de la figure matplotlib
                cnt = cnt + 1
                if (cnt > nb_points):
                    data.pop(0)  # Si plus de nb_points, enlever la première valeur de la liste data
        except:
            pass


def write_data(ser):
    while True:
        message = input("Entrez le message à envoyer via le port série : ")
        ser.write(message.encode())


if __name__ == '__main__':
    ser = serial.Serial(arduino_port, arduino_baudrate)  # Lancement de la communication série avec Arduino
    serial_thread = threading.Thread(target=write_data, args=(ser,))
    serial_thread.daemon = True
    serial_thread.start()
    read_data(ser)
