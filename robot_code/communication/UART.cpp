// implementation of UART for communication between the Raspberry Pi dynamixel motors
// GPIO14 and GPIO15 are used for UART communication
#include "UART.hh"
 
using namespace std;

int uart_fd;

void setup_uart() {
    // Open UART device
    //uart_fd = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY); // marche sur le rasp 4 selon le gas mais je pense le 5 aussi
    uart_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);   //ça c'est pour les pins 14 et 15
    //uart_fd = open("/dev/ttyAMA10", O_RDWR | O_NOCTTY | O_NDELAY); // ça c'est pour le nouveau port uart   en vrai je pense balec 
    if (uart_fd < 0) {                                                 // pcq quand on fais ls -l dans dev on voit que serial0 est lié à AMA10
        perror("Failed to open UART device");
        exit(EXIT_FAILURE);
    }

    // Configure UART
    struct termios options;   // Settings du port série
    tcgetattr(uart_fd, &options);                        // fd et endroit ou on veut stocker les settings
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;    // baud rate, 8-bit, ignore modem status lines, enable receiver
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);
}

void send_uart(float value) {

    // Convertir le float en tableau d'octets
    unsigned char bytes[sizeof(float)];
    memcpy(bytes, &value, sizeof(float));

    // Envoyer les octets via UART
    if (write(uart_fd, bytes, sizeof(float)) < 0) {
        perror("Failed to write to UART device");
        exit(EXIT_FAILURE);
    }
}

float receive_uart() {
    float value;
    unsigned char bytes[sizeof(float)];

    // Recevoir les octets depuis UART
    read(uart_fd, bytes, sizeof(float));

    // Convertir les octets en float
    memcpy(&value, bytes, sizeof(float));
    return value;
}

int main() {
    setup_uart();       // à appeler seulement une fois dans le init

    // Envoi de bytes via UART
    float data_to_send = 1.05;
    send_uart(data_to_send);

    // Réception de bytes depuis UART
    float received_data = receive_uart();

    // Fermeture du port UART
    close(uart_fd);

    return 0;
}

