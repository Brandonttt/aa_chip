#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Definiciones para el SPI
#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define SS PB2
#define MOSI PB3
#define SCK PB5

// Definiciones para el MAX7219
#define MAX7219_LOAD_LOW() (SPI_PORT &= ~(1 << SS))
#define MAX7219_LOAD_HIGH() (SPI_PORT |= (1 << SS))

// Inicialización de SPI en modo maestro
void SPI_MasterInit(void) {
    SPI_DDR |= (1 << MOSI) | (1 << SCK) | (1 << SS); // Configura MOSI, SCK y SS como salidas
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);   // Habilita SPI, modo maestro, prescaler fosc/16
    MAX7219_LOAD_HIGH();                             // Asegúrate de que SS esté inicialmente en alto
}

// Enviar un byte a través de SPI
void SPI_MasterTransmit(uint8_t data) {
    SPDR = data;                      // Carga el dato en el registro de datos
    while (!(SPSR & (1 << SPIF)));    // Espera a que se complete la transmisión
}

// Enviar un comando al MAX7219
void MAX7219_Send(uint8_t address, uint8_t data) {
    MAX7219_LOAD_LOW();               // Baja el pin SS para iniciar la comunicación
    SPI_MasterTransmit(address);      // Enviar la dirección del registro
    SPI_MasterTransmit(data);         // Enviar el dato al registro
    MAX7219_LOAD_HIGH();              // Sube el pin SS para finalizar la comunicación
}

// Configuración inicial del MAX7219
void MAX7219_Init(void) {
    MAX7219_Send(0x0C, 0x01); // Salir del modo shutdown
    MAX7219_Send(0x0F, 0x00); // Desactivar el modo de prueba
    MAX7219_Send(0x09, 0xFF); // Decodificar todos los dígitos (modo BCD)
    MAX7219_Send(0x0B, 0x07); // Mostrar todos los dígitos (0-7)
    MAX7219_Send(0x0A, 0x0F); // Intensidad máxima
    // Apagar todos los dígitos al inicio
    for (uint8_t i = 1; i <= 8; i++) {
        MAX7219_Send(i, 0x0F); // 0x0F apaga el dígito en modo decodificación BCD
    }
}

// Inicializar USART
void USART_Init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // Habilita RX, TX e interrupción por RX
    UCSR0C = (1 << USBS0) | (3 << UCSZ00); // 8 bits, 2 stop bits
}

// Leer datos desde USART
ISR(USART_RX_vect) {
    unsigned char receivedChar = UDR0; // Leer dato recibido
    // Apaga todos los dígitos primero
    for (uint8_t i = 1; i <= 8; i++) {
        MAX7219_Send(i, 0x0F);
    }
    // Si es un número, mostrarlo solo en el primer dígito
    if (receivedChar >= '0' && receivedChar <= '9') {
        uint8_t digito = receivedChar - '0'; // Convierte el carácter a número
        MAX7219_Send(0x01, digito);          // Muestra el número en el primer dígito
    }
}

int main(void) {
    cli(); // Desactiva interrupciones globales

    USART_Init(103); // UBRR para 9600 baudios con F_CPU = 16 MHz
    SPI_MasterInit();
    MAX7219_Init();

    sei(); // Activa interrupciones globales

    while (1) {
        // El programa principal no necesita hacer nada
        // Todo se maneja en la interrupción USART_RX_vect
             MAX7219_Send(0x00, 0x05);  // Envía algo constantemente
        _delay_ms(100); 
    }
}