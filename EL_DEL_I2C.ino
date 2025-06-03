#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define FOSC 16000000 // Frecuencia del reloj
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define PCF8574_ADDR 0x40 // Dirección base del PCF8574 (ajustar según configuración de pines A0, A1, A2)

// Variables globales
volatile uint8_t digitoRecibido = 0;

void USART_Init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // Habilita RX, TX e interrupción por RX
    UCSR0C = (1 << USBS0) | (3 << UCSZ00); // 8 bits, 2 stop bits
}

void USART_Transmit(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0))); // Espera a que el buffer de transmisión esté vacío
    UDR0 = data; // Envía el dato
}

ISR(USART_RX_vect)
{
    unsigned char receivedChar = UDR0;
    if (receivedChar >= '0' && receivedChar <= '9') {
        digitoRecibido = receivedChar - '0';
    }
}

void TWI_Init(void)
{
    TWSR = 0x00; // Configura prescaler a 1
    TWBR = 0x48; // Configura velocidad de reloj (~100kHz con F_CPU=16MHz)
    TWCR = (1 << TWEN); // Habilita TWI
}

void TWI_Start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Enviar condición de START
    while (!(TWCR & (1 << TWINT))); // Esperar a que se complete
}

void TWI_Stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // Enviar condición de STOP
}

void TWI_Write(uint8_t data)
{
    TWDR = data; // Cargar dato
    TWCR = (1 << TWINT) | (1 << TWEN); // Iniciar transmisión
    while (!(TWCR & (1 << TWINT))); // Esperar a que se complete
}

void PCF8574_Write(uint8_t data)
{
    TWI_Start();
    TWI_Write(PCF8574_ADDR); // Dirección del PCF8574 (escritura)
    TWI_Write(data); // Enviar dato
    TWI_Stop();
}

void generarPWM(uint8_t dutyCycle)
{
    uint8_t signal = 0x00;
    for (int i = 0; i < 100; i++) {
        if (i < dutyCycle) {
            signal = 0xFF; // Encender señal
        } else {
            signal = 0x00; // Apagar señal
        }
        PCF8574_Write(signal);
        _delay_ms(1);
    }
}

int main(void)
{
    cli(); // Desactiva interrupciones globales

    USART_Init(MYUBRR);
    TWI_Init();

    sei(); // Activa interrupciones globales

    while (1)
    {
        generarPWM(digitoRecibido * 10); // Generar señal PWM según el dígito recibido
    }
}