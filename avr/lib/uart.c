#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/atomic.h>
#include "buffer_utils.h"

static const char * header = "FanDuino-Header0";
#define header_size strlen(header)

#define UART_BUFFER_SIZE 256

typedef struct UART {
  uint8_t tx_buffer[UART_BUFFER_SIZE];
  volatile uint8_t tx_start;
  volatile uint8_t tx_end;
  volatile uint8_t tx_size;

  uint8_t rx_buffer[UART_BUFFER_SIZE];
  volatile uint8_t rx_start;
  volatile uint8_t rx_end;
  volatile uint8_t rx_size;

  int baud;
  int uart_num; // hardware uart;
} UART;

static UART uart_0;

struct UART * UART_init(void) {
  UART* uart=&uart_0;
  uart->uart_num=0;

  UBRR0H = (uint8_t)(MYUBRR>>8);
  UBRR0L = (uint8_t)MYUBRR;

  uart->tx_start=0;
  uart->tx_end=0;
  uart->tx_size=0;
  uart->rx_start=0;
  uart->rx_end=0;
  uart->rx_size=0;

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);   /* Enable RX and TX */
  sei();
  return &uart_0;
}

// returns the free space in the buffer
int UART_rxbufferSize(struct UART* uart __attribute__((unused))) {
  return UART_BUFFER_SIZE;
}

// returns the free occupied space in the buffer
int  UART_txBufferSize(struct UART* uart __attribute__((unused))) {
  return UART_BUFFER_SIZE;
}

// number of chars in rx buffer
int UART_rxBufferFull(UART* uart) {
  return uart->rx_size;
}

// number of chars in rx buffer
int UART_txBufferFull(UART* uart) {
  return uart->tx_size;
}

// number of free chars in tx buffer
int UART_txBufferFree(UART* uart){
  return UART_BUFFER_SIZE-uart->tx_size;
}

void UART_putChar(struct UART* uart, uint8_t c) {
  // loops until there is some space in the buffer
  while (uart->tx_size>=UART_BUFFER_SIZE);
  ATOMIC_BLOCK(ATOMIC_FORCEON){
    uart->tx_buffer[uart->tx_end]=c;
    BUFFER_PUT(uart->tx, UART_BUFFER_SIZE);
  }
  UCSR0B |= _BV(UDRIE0); // enable transmit interrupt
}

uint8_t UART_getChar(struct UART* uart){
  while(uart->rx_size==0);
  uint8_t c;
  ATOMIC_BLOCK(ATOMIC_FORCEON){
    c=uart->rx_buffer[uart->rx_start];
    BUFFER_GET(uart->rx, UART_BUFFER_SIZE);
  }
  return c;
}


ISR(USART0_RX_vect) {
      uint8_t c=UDR0;
      if (uart_0.rx_size<UART_BUFFER_SIZE){
        uart_0.rx_buffer[uart_0.rx_end] = c;
        BUFFER_PUT(uart_0.rx, UART_BUFFER_SIZE);
      }
}

ISR(USART0_UDRE_vect){
  if (! uart_0.tx_size){
    UCSR0B &= ~_BV(UDRIE0);
  } else {
    UDR0 = uart_0.tx_buffer[uart_0.tx_start];
    BUFFER_GET(uart_0.tx, UART_BUFFER_SIZE);
  }
}


void UART_putString(struct UART * uart,uint8_t * buf, size_t size){
  uint8_t * msg = malloc(size+header_size);
  memcpy(msg,header,header_size);
  memcpy(msg+header_size,buf,size);
  size+=header_size;
  for(int i = 0;i<size;i++){
    UART_putChar(uart, *(msg+i));
  }
  free(msg);
}

void UART_putData(struct UART * uart, uint8_t * data, uint32_t data_size, uint8_t data_type){
  struct data d = fill_data(data,data_size,data_type);
  UART_putString(uart, (uint8_t*)&d,sizeof(struct data));
}


//secondo me sta funzione va migliorata
//alla fine non ha senso resettare i valori con uart->rx_end potrebbero esserci dei byte non letti.
//nel caso due paccetti non siano stati letti in tempo.
uint8_t UART_getData(struct UART * uart, uint8_t * buf, size_t data_size){
  ATOMIC_BLOCK(ATOMIC_FORCEON){
    uint16_t i;
    uint8_t k;
    const int internal_buffer_size = UART_rxbufferSize(uart);

    for(i = 0;i<internal_buffer_size;i++){
      //Searching for header head
      for(k=0;k<strlen(header);k++){
        if(uart->rx_buffer[(i+k)%internal_buffer_size]!=header[k])
          break;
      }
      if(k==strlen(header))//Data start at index i
      {
        i = (i+k)%internal_buffer_size;//I contains first index of data
        if(uart->rx_end < i+data_size)
          return 0;

        for(k=0;k<data_size;k++)
          buf[k]=uart->rx_buffer[(i+k)%internal_buffer_size];

        uart->rx_end=0;
        memset(uart->rx_buffer,0,internal_buffer_size);
        return 1;
      }
    }
    return 0;
  }
  return 0;
}
