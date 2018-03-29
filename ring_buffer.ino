#include <avr/io.h>

#include <avr/interrupt.h>

#include <avr/sleep.h>

#define USART_BAUDRATE 9600

#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//define max buffer size

#define BUF_SIZE 300

//type definition of buffer structure
uint8_t strtimer = 0;
uint8_t received = 0;
uint8_t tim=1;
uint8_t tovf = 0;

uint8_t n_elem;


typedef struct{
    

    uint8_t buffer[BUF_SIZE];
    uint8_t head;
    uint8_t tail;
    
   

}u8buf;

//declare buffer

u8buf buf;

//initialize buffer

void BufferInit(u8buf *buf)

{

    //set index to start of buffer

    buf->head=0;
    buf->tail=0;

}

//write to buffer routine

uint8_t BufferWrite(u8buf *buf, uint8_t u8data)
{ 
    if (buf->head!=BUF_SIZE)

    {

        buf->buffer[buf->head] = u8data;

        //increment buffer index

        buf->head++;

        n_elem=buf->head;

        return 0;

    }

        else 
        {
          buf->head=0;
          return 1;
        }

}

uint8_t BufferRead(u8buf *buf, volatile uint8_t *u8data)

{

    if(buf->tail!=n_elem)

    {

       *u8data=buf->buffer[buf->tail];
        buf->tail++;      
        
        return 0;

    }

    else
    { 
      buf->tail=0;
      return 1;
    }

}

void USART0Init(void)

{
    cli();
    // Set baud rate

    UBRR0H = (uint8_t)(UBRR_VALUE>>8);

    UBRR0L = (uint8_t)UBRR_VALUE;

    // Set frame format to 8 data bits, no parity, 1 stop bit

    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);

    //enable reception and RC complete interrupt

    UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);

}

void TIMER0Init(void)
{
  TCCR1B |= (1 << CS11);
  
  TIMSK1 |= (1 << TOIE1);
  
  TCNT1=0;
  }

  
ISR(USART_RX_vect)

{  uint8_t u8temp;
  
    u8temp=UDR0;
   
    received = 1;
    strtimer = 0;
    //check if period char or end of buffer

    if ((BufferWrite(&buf, u8temp)==1))

    {   //disable timer interrupt
        //TIMSK1 &= ~(1 << TOIE1);
        //disable reception and RX Complete interrupt

        UCSR0B &= ~((1<<RXEN0)|(1<<RXCIE0));

        //enable transmission and UDR0 empty interrupt

        UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);

    }

}

//UDR0 Empty interrupt service routine

ISR(USART_UDRE_vect)

{

    //if index is not at start of buffer
    PORTB ^=(1<<PORTB5); 
    if (BufferRead(&buf, &UDR0)==1)

    {

        //start over

        //reset buffer

        BufferInit(&buf);

        //disable transmission and UDR0 empty interrupt

        UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));

        //enable reception and RC complete interrupt

        UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
        //enable timer 
        TIMSK1 |= (1 << TOIE1);
        
    }

}

void tick(){
  strtimer++;
  //PORTB ^=(1<<PORTB5); 
  if((strtimer > 4 ) && (received)){
    
    UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);
    received = 0;
    strtimer = 0;
  }
  
}

ISR(TIMER1_OVF_vect)
{ 
  
 //enable transmission and UDR0 empty interrupt
   //UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);
   tovf++;
 if(tovf>=31)
  {
    //PORTB ^=(1<<PORTB5);
     //UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
     tick();
    tim=0;
    tovf=0;
     
  } 
}

int main (void)

{

    //Init buffer

    BufferInit(&buf);

    //set sleep mode

    set_sleep_mode(SLEEP_MODE_IDLE);

    DDRB = (1<< PORTB5);
      //Initialize TIMER
    
    TIMER0Init();
     cli();
    //Initialize USART0

    USART0Init();
    
  

    //enable global interrupts

    sei();

    while(1)

        {

            //put MCU to sleep

            sleep_mode();

        }

}


