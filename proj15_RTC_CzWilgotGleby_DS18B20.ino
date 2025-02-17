//Program wykorzystujący zegar czasu rzeczywsitego z PCF8563 I2C, służący do ustawienia daty, godziny i czasu alarmu.
//Alarm wywołuje przerwanie zewnętrzne na pinie PD2 (INT0) arduino nano, którego reprezentacją jest sprawdzenie wigotności gleby.


#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

#define CONTROL1            0x00
#define CONTROL2            0x01
#define SECOND              0x02
#define MINUTE              0x03
#define HOUR                0x04
#define DAY                 0x05
#define WEEKDAY             0x06
#define MONTH               0x07
#define YEAR                0x08
#define MINUTE_ALARM        0x09
#define HOUR_ALARM          0x0A
#define DAY_ALARM           0x0B
#define WEEKDAY_ALARM       0x0C

#define I2C_course_write    0b00000000
#define I2C_course_read     0b00000001  

//ROM COMMANDS
#define READ_ROM          0x33
#define MATCH_ROM         0x55
#define SKIP_ROM          0xCC
#define ALARM_SEARCH      0xEC
#define SEARCH_ROM        0xF0

//FUNCTION COMMANDS
#define CONVERT           0x44
#define W_SCRATCHPAD      0x4E
#define R_SCRATCHPAD      0xBE
#define COPY_SCRATCHPAD   0x48
#define RECALL            0xB8

//Configuration
#define TH                0x18            //24C
#define TL                0x0F            //15C
#define CONF              0b01111111      //rozdzielczość 12-bitowa
#define PIN               PD4             //pin na którym podłączone jest DS18B20
 
uint8_t PCF8563_address=0;  //zapis dec adresu urządzenia
int i = 0;
volatile bool alarm_flag = 0;
float temperature=0.0;


//......................................................... I2C .................................................................
void I2C_init()
{
  //SCL frequency - 100000Hz
  //scl_freq = 16000000/(16+2(72)*1)= 100000Hz
  //TWBR = (f_cpu/(f_scl)-16)/(2*N)
  TWBR = 0b01001000; //72

  // prescaler = 1
  TWSR &= ~(1<<TWPS1);
  TWSR &= ~(1<<TWPS0);

  addressScanner();
}

void addressScanner()
{
  //iteracja przez wszyskite możliwe wartości zapisu 7-bitowego adresu
  for(int i=1; i<127; i++)
  {
    I2C_start();
    
    PCF8563_address = (i<<1)|(I2C_course_write); 
    TWDR = PCF8563_address;
    TWCR = (1<<TWINT) | (1<<TWEN);
  
    //Poczekaj na ustawienie flagi TWINT. Oznacza to, że DANE zostały przesłane i odebrano ACK/NACK
    while (!(TWCR & (1<<TWINT)))
    {}
  
    //SLA+W(adres urządzenia slave + tryb zapisu do slave) została przekazana, ACK został odebrany
    if ((TWSR & 0xF8) == 0x18)
    {
       PCF8563_address = i;
       I2C_stop();    
       return;
    }

    I2C_stop(); 
    _delay_ms(10);
  }  
}

void I2C_start()
{
  //Master Transmitter Mode
  //start
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  
  //Poczekaj na ustawienie flagi TWINT. Oznacza to, że warunek START został przesłany
  while (!(TWCR & (1<<TWINT)))
  {}

  //Sprawdzenie czy bajt danych został przesłany, ACK został odebrany
  error(0x08);
}

void I2C_firstByte(char direction)
{
  uint8_t address = 0;
  //adres urządzenia i bit kierunku (0, bo zapis danych z master do slave)
  if(direction == 'W')
  {
    address = (PCF8563_address<<1)|(I2C_course_write); 
  }
  else if(direction == 'R')
  {
    address = (PCF8563_address<<1)|(I2C_course_read); 
  }
  TWDR = address;
  TWCR = (1<<TWINT) | (1<<TWEN);

  //Poczekaj na ustawienie flagi TWINT. Oznacza to, że DANE zostały przesłane i odebrano ACK/NACK
  while (!(TWCR & (1<<TWINT)))
  {}

  //Sprawdzenie czy bajt danych został przesłany, ACK został odebrany
  if(direction == 'W')
  {
    error(0x18);
  }
  else if(direction == 'R')
  {
    error(0x40);
  }
}

void I2C_writeData(uint8_t address, uint8_t DATA)
{
  //Załaduj adres do rejestru TWDR. Wyczyść bit TWINT w TWCR, aby rozpocząć transmisję danych
  TWDR = address;
  TWCR = (1<<TWINT) | (1<<TWEN);

  //Poczekaj na ustawienie flagi TWINT. Oznacza to, że DANE zostały przesłane i odebrano ACK/NACK
  while (!(TWCR & (1<<TWINT)))
  {}

  //Sprawdzenie czy bajt danych został przesłany, ACK został odebrany
  error(0x28);

  //Załaduj dane do rejestru TWDR. Wyczyść bit TWINT w TWCR, aby rozpocząć transmisję danych
  TWDR = DATA;
  TWCR = (1<<TWINT) | (1<<TWEN);

  //Poczekaj na ustawienie flagi TWINT. Oznacza to, że DANE zostały przesłane i odebrano ACK/NACK
  while (!(TWCR & (1<<TWINT)))
  {}

  //Sprawdzenie czy bajt danych został przesłany, ACK został odebrany
  error(0x28);
 
}

uint8_t I2C_readData(uint8_t registerAddress)
{
  I2C_start();
  I2C_firstByte('W');
  
  //Wysłanie adresu rejestru, z którego chcemy odczytać dane
  TWDR = registerAddress;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));    
  error(0x28);  
    
  I2C_start();
  I2C_firstByte('R');
  
  //Odbiór danych
  TWCR = (1 << TWINT) | (1 << TWEN); 
  while (!(TWCR & (1 << TWINT)));  
  uint8_t data = TWDR;       
  
  I2C_stop();
  
  return data; 
}

void I2C_stop()
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  while(TWCR & (1<<TWSTO)) 
  {}
}

void error(uint8_t value)
{
  if ((TWSR & 0xF8) != value)
  {
    PORTB |= (1<<PB5);  
  }
  else
  {
    PORTB &= ~(1<<PB5);
  }
}



//....................................................... PCF8563 ...............................................................
void setData(uint8_t day, uint8_t weekday, uint8_t month, uint8_t year)
{
  setDay(day);
  setWeekday(weekday);
  setMonth(month);
  setYear(year);
}

void setTime(uint8_t second, uint8_t minute, uint8_t hour)
{
  setSecond(second);
  setMinute(minute);
  setHour(hour);
}

void setAlarm(uint8_t minute, bool m_on, uint8_t hour, bool h_on, uint8_t day, bool d_on, uint8_t weekday, bool wd_on)
{
  minuteAlarm(minute, m_on);
  hourAlarm(hour, h_on);
  dayAlarm(day, d_on);
  weekdayAlarm(weekday, wd_on);
 
  I2C_start();
  I2C_firstByte('W');
  
  //ustawienie bitu AIE (flaga przerwania)
  I2C_writeData(CONTROL2, 0b00000010);  
  
  I2C_stop();
}

void setDay(uint8_t day)
{
  I2C_start();
  I2C_firstByte('W');

  I2C_writeData(DAY, BDC_DecToBin(day));
 
  I2C_stop();
}

void setWeekday(uint8_t weekday)
{
  I2C_start();
  I2C_firstByte('W');

  I2C_writeData(WEEKDAY, weekday);
  
  I2C_stop();
}

void setMonth(uint8_t month)
{
  I2C_start();
  I2C_firstByte('W');

  I2C_writeData(MONTH, BDC_DecToBin(month));
  
  I2C_stop();
}

void setYear(uint8_t year)
{
  I2C_start();
  I2C_firstByte('W');

  I2C_writeData(YEAR, BDC_DecToBin(year));
  
  I2C_stop();
}

void setSecond(uint8_t second)
{
  I2C_start();
  I2C_firstByte('W');

  I2C_writeData(SECOND, BDC_DecToBin(second));
  
  I2C_stop();
}

void setMinute(uint8_t minute)
{
  I2C_start();
  I2C_firstByte('W');

  I2C_writeData(MINUTE, BDC_DecToBin(minute));
  
  I2C_stop();
}

void setHour(uint8_t hour)
{
  I2C_start();
  I2C_firstByte('W');
  
  I2C_writeData(HOUR, BDC_DecToBin(hour));
  
  I2C_stop();
}

void minuteAlarm(uint8_t minute, bool m_on)
{
  I2C_start();
  I2C_firstByte('W');
  
  if(m_on == 0)
  {
    I2C_writeData(MINUTE_ALARM, (0b10000000));
  }
  else
  {
    I2C_writeData(MINUTE_ALARM, (BDC_DecToBin(minute) & (0b01111111)));
  }

  I2C_stop();
}

void hourAlarm(uint8_t hour, bool h_on)
{
  I2C_start();
  I2C_firstByte('W');
  
  if(h_on == 0)
  {
    I2C_writeData(HOUR_ALARM, (0b10000000));
  }
  else
  {
    I2C_writeData(HOUR_ALARM, (BDC_DecToBin(hour) & (0b00111111)));
  }

  I2C_stop();
}

void dayAlarm(uint8_t day, bool d_on)
{
  I2C_start();
  I2C_firstByte('W');
  
  if(d_on == 0)
  {
    I2C_writeData(DAY_ALARM, (0b10000000));
  }
  else
  {
    I2C_writeData(DAY_ALARM, (BDC_DecToBin(day) & (0b00111111)));
  }

  I2C_stop();
}

void weekdayAlarm(uint8_t weekday, bool wd_on)
{
  I2C_start();
  I2C_firstByte('W');
  
  if(wd_on == 0)
  {
    I2C_writeData(WEEKDAY_ALARM, (0b10000000));
  }
  else
  {
     I2C_writeData(WEEKDAY_ALARM, (weekday & (0b00000111)));
  }

  I2C_stop();
}

uint8_t getSecond()
{
  uint8_t sec = I2C_readData(SECOND);
  return BDC_BinToDec(sec);  
}

uint8_t getMinute()
{
  uint8_t min = I2C_readData(MINUTE);
  min = min & 0b01111111;
  return BDC_BinToDec(min);  
}

uint8_t getHour()
{
  uint8_t h = I2C_readData(HOUR);
  h = h & 0b00111111;
  return BDC_BinToDec(h);  
}

uint8_t getDay()
{
  uint8_t d = I2C_readData(DAY);
  d = d & 0b00111111;
  return BDC_BinToDec(d);  
}

uint8_t getWeekday()
{
  uint8_t wd = I2C_readData(WEEKDAY);
  wd = wd & 0b00000111;
  return BDC_BinToDec(wd);  
}

uint8_t getMonth()
{
  uint8_t m = I2C_readData(MONTH);
  m = m & 0b00011111;
  return BDC_BinToDec(m);  
}

uint8_t getYear()
{
  uint8_t y = I2C_readData(YEAR);
  return BDC_BinToDec(y); 
}

uint8_t getminuteAlarm()
{
  uint8_t mA = I2C_readData(MINUTE_ALARM);
  mA = mA & 0b01111111;
  return BDC_BinToDec(mA);  
}

uint8_t gethourAlarm()
{
  uint8_t hA = I2C_readData(HOUR_ALARM);
  hA = hA & 0b00111111;
  return BDC_BinToDec(hA);  
}

uint8_t BDC_DecToBin(uint8_t value)
{
  uint8_t BDC = ((value/10)<<4) | (value%10);
  return BDC; 
}

uint8_t BDC_BinToDec(uint8_t value)
{
  uint8_t BDC1 = (value>>4) * 10;
  uint8_t BDC2 = (value & 0b00001111);
  uint8_t BDC = BDC1 + BDC2;
  return BDC;
}



//........................................................ DS18B20 ................................................................
void reset()
{
  //Master jako transmiter
  DDRD |= (1<<PIN);
  PORTD &= ~(1<<PIN);
  _delay_us(480);

  //Zwolnienie magistrali 
  DDRD &= ~(1<<PIN);
  _delay_us(60);
}

void presence()
{
  //Czas na odpowiedz czujnika
  _delay_us(60);

  //Sprawdzenie czy czujnik odpowiedział (PIN jest w stanie niskim) 
  if(!(PIND & (1<<PIN)))
  {
    _delay_us(240); 
  }
}

void writeByte(uint8_t byte_)
{
  for(int i=0; i<8; i++)
  {
    writeBit((byte_>>i) & 0b00000001);
  }
}

void writeBit(uint8_t bit_)
{
  //Master jako transmiter
  DDRD |= (1<<PIN);

  //Magistrala w dół
  PORTD &= ~(1<<PIN);
  
  if(bit_== 1)
  {
    _delay_us(2);

    //Zwolnienie magistrali
    DDRD &= ~(1<<PIN);

    _delay_us(60);
  }
  else
  {
    _delay_us(60);
    
    //Zwolnienie magistrali
    DDRD &= ~(1<<PIN);
    
    _delay_us(2);
  } 
}

uint8_t readBit()
{
  uint8_t bit_;
  
  DDRD |= (1<<PIN);
  PORTD &= ~(1<<PIN);
  _delay_us(2);
    
  //Zwolnienie magistrali
  DDRD &= ~(1<<PIN);
  _delay_us(15);
    
  if(PIND & (1<<PIN))
  {
    bit_ = 1;
  }
  else
  {
    bit_ = 0;
  }
  
  _delay_us(45);

  return bit_;
}

uint8_t readByte()
{
  uint8_t byte_ = 0b00000000;
  uint8_t bit_;
  
  for(int i=0; i<8; i++)
  {
    bit_ = readBit();

    if(bit_ == 1)
    {
      byte_ = byte_ | (1<<i);
    }

     _delay_us(45);
  }
  
  return byte_;
}

void wScratchpad()
{
  writeByte(TH);
  writeByte(TL);
  writeByte(CONF);
}

void rScratchpad()
{
  uint8_t ROWS[9];

  for(int i=0; i<9; i++)
  {
    ROWS[i] = readByte();
  }

  uint8_t crc = countCRC(ROWS);

  if(crc != ROWS[8])
  {
    //Błędny crc, odczyt powtórzony
    DDRB |= (1<<PB5);
    PORTB |= (1<<PB5);  
    
    writeByte(R_SCRATCHPAD);
    rScratchpad();
  }
  else
  {
    return;
  }
}

uint8_t countCRC(uint8_t *rows)
{
  uint8_t crc = 0b00000000;
  
  for(int i=0; i<8; i++)
  {
    crc ^= rows[i];

    for(int j=0; j<8; j++)
    {
      if((crc&0b00000001)==1)
      {
        crc = (crc >> 1) ^ 0x8C;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void cScratchpad()
{
  DDRD |= (1<<PIN);
  PORTD |= (1<<PIN);
  _delay_us(10);

  DDRD &= ~(1<<PIN);
  _delay_ms(10);
}

void AddressScanner()
{
  uint8_t address_array[8];
  char string[50];

  reset();
  presence();
  
  writeByte(READ_ROM);
    
  for(int j=0; j<8; j++)
  {
    address_array[j]=readByte();
  }

  USART_String("address = ");
  for(int j=0; j<8; j++)
  {    
    sprintf(string, "%02X", address_array[j]);
    USART_String(string);
    USART_String(" ");
  }
   
  USART_String("\n");
}

void checkTemperature()
{
    reset();
    presence();

    writeByte(SKIP_ROM);
    writeByte(CONVERT);
    //czekaj dopóki konwersja trwa
    while(!(PIND & (1<<PIN))) {}

    reset();
    presence();
    writeByte(SKIP_ROM);
    writeByte(R_SCRATCHPAD);

    uint8_t ROWS[4];
    for(int i = 0; i <4; i++)
    {
       ROWS[i] = readByte();
    }

    //odczyt temperatury
    uint16_t read_temp = (ROWS[1] << 8) | ROWS[0]; 
    float temp = read_temp / 16.0; 

    uint8_t th = ROWS[2];
    uint8_t tl = ROWS[3];

    temperature = temp;
    
    checkAlarm(th, tl, temp);
}

void checkAlarm(uint8_t th, uint8_t tl, float temp)
{
  char str_temp1[20];
  char str_temp2[20];
  
  reset();
  presence();
  writeByte(ALARM_SEARCH);

  _delay_us(60);

  uint8_t address = readByte();
  if (address != 0xFF){
    
    if(temp>=th){
      DDRB |= (1<<PB5);
      PORTB |= (1<<PB5);
      dtostrf(temp, 7, 4, str_temp1);
      sprintf(str_temp2, "ALARM temperatura: %sC", str_temp1);
      USART_String(str_temp2);
      alarm_flag = 1;
    }
    else if(temp<=tl){
      DDRB |= (1<<PB5);
      PORTB |= (1<<PB5);
      dtostrf(temp, 7, 4, str_temp1);
      sprintf(str_temp2, "ALARM temperatura: %sC", str_temp1);
      USART_String(str_temp2);
      alarm_flag = 1;
    }
  }
  
  else{
      PORTB &= ~(1<<PB5);
  }
}

void DS18B20_init()
{
  reset();
  presence();
  
  writeByte(SKIP_ROM);
  writeByte(W_SCRATCHPAD);
  wScratchpad();
  
  reset();
  presence();
  
  writeByte(SKIP_ROM);
  writeByte(R_SCRATCHPAD);
  rScratchpad();

  reset();
  presence();
  
  writeByte(SKIP_ROM);
  writeByte(COPY_SCRATCHPAD);
  cScratchpad();
}




//.................................................... EXTERNAL_INT .............................................................
void ExternalInterupts_init()
{
  //failling edge
  EICRA |= (1<<ISC01);
  EIMSK |= (1<<INT0);

  //wlaczenie przerwań 
  sei();
}

ISR(INT0_vect)
{ 
  uint8_t value = I2C_readData(CONTROL2);
  //wyczyszczenie bitu AF (flaga alarmu)
  value = value & 0b11110111;
  
  I2C_start();
  I2C_firstByte('W');
  I2C_writeData(CONTROL2, value);
  I2C_stop();

  alarm_flag = 1;
}



//.................................................... INTERNAL_INT .............................................................
void InternalInterupts_init()
{
  //tryb pracy - CTC
  TCCR1B |= (1<<WGM12);

  //prescaler - 1024
  TCCR1B |= (1<<CS12);
  TCCR1B |= (1<<CS10);

  //porównywacz (co 500ms)-> 16000000/1024 = 15625 -> 15625*0,5s = 7812
  OCR1A = 15625;

  //włączenie przerwań
  TIMSK1 |= (1<<OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
  char string1[50];
  char string2[50];
  
  //USART_String("p\n");
  
  if(alarm_flag == 1)
  {
    float soil_moisture = checkMoisture();
    dtostrf(soil_moisture, 5,1, string1);
    sprintf(string2, "\nPomiar wilgotności gleby: %s%%\n", string1);
    USART_String(string2);
     
    alarm_flag = 0;
  }
}



//......................................................... PWM .................................................................
void PWM_init()
{
  //Tryb pracy - Fast PWM 8bit
  TCCR1A |= (1<<WGM10);
  TCCR1B |= (1<<WGM12);
  TCCR2A |= (1<<WGM20);
  TCCR2A |= (1<<WGM21);

  //Compare output mode
  TCCR1A |= (1<<COM1A1);
  TCCR1A |= (1<<COM1B1);
  TCCR2A |= (1<<COM2B1);
 
  //Prescaler = 1  
  TCCR1B |= (1<<CS10);
  TCCR2B |= (1<<CS20);
  
  //Częstotliwosc = 16000000/(1*(1+255)) = 62500Hz (62500 razy na sekunde zliczanie od 0 do 255)

  //Wypwełnienie
  OCR1A = 255;
  OCR1B = 255;
  OCR2B = 255;
}

void setColor(int r, int g, int b)
{
  r = 255-r;
  g = 255-g;
  b = 255-b;
  OCR1A = r;
  OCR1B = g;
  OCR2B = b;
}



//........................................................ USART ................................................................
void USART_init()
{
  UBRR0H = (unsigned char)(103>>8);
  UBRR0L = (unsigned char)103;
    
  //Adres we/wy rejestrów danych nadawczych USART i rejestry odbierania danych USART
  //UDR0

  //By bufor transmisji mógłbyć zapisany
  UCSR0A |= (1<<UDRE0);

  //Włączenie odbiornika
  UCSR0B |= (1<<RXEN0);

  //Włączenie nadajnika
  UCSR0B |= (1<<TXEN0);

  //Liczba bitów danych w ramce
  UCSR0C |= (1<<UCSZ00);
  UCSR0C |= (1<<UCSZ01);
}


void USART_Transmit( unsigned char data )
 {
 /* Wait for empty transmit buffer */
 while ( !( UCSR0A & (1<<UDRE0)) )
 ;
 /* Put data into buffer, sends the data */
 UDR0 = data;
 }

unsigned char USART_Receive()
 {
 /* Wait for data to be received */
 while ( !(UCSR0A & (1<<RXC0)) )
 ;
 /* Get and return received data from buffer */
 return UDR0;
 }

void USART_String(const char *array)
{
  int i=0;
  while(array[i]!='\0')
  {
    USART_Transmit(array[i]);
    i++;
  }
}



//.......................................................... ADC ..................................................................
void ADC_init()
{
  //zasilanie
  ADMUX |= (1<<REFS0);
  ADMUX &= ~(1<<REFS1);

  //division factor
  ADCSRA |= (1<<ADPS0);
  ADCSRA |= (1<<ADPS1);
  ADCSRA |= (1<<ADPS2);
  
  //włączenie ADC
  ADCSRA |= (1<<ADEN);
}

int channelA0()
{
  //kanał A0
  ADMUX &= ~(1<<MUX0);
  ADMUX &= ~(1<<MUX1);
  ADMUX &= ~(1<<MUX2);
  ADMUX &= ~(1<<MUX3);
  
  ADCSRA |= (1<<ADSC);

  while(ADCSRA & (1<<ADSC))
  {}

  //zwracanie zmierzonego napięcia
  return ADC;
}

float checkMoisture()
{
  int resultA0 = 0;
  float moisture_percent = 0.0;

  resultA0 = channelA0();
  moisture_percent = (1023.0 - resultA0)/1023.0*100.0;

  return moisture_percent;
}




//.......................................................... MAIN ...............................................................
int main()
{
  char string1[50];
  char string2[50];
  float soil_moisture=0;

  //dioda RGB
  DDRB |= (1<<PB1);
  DDRB |= (1<<PB2);
  DDRD |= (1<<PD3);
  
  //dioda do errors
  DDRB |= (1<<PB5);

  //Czujnik DS18B20
  DDRD &= ~(1<<PIN);
  PORTD |= (1<<PIN);

  //pull-up INT0 
  DDRD &= ~(1 << PD2); 
  PORTD |= (1 << PD2); 
  
  ExternalInterupts_init();
  I2C_init();
  DS18B20_init();
  ADC_init();
  PWM_init();
  USART_init();
  
  setData(01,5,11,24);
  setTime(50, 24, 11);
  setAlarm(25, 1, 11, 1, 23, 0, 3,0);

  //startowa wartość wilgotności gleby zaraz po podłączeniu urządzenia
  soil_moisture = checkMoisture();
  
  while (1) 
  {
    
    if(i >= 15)
    {
      setTime(50, 24, 11);
      i=0;
    }
  
    checkTemperature();
    if(alarm_flag == 1)
    {
      dtostrf(soil_moisture, 5,1, string1);
      sprintf(string2, "\nPomiar wilgotności gleby: %s%%\n", string1);
      USART_String(string2);
      
      alarm_flag = 0;
    }
      
    if(soil_moisture <= 30)
    {
      //odcienie czerwieni
      int g = soil_moisture/2;
      int r = 250-soil_moisture/2;
      int b = 0;
      setColor(r,g,b);
    }
    else if((soil_moisture > 30)&&(soil_moisture <= 40))
    {
      //czerwony->żółty
      int g = soil_moisture;
      int r = 235;
      int b = 0;
      setColor(r,g,b);
    }
    else if((soil_moisture > 40)&&(soil_moisture <= 60))
    {
       //żółty->zielony
       int g = soil_moisture*1.2;
       int r = 235-soil_moisture;
       int b = 0;
       setColor(r,g,b);
     }
     else if((soil_moisture > 60))
     {
        //odcienie zieleni
        int g = soil_moisture*1.2;
        int r = 174-soil_moisture;
        int b = 0;
        setColor(r,g,b);
     }
        
      _delay_ms(1000);

      i++;
  }
  
  return 0;
}
