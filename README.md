# c-atmega328p-rtc-soil-monitoring

**Project Description - English**

This project is a soil moisture and temperature monitoring system based on ATmega328P. It integrates:

RTC PCF8563 (I2C) for timekeeping and alarms,

DS18B20 (One-Wire) for temperature measurement,

Soil moisture sensor (ADC) for monitoring soil hydration levels.

The system uses an RGB LED (PWM) to indicate soil moisture levels, and INT0 external interrupt for alarm handling.


Features:

- RTC PCF8563 – read, set, and synchronize time.

- DS18B20 – temperature measurement and alarm triggering.

- Soil moisture sensor – ADC-based humidity measurement.

- INT0 & Timer1 interrupts – handling alarms and periodic readings.

- RGB LED (PWM) – visual moisture level indication.

- USART communication – monitoring data in a serial terminal.

Hardware Requirements:

- ATmega328P (e.g., Arduino Nano or standalone AVR)

- RTC PCF8563 (I2C: SDA – PC4, SCL – PC5)

- DS18B20 temperature sensor (One-Wire, PD4)

- Soil moisture sensor (ADC, A0)

- RGB LED (PWM: PB1, PB2, PD3)

- USB-UART converter (e.g., CP2102, FT232RL)

- Wiring cables

- 5V power supply

Usage Instructions:

- Compile the code using Atmel Studio, PlatformIO, or AVR-GCC.

- Connect the sensors to the microcontroller following the circuit diagram.

- Connect ATmega328P to the computer via USB-UART.

- Open a serial terminal (9600 baud) to observe measurements.

- Monitor the RGB LED color, which indicates moisture levels.

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

**Opis projektu - Polski**

Ten projekt to system monitorowania wilgotności gleby i temperatury przy użyciu mikrokontrolera ATmega328P, który komunikuje się z:

RTC PCF8563 (I2C) do obsługi zegara i alarmów,

DS18B20 (One-Wire) do pomiaru temperatury,

Czujnikiem wilgotności gleby (ADC) do monitorowania poziomu nawodnienia.

Dodatkowo, wartości wilgotności są prezentowane poprzez kolor diody RGB (PWM), a alarmy są sygnalizowane przerwaniem INT0.


Funkcjonalność:

- RTC PCF8563 – odczyt, ustawianie i synchronizacja czasu.

- DS18B20 – pomiar temperatury i wyzwalanie alarmów.

- Czujnik wilgotności gleby – odczyt wartości z przetwornika ADC.

- Przerwania INT0 i Timer1 – obsługa alarmu i cyklicznych pomiarów.

- Dioda RGB – wizualizacja poziomu wilgotności przez PWM.

- USART – monitorowanie danych w terminalu szeregowym.

Wymagania sprzętowe:

- ATmega328P (np. Arduino Nano lub standalone AVR)

- RTC PCF8563 (podłączony do I2C: SDA – PC4, SCL – PC5)

- Czujnik temperatury DS18B20 (One-Wire, PD4)

- Czujnik wilgotności gleby (podłączony do ADC, A0)

- Dioda LED RGB (PWM: PB1, PB2, PD3)

- Konwerter USB-UART (np. CP2102, FT232RL)

- Przewody połączeniowe

- Zasilanie 5V

Instrukcja użytkowania:

- Skompiluj kod w Atmel Studio, PlatformIO lub AVR-GCC.

- Podłącz czujniki do mikrokontrolera zgodnie ze schematem.

- Podłącz ATmega328P do komputera przez USB-UART.

- Uruchom terminal szeregowy (9600 baud) i obserwuj pomiary.

- Monitoruj kolor diody RGB, który wskazuje poziom wilgotności.

