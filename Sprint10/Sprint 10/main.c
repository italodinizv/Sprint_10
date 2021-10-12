/*
 * Sprint 10.c
 *
 * Created: 06/10/2021 21:48:59
 * Author : italo
 */ 
 
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Nokia5110/nokia5110.h"

//Novos tipos
typedef enum enum_parametros {Sel_modo, Sel_tempo_verde, Sel_tempo_vermelho, Sel_tempo_amarelo, Size_enum_parametros} enum_parametros;
typedef struct stc_semaforo //Struct para armazenar os tempos dos estados do semáforo
{
	uint8_t modo;					//Manual (0) ou Automático (1)
	uint16_t tempo_verde_ms;		//Tempo estado verde (entre 1000ms e 9000ms)
	uint16_t tempo_vermelho_ms;		//Tempo estado vermelho (entre 1000ms e 9000ms)
	uint16_t tempo_amarelo_ms;		//Tempo estado amarelo (entre 1000ms e 9000ms)
	uint16_t carros_por_min;		//Fluxo de carros medido
	uint16_t sensor_lux;			//Sensor de intensidade luminosa do ambiente (LUX)
}stc_semaforo;						//Define o nome do novo tipo criado

//Variáveis globais
stc_semaforo semaforo = {.modo = 0, .tempo_verde_ms = 5000, .tempo_vermelho_ms = 3000, .tempo_amarelo_ms = 1000, .carros_por_min = 0, .sensor_lux = 0}; //inicializando valores
enum_parametros selecao_parametro = Sel_modo;
uint8_t flag_5000ms = 0, flag_500ms = 0;	//Flags para disparar tarefas cíclicas
uint32_t tempo_ms = 0;						//Contador geral de tempo decorrido em ms
uint16_t num_carros = 0;					//Contador do número de carros que passou pelo semáforo em 5seg

//Protótipos
void anima_semaforo(stc_semaforo Semaforo, uint32_t Tempo_ms);
void anima_LCD(stc_semaforo Semaforo);
void estima_carros_por_min(uint8_t *flag_disparo);
void leituraADC_sensor_LUX(uint8_t *flag_disparo);

//Tratamento de interrupções
ISR(TIMER0_COMPA_vect) //interrupção do TC0 a cada 1ms = (64*(249+1))/16MHz
{
	tempo_ms++;
	//PORTD ^=0b01000000;
	if((tempo_ms % 5000) == 0) //True a cada 5000ms
	{
		flag_5000ms = 1;
	}
	if((tempo_ms % 500) == 0) //True a cada 500ms
	{
		flag_500ms = 1;
	}
} 

ISR (INT0_vect)  //interrupção externa 0, PIND2, Sensor de presença de carros
{
	num_carros++;
}

ISR(PCINT2_vect) //interrupção 2 por mudança de pino na porta D
{
	if((PIND&0b00000010) == 0) //Botão "Pedestre" pressionado (PD1)
	{
		if(semaforo.tempo_verde_ms > 3000)   //Reduz tempo do sinal verde para 3 segundos, caso seja maior que isso, para que pedestre não espere muito
			semaforo.tempo_verde_ms = 3000;
		if(semaforo.tempo_amarelo_ms > 2000) //Reduz tempo do sinal amarelo para 2 segundos, caso seja maior que isso, para que pedestre não espere muito
			semaforo.tempo_amarelo_ms = 2000;
		
	}
	
	if ((PIND&0b00010000) == 0) //botão "+" pressionado (PD4)
	{
		switch (selecao_parametro)
		{
			case Sel_modo: //Modo
				semaforo.modo = !semaforo.modo;
				break;
			case Sel_tempo_verde:
				if(semaforo.tempo_verde_ms <= 8000)
					semaforo.tempo_verde_ms += 1000;
				break;
			case Sel_tempo_vermelho:
				if(semaforo.tempo_vermelho_ms <= 8000)
					semaforo.tempo_vermelho_ms += 1000;
				break;
			case Sel_tempo_amarelo:
				if(semaforo.tempo_amarelo_ms <= 8000)
					semaforo.tempo_amarelo_ms += 1000;
				break;
		}
	}
	
	if ((PIND&0b00100000) == 0) //botão "-" pressionado (PD5)
	{
		switch (selecao_parametro)
		{
			case Sel_modo: //Modo
				semaforo.modo = !semaforo.modo;
				break;
			case Sel_tempo_verde: //Tempo verde
				if(semaforo.tempo_verde_ms >= 2000)
					semaforo.tempo_verde_ms -= 1000;
				break;
			case Sel_tempo_vermelho: //Tempo vermelho
				if(semaforo.tempo_vermelho_ms >= 2000)
					semaforo.tempo_vermelho_ms -= 1000;
				break;
			case Sel_tempo_amarelo: //Tempo amarelo
				if(semaforo.tempo_amarelo_ms >= 2000)
					semaforo.tempo_amarelo_ms -= 1000;
				break;
		}
	}
	
	if((PIND&0b01000000) == 0) //Botão "S" pressionado (PD6)
	{
		if(selecao_parametro < (Size_enum_parametros - 1))
			selecao_parametro++;
		else
			selecao_parametro = Sel_modo;
	}
	
	anima_LCD(semaforo);
}

void anima_semaforo(stc_semaforo Semaforo, uint32_t Tempo_ms)
{
	const uint16_t estados[9] = {0b000001111, 0b000000111, 0b000000011, 0b000000001, 0b100000000, 0b011110000, 0b001110000, 0b000110000, 0b000010000};
	static int8_t i = 0;
	static uint32_t tempo_anterior_ms = 0;
	
	PORTB = estados[i] & 0b011111111;
	if (estados[i] & 0b100000000) //PORTD = (estados[i] & 0b100000000) >> 1;
		PORTD |= 0b10000000;
	else
		PORTD &= 0b01111111;	
		
	if (i <= 3)
	{
		if ((Tempo_ms - tempo_anterior_ms) >= (Semaforo.tempo_verde_ms/4))
		{
			i++;
			tempo_anterior_ms += (Semaforo.tempo_verde_ms/4);
		}
		
		//Acionamento do Buzzer
		DDRD  |= 0b00000001;
		PORTD |= 0b00000001;
	}
	else
	{
		if (i <= 4)
		{
			if ((Tempo_ms - tempo_anterior_ms) >= (Semaforo.tempo_amarelo_ms))
			{
				i++;
				tempo_anterior_ms += (Semaforo.tempo_amarelo_ms);
			}
			
			//Acionamento do Buzzer
			DDRD  |= 0b00000001;
			PORTD |= 0b00000001;
		}
		else 
		{
			if (i <= 8)
			{
				if ((Tempo_ms - tempo_anterior_ms) >= (Semaforo.tempo_vermelho_ms/4))
				{
					i++;
					tempo_anterior_ms += (Semaforo.tempo_vermelho_ms/4);
				}
				
				//Buzzer desligado
				PORTD &= 0b11111110;
			}
			else
			{
				i = 0;
				tempo_anterior_ms = Tempo_ms;
			}
		}
	}
}

void anima_LCD(stc_semaforo Semaforo)
{
	unsigned char modo_string[2];
	unsigned char tempo_verde_s_string[2];
	unsigned char tempo_vermelho_s_string[2];
	unsigned char tempo_amarelo_s_string[2];
	unsigned char carros_por_min_string[2];
	unsigned char sensor_lux_string[2];
	
	modo_string[0] = (Semaforo.modo) ? 'A' : 'M'; modo_string[1] = '\0';
	sprintf(tempo_verde_s_string, "%u", Semaforo.tempo_verde_ms/1000);
	sprintf(tempo_vermelho_s_string, "%u", Semaforo.tempo_vermelho_ms/1000);
	sprintf(tempo_amarelo_s_string, "%u", Semaforo.tempo_amarelo_ms/1000);
	sprintf(carros_por_min_string, "%u", Semaforo.carros_por_min);
	sprintf(sensor_lux_string, "%u", Semaforo.sensor_lux);
	
	nokia_lcd_clear(); //Limpa o LCD
	
	//TELA ESQUERDA
	nokia_lcd_set_cursor(0, 5);
	nokia_lcd_write_string("Modo", 1);
	nokia_lcd_set_cursor(30, 5);
	nokia_lcd_write_string(modo_string, 1);
	nokia_lcd_set_cursor(0, 15);
	nokia_lcd_write_string("T.Vd", 1);
	nokia_lcd_set_cursor(30, 15);
	nokia_lcd_write_string(tempo_verde_s_string, 1); nokia_lcd_write_string("s", 1);
	nokia_lcd_set_cursor(0, 25);
	nokia_lcd_write_string("T.Vm", 1);
	nokia_lcd_set_cursor(30, 25);
	nokia_lcd_write_string(tempo_vermelho_s_string, 1); nokia_lcd_write_string("s", 1);
	nokia_lcd_set_cursor(0, 35);
	nokia_lcd_write_string("T.Am", 1);
	nokia_lcd_set_cursor(30, 35);
	nokia_lcd_write_string(tempo_amarelo_s_string, 1); nokia_lcd_write_string("s", 1);
	
	nokia_lcd_set_cursor(40, (selecao_parametro * 10) + 5);
	nokia_lcd_write_string("<", 1);
	
	//Construcao da barra na tela
	nokia_lcd_set_cursor(45, 0);
	nokia_lcd_write_string("|", 1);
	nokia_lcd_set_cursor(45, 5);
	nokia_lcd_write_string("|", 1);
	nokia_lcd_set_cursor(45, 10);
	nokia_lcd_write_string("|", 1);
	nokia_lcd_set_cursor(45, 15);
	nokia_lcd_write_string("|", 1);
	nokia_lcd_set_cursor(45, 20);
	nokia_lcd_write_string("|", 1);
	nokia_lcd_set_cursor(45, 25);
	nokia_lcd_write_string("|", 1);
	nokia_lcd_set_cursor(45, 30);
	nokia_lcd_write_string("|", 1);
	nokia_lcd_set_cursor(45, 35);
	nokia_lcd_write_string("|", 1);
	
	//TELA DIREITA
	nokia_lcd_set_cursor(51, 25);
	nokia_lcd_write_string(carros_por_min_string, 2);
	nokia_lcd_set_cursor(55, 40);
	nokia_lcd_write_string("c/min", 1);
	
	nokia_lcd_set_cursor(51, 0);
	nokia_lcd_write_string(sensor_lux_string, 2);
	nokia_lcd_set_cursor(55, 15);
	nokia_lcd_write_string("lux", 1);
	
	nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
}

void estima_carros_por_min(uint8_t *flag_disparo)
{
	static uint16_t aux = 0;
	
	if (*flag_disparo)
	{
		*flag_disparo = 0;
		aux = num_carros;
		num_carros = 0;
		semaforo.carros_por_min = aux * 12; // *60/5 => converter de carros/5seg para carros/min
		
		if (semaforo.modo)
		{
			semaforo.tempo_verde_ms = 1000 + ((uint16_t)(semaforo.carros_por_min * 16.7)/1000) * 1000;
			if (semaforo.tempo_verde_ms > 9000)
				semaforo.tempo_verde_ms = 9000;
			semaforo.tempo_vermelho_ms = 9000 - ((uint16_t)(semaforo.carros_por_min * 16.7)/1000) * 1000;
			if (semaforo.tempo_vermelho_ms > 32000)
				semaforo.tempo_vermelho_ms = 1000;
		}
		
		//anima_LCD(semaforo);
	}
	
}

void leituraADC_sensor_LUX(uint8_t *flag_disparo)
{
	if (*flag_disparo)
	{
		semaforo.sensor_lux = 1023000/ADC - 1000;
		if (semaforo.sensor_lux > 300)
		{
			OCR2B = 0;
		}
		else 
		{
			if (((PINC & 0b1000000) ==0) || (semaforo.carros_por_min > 0))
			{
				OCR2B = 255;
			}
			else 
			{
				OCR2B = 85;
			}
		}
		
		*flag_disparo = 0;
		anima_LCD(semaforo);
	}
}

int main (void)
{
	//Definições de GPIO
	DDRB = 0b11111111; //Habilita os pinos PB0..7 como saídas
	DDRC &= 0b0111111; //Habilita PC6 como entrada
	DDRD = 0b10001000; //Habilita os pinos PD3 e PD7 como saídas e os demais como entradas
	PORTC |= 0b1000000;
	PORTD = 0b01110110; //Habilita pullups de PD1..6
	
	//Configurações das interrupções externas
	EICRA = 0b00001010; //interrupção externa INT0 e INT1 na borda de descida
	EIMSK = 0b00000011; //Habilita a interrupção externa INT0 e INT1
	PCICR = 0b00000100; //Habilita interrupção pin change 2 (porta D)
	PCMSK2 = 0b01110010; //Habilita interrupção PD1, PD4, PD5 e PD6
	
	//Configuração do Timer 0
	TCCR0A = 0b00000010; //Habilita modo CTC do TC0
	TCCR0B = 0b00000011; //Liga TC0 com prescaler = 64
	OCR0A = 249;		 //Ajusta o comparador para TC0 contar até 249
	TIMSK0 = 0b00000010; //Habilita a interrupção na igualdade de comparação com OCR0A. A interrupção ocorre a cada 1ms = (64*(249+1))/16MHz
	
	//Configuração do Timer 2
	TCCR2A = 0b00100011; //PWM não invertido no pino OC0B - modo Fast PWM
	TCCR2B = 0b00000110; //Liga TC0, prescaler = 256, fpwm = f0sc/(256*prescaler) - 16MHz/(256*256) = 244MHz
	OCR2B = 128; //controle do ciclo ativo do PWM OC0B - duty = 128/256 = 50%
	
	//Configuração do ADC
	ADMUX = 0b01000000;  //VCC como ref, canal 0
	ADCSRA = 0b11100111; //Habilita o AD, modo de conversão continua, prescaler = 128
	ADCSRB = 0b00000000; //Modo de conversão continua
	DIDR0 = 0b00000001;  //Desabilita pino PC0 como entrada digital
	
	//Inicialização do LCD
	nokia_lcd_init(); //Inicia LCD
	anima_LCD(semaforo);
	
	//Habilita o flag de interrupções globais
	sei();
	
	while(1)
	{
		anima_semaforo(semaforo, tempo_ms);
		estima_carros_por_min(&flag_5000ms);
		leituraADC_sensor_LUX(&flag_500ms);
	}
}