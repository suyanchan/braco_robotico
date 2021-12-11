/****************************************************************************
 * Copyright (C) 2021 by Programacao de Hardware Engenharia de Computacao   *
 *                                                                          *
 * This file is part of Codigo_Final.                                       *
 *                                                                          *
 * Codigo_Final e um software livre: voce pode redistribui-lo e / ou        *
 * modifica-lo sob os termos da GNU Lesser General Public License conforme  *
 * publicada pela Free Software Foundation, seja a versao 3 da Licenca, ou  *
 * (a sua escolha) qualquer versao posterior.                               *
 *                                                                          *
 * O Codigo_Final e distribuido na esperanca de que seja util,              *
 * mas SEM QUALQUER GARANTIA; sem mesmo a garantia implicita de             *
 * COMERCIALIZACAO ou ADEQUACAO A UM DETERMINADO FIM. Veja o                *
 * GNU Lesser General Public License para obter mais detalhes.              *
 *                                                                          *
 * Voce deve ter recebido uma copia do GNU Lesser General Public            *
 * Licenca junto com Codigo_Final. Caso contrario, consulte                 *
 * <http://www.gnu.org/licenses/>.                                          *
 * <https://www.doxygen.nl/manual/index.html>                               *
 ****************************************************************************/

/**
 * @file Codigo_Manual.c
 * @author <b>Suyan Rocha Vidal RA 080350, Giovanna Pimenta RA 171886, Vinicius Mazitelli RA 173997
 * e Aguinaldo Parreira RA 176888</b>
 * @date 11 Dec 2021
 * @brief <b Codigo_Final_Manual do projeto Braco Robotico da materia Programacao de Hardware do 6 ciclo de Engenharia de Computacao.</b>
 *
 * Requisito do Projeto: Deverá ser projetado, construído e apresentado um braço robótico para movimentação de peças. 
 * É indicado que o braço robótico seja construído no FABLAB.
 * O design e a escolha dos materiais necessários para a construção do braço robótico e garra, ficam a escolha dos integrantes do grupo. 
 * Sugere-se levar em conta as forças necessárias para a execução das movimentações, possíveis torções e peso dos materiais envolvidos no projeto.
 * O braço robótico deverá ser capaz de realizar o transporte de peças entre os quatro pontos de um tablado previamente definido, logo, 
 * não teremos alterações dos pontos de captação e deposito de peças no dia da apresentação.
 * O braço deverá possuir o tamanho de sua base, compatível com o espaço definido para encaixe no tablado.
 * O braço deverá realizar as operações solicitadas no momento da apresentação, sendo que o comitê avaliador poderá indicar à sua escolha 
 * os pontos de retirada e depósito de peças entre os quatro disponíveis no tablado.
 * As movimentações deverão ser executadas em dois momentos; uma sequência de movimentação executada com o auxílio de um operador (modo manual) 
 * e uma segunda movimentação realizada em modo automático. Os pontos dessas duas movimentações não serão os mesmos.
 * Para a movimentação manual o grupo pode escolher entre um joystick, botões, etc.
 * A escolha do microcontrolador de controle ficará por responsabilidade do grupo.
 * Será obrigatório, caso necessário, uma placa para instalação dos periféricos (transistores, resistores, etc). Ou seja, não será permitido a utilização de protoboard no dia da apresentação.
 * Na data da apresentação deverá ser entregue um relatório com toda a documentação de componentes, programação, montagem, materiais e testes realizados. O relatório deverá obedecer o padrão ABNT.
 * Cada movimentação solicitada será avaliada e, a nota final será definida a partir da composição das duas avaliações conjuntamente ao relatório entregue.
 
 * O aluno(a) devera desenvolver um codigo em C, em que ele devera criar as funcoes de manipulacao de IO, 
 * configuracao de PWM e conversores analogicos e assim poder controlar o braco robotico.
 * 
 *
 * @note https://github.com/suyanchan/braco_robotico
 * 
 * @see http://inovfablab.unisanta.br
 * @see http://fabmanager.unisanta.br
 */


#include <stdio.h>
#include <avr/io.h>
#define F_CPU
#define TstBit(RES, BIT)(RES &  (1 << BIT))                     /**<Testa BIT, para escolha do modo, retorna 0 ou 1. */ 

/**
 * @brief Inicializa o Comparador Analogico.
 */

void InitADC(){
ADMUX |= (1<<REFS0);                                            /**<Seleciona Vref = AVcc */
ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN);     /**<Configura o Prescaller para 128 e habilita o ADC*/
}

/**
 * @brief Ler o valor de ADC.
 */
uint16_t ReadADC (uint8_t ADCchannel){
ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);                   /**<Seleciona o canal do ADC com uma mascara de seguranca*/
ADCSRA |= (1<<ADSC);                                            /**<Coloca no modo single mode*/
while( ADCSRA & (1<<ADSC));                                     /**<Espera ata a conversao do ADC finalizar*/
return ADC;                                                     /**<Returna o valor de ADC*/
}

/**
 * @brief Habilita e Configura as portas PWM, para o Timer 0.
 */
void PWMGarraBase_Timer0(){
 DDRD |= (1 << PD5) | (1 << PD6) | (0 << PD2)| (0 << PD3)| (0 << PD4)| (0 << PD7) ;     /**<Registra com 1 nas portas PWM PD5 e PD6, habilitando elas*/
 TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);                  /**<Inicializa o timer 0 no modo Fast PWM*/
 TCCR0B = (1 << CS02) | (1 << CS00);                                                    /**<Configura com Prescaller de 1024, em 255 da 16,32ms*/
}

/**
 * @brief Habilita e Configura as portas PWM, para o Timer 1.
 */
void PWMFrenteCima_Timer1(){
  DDRB = 0b00000110;                                                                    /**<Registra com 1 nas portas PWM PB1 e PB2, habilitando elas*/
  TCCR1A = 0b10100010;                                                                  /**<Configura o valor do prescaller*/
  TCCR1B = 0b00011010;                                                                  /**tres ultimos bits = prescaler de 8, (16MHz/8)*40000 = 50Hz, servo PWM = 50Kz*/
  ICR1 = 40000;                                                                         /**<Valor de TOP do prescale do Timer 1*/
  OCR1A =10;                                                                            /**<Valor inicial do registrador OCR1A*/
  OCR1B = 0;                                                                            /**<Valor inicial do registrador OCR1B*/
}


  int P1 = 0;               /**<Declara a variavel para atribuir o valor do analogico P1*/
  int P2 = 0;               /**<Declara a variavel para atribuir o valor do analogico P2*/
  int P3 = 0;               /**<Declara a variavel para atribuir o valor do analogico P3*/
  int P4 = 0;               /**<Declara a variavel para atribuir o valor do analogico P4*/
  int modo;
                                                                          
  int origem=0;
  int destino=0;

  
/**
 * @brief Funcao principal.
 */
int main(){                                 
  InitADC();                
  PWMGarraBase_Timer0();
  PWMFrenteCima_Timer1();
  delay(500);
  Serial.begin(9600);
  OCR0A=0;
  OCR0B=0;
  OCR1A=0;
  OCR1B=0;


  
while(1){
  manual();
}
}

/**
 * @brief Habilita o modo manual.
 */
void manual(){
  P1 = ReadADC(0);        /**<Atribui o valor lido no ADC da porta A0 na variavel P1*/
  P2 = ReadADC(1);        /**<Atribui o valor lido no ADC da porta A1 na variavel P2*/
  P3 = ReadADC(2);        /**<Atribui o valor lido no ADC da porta A2 na variavel P3*/
  P4 = ReadADC(3);        /**<Atribui o valor lido no ADC da porta A3 na variavel P4*/
 
  Serial.print("0A: ");   
  Serial.println(OCR0A);  
  Serial.print("0B: ");   
  Serial.println(OCR0B);  
  Serial.print("1A: ");   
  Serial.println(OCR1A);   
  Serial.print("1B: ");   
  Serial.println(OCR1B);  
  delay(500);


  if(P1>=812){             /**<A0 contralando PWM da porta 9, timer 1A*/
    if(OCR1A>=3900){
      OCR1A=3900;
      _delay_ms(50);
    }
    else{
      OCR1A=OCR1A+100;
      _delay_ms(50);
    }
  }
    
  else if(P1<=50){
    if(OCR1A<=0){
      OCR1A=0;
      _delay_ms(50);
    }
    else {
      OCR1A=OCR1A-100;
      _delay_ms(50);
    }
  }


  if(P2>=812){                /**<A1 contralando PWM da porta 10, timer 1B*/
    if(OCR1B>=3900){
      OCR1B=3900;
      _delay_ms(80);
    }
    else{
      OCR1B=OCR1B+100;
      _delay_ms(80);  
      }
  }
    
  else if(P2<=50){
    if(OCR1B<=0){
      OCR1B=0;
      _delay_ms(80);
    }
    else {
      OCR1B=OCR1B-100;
       _delay_ms(80);
    }
  }
 

  if(P3>=812){                /**<A2 contralando PWM da porta 5, timer 0A*/
    if(OCR0A>=18){
      OCR0A=18;
      _delay_ms(80);
    }
    else{
      OCR0A=OCR0A+1;
      _delay_ms(80);
      }
  }
    
  else if(P3<=50){
    if(OCR0A<=0){
      OCR0A=0;
      _delay_ms(80);
    }
    else{
      OCR0A=OCR0A-1;
      _delay_ms(80);
    }
  }


  if(P4>=812){                /**<A3 controlando PWM da porta 6, timer 0B*/
    if(OCR0B>=32){
      OCR0B=32;
      _delay_ms(80);
    }
    else{
      OCR0B=OCR0B+1;
      _delay_ms(80);
    }
  }
    
  else if(P4<=50){
    if(OCR0B<=0){
      OCR0B=0;
      _delay_ms(80);
    }
    else{
      OCR0B=OCR0B-1;
      _delay_ms(80);
    }
  }
 
}
