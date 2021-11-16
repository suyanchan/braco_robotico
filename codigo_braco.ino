#include <stdio.h>
#include <avr/io.h>
#define F_CPU
#define TstBit(RES, BIT)(RES &  (1 << BIT)) // testar BIT, retorna 0 ou 1

void InitADC(){

ADMUX |= (1<<REFS0);
ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN);
}

uint16_t ReadADC (uint8_t ADCchannel){

ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
ADCSRA |= (1<<ADSC);

while( ADCSRA & (1<<ADSC));
return ADC;
}

void PWMGarraBaseTimer0(){
  DDRD |= (1 << PD5) | (1 << PD6) | (0 << PD2)| (0 << PD3)| (0 << PD4)| (0 << PD7) ;
 // TCCR0A = (1 << COM0A1) | (1 << COM0B1)| (1 << WGM02) | (1 << WGM01) | (1 << WGM00);
 TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << CS02) | (1 << CS00); //prescaller de 1024, em 255 da 16,32ms
 // OCRA=255;
 OCR0A = 0;
 OCR0B=0;
}

void PWMHorVertTimer1(){
  DDRB = 0b00000110;
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011010;//tres ultimos bits = prescaler de 8, (16MHz/8)*40000 = 50Hz, servo PWM = 50Kz
  ICR1 = 40000;
    OCR1A = 0;
      OCR1B = 0;
}




  int P1 = 0;
  int P2 = 0;
  int P3 = 0;
  int P4 = 0;
  int P5 = 0;
  int P6 = 0;
  int modo=0;//0 para manual, 1 para automatico
  int origem=0;
  int destino=0;
 /* 
  int AD0A=;
  int AD0B=;
  int AD1A=;
  int AD1B=;
*/

int main(){
  InitADC();
  PWMGarraBaseTimer0();
  PWMHorVertTimer1();
  delay(500);
  Serial.begin(9600);


  while(1){

if(TstBit(PINB, PB0) == false){
  modo=0;
}
else{
  modo=1;
}


  if(modo==0){
    manual();
  }
  else{
    automatico();
  }
   
 // _delay_ms(100);

 

  }
}





void automatico(){

  origem = ler_botoes();
   _delay_ms(1000);
  destino = ler_botoes();
   _delay_ms(1000);
   mover(origem,destino);
  
}






void mover(int origem,int destino){

  if(origem==1){
    pega_1();
  }
  else if(origem==2){
    pega_2();
  }
   else if(origem==3){
    pega_3();
  }
   else if(origem==4){
    pega_4();
  }


  if(destino==1){
    leva_1();
  }
  else if(destino==2){
    leva_2();
  }
  else if(destino==3){
    leva_3();
  }
  else if(destino==4){
    leva_4();
  }
    
}




void pega_1(){
  
  //motor base
  OCR0B=33; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(2250);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  //motor vertical
  OCR1A=280; //0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //motor horizontal
  OCR1B=940;//0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  //garra
  OCR0A=33; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato
 
  
  
}

void pega_2(){

   //motor vertical
  OCR1A=0; //0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //motor horizontal
  OCR1B=1040;//0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //motor base
  OCR0B=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

    //motor garra
  OCR0A=33; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato
  
}

void pega_3(){

   //motor base
  OCR0A=12; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(2000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

   //motor vertical
  OCR1A=600; //0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //motor horizontal
  OCR1B=1380;//0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //motor garra
  OCR0B=33; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato
  
}

void pega_4(){

   //motor vertical
  OCR1A=0; //0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //motor horizontal
  OCR1B=1560;//0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //motor base
  OCR0A=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //motor garra
  OCR0B=33; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato
  
}

void leva_1(){

   //um dos motores
  OCR1A=0; //0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR1B=0;//0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR0A=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR0B=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato
  
}

void leva_2(){

   //um dos motores
  OCR1A=0; //0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR1B=0;//0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR0A=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR0B=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato
  
}

void leva_3(){

   //um dos motores
  OCR1A=0; //0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR1B=0;//0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR0A=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR0B=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato
  
}

void leva_4(){

   //um dos motores
  OCR1A=0; //0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR1B=0;//0 é parado, 2000 pra um lado, 4000 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR0A=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato

  
  //outro motor
  OCR0B=0; //0 é parado, 12 pra um lado, 33 pro outro
  _delay_ms(1000);//aqui vc ajusta pra ver quanto tempo o motor tem que se mover até chegar no ponto exato
  
}




int ler_botoes(){

  if(TstBit(PIND, PD7) == true){
    return 1;
  }
  else  if(TstBit(PIND, PD2) == true){
    return 2;
  }
  else  if(TstBit(PIND, PD4) == true){
    return 3;
  }
  else  if(TstBit(PIND, PD3) == true){
    return 4;
  }

  
}















void manual(){
   int P1 = ReadADC(0);
    int P2 = ReadADC(1);
    int P3 = ReadADC(2);
    int P4 = ReadADC(3);
    int P5 = ReadADC(4);
    int P6 = ReadADC(5);
   // Serial.print("AD: ");
   // Serial.println(P1);
    Serial.print("OCR1A: ");
    Serial.println(OCR1A);
     Serial.print("OCR1B: ");
    Serial.println(OCR1B);
     Serial.print("OCR0A: ");
    Serial.println(OCR0A);
     Serial.print("OCR0B: ");
    Serial.println(OCR0B);
    delay(500);



//A1 controlando PWM da porta 9, Timer1A
        if(P1>=812){
           if(OCR1A>=4000){
          OCR1A=4000;
          //_delay_ms(80);
        }
        else{
          OCR1A=OCR1A+20;
          //_delay_ms(80);
        }
    }
    
    else if(P1<=212){

            if(OCR1A<=0){
        OCR1A=0;
      }
      else {
        OCR1A=OCR1A-20;
      }
      
      
    }
    else{
      OCR1B=OCR1B;
    }

//A1 controlando PWM da porta 10, Timer1B
        if(P2>=812){
           if(OCR1B>=4000){
          OCR1B=4000;
          //_delay_ms(80);
        }
        else{
          OCR1B=OCR1B+20;
          //_delay_ms(80);
        }
    }
    
    else if(P2<=212){

            if(OCR1B<=0){
        OCR1B=0;
      }
      else {
        OCR1B=OCR1B-20;
      }
      
      
    }
    else{
      OCR1B=OCR1B;
    }

    //A2 controlando PWM da porta 5, Timer0A
        if(P3>=812){
      
    
        OCR0A=33;
        }
    
        else if(P3<=212){

        OCR0A=0;
      
        }

      //A3 controlando PWM da porta 6, Timer0B
        if(P4>=812){
      
    
        OCR0B=33;
    }
    
    else if(P4<=212){

      OCR0B=12;
      
    }
    else{
      OCR0B=0;
    }


    
  
  

}
