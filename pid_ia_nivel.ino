/* 
 * Trabalho de graduação apresentado à FATEC Tatuí, com tema: 
 * DESENVOLVIMENTO E ANÁLISE DE CONTROLE PID INTEGRADO À INTELIGÊNCIA ARTIFICIAL 
 * COM APRENDIZADO DE MÁQUINA ATRAVÉS DE REDES NEURAIS ARTIFICIAIS
 *
 * Created:   qua abr 14 2021
 * Processor: Mega 2560 (Arduino Mega)
 * Autor: João Marcos, 
 * github: https://github.com/joaom007
 * linkedin: https://www.linkedin.com/in/joaomarcos17/
 * 
 * Bibliotecas usadas:
 * Comunicação Modbus: https://github.com/stephane/modbusino
 * 
 * CREDITOS: 
 * Debugger, Brincando com ideias: https://www.youtube.com/watch?v=V41FtmNrNyk
 * Configuração Interrupção por timer, WR Kits: https://www.youtube.com/watch?v=BHa6u096Svo
 */

/*
#define DEBUG 22

float debugValue = 0;
#define habilitaDebugSerial false //define se envia informações do funcionamento para o monitor serial. "true" envia e "false" não envia. Utilizado apenas para identificar problemas de funcionamento atraves do monitor serial do IDE Arduino. Em situações normais, definir este parametro como "false". Quando usar o monitor, ele deve ser configurado para a velocidade de 115200.

#if habilitaDebugSerial == true
void debug(
  int pontoParada, String nomeVariavel, String valorVariavel, int tempoParada = -1) {   //TempoParada faz delay. Com -1, para até porta 13 mudar de nível
  
  Serial.print("(");
  Serial.print(pontoParada);
  Serial.print(") ");

  Serial.print(nomeVariavel);  
  Serial.print(": ");
  Serial.print(valorVariavel);    
  Serial.println();
  
  debugValue = valorVariavel.toFloat();

  if (tempoParada == -1) {

     static bool estadoBotaoAnt = digitalRead(DEBUG);
     static unsigned long delayDebounce;
     bool estadoBotao;
     bool aguarda = true;
     while (aguarda) {
       estadoBotao = digitalRead(DEBUG);
       if ( (estadoBotao != estadoBotaoAnt) && !estadoBotao ) {
          if ((millis() - delayDebounce) > 100) {
             aguarda = false;
             delayDebounce = millis();
          }
       } else if (estadoBotao != estadoBotaoAnt) {
         delayDebounce = millis(); 
       }
       estadoBotaoAnt = estadoBotao; 
     } 
  } else if (tempoParada > 0) {
     delay(tempoParada);
  }
}
#endif

*/

#include <Modbusino.h>            //Incluindo biblioteca de comunicação Serial Modbus
#include <SPI.h>
#include <SD.h>                   //Incluindo biblioteca read write SD card

File dataFile;                      //Cria um ponteiro para arquivo

ModbusinoSlave modbusino_slave(1);//Endereçando arduino com ID 1

uint16_t tab_reg[20];             //Alocando 20 variáveis para comunicação Modbus

// Configuração de pilhas
#define STACK 2                   // constante de quantas pilhas vou usar pilha 1 = posição 0
#define MAXSTACK 3                // constante de quantos elementos na pilha
int top[STACK];                   // ponteiro das pilhas top[0], top[1] e top[2]...
float stack[MAXSTACK][STACK];     // é a matriz Pilha

// Mapeamento de portas
    // Interface de potência relé p/ válvulas 
#define V2 2                      // V2 será relé que acionará válvula 2
#define V1 3                      // V1 será relé que acionará válvula 1

   // Sensor ultrassônico (Jsn-sr04t/aj-sr04m)
#define ECHO 4                    // ECHO será a entrada de echo do sendor ultrassônico
#define TRIGGER 5                 // TRIGGER será a saída de trigger do sensor ultrassônico

   // Driver Motor Ponte H Dupla (L298n) Shield
#define PWM2 6                    // PWM2 será o controle de velocidade da bomba 2da bomba 2
#define INT4 7                    // INT4 será o controle do sentido anti-horário de giro da bomba 2
#define INT3 8                    // INT3 será o controle do sentido horário de giro da bomba 2
#define INT2 9                    // INT2 será o controle do sentido anti-horário de giro da bomba 1
#define INT1 10                   // INT1 será o controle do sentido horário de giro da bomba 1  
#define PWM1 11                   // PWM1 será o controle de velocidade da bomba 1

  //Led para debug interrupção
#define LEDPIN 13                 // LEDPIN será um led no pino 13

  //Chaves de nível
#define LSH2 14                   // LSH2 será chave de level alto 2
#define LSH1 15                   // LSH1 será chave de level alto 1
#define LSL2 16                   // LSL2 será chave de level baixo 2  
#define LSL1 17                   // LSL1 será chave de level baixo 1

  //Led para debug botão
#define LED 42

  //SD card adapter 
#define CS 53


//Variaveis p/ sensor ultrassônico
float pulse,                      //Variável p/ armazenar o tempo de pulso echo
      distance;                   //Variável p/ armazenar a distância em centímetros

// Configurações do PID
float pv = 0,                     //valor do processo (nível em cm)
      level = 0,                  //valor do nível = 38-pv
      sp = 0,                     //valor desejável ou escolhido para o processo (nível em cm)
      error = 0,                  //erro ou offset do sistema (sp-pv)
      lastErr = 0,                //erro anterior do processo
      kp = 0,                     //constante kp
      ki = 0,                     //constante ki
      kd = 0,                     //constante kd
      proportional = 0,           //armazena valor proporcional
      integral = 0,               //armazena valor integral
      derivative = 0,             //armazena valor derivativo
      mv = 0,                     //armazena resultado PID
      pid = 0,                    //inicializa pid 
      time_process = 1;           //tempo de 1 segundo para processo
      
// Inicialização de tempo anterior para PID
unsigned long lastProcess = 0; //tempo anterior

// Variável p/ funcionamento da planta
int mode = 0,                     //Modo de funcionamento: Manual = 1, PID = 2, IA = 3
    receita = 0;                  //configuração de processo da planta, ex: válvula 1 on a cada 2 seg, válvula 2 off.    

int test_manual = 0,
    windup = 0;                   //Chave windup para detectar saturação de mv e desligar Integrador
    
// Constantes Timer
const uint16_t T5_init = 0;
const uint16_t T5_comp = (time_process * 16000000) / 256;
  /*  cálculo para timer de 1 segundo
   *  (1 * 16E6)/256 = 65000
   */

// Ajustes iniciais
void setup()
{
  //Debugger
  //#if habilitaDebugSerial == true
  //    pinMode(DEBUG, INPUT_PULLUP); 
  //#endif
    
  //Desabilita interrupções para escrever analogicamente sem problemas e configurar interrupções
  cli();
  
  // Inicia pilha 1, vazia, retorno 1 = criada, retorno 0 = falha na criação
    // Pilha vazia, o ponteiro é igual a -1
  StackInit(0);
  StackInit(1);  
    
  //Definindo velocidade de comunicação em 9600 bauds (max 115200)
  modbusino_slave.setup(115200);
  
  // Definindo a forma de utilização das chaves de nível no Arduino
  pinMode(LSL1, INPUT_PULLUP);    //Pino da chave de level baixo 1 será entrada digital em nível 1
  pinMode(LSL2, INPUT_PULLUP);    //Pino da chave de level baixo 2 será entrada digital em nível 1
  pinMode(LSH1, INPUT_PULLUP);    //Pino da chave de level alto 1 será entrada digital em nível 1
  pinMode(LSH2, INPUT_PULLUP);    //Pino da chave de level alto 2 será entrada digital em nível 1
  
  pinMode(LEDPIN, OUTPUT);        //Pino com LED da placa p/ teste de interrupção
  pinMode(LED, OUTPUT);           //Pino com LED para teste do botão debug

  // Definindo a forma de utilização do Driver Motor Ponte H no Arduino
  pinMode(PWM1, OUTPUT);          //Pino de Enable bomba 1 será saída digital
  pinMode(PWM2, OUTPUT);          //Pino de Enable bomba 2 será saída digital
  pinMode(INT1, OUTPUT);          //Pino de Rotação horária da bomba 1 será saída digital
  pinMode(INT2, OUTPUT);          //Pino de Rotação anti-horária da bomba 1 será saída digital
  pinMode(INT3, OUTPUT);          //Pino de Rotação horária da bomba 2 será saída digital
  pinMode(INT4, OUTPUT);          //Pino de Rotação anti-horária da bomba 2 será saída digital

  // Definindo a forma de utilização do Sensor ultrassônico no Arduino
  pinMode(TRIGGER, OUTPUT);       //Pino de trigger será saída digital
  pinMode(ECHO, INPUT);           //Pino de echo será entrada digital

  // Definindo a forma de utilização do shild de relés no Arduino
  pinMode(V1, OUTPUT);            //Pino do relé da válvula 1 será saída digital
  pinMode(V2, OUTPUT);            //Pino do relé da válvula 2 será saída digital

  //Definindo a forma de utilização do shild SD card
  pinMode(CS, OUTPUT);            //Configura saída para Chip Select
 
 
  // Nível lógico inicial das portas
  digitalWrite(LEDPIN, LOW);      //Saída do pino LED inicia em nível baixo
  digitalWrite(LED, LOW);         //Saída do pino LED inicia em nível baixo
  
  analogWrite(PWM1, 0);           //Saída analógica de PWM da bomba 1 inicia em 0
  analogWrite(PWM2, 0);           //Saída analógica de PWM da bomba 2 inicia em 0
  digitalWrite(INT1, LOW);        //Saída direta bomba 1 inicia em nível baixo
  digitalWrite(INT2, LOW);        //Saída reversa bomba 1 inicia em nível baixo
  digitalWrite(INT3, LOW);        //Saída reversa bomba 2 inicia em nível baixo
  digitalWrite(INT4, LOW);        //Saída direta bomba 2 inicia em nível baixo

  digitalWrite(TRIGGER, LOW);     //Saída trigger inicia em nível baixo
  digitalWrite(ECHO, LOW);        //Entrada echo inicia em nível baixo

  digitalWrite(V1, HIGH);         //Saída do relé inicia em 1 mantendo válvula NF
  digitalWrite(V2, HIGH);         //Saída do relé inicia em 1 mantendo válvula NF

  // Configuração do timer 5
    //Modo de Comparação
  TCCR5A = 0;
  
  //Prescaler 1:256
  TCCR5B |=  (1 << CS52);
  TCCR5B &= ~(1 << CS51);
  TCCR5B &= ~(1 << CS50);
  
  //Inicializa Registradores
  TCNT5 = T5_init;                //Inicio contagem
  OCR5A = T5_comp;                //comparação com contagem crescente timer 1 p/ estouro
  
  //Habilita Interrupção do Timer 5
  TIMSK5 = (1 << OCIE5A);
  
  //Habilita interrupções globais após configurações
  sei();

  //Inicializando leitor de SD card
  SD.begin(CS);
  dataFile = SD.open("data.csv", FILE_WRITE);     //Abre arquivo para escrita
   
  if(dataFile)                                     //Arquivo aberto com sucesso?
  {                                               //Sim...  
    String header = "Time;Pv;Error;Mv";
    dataFile.println(header);
    dataFile.close();
  } 
}

// Processo do Arduíno
void loop()
{
  
  //Teste se pilha está vazia, se não estiver, desempilha o erro e PID
  if(EmptyStack(0) == 0) level = Pop(0);
  if(EmptyStack(1) == 0) mv = Pop(1);

  //Atribuindo variáveis nas tags de comunicação   
  /*
   * Conversão 0~38cm (38.3) em resolução 10bits 2^10 = 1023
   * (level / 38)*1023
   */  
  //tab_reg[2] = (level / 38.3) * 1023;
  tab_reg[2] = (level / 37.65) * 1023;
  
  //Debug
  //#if habilitaDebugSerial == true
  //  debug(1, "Level: ", String(level));
  //  digitalWrite(LED, digitalRead(LED) ^ 1);
  //#endif
  
  //Válvulas
  tab_reg[8] = digitalRead(V1);
  tab_reg[9] = digitalRead(V2);
  
  //Chaves de nível
  tab_reg[10] = digitalRead(LSL1);
  tab_reg[11] = digitalRead(LSL2);
  tab_reg[12] = digitalRead(LSH1);
  tab_reg[13] = digitalRead(LSH2);

  //Cheves motores horário
  tab_reg[14] = digitalRead(INT1);
  tab_reg[15] = digitalRead(INT4);
  
  //Comunicando com supervisório
  modbusino_slave.loop(tab_reg, 20);

  //Atribuindo atualização do supervisório no Arduíno
  sp = tab_reg[3];
  kp = tab_reg[4];
  ki = tab_reg[5];
  kd = tab_reg[6];
  sp = (sp / 4095) * 35;
  kp = (kp / 1023) * 100;
  ki = (ki / 1023) * 50;
  kd = (kd / 1023) * 50;
  
  //Recebe receita de funcionamento do supervisório
  receita = tab_reg[7];
  
  //Recebe modo de funcionamento da planta do supervisório
  mode = tab_reg[16];
  
  //Modo de funcionamento manual selecionado
  if(mode == 1)
  { 
    //Mantem Sp e parametros kp, ki e kd em 0
    tab_reg[3] = 0;
    tab_reg[4] = 0;
    tab_reg[5] = 0;
    tab_reg[6] = 0;
        
    //Segurança p/ nível minimo
      //Desliga válvulas se chegar em nível mínimo (~7cm)
    if(digitalRead(LSL1) == 0 | digitalRead(LSL2) == 0) 
    {
      digitalWrite(V1, HIGH);           //Saída do relé em 1 mantendo válvula NF
      digitalWrite(V2, HIGH);           //Saída do relé em 1 mantendo válvula NF
    } else
    {
      digitalWrite(V1, tab_reg[8]);
      digitalWrite(V2, tab_reg[9]);
    }

    //Segurança p/ nível máximo
      //Desliga válvulas se chegar em nível máximo (~35cm)
    if(digitalRead(LSH1) == 0 | digitalRead(LSH2) == 0)
    {
      digitalWrite(INT1, LOW);
      digitalWrite(INT4, LOW);
      
    } else
    {
      //Desabilita interrupções para escrever analogicamente sem problemas
      cli();
      if(receita == 1)
      {
        //tab_reg[0] = 204;
        //tab_reg[1] = 204;
        //test_manual = 204;
        tab_reg[0] = 255;
        tab_reg[1] = 255;
        test_manual = 255;
      }
      else
      {
        test_manual = tab_reg[0];  
      }
      
      
      analogWrite(PWM1, tab_reg[0]);         // Escreve valor pwm recebido do supervisório na bomba 1
      analogWrite(PWM2, tab_reg[1]);         // Escreve valor pwm recebido do supervisório na bomba 2
      
      //Habilita interrupções
      sei(); 
      
      digitalWrite(INT1, tab_reg[14]);
      digitalWrite(INT4, tab_reg[15]);
    }     
  }
  
  //Modo Automático (PID) selecionado
  else if(mode == 2)
  {
    
    //Bombas 1 e 2 recebem parametro PID 
    tab_reg[0] = mv;
    tab_reg[1] = mv;
    tab_reg[17] = mv;
        
    if(mv <= 77)                                        //PID menor ou igual que zero? 77 equivale a inicio de funcionamento das bombas
    {                                                     //se sim
      digitalWrite(INT1, LOW);                              //Desativa funcionamento da bomba 1
      digitalWrite(INT4, LOW);                              //Desativa funcionamento da bomba 2
    } 
    else                                                  
    {                                                     //senão
      //Segurança p/ nível máximo
        //Desliga válvulas se chegar em nível máximo (35cm)
      if(digitalRead(LSH1) == 0 | digitalRead(LSH2) == 0) 
      {
        digitalWrite(INT1, LOW);
        digitalWrite(INT4, LOW);
        
      } else
      {
        digitalWrite(INT1, HIGH);                          //Ativa funcionamento da bomba 1
        digitalWrite(INT4, HIGH);                          //Ativa funcionamento da bomba 2
      }      
    }
        
    //Desabilita interrupções para escrever analogicamente sem problemas
    cli();
       
    analogWrite(PWM1, tab_reg[0]);                      // Escreve valor pwm do PID na bomba 1
    analogWrite(PWM2, tab_reg[1]);                      // Escreve valor pwm do PID na bomba 2
    
    //Habilita interrupções
    sei();

    //Segurança para nível minimo
      //Desliga válvulas se chegar em nível mínimo
    if(digitalRead(LSL1) == 0 | digitalRead(LSL2) == 0) 
    {
      digitalWrite(V1, HIGH);                           //Saída do relé em 1 mantendo válvula NF
      digitalWrite(V2, HIGH);                           //Saída do relé em 1 mantendo válvula NF
    } 
    
  }
  
  //Modo Inteligência Artificial (python) selecionado 
  else if (mode == 3)
  {

    
  }
  
  //Nenhum ou muitos modos selecionados, planta desligada, apenas realiza leituras
  else
  {
    //Zera varáveis não utilizadas
    tab_reg[0] = 0;
    tab_reg[1] = 0;
    tab_reg[3] = 0;
    tab_reg[4] = 0;
    tab_reg[5] = 0;
    tab_reg[6] = 0;
    tab_reg[17] = 0;
    
    //Desativa bombas
    digitalWrite(INT1, LOW);
    digitalWrite(INT4, LOW);
    
    //Desabilita interrupções para escrever analogicamente sem problemas
    cli();
    
    analogWrite(PWM1, 0);             // Escreve valor pwm 0 na bomba 1
    analogWrite(PWM2, 0);             // Escreve valor pwm 0 na bomba 2
    
    //Habilita interrupções
    sei();
    
    //Desativa relés de válvulas
    digitalWrite(V1, HIGH);              
    digitalWrite(V2, HIGH);

  }    
}

// Função para iniciar a pilha s
  // Se StackInit for 0, indica falha na inicialização da pilha
int StackInit(int s)
{
   if((s<0)||(s>STACK-1))
   {
      return 0;
   }
   else
   {
      top[s] = -1;
      return 1;
   }
}

//Função para verificar se a pilha está vazia, vazia = 1, não vazia = 0
int EmptyStack(int s)
{
   //Verificar se o valor de s é válido
   if((s<0)||(s>STACK-1))
   {    
      return 0;
   }
   else
   {
      //Verificar se pilha está vazia
      if(top[s] == -1)
      {
        //a pilha está vazia
        return 1;
      }
      else
      {
        //a pilha não está vazia
        return 0;
      }
   }
}

//Função para verificar se a pilha está cheia, cheia = 1, não cheia = 0
int FullStack(int s)
{
   //Verificar se o valor de s é válido
   if((s<0)||(s>STACK-1))
   {    
     return 0;
   }
   else
   {
     //Verificar se pilha está cheia
      if(top[s] == MAXSTACK-1)
      {
        //Pilha está cheia
        return 1;
      }
      else
      {
        //Pilha não está cheia
        return 0;
      }
   }
}

//Função para empilhar elementos na pilha, empilhado = 1, não empilhado = 0 
  //Empilha na pilha "s" o valor "x" flutuante
int Push(int s, float x)
{
   //chama função para validar o valor de s
   int full = FullStack(s);
   if((s<0)||(s>STACK-1)||(full==1))
   {
     //não posso colocar elemento na pilha
     return 0;
   }
   else
   {
     //posso empilhar: mudar o ponteiro (cria posição)
     top[s] = top[s] + 1;
     
     //Empilhar valor x na nova posição
     stack[top[s]][s] = x;
     
     return 1;
   }
}

// Função para tirar elementos da pilha, desempilhado = valor no topo da pilha, não desempilhado = 0
float Pop(int s)
{
   //chama função para validar o valor de s
   int empty=EmptyStack(s);
   if((s<0)||(s>STACK-1)||(empty==1))
   {
     //não posso retirar elemento na pilha
     return 0;
   }
   else
   {
     float unStack=stack[top[s]][s];
     top[s]=top[s]-1;
     return unStack;
     
   }
}

// Variável de processo com retorno de distância em cm
float ProcessValue()
{ 
  triggerPulse();                //Aciona função do trigger do módulo ultrassônico  
  pulse = pulseIn(ECHO, HIGH, 200000);   //Medindo o tempo de ECHO em nível alto
  distance = pulse/58.82;        //converte resultado da distância em centímetros
  
  return distance;
}

//Função para gerar pulso de trigger
void triggerPulse()
{
  digitalWrite(TRIGGER, HIGH);  //Pulso de trigger em nível alto
  delayMicroseconds(10);        //tempo de 10 ms
  digitalWrite(TRIGGER, LOW);   //Pulso de trigger em nível baixo
}

// Função de comparação do timer 5 para interrupção T5_init ---> T5_comp.
ISR(TIMER5_COMPA_vect)
{
  TCNT5 = T5_init;      //reinicializa TIMER5
  digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);

  pv = 0;
  float dist[5];
  for(int i = 0; i < 5; i++){
    dist[i] = 0;
    //captura distância no tanque
    dist[i] = ProcessValue();
    if(dist[i] < 0) dist[i] = 0;

    if(i != 0)
    {
      if((dist[i] > (dist[i-1]+1.0)) || (dist[i] < (dist[i-1]-1.0)))
      { 
        dist[i] = dist[i-1]; 
      }
    }
        
    pv += dist[i];
    delay(90);
  }
  
  //Média de 5 medidas
  pv /= 5;
  
  
  //Converte distância em nível
  //pv = 38.3 - pv;
  pv = 37.65 - pv;
  
  //empilha erro na pilha 0
  Push(0, pv);
  
  //Calcula erro do processo
  error = sp - pv;
  
  //Calcula diferença de tempo entre as medições
  unsigned long deltaTime = (millis() - lastProcess);
  lastProcess = millis();

  if(mode == 1)
  {
    //Cria string de dados para armazenar no cartão SD
    String dataString = String(lastProcess/1000.0) + ";" + String(pv) + ";" + String(error) + ";" + String(test_manual);
    dataFile = SD.open("data.csv", FILE_WRITE);
    //Abre o arquivo para escrita
    if(dataFile)                                     //Arquivo aberto com sucesso?
    {                                               //Sim...  
      dataFile.println(dataString);
      dataFile.close();
      digitalWrite(LED, digitalRead(LED) ^ 1);
    } 
  }
  if(mode == 2)
  {
    //Troca estado das válvulas a cada duas interrupções, visto que o led troca de estado a cada interrupção
    if(digitalRead(LEDPIN) == 1)
    {
      digitalWrite(V1, digitalRead(V1) ^ 1);
      
      if(pv < 15.0)
      {
        digitalWrite(V2, digitalRead(V1));
      }
      else
      {
        digitalWrite(V2, HIGH);
      }
    }
    
    //Controlador PID
    proportional = kp * error;                           //calcula valor de proporcional
    if(!windup)
    {
      integral += ki * error * (deltaTime/1000.0);         //calcula valor de integral considerando diferença de tempo
    }
    derivative = kd * ((error - lastErr) / (deltaTime/1000.0));//calcula valor de derivativo em função da variação do erro
    lastErr = error;
    
    pid = proportional + integral + derivative + 77;    // Calcula pid e atribuí viés de 77, visto que bombas necessitam de 30% de carga para iniciar
   
    //pid menor que zero, força valor 77 que é o limite para bombas iniciar atuação
    if(pid <= 77)
    {
      pid = 77;
      windup = 1;
    }
    //pid maior que 255, força valor para 255
    else if(pid >= 255)
    {
      pid = 255;
      windup = 1;
    }
    else
    {
      windup = 0;
    }

    //Empilha pid na pilha 1
    Push(1, pid);
    
    Aquisition_data();
  }
  //Caso nenhum modo esteja selecionado reseta pid e suas variaveis
  else 
  {
    proportional = 0;
    integral = 0;
    derivative = 0;
    pid = 0;
  }
  
}

void Aquisition_data()
{
  
  //Cria string de dados para armazenar no cartão SD
  String dataString = String(lastProcess/1000.0) + ";" + String(pv) + ";" + String(error) + ";" + String(pid);
  dataFile = SD.open("data.csv", FILE_WRITE);
  //Abre o arquivo para escrita
  if(dataFile)                                     //Arquivo aberto com sucesso?
  {                                               //Sim...  
    dataFile.println(dataString);
    dataFile.close();
    digitalWrite(LED, digitalRead(LED) ^ 1);
  } 
   
}