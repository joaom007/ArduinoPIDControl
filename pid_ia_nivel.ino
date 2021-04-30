/* 
 * Trabalho de graduação apresentado à FATEC Tatuí, com tema: 
 * DESENVOLVIMENTO E ANÁLISE DE CONTROLE PID INTEGRADO À INTELIGÊNCIA ARTIFICIAL 
 * COM REDES NEURAIS ARTIFICIAIS E APRENDIZADO DE MÁQUINA
 *
 * Created:   qua abr 14 2021
 * Processor: ATmega328P (Arduino Uno)
 * Autor: João Marcos, 
 * github: https://github.com/joaom007
 * linkedin: https://www.linkedin.com/in/joaomarcos17/
 */
 
//Debugger brincando com ideias: https://www.youtube.com/watch?v=V41FtmNrNyk
#define pinBotaoDebug 18
#define habilitaDebugSerial true //define se envia informações do funcionamento para o monitor serial. "true" envia e "false" não envia. Utilizado apenas para identificar problemas de funcionamento atraves do monitor serial do IDE Arduino. Em situações normais, definir este parametro como "false". Quando usar o monitor, ele deve ser configurado para a velocidade de 115200.

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

  if (tempoParada == -1) {

     static bool estadoBotaoAnt = digitalRead(pinBotaoDebug);
     static unsigned long delayDebounce;
     bool estadoBotao;
     bool aguarda = true;
     while (aguarda) {
       estadoBotao = digitalRead(pinBotaoDebug);
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


//Incluindo biblioteca de comunicação Serial Modbus
#include <Modbusino.h> 

//Endereçando arduino com ID 1
ModbusinoSlave modbusino_slave(1);

//Alocando 20 variáveis para comunicação Modbus
uint16_t tab_reg[20];

// Configuração de pilhas
#define STACK 1       // constante de quantas pilhas vou usar pilha 1 = posição 0
#define MAXSTACK 3    // constante de quantos elementos na pilha
int top[STACK];       // ponteiro das pilhas top[0], top[1] e top[2]...
float stack[MAXSTACK][STACK];   // é a matriz Pilha

// Mapeamento de portas
#define LSL1 17       // LSL1 será chave de level baixo 1
#define LSL2 16       // LSL2 será chave de level baixo 2
#define LSH1 15       // LSH1 será chave de level alto 1
#define LSH2 14       // LSH2 será chave de level alto 2
#define LEDPIN 13     // LEDPIN será um led no pino 13

   // Driver Motor Ponte H Dupla (L298n) Shield
#define PWM1 11       // PWM1 será o controle de velocidade da bomba 1
#define INT1 10       // INT1 será o controle do sentido horário de giro da bomba 1
#define INT2 9        // INT2 será o controle do sentido anti-horário de giro da bomba 1
#define INT3 8        // INT3 será o controle do sentido horário de giro da bomba 2
#define INT4 7        // INT4 será o controle do sentido anti-horário de giro da bomba 2
#define PWM2 6        // PWM2 será o controle de velocidade da bomba 2da bomba 2
   
   // Sensor ultrassônico (Jsn-sr04t/aj-sr04m)
#define TRIGGER 5     // TRIGGER será a saída de trigger do sensor ultrassônico
#define ECHO 4        // ECHO será a entrada de echo do sendor ultrassônico
    
    // Interface de potência relé p/ válvulas 
#define V1 3          // V1 será relé que acionará válvula 1
#define V2 2          // V2 será relé que acionará válvula 2

//Variaveis p/ sensor ultrassônico
float pulse,          //Variável p/ armazenar o tempo de pulso echo
      distance;       //Variável p/ armazenar a distância em centímetros

      
// Configurações do PID
float pv = 0,         //valor do processo (nível em cm)
      sp = 0,         //valor desejável ou escolhido para o processo (nível em cm)
      erro = 0,       //erro ou offset do sistema (sp-pv)
      kp = 0,         //constante kp
      ki = 0,         //constante ki
      kd = 0,         //constante kd
      proportional = 0,  //armazena valor proporcional
      integral = 0,   //armazena valor integral
      derivative = 0, //armazena valor derivativo
      PID = 0,        //armazena resultado PID
      time_process = 1;  //tempo de 1 segundo

// Variável p/ modo de funcionamento da planta
int mode = 0;

// Ajustes iniciais
void setup()
{
  //Debugger
  #if habilitaDebugSerial == true
      Serial.begin(115200);
      pinMode(pinBotaoDebug, INPUT_PULLUP); 
  #endif
  
  // Inicia pilha 1, vazia, retorno 1 = criada, retorno 0 = falha na criação
    // Pilha vazia, o ponteiro é igual a -1
  StackInit(0);  
    
  
  //Definindo velocidade de comunicação em 9600 bauds (max 115200)
  modbusino_slave.setup(9600);
  
  // Definindo a forma de utilização das chaves de nível no Arduino
  pinMode(LSL1, INPUT_PULLUP);    //Pino da chave de level baixo 1 será entrada digital em nível 1
  pinMode(LSL2, INPUT_PULLUP);    //Pino da chave de level baixo 2 será entrada digital em nível 1
  pinMode(LSH1, INPUT_PULLUP);    //Pino da chave de level alto 1 será entrada digital em nível 1
  pinMode(LSH2, INPUT_PULLUP);    //Pino da chave de level alto 2 será entrada digital em nível 1
  
  pinMode(LEDPIN, OUTPUT);        //Pino para teste de interrupção

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


  // Nível lógico inicial das portas
  digitalWrite(LEDPIN, LOW);      //Saída do pino LED inicia em nível baixo

  analogWrite(PWM1, 0);           //Saída analógica de PWM da bomba 1 inicia em 0
  analogWrite(PWM2, 0);           //Saída analógica de PWM da bomba 2 inicia em 0
  digitalWrite(INT1, LOW);        //Saída direta bomba 1 inicia em nível baixo
  digitalWrite(INT2, LOW);        //Saída reversa bomba 1 inicia em nível baixo
  digitalWrite(INT3, LOW);        //Saída direta bomba 2 inicia em nível baixo
  digitalWrite(INT4, LOW);        //Saída reversa bomba 2 inicia em nível baixo

  digitalWrite(TRIGGER, LOW);     //Saída trigger inicia em nível baixo
  digitalWrite(ECHO, LOW);        //Entrada echo inicia em nível baixo

  digitalWrite(V1, HIGH);         //Saída do relé inicia em 1 mantendo válvula NF
  digitalWrite(V2, HIGH);         //Saída do relé inicia em 1 mantendo válvula NF
  
}

// Processo do Arduíno
void loop()
{
    
  triggerPulse();                //Aciona função do trigger do módulo ultrassônico  
  pulse = pulseIn(ECHO, HIGH, 200000);   //Medindo o tempo de ECHO em nível alto
  distance = pulse/58;           //converte resultado da distância em centímetros

  #if habilitaDebugSerial == true
      debug(1, "variavel1", String());
  #endif
  
  //converte distância em nível no tanque
  pv = 35-distance;

  //calcula o erro do processo
  erro = sp-pv;
  
  //Atribuindo variáveis com tags de comunicação
      
  //Medição do nível sensor ultrassônico
  tab_reg[2] = pv;
  
  //Valor desejável para o nível
  tab_reg[3] = sp;
  
  //Constantes PID
  tab_reg[4] = kp;
  tab_reg[5] = ki;
  tab_reg[6] = kd;
  
  //Receitas
  //tab_reg[7] = receita;
  
  //Válvulas (shield Relé)
  tab_reg[8] = digitalRead(V1);
  tab_reg[9] = digitalRead(V2);
  
  //Chaves de nível
  tab_reg[10] = digitalRead(LSL1);
  tab_reg[11] = digitalRead(LSL2);
  tab_reg[12] = digitalRead(LSH1);
  tab_reg[13] = digitalRead(LSH2);

  //Chaves sentido horário bombas 1 e 2
  tab_reg[14] = digitalRead(INT1);
  tab_reg[15] = digitalRead(INT3);

  //Modo de funcionamento da planta
  tab_reg[16] = mode;
  
    
  //Comunicando com supervisório
  modbusino_slave.loop(tab_reg, 20);

  /*Atribuindo atualização do supervisório no Arduíno*/
  
  mode = tab_reg[16];
  

  //Modo de funcionamento manual selecionado
  if(mode == 1)
  {
    
    //Velocidades das bombas 0 à 100
    /* convertendo para escala de 0 à 255
          255 -- 100
            x -- 1
            x = 2.55 */
    float aw_pwm1 = tab_reg[0]*2.55;
    float aw_pwm2 = tab_reg[1]*2.55;
    
    analogWrite(PWM1, aw_pwm1);
    analogWrite(PWM2, aw_pwm2);
    
    sp = tab_reg[3];
    kp = tab_reg[4];
    ki = tab_reg[5];
    kd = tab_reg[6];
    
    digitalWrite(V1, tab_reg[8]);
    digitalWrite(V2, tab_reg[9]);
    
    digitalWrite(INT1, tab_reg[14]);
    digitalWrite(INT3, tab_reg[15]);
    mode = tab_reg[16];    
    
  }
  //Modo Automático (PID) selecionado
  else if (mode == 2)
  {
    
    
  }
  //Modo Inteligência Artificial (python) selecionado 
  else if (mode == 3)
  {

    
  }
  //Nenhum ou muitos modos selecionados, planta desligada, apenas realiza leituras
  else
  {
    
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    
    sp = tab_reg[3];
    kp = tab_reg[4];
    ki = tab_reg[5];
    kd = tab_reg[6];

    digitalWrite(V1, 1);
    digitalWrite(V2, 1);
    
    digitalWrite(INT1, 0);
    digitalWrite(INT3, 0);
  }

  
}

//Função para gerar pulso de trigger
void triggerPulse()
{
  digitalWrite(TRIGGER, HIGH);  //Pulso de trigger em nível alto
  delayMicroseconds(10);        //tempo de 10 micro segundos
  digitalWrite(TRIGGER, LOW);   //Pulso de trigger em nível baixo
}

// Função para iniciar a pilha s
  // Se StackInit for 0, indica falha na inicialização da pilha
int StackInit(int s)
{
   if ((s<0)|(s>STACK-1))
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
   if ((s<0)|(s>STACK-1))
   {    
      return 0;
   }
   else
   {
      //Verificar se pilha está vazia
      if (top[s]==-1)
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
   if ((s<0)|(s>STACK-1))
   {    
     return 0;
   }
   else
   {
     //Verificar se pilha está cheia
      if (top[s]==MAXSTACK-1)
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
   int full=FullStack(s);
   if ((s<0)|(s>STACK-1)|(full==1))
   {
     //não posso colocar elemento na pilha
     return 0;
   }
   else
   {
     //posso empilhar: mudar o ponteiro (cria posição)
     top[s]=top[s] + 1;
     
     //Empilhar valor x na nova posição
     stack[top[s]][s] = x;
     
     return 1;
   }
}

// Função para tirar elementos da pilha, desempilhado = valor no topo da pilha, não desempilhado = 0
int Pop(int s)
{
   //chama função para validar o valor de s
   int empty=EmptyStack(s);
   if ((s<0)|(s>STACK-1)|(empty==1))
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
