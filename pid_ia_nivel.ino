/* 
 * Trabalho de graduação apresentado à FATEC Tatuí, com tema: 
 * DESENVOLVIMENTO E ANÁLISE DE CONTROLE PID INTEGRADO À INTELIGÊNCIA ARTIFICIAL 
 * COM REDES NEURAIS ARTIFICIAIS E APRENDIZADO DE MÁQUINA
 *
 * Created:   qua abr 14 2021
 * Processor: ATmega328P (Arduino Uno)
 * Autor: João Marcos
 */

//Incluindo biblioteca de comunicação Serial Modbus
#include <Modbusino.h> 

//Endereçando arduino com ID 1
ModbusinoSlave modbusino_slave(1);

//Alocando 20 variáveis para comunicação Modbus
uint16_t tab_reg[20];

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

  //converte distância em nível no tanque
  pv = 35-distance;

  //calcula o erro do processo
  erro = sp-pv;
  
  //Atribuindo variáveis com tags de comunicação
    //Velocidades das bombas 0 à 1023
    /* convertendo para escala de 0 à 100
          1023 -- 100
             x -- 1
             x = 10.23 */
  float ar_pwm1 = analogRead(PWM1)/10.23;
  float ar_pwm2 = analogRead(PWM2)/10.23;

  //tab_reg[0] = ar_pwm1;
  //tab_reg[1] = ar_pwm2;
  
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
void triggerPulse()
{
  digitalWrite(TRIGGER, HIGH);  //Pulso de trigger em nível alto
  delayMicroseconds(10);        //tempo de 10 micro segundos
  digitalWrite(TRIGGER, LOW);   //Pulso de trigger em nível baixo
}
