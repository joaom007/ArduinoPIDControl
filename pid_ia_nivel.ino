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

// Funções e variaveis p/ sensor ultrassônico
void trigPuls();      //Função p/ gerar o pulso de trigger
float pulse;          //Variável p/ armazenar o tempo de pulso echo
float distance;       //Variável p/ armazenar a distância em cm

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

void loop()
{
  
}
