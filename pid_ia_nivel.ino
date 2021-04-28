/* 
   Trabalho de graduação apresentado à FATEC Tatuí, com tema: DESENVOLVIMENTO E ANÁLISE DE CONTROLE PID INTEGRADO À INTELIGÊNCIA ARTIFICIAL COM REDES NEURAIS ARTIFICIAIS E APRENDIZADO DE MÁQUINA
 *
 * Created:   qua abr 14 2021
 * Processor: ATmega328P (Arduino Uno)
 * Autor: João Marcos
 */

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
#define V1 3        // V1 será relé que acionará válvula 1
#define V2 2        // V2 será relé que acionará válvula 2      