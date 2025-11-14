```
/ Definições de Pinos e Constantes



// ADICIONEI ESTES PINOS E CONSTANTES NECESSÁRIAS

const int8_t TRIG_PIN = 32; // Pino TRIG do sensor ultrassônico

const int8_t ECHO_PIN = 34; // Pino ECHO do sensor ultrassônico



#define LIMIAR_NOITE 500       // Valor LDR abaixo do qual é considerado "noite"

#define LIMIAR_DISTANCIA 30    // Distância em cm para detecção de veículo/pedestre

#define TEMPO_AMARELO_NOITE 500 // 0.5 segundos para piscar no modo noturno



// Função para ler a distância do sensor ultrassônico

long readDistance() {

  // Limpa o pino Trig

  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(2);

  // Define o pino Trig em HIGH por 10 microssegundos

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  

  // Lê o tempo de duração do pulso de ECHO

  long duracao = pulseIn(ECHO_PIN, HIGH);

  // Converte a duração para distância em cm

  long distancia = duracao * 0.0343 / 2; 

  return distancia;

}







// Classe responsável por monitorar o sensor piezo e exibir no LCD

class Semaforo {

private:

  const int8_t vermelho1Pin;           // Pino led1 vermelho

  const int8_t verde1Pin;              // Pino led1 verde

  const int8_t amarelo1Pin;            // Pino led1 amarelo

  const int8_t vermelho2Pin;           // Pino led2 vermelho

  const int8_t verde2Pin;              // Pino led2 amarelo (será usado como Amarelo da Rua/Pedestre)

  const int8_t amarelo2Pin;            // Pino led2 verde (será usado como Verde da Rua/Pedestre)

  const int8_t ldr;                    // porta do sensor ldr

  int ultimoTempo1;                    // Variável de controle do tempo semáforo 1 

  int ultimoTempo2;                    // Variável de controle do tempo semáforo 2

  

  // ADICIONEI UMA VARIÁVEL PARA CONTROLE DO PISCA-PISCA NOTURNO

  unsigned long tempoPisca;



  // ADICIONEI UMA FUNÇÃO AUXILIAR PARA O MODO NOTURNO

  void setTrafficLightYellowBlinking(unsigned long tempoAtual) {

    if (tempoAtual - tempoPisca >= TEMPO_AMARELO_NOITE) {

      // Inverte o estado de todos os amarelos

      bool estado = digitalRead(amarelo1Pin) == LOW; 

      

      // Desliga tudo

      digitalWrite(vermelmo1Pin, LOW);

      digitalWrite(verde1Pin, LOW);

      digitalWrite(vermelmo2Pin, LOW);

      digitalWrite(amarelo2Pin, LOW); // Pino verde do semáforo 2

      digitalWrite(verde2Pin, LOW); // Pino amarelo do semáforo 2



      // Liga os amarelos

      digitalWrite(amarelo1Pin, estado ? HIGH : LOW);

      digitalWrite(verde2Pin, estado ? HIGH : LOW); // Usa o pino 'verde2' como amarelo do semáforo 2

      

      tempoPisca = tempoAtual;

    }

  }





public:

  // Construtor: define pinos (NÃO MUDADO)

  Semaforo(int8_t vermelho1Pin, int8_t verde1Pin, int8_t amarelo1Pin, int8_t vermelho2Pin, int8_t verde2Pin, int8_t amarelo2Pin, int8_t ldr): 



    vermelho1Pin(vermelho1Pin),

    verde1Pin(verde1Pin),

    amarelo1Pin(amarelo1Pin),

    vermelho2Pin(vermelho2Pin),

    verde2Pin(verde2Pin),

    amarelo2Pin(amarelo2Pin),

    ldr(ldr)

    

  {}





  // Inicialização do sistema (ADICIONEI CONFIGURAÇÃO DO ULTRASSÔNICO E TEMPO PISCA)

  void begin() {

    Serial.begin(115200); 

    pinMode(vermelho1Pin, OUTPUT);

    pinMode(verde1Pin, OUTPUT);

    pinMode(amarelo1Pin, OUTPUT);

    pinMode(vermelho2Pin, OUTPUT);

    pinMode(verde2Pin, OUTPUT);

    pinMode(amarelo2Pin, OUTPUT);

    

    // ADICIONEI A CONFIGURAÇÃO DOS PINOS DO ULTRASSÔNICO

    pinMode(TRIG_PIN, OUTPUT);

    pinMode(ECHO_PIN, INPUT);



    digitalWrite(vermelho1Pin, HIGH);

    digitalWrite(verde2Pin, HIGH); // Assumindo verde2Pin é o 'Amarelo' do semáforo 2, que começa ligado junto com o vermelho

    

    ultimoTempo1 = 0;

    ultimoTempo2 = 0;

    tempoPisca = millis(); // Inicializa o tempo de pisca

  }





  // Atualiza leitura e exibição (REIMPLEMENTADA A LÓGICA MISTURANDO TEMPO E SENSORES)

  void update() {



    int tempoAtual = millis();

    

    // 1. Leitura de Sensores

    int leituraLDR = analogRead(ldr);

    long distanciaCM = readDistance();

    

    Serial.print("LDR: ");

    Serial.print(leituraLDR);

    Serial.print(" | Distância: ");

    Serial.print(distanciaCM);

    Serial.println(" cm");

    

    // 2. Lógica do Modo Noturno (Prioridade Máxima)

    if (leituraLDR < LIMIAR_NOITE) {

      setTrafficLightYellowBlinking(tempoAtual);

      delay(10); // Reduzindo o delay fixo

      return; // Sai se for noite

    }



    // 3. Lógica Normal (Dia) - INTEGRAÇÃO COM SENSOR DE DISTÂNCIA

    

    // VARIÁVEL DE CONTROLE: Detectou veículo/pedestre na rua secundária (Semáforo 2)?

    bool deteccaoRua2 = (distanciaCM <= LIMIAR_DISTANCIA);

    

    // Transição 1: Vermelho1 (Avenida) -> Verde1 (Avenida)

    // Condição original: 4 segundos de Vermelho1

    if (tempoAtual - ultimoTempo1 >= 4000 && digitalRead(vermelho1Pin) == HIGH){

      // Ocorre apenas se a Rua/Pedestre (Semáforo 2) tiver completado o seu ciclo e estiver em Vermelho.

      // O Vermelho do Semáforo 2 é ativado na Transição 5.

      if (digitalRead(vermelho2Pin) == HIGH) { 

        digitalWrite(vermelho1Pin, LOW);

        digitalWrite(verde1Pin, HIGH);

        ultimoTempo1 = tempoAtual;

      }

    }

    

    // Transição 2: Verde1 (Avenida) -> Amarelo1 (Avenida)

    // Condição original: 2 segundos de Verde1. MUDANÇA: Agora, se detectar tráfego na rua secundária ANTES, ele deve ir para o Amarelo.

    if (digitalRead(verde1Pin) == HIGH) { // Se o semáforo 1 estiver verde

      if (tempoAtual - ultimoTempo1 > 4000 || (tempoAtual - ultimoTempo1 > 2000 && deteccaoRua2)) { 

        // Se já passou 4s OU (se passou 2s E detectou na rua 2)

        digitalWrite(verde1Pin, LOW);

        digitalWrite(amarelo1Pin, HIGH);

        ultimoTempo1 = tempoAtual;

      }

    }



    // Transição 3: Amarelo1 (Avenida) -> Vermelho1 (Avenida)

    // Condição original: 2 segundos de Amarelo1

    if (tempoAtual - ultimoTempo1 > 2000 && digitalRead(amarelo1Pin) == HIGH){

      digitalWrite(amarelo1Pin, LOW);

      digitalWrite(vermelho1Pin, HIGH);

      ultimoTempo1 = tempoAtual;

      // Neste ponto, o Semáforo 1 está Vermelho, liberando a sequência do Semáforo 2

    }





    // LÓGICA DO SEMÁFORO 2 (RUA/PEDESTRE)

    

    // Transição 4: Verde2 (Rua) -> Amarelo2 (Rua)

    // Condição original: 2 segundos de Verde2 E Vermelho1 ativo. MUDANÇA: Agora, apenas tempo.

    if (tempoAtual - ultimoTempo2 >= 2000 && digitalRead(verde2Pin) == HIGH){

      digitalWrite(verde2Pin, LOW);

      digitalWrite(amarelo2Pin, HIGH);

      ultimoTempo2 = tempoAtual;

    }



    // Transição 5: Amarelo2 (Rua) -> Vermelho2 (Rua)

    // Condição original: 2 segundos de Amarelo2

    if (tempoAtual - ultimoTempo2 >= 2000 && digitalRead(amarelo2Pin) == HIGH) {

      digitalWrite(amarelo2Pin, LOW);

      digitalWrite(vermelho2Pin, HIGH);

      ultimoTempo2 = tempoAtual;

    }



    // Transição 6: Vermelho2 (Rua) -> Verde2 (Rua)

    // Condição original: 4 segundos de Vermelho2. MUDANÇA: O Semáforo 2 SÓ DEVE ABRIR (ir para Verde)

    // SE o Semáforo 1 ESTIVER VERMELHO (Garantia de intertravamento) E se houver detecção na rua 2.

    if (digitalRead(vermelho2Pin) == HIGH) { // Se Semáforo 2 estiver Vermelho

      if (digitalRead(vermelho1Pin) == HIGH) { // E Semáforo 1 estiver Vermelho

         // Agora verifica a necessidade: se a detecçãoRua2 for verdadeira, ele abre o Verde2

        if (tempoAtual - ultimoTempo2 >= 4000 || deteccaoRua2) {

          digitalWrite(vermelho2Pin, LOW);

          digitalWrite(verde2Pin, HIGH);

          ultimoTempo2 = tempoAtual;

        }

      }

    }



    // O delay original de 1000ms foi substituído por um delay menor no modo noturno

    // e foi deixado aqui um pequeno delay para estabilidade.

    delay(10); 

  }

};





// Instância da classe 

// NOTA: O construtor original só aceita 7 parâmetros. Os pinos TRIG e ECHO foram definidos globalmente.

// Semaforo rodar(vermelho1, verde1, amarelo1, vermelho2, amarelo2(pino), verde2(pino), ldr);

Semaforo rodar(33, 26, 25, 21, 22, 23, 35); 





void setup() {

  rodar.begin();

}





void loop() {

  rodar.update();

}



COMPARE O CÓDIGO ACIMA COM O DE BAIXO:



#include "UbidotsEsp32Mqtt.h"



#define TRIG_PIN              19

#define ECHO_PIN              18

#define LDR_RUA_PIN           35

#define LDR_CALCADA_PIN       32

#define LED_GREEN_PIN_1       26

#define LED_YELLOW_PIN_1      25

#define LED_RED_PIN_1         33

#define LED_GREEN_PIN_2       23

#define LED_YELLOW_PIN_2      22

#define LED_RED_PIN_2         21



const char *UBIDOTS_TOKEN = "BBUS-0KKB8VTHx1jUYwZ3rUZJ659aEkq4bq";

const char *WIFI_SSID = "Inteli.Iot";

const char *WIFI_PASS = "@Intelix10T#";

const char *DEVICE_LABEL = "esp32_t11_g01";

const char *VARIABLE_LABEL_RED_1 = "luz_vermelha_1";

const char *VARIABLE_LABEL_YELLOW_1 = "luz_amarela_1";

const char *VARIABLE_LABEL_GREEN_1 = "luz_verde_1";

const char *VARIABLE_LABEL_LDR_RUA = "ldr_rua";

const char *VARIABLE_LABEL_LDR_CALCADA = "ldr_calcada";

const char *VARIABLE_LABEL_DISTANCE = "distance";

const int PUBLISH_FREQUENCY = 5000;



float duration_us, distance_cm;

int luz_calcada, luz_rua;

long int timer;



unsigned long greenStartTime = 0;

unsigned long yellowStartTime = 0;

bool isGreen = false;

bool isYellow = false;



Ubidots ubidots(UBIDOTS_TOKEN);



void callback(char *topic, byte *payload, unsigned int length) {

  Serial.print("Message arrived [");

  Serial.print(topic);

  Serial.print("] ");

 

  String message = "";

  for (int i = 0; i < length; i++) {

    message += (char)payload[i];

  }

 

  Serial.println(message);

}



void setup() {

  Serial.begin(115200);

  ubidots.setDebug(true);

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);

  ubidots.setCallback(callback);

  ubidots.setup();

  ubidots.reconnect();

  pinMode(LED_GREEN_PIN_1, OUTPUT);

  pinMode(LED_YELLOW_PIN_1, OUTPUT);

  pinMode(LED_RED_PIN_1, OUTPUT);

  pinMode(LED_GREEN_PIN_2, OUTPUT);

  pinMode(LED_YELLOW_PIN_2, OUTPUT);

  pinMode(LED_RED_PIN_2, OUTPUT);



  timer = millis();



  pinMode(TRIG_PIN, OUTPUT);

  pinMode(ECHO_PIN, INPUT);

  pinMode(LDR_CALCADA_PIN, INPUT);

  pinMode(LDR_RUA_PIN, INPUT);

}



void loop() {  

  readDistance();

  readLight();

  delay(100);



  if (!ubidots.connected()) {

    ubidots.reconnect();

  }



  controlTrafficLight();



  if (millis() - timer >= PUBLISH_FREQUENCY) {

    publishData();

    timer = millis();

  }



  ubidots.loop();

}


```


# Lógica do Código de Semáforo Inteligente

## 1. Definições Iniciais e Parâmetros
- **Pinos Globais**: Define os pinos para o sensor ultrassônico (`TRIG_PIN`, `ECHO_PIN`) e o sensor LDR.
- **Limiares**: `LIMIAR_NOITE` para ativar modo noturno, e `LIMIAR_DISTANCIA` para detectar objetos próximos (ex: pedestres ou veículos).

```
// Definições de Pinos e Constantes
// **********************************
const int8_t TRIG_PIN = 32; // Pino TRIG do sensor ultrassônico
const int8_t ECHO_PIN = 34; // Pino ECHO do sensor ultrassônico

#define LIMIAR_NOITE 500       // Valor LDR abaixo do qual é considerado "noite"
#define LIMIAR_DISTANCIA 30    // Distância em cm para detecção de veículo/pedestre
#define TEMPO_AMARELO_NOITE 500 // 0.5 segundos para piscar no modo noturno

// Função para ler a distância do sensor ultrassônico
long readDistance() {

  // Envia um pulso curto no TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Recebe a duração do pulso de ECHO
  long duracao = pulseIn(ECHO_PIN, HIGH);
  
  // Converte o tempo de duração em distância (cm)
  long distancia = duracao * 0.0343 / 2; 
  return distancia;
}
```
####  Bloco 1:

= Pinos Globais: Os pinos do Sensor Ultrassônico (TRIG_PIN e ECHO_PIN) foram definidos globalmente porque o construtor da sua classe original não permitia a adição de mais parâmetros.

Limiares de Decisão (#define):

LIMIAR_NOITE: Define o valor de luz (leitura do LDR) que aciona o Modo Noturno.

LIMIAR_DISTANCIA: Define a distância (30 cm) que indica a presença de algo, acionando a prioridade na via secundária.

Função readDistance(): Essa função implementa o protocolo para medir a distância: envia um sinal de ultrassom (pulso no TRIG) e mede quanto tempo leva para o sinal retornar (no ECHO). O resultado é convertido para centímetros.


## Classe Semaforo e Funções Auxiliares.
```
// Classe responsável por monitorar o sensor piezo e exibir no LCD (Nota: Piezo foi substituído por Ultrassônico)

class Semaforo {
private:
  const int8_t vermelho1Pin; // ... (definição de todos os pinos de LEDs e LDR)
  const int8_t ldr;
  int ultimoTempo1; // Variável de controle do tempo semáforo 1 
  int ultimoTempo2; // Variável de controle do tempo semáforo 2
  
  // ADICIONEI UMA VARIÁVEL PARA CONTROLE DO PISCA-PISCA NOTURNO
  unsigned long tempoPisca;

  // ADICIONEI UMA FUNÇÃO AUXILIAR PARA O MODO NOTURNO
  void setTrafficLightYellowBlinking(unsigned long tempoAtual) {
    if (tempoAtual - tempoPisca >= TEMPO_AMARELO_NOITE) {
      // Inverte o estado de todos os amarelos
      bool estado = digitalRead(amarelo1Pin) == LOW; 
      
      // Desliga todas as luzes antes de acender o amarelo
      // (códigos omitted para brevidade, mas o objetivo é apagar tudo)

      // Liga os amarelos
      digitalWrite(amarelo1Pin, estado ? HIGH : LOW);
      digitalWrite(verde2Pin, estado ? HIGH : LOW); // Usa o pino 'verde2' como amarelo do semáforo 2
      
      tempoPisca = tempoAtual; // Atualiza o tempo do último pisca
    }
  }

public:
  // Construtor: Inicializa os pinos com os valores passados na instância
  Semaforo(int8_t v1, int8_t g1, int8_t a1, int8_t v2, int8_t g2, int8_t a2, int8_t ldr) : 
    vermelho1Pin(v1), verde1Pin(g1), amarelo1Pin(a1), 
    vermelho2Pin(v2), verde2Pin(g2), amarelo2Pin(a2), ldr(ldr) {}
  
  // ... (função begin() abaixo)
  // ... (função update() abaixo)
};

``` 
#### Explicação do Bloco 2:
Variável tempoPisca: Uma variável crucial adicionada para implementar o Modo Noturno. Ela rastreia o último momento em que os LEDs amarelos piscaram.

Função setTrafficLightYellowBlinking(): Esta função é chamada quando o LDR detecta que é noite. Ela usa tempoPisca para garantir que o LED mude de estado (ligar/desligar) a cada 500ms (TEMPO_AMARELO_NOITE), criando o efeito intermitente sem usar o delay().

## 3. Classe Semaforo (Controle) 
##### Função begin() (Configuração de Pinos)

```
void begin() {
  Serial.begin(115200); 
  // Configuração de todos os pinos de LEDs como OUTPUT
  // ...
  
  // ADICIONEI A CONFIGURAÇÃO DOS PINOS DO ULTRASSÔNICO
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Estado inicial: Semáforo 1 (Avenida) em Vermelho, Semáforo 2 (Rua) em Verde (ou Amarelo/Vermelho, dependendo da sua fiação)
  digitalWrite(vermelho1Pin, HIGH);
  digitalWrite(verde2Pin, HIGH); 
  
  ultimoTempo1 = 0;
  ultimoTempo2 = 0;
  tempoPisca = millis(); // Inicializa o contador do pisca-pisca
}
```

#### Explicação do Bloco 3:
- Configura todos os pinos de LEDs e, fundamentalmente, os pinos globais do Ultrassônico (TRIG_PIN e ECHO_PIN).

- Define o estado inicial dos semáforos, preparando o sistema para o ciclo de tráfego.

- **Atributos Privados**: Pinos dos LEDs, variáveis de controle de tempo (`ultimoTempo1`, `ultimoTempo2`, `tempoPisca`).  

- **Método `begin()`**: Configura os pinos como saída e define o estado inicial (semáforo de carro: vermelho, semáforo de pedestre: verde).  

- **Método `update()`**: Executado continuamente, implementa a lógica de controle dos sinais, considerando:

  - **Modo Noturno**: Se a leitura do LDR estiver abaixo de `LIMIAR_NOITE`, aciona o pisca-pisca amarelo (`setTrafficLightYellowBlinking()`).

  - **Controle de Semáforo No Dia**: Usa o valor da distância do sensor ultrassônico para detectar pedestres ou veículos na via secundária.

  - **Transições**:

  - **Carro (Semáforo 1)**: Verde para amarelo após 4 segundos, amarelo para vermelho após 2 segundos, e então o ciclo se reinicia com o vermelho.
    
  - **Pedestre (Semáforo 2)**: Verde para amarelo após 2 segundos, amarelo para vermelho após 2 segundos, vermelho para verde baseado na detecção de presença.


## Função update() (Lógica de Decisão e Prioridade) 

```
void update() {
  int tempoAtual = millis();
  int leituraLDR = analogRead(ldr);
  long distanciaCM = readDistance();
  
  // 1. Lógica do Modo Noturno (Prioridade Máxima)
  if (leituraLDR < LIMIAR_NOITE) {
    setTrafficLightYellowBlinking(tempoAtual);
    delay(10); 
    return; // Sai daqui! Nenhuma outra lógica de tráfego é executada se for noite.
  }

  // 2. Lógica Normal (Dia) - INTEGRAÇÃO COM SENSOR DE DISTÂNCIA
  bool deteccaoRua2 = (distanciaCM <= LIMIAR_DISTANCIA);
  
  // Transição 2: Verde1 (Avenida) -> Amarelo1 (Avenida)
  if (digitalRead(verde1Pin) == HIGH) {
    if (tempoAtual - ultimoTempo1 > 4000 || (tempoAtual - ultimoTempo1 > 2000 && deteccaoRua2)) { 
      // MUDANÇA: Passa para Amarelo se passou 4s OU se passou 2s E detectou na Rua (prioridade)
      digitalWrite(verde1Pin, LOW);
      digitalWrite(amarelo1Pin, HIGH);
      ultimoTempo1 = tempoAtual;
    }
  }

  // ... Transições 1 e 3 (padrão)

  // Transição 6: Vermelho2 (Rua) -> Verde2 (Rua)
  if (digitalRead(vermelho2Pin) == HIGH) { 
    if (digitalRead(vermelho1Pin) == HIGH) { 
      // SÓ ABRE se a Avenida estiver Vermelha E...
      if (tempoAtual - ultimoTempo2 >= 4000 || deteccaoRua2) {
        // ...passou 4s OU se houver detecção na Rua (prioridade)
        digitalWrite(vermelho2Pin, LOW);
        digitalWrite(verde2Pin, HIGH);
        ultimoTempo2 = tempoAtual;
      }
    }
  }
  // ... (continua com as outras transições de tempo fixo)
  delay(10); 
}
```
#### Explicação do Bloco 4:
- Leitura e return: A função lê os sensores. Se a condição noturna for atendida, ela chama a função de pisca e usa o comando return; para sair imediatamente, bloqueando a lógica de tráfego do dia.

- deteccaoRua2 (A Flag de Prioridade): Esta variável booleana rastreia a detecção do ultrassônico.

- Inteligência na Transição 2 (Avenida): O tempo de Verde da Avenida é reduzido (de 4s para 2s) se a deteccaoRua2 for verdadeira. Isso acelera o ciclo quando há tráfego esperando na via secundária.

- Inteligência na Transição 6 (Rua/Pedestre): O Verde da Rua agora só é acionado se houver intertravamento (Avenida Vermelha) e se o tempo tiver esgotado OU se houver detecção de veículo/pedestre. Isso evita que o semáforo da Rua abra desnecessariamente.

## 5. Bloco Principal do Arduino.

```
// Instância da classe 
Semaforo rodar(33, 26, 25, 21, 22, 23, 35); 

void setup() {
  rodar.begin();
}

void loop() {
  rodar.update();
}
```

#### Explicação do Bloco 5:
- Semaforo rodar(...): Cria o objeto (rodar) que representa todo o seu sistema. Os números entre parênteses são os pinos do seu microcontrolador (LEDs e LDR).

- setup(): É a função de configuração do Arduino. Chama rodar.begin(), que configura todos os pinos.

- loop(): É a função de execução contínua. Chama rodar.update() repetidamente, mantendo a lógica de leitura de sensores e controle de luzes em funcionamento constante.

- **Sensor de Luminosidade**: Para acionar o modo noturno (`setTrafficLightYellowBlinking()`), normalmente usado ao anoitecer.

- **Sensor Ultrassônico**: Para detectar presença na via secundária, ativando o semáforo de pedestres.

## 6. Controle de Estado dos Semáforos
- Os semáforos não ficam verdes ao mesmo tempo, garantindo descentralização do tráfego com as condições de detecção.
- Temporizações ajustáveis em variáveis constantes, facilitando testes e ajustes finais.

## 7. Implementação na Prática
- Você já tem uma classe `Semaforo` e uma instância `rodar`.
- Basta incluir a leitura do sensor ultrassônico na lógica do método `update()`, com uma condicional que ajusta os sinais de acordo com a distância detectada.
- Implementar o modo noturno acionando o `setTrafficLightYellowBlinking()` baseado na leitura do LDR.


