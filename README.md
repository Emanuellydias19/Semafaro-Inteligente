# Semáforo Inteligente - Arduino

Este repositório documenta o desenvolvimento de um semáforo inteligente utilizando Arduino, com sensores LDR para detectar a presença de veículos e com modo noturno automático. O sistema ajusta o fluxo de veículos conforme a luminosidade, permitindo uma operação eficiente e adaptativa no controle do tráfego.

## Etapas Realizadas

1. Montagem de dois semáforos físicos com LEDs para sinalização padrão.
2. Integração do sensor de luz (LDR) para detectar variações e presença de veículos.
3. Programação da lógica para adaptação automática ao modo noturno conforme a luminosidade.
4. Desenvolvimento da interface online para controle e visualização dos dados captados.
5. Testes de funcionalidade e validação do sistema integrado.

## Montagem do Circuito

#### Componentes:

- Arduino Uno ou similar.
- Protoboard.
- LEDs vermelho, amarelo e verde para cada semáforo.
- Sensor LDR (Light Dependent Resistor).
- Resistores para proteção dos LEDs e estabilização do sensor.
- Fios jumper para conexões.

#### Conexões:

- LEDs conectados aos pinos digitais, com resistores adequados.
- Sensor LDR conectado a um pino analógico para leituras de luminosidade.
- Alimentação e GND conforme padrão Arduino.

## Interface Online

- Interface simples para ajustar o comportamento dos semáforos e ativar o modo noturno.
- Visualização em tempo real dos dados captados pelo sensor LDR.
- Link e código da interface deverão ser disponibilizados no repositório.

## Evidências

- Vídeo demonstrando o funcionamento do semáforo inteligente.
- Capturas de tela da montagem no ambiente físico ou simulado.
- Código fonte completo disponível no repositório.
