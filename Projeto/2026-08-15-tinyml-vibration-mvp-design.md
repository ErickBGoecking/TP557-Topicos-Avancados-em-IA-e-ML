# Design do MVP — Detecção de Anomalias por Vibração com TinyML

## Objetivo
Construir uma bancada mínima e reproduzível para adquirir vibração de uma pequena máquina rotativa usando ESP32 DevKit e MPU6050, gerar dados rotulados de condição normal e condição anômala controlada, e validar que os sinais são separáveis antes de treinar qualquer modelo TinyML.

## Escopo da Fase 1
A Fase 1 inclui montagem elétrica/mecânica, validação do MPU6050, aquisição estável a 100 Hz, geração de CSV e comparação básica entre duas condições. Não inclui TensorFlow Lite Micro, autoencoder, FFT embarcada, MQTT, dashboard ou inferência no ESP32.

## Hardware-base
- ESP32 DevKit V1 (ou equivalente com I2C nos GPIO 21/22)
- Módulo MPU6050/GY-521
- Cooler ou pequeno motor DC de baixa tensão
- Fonte separada adequada ao motor/cooler
- Cabo USB para o ESP32
- Protoboard e jumpers
- Fita dupla face forte ou fixação rígida para o MPU6050

## Alimentação e segurança
O motor/cooler não deve ser alimentado diretamente pelo ESP32. Na Fase 1, recomenda-se alimentação separada para o motor e USB para o ESP32. O sensor deve ser fixado rigidamente à carcaça/suporte da máquina. Não usar massas soltas em partes girantes. A condição anômala deve ser criada por alteração de fixação, velocidade controlada ou carga externa segura.

## Ligações do MPU6050
- VCC do GY-521 -> 3V3 do ESP32
- GND -> GND do ESP32
- SDA -> GPIO 21
- SCL -> GPIO 22
- AD0 -> GND ou deixado no estado padrão do módulo, resultando normalmente no endereço 0x68
- INT não é necessário na Fase 1

## Arquitetura de dados
Máquina rotativa -> vibração -> MPU6050 -> I2C -> ESP32 -> Serial USB -> computador -> CSV.

O ESP32 coletará ax, ay e az a 100 Hz. Cada linha enviada terá timestamp e três eixos. O computador será responsável por salvar os dados.

## Condições experimentais
### C0 — Normal
Equipamento na fixação nominal e em velocidade estável.

### C1 — Anomalia controlada
Preferência: alterar a rigidez/fixação do conjunto de forma segura e reprodutível. Alternativas: usar uma segunda velocidade fixa ou introduzir carga externa controlada, sem colocar objetos soltos em partes rotativas.

## Protocolo de coleta inicial
1. Aquecer/estabilizar o motor por aproximadamente 30 s antes da sessão.
2. Descartar os primeiros 2 s de leitura após iniciar cada coleta.
3. Registrar pelo menos 60 s por condição na primeira validação.
4. Repetir cada condição em 3 sessões independentes para verificar reprodutibilidade.
5. Não misturar C0 e C1 no mesmo arquivo bruto.

## Formato esperado do CSV
Campos mínimos:
- timestamp_ms
- ax_g
- ay_g
- az_g
- label
- session_id

## Critérios de sucesso da Fase 1
- MPU6050 detectado de forma consistente no endereço esperado.
- Taxa média de amostragem próxima de 100 Hz, com jitter pequeno o suficiente para não comprometer as janelas.
- Dados sem saturação evidente do acelerômetro.
- C0 apresenta comportamento relativamente repetível entre sessões.
- C1 produz diferença mensurável em pelo menos uma característica simples, como RMS, desvio padrão, pico-a-pico ou magnitude.
- Os dados podem ser salvos em CSV e reproduzidos para análise.

## Sequência de testes prevista
1. Scanner I2C e confirmação do endereço do MPU6050.
2. Leitura estática dos eixos e verificação aproximada da gravidade.
3. Leitura contínua com motor desligado e ligado.
4. Aquisição temporizada a 100 Hz e medição da taxa real.
5. Coleta C0.
6. Coleta C1.
7. Script no computador para salvar serial em CSV.
8. Script de análise para calcular magnitude, média, RMS, desvio padrão e pico-a-pico por janela.
9. Critério go/no-go para avançar ao treinamento.

## Decisões deliberadamente adiadas
- FFT e análise espectral
- autoencoder
- comparação entre vários modelos
- quantização INT8
- TFLite Micro no ESP32
- Wi-Fi/MQTT/dashboard

## Resultado esperado ao final da Fase 1
Uma bancada capaz de gerar um dataset confiável e reprodutível com duas condições distintas, pronto para janelamento e treinamento do primeiro modelo de Machine Learning.
