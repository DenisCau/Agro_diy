// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: config.h
// Descrição: Definições de configuração de hardware, constantes e parâmetros padrão.
// ========================================================

#ifndef CONFIG_H
#define CONFIG_H

// === Configuração de Hardware ===
// Pinos dos sensores de pulso
#define PIN_SENSOR_RODA     34 // Pino para o sensor de pulso da roda (Velocidade/Distancia)
// Array de pinos para os sensores de semente por linha
// Exemplo para 11 linhas:
#define NUM_LINHAS       11
const int PIN_SENSOR_SEMENTE[NUM_LINHAS] = {2, 4, 16, 17, 5, 18, 19, 23, 25, 26, 32}; // Pinos para sensores de semente por linha
#define PIN_SENSOR_INOCULANTE  33 // Pino para o sensor de pulso do inoculante (Vazao)
#define PIN_SENSOR_ACIONAMENTO  35 // Pino para o sensor de acionamento da plantadeira (detecta se esta baixada/levantada) - REMOVIDO DE USADO POR ENQUANTO NO CALCULOS.CPP

// Pinos dos atuadores
#define PIN_BOMBA_PWM      2  // Pino para controle PWM da bomba de inoculante (ajuste) - REPETIDO? NAO USAR PINO 2. USAR OUTRO PINO.

// Pinos de sinalizacao (LEDs, Buzzer) - TODO: Definir pinos reais
#define PIN_LED_STATUS     15 // Pino para LED de status geral (Exemplo)
#define PIN_LED_ALARME_GERAL  16 // Pino para LED de alarme geral (Exemplo)
// #define PIN_BUZZER      XX // Pino para o Buzzer (Exemplo)

// Pinos dos botoes da UI - TODO: Definir pinos reais
#define PIN_BTN_UP       36 // Pino para botao UP (Exemplo)
#define PIN_BTN_DOWN      39 // Pino para botao DOWN (Exemplo)
#define PIN_BTN_SELECT     34 // Pino para botao SELECT (Exemplo) - NAO USAR PINO 34, CONFLITO COM SENSOR RODA
#define PIN_BTN_BACK      35 // Pino para botao BACK (Exemplo) - NAO USAR PINO 35, CONFLITO COM SENSOR ACIONAMENTO
// AJUSTE REAL DOS PINOS DIGITAL (GPIO) - ESP32-S3-DEVKITC-1 tem GPIOs 0-21, 38-48
// PIN_SENSOR_RODA    -> GPIO34 (Input)
// PIN_SENSOR_SEMENTE[11] -> GPIOs 2,4,5,16,17,18,19,23,25,26,32 (Input)
// PIN_SENSOR_INOCULANTE -> GPIO33 (Input)
// PIN_SENSOR_ACIONAMENTO -> GPIO35 (Input)
// PIN_BOMBA_PWM     -> GPIO2 (Output, PWM) - PINO 2 JA ESTA SENDO USADO, TROCAR! EX: GPIO17
// PIN_LED_STATUS    -> GPIO15 (Output)
// PIN_LED_ALARME_GERAL -> GPIO16 (Output)
// PIN_BTN_UP      -> GPIO36 (Input Pullup)
// PIN_BTN_DOWN     -> GPIO39 (Input Pullup)
// PIN_BTN_SELECT    -> GPIO14 (Input Pullup)
// PIN_BTN_BACK     -> GPIO27 (Input Pullup)
// ATUALIZANDO PINOS DE ACORDO COM A SUGESSTAO (EVITANDO CONFLITOS):
#undef PIN_BOMBA_PWM
#define PIN_BOMBA_PWM      17 // Exemplo: Pino para controle PWM da bomba de inoculante (Output)
#undef PIN_BTN_SELECT
#define PIN_BTN_SELECT     14 // Pino para botao SELECT (Input Pullup)
#undef PIN_BTN_BACK
#define PIN_BTN_BACK      27 // Pino para botao BACK (Input Pullup)

// === Constantes de Calibracao Padrao ===
// Estes sao valores padrao que serao usados se nao houver configuracao salva na NVS.
// Eles podem ser ajustados pela UI e salvos na NVS.
// As variaveis globais correspondentes estao declaradas em ui/ui_task.h
#define CAL_RODA_M_POR_PULSO_DEFAULT   0.5  // Metros percorridos por cada pulso do sensor da roda (diametro da roda * PI / pulsos por volta)
#define CAL_SEMENTE_S_POR_METRO_DEFAULT 0.0  // Sementes por metro esperado (apenas um valor de referencia, SPM real eh por linha)
#define CAL_INOCULANTE_L_POR_1000PULSOS_DEFAULT 10.0  // Litros de inoculante por 1000 pulsos do sensor (calibrar na prática)
#define LARGURA_PLANTADEIRA_METROS   3.0  // Largura total da plantadeira em metros (para calculo de area e vazao/ha)

// === Configuracoes Padrao ===
// Estes sao valores padrao para settings que serao usados se nao houver configuracao salva na NVS.
// Eles podem ser ajustados pela UI e salvos na NVS.
// As variaveis globais correspondentes estao declaradas em ui/ui_task.h
#define SETTING_ALARME_SPM_MIN_DEFAULT 4.0  // Limite inferior de Sementes por Metro para disparar alarme
#define SETTING_ALARME_SPM_MAX_DEFAULT 6.0  // Limite superior de Sementes por Metro para disparar alarme
#define SETTING_VAZAO_ALVO_LHA_DEFAULT 50.0  // Vazao alvo de inoculante em Litros por Hectare (para controle ou alarme)

// === Configuracao NVS (Non-Volatile Storage) ===
#define NVS_NAMESPACE "plantadeira" // Nome do namespace na NVS para as configuracoes do projeto. ADICIONADO.

// === Configuracao de Sensores ===
// Tipos de sensores de pulso gerenciados pelo ESP32 Principal
#define SENSOR_TYPE_RODA     0
#define SENSOR_TYPE_INOCULANTE  1
#define SENSOR_TYPE_ACIONAMENTO  2
// Os sensores de semente por linha terao tipos baseados no NUM_LINHAS
// SENSOR_TYPE_SEMENTE_BASE = 3
// SENSOR_TYPE_SEMENTE_LINHA_0 = SENSOR_TYPE_SEMENTE_BASE + 0
// SENSOR_TYPE_SEMENTE_LINHA_1 = SENSOR_TYPE_SEMENTE_BASE + 1, etc.
#define SENSOR_TYPE_SEMENTE_BASE 3

// === Configuracao de Comunicacao UART com ESP32 Secundario ===
#define UART_NUM_SECUNDARIO   UART_NUM_2 // Numero da UART a ser usada para comunicacao com o Secundario
#define PIN_UART_TX_SECUNDARIO 17    // Pino TX da UART (ajuste) - CONFLITO COM PIN_BOMBA_PWM, TROCAR! EX: GPIO43
#define PIN_UART_RX_SECUNDARIO 18    // Pino RX da UART (ajuste) - CONFLITO COM PINOS DE SEMENTE, TROCAR! EX: GPIO44
// ATUALIZANDO PINOS UART:
#undef PIN_UART_TX_SECUNDARIO
#define PIN_UART_TX_SECUNDARIO 43    // Pino TX da UART (ajuste)
#undef PIN_UART_RX_SECUNDARIO
#define PIN_UART_RX_SECUNDARIO 44    // Pino RX da UART (ajuste)

#define UART_BAUD_RATE_SECUNDARIO 115200 // Baud rate da comunicacao UART

// === Configuracao de Comunicacao RPi (WiFi/HTTP) ===
// TODO: Definir SSID e Password em config.h ou carregar da NVS na tarefa de comunicacao
// #define WIFI_SSID "SeuSSID"
// #define WIFI_PASSWORD "SuaSenha"

// Endereco base do servidor HTTP da Raspberry Pi
#define RPI_HTTP_SERVER_URL "http://192.168.1.100:5000" // <<<< AJUSTE CONFORME O IP E PORTA DA SUA RPi!

// === Configuracao Geral do Sistema ===
// Stack size padrao para tarefas FreeRTOS (em bytes) - Ajuste conforme a necessidade real
#define DEFAULT_TASK_STACK_SIZE  2048 // Tamanho padrao, algumas tarefas precisam de mais (UI, Comunicacao)

// Prioridade padrao para tarefas FreeRTOS (ajuste)
#define DEFAULT_TASK_PRIORITY   5 // Prioridade media

// Core padrao para tarefas (0 ou 1) - Core 0 geralmente eh mais livre, Core 1 roda Arduino loop
#define DEFAULT_TASK_COREID    0

// Macros para obter bits de evento por linha - REPETIDOS DO main.h, REMOVER DAQUI!
// #define GET_BIT_SEMENTES_LINHA_ABAIXO(i) (1 << ((i) * 2))
// #define GET_BIT_SEMENTES_LINHA_ACIMA(i) (1 << ((i) * 2 + 1))


#endif // CONFIG_H