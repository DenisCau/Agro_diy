// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: communication/uart_comm.h
// Descrição: Definições e protótipos para a comunicação UART com o ESP32 Secundário.
// ========================================================

#ifndef UART_COMM_H
#define UART_COMM_H

// === Includes Necessários ===
// Inclui headers do FreeRTOS para a declaracao da tarefa


// Inclui headers do Arduino para tipos como HardwareSerial
#include <Arduino.h>

// Inclui o arquivo de configuracao para pinos e baud rate
#include "config.h"

// Inclui o arquivo principal para Event Groups e Handles globais
#include "main.h"


// === Constantes de Configuração da UART ===
// Defina qual UART sera usada (UART0, UART1, UART2)
// UART0 eh geralmente usada para o Serial Monitor USB
// UART1 e UART2 estao disponiveis para uso geral
#define UART_NUM          2    // Usando UART2 (ajuste se necessario)
#define UART_BAUD_RATE     115200 // Baud rate (deve ser o mesmo no ESP32 Secundario)
// Tamanho do buffer de recepcao em bytes (ajuste conforme o tamanho maximo do pacote + folga)
// MAX_PACKET_SIZE (header + payload max + checksum) + folga
#define UART_RX_BUFFER_SIZE   512
// Tamanho do buffer de transmissao em bytes (se o Principal precisar enviar algo)
#define UART_TX_BUFFER_SIZE  255


// === Definições do Protocolo UART ===

// Bytes de início do pacote para sincronização
#define PACKET_START_BYTE1 0xAA
#define PACKET_START_BYTE2 0x55

// Definição dos tipos de pacotes que serao trocados
typedef enum {
 PACKET_TYPE_NONE = 0,      // Tipo invalido ou nao definido
 PACKET_TYPE_STATUS_DATA,   // Dados de status/sensores do Secundario para o Principal (MPU, niveis, temp, RTC, pulsos)
 PACKET_TYPE_COMMAND_PRINCIPAL, // Comando do Principal para o Secundario (ex: ligar/desligar sinalizador, pedir dados especificos)
 PACKET_TYPE_ACK,      // Pacote de reconhecimento (Acknowledgement)
 PACKET_TYPE_ERROR,     // Pacote indicando um erro
 // TODO: Adicionar mais tipos de pacote conforme a necessidade (ex: dados de calibracao)
} packet_type_t;

// Estrutura do Payload para PACKET_TYPE_STATUS_DATA
// Define a ordem e o tipo dos dados enviados pelo ESP32 Secundario
// Use __attribute__((packed)) para garantir que nao ha padding entre os campos
typedef struct __attribute__((packed)) {
 uint8_t mpu_status_byte;            // Status dos MPUs (1 byte, ex: bit 0 = MPU0 OK, bit 1 = MPU1 OK, etc.)
 float  analog_level_semente;        // Leitura do sensor de nivel analogico de semente
 float  ultrasonic_level_inoculante_cm;  // Leitura do sensor de nivel ultrasonico de inoculante (em cm)
 float  temp_inoculante_celsius;      // Leitura do sensor de temperatura do inoculante (em Celsius)
 uint64_t rtc_timestamp_seconds;       // Timestamp do RTC (segundos desde a época)
 long  adubo_esq_pulses;         // Pulsos do sensor de rotacao do eixo de adubo esquerdo
 long  adubo_dir_pulses;         // Pulsos do sensor de rotacao do eixo de adubo direito
 long  semente_esq_pulses;         // Pulsos do sensor de rotacao do eixo de semente esquerdo
 long  semente_dir_pulses;         // Pulsos do sensor de rotacao do eixo de semente direito
 // TODO: Adicionar outros dados brutos do Secundario se necessario
} status_data_payload_t;

// Tamanho do payload para o pacote de status/dados
#define STATUS_DATA_PAYLOAD_SIZE (sizeof(status_data_payload_t))

// Tamanho máximo do payload (deve ser >= STATUS_DATA_PAYLOAD_SIZE)
#define MAX_PAYLOAD_SIZE STATUS_DATA_PAYLOAD_SIZE // Definido como o tamanho do payload de status por enquanto

// Estrutura básica de um pacote UART
// Esta estrutura define o formato dos dados que serao enviados/recebidos pela UART
typedef struct __attribute__((packed)) { // Use __attribute__((packed)) para evitar padding e garantir tamanho exato
 uint8_t start1;    // Primeiro byte de inicio (0xAA)
 uint8_t start2;    // Segundo byte de inicio (0x55)
 uint8_t type;     // Tipo do pacote (enum packet_type_t)
 uint8_t length;    // Comprimento (em bytes) do payload
 uint8_t payload[MAX_PAYLOAD_SIZE]; // Dados do pacote (payload)
 uint8_t checksum;    // Checksum simples (XOR de todos os bytes anteriores)
} uart_packet_t;

// Tamanho total máximo de um pacote (header + payload max + checksum)
#define MAX_PACKET_SIZE (sizeof(uart_packet_t))


// === Variáveis Globais para Dados Recebidos (DECLARADAS como extern volatile) ===
// Estas variaveis serao atualizadas pela tarefa UART e lidas por outras tarefas (ex: Calculos).
// Devem ser protegidas pelo dataMutex.

// Variaveis para armazenar os dados DECODIFICADOS recebidos do Secundario
extern volatile uint8_t received_mpu_status_byte;
extern volatile float  received_analog_level_semente;
extern volatile float  received_ultrasonic_level_inoculante_cm;
extern volatile float  received_temp_inoculante_celsius;
extern volatile uint64_t received_rtc_timestamp_seconds;
extern volatile long  received_adubo_esq_pulses;
extern volatile long  received_adubo_dir_pulses;
extern volatile long  received_semente_esq_pulses;
extern volatile long  received_semente_dir_pulses;
// TODO: Adicionar outras variaveis globais para dados futuros se necessario


// === Protótipos das Funções do Módulo ===

/**
 * @brief Inicializa a porta UART para comunicação com o ESP32 Secundário.
 * Configura a interface UART, pinos e buffers.
 */
void uart_comm_init();

/**
 * @brief Função para enviar um pacote pela UART (do Principal para o Secundario).
 * Retorna true se o pacote foi enviado com sucesso, false caso contrario.
 * @param type Tipo do pacote a ser enviado.
 * @param data Ponteiro para o buffer de dados (payload).
 * @param data_size Tamanho dos dados no buffer.
 * @return true se enviado com sucesso, false se falhou (ex: fila cheia, UART nao inicializada).
 */
bool uartComm_sendPacket(packet_type_t type, const uint8_t* data, size_t data_size);


// === Protótipo da Tarefa FreeRTOS do Módulo ===

/**
 * @brief Tarefa FreeRTOS para gerenciar a comunicação UART.
 * Fica em loop lendo dados da UART, montando pacotes, parseando e atualizando variaveis globais.
 */
void tarefaComunicacaoUART(void *pvParameters);


#endif // UART_COMM_H
