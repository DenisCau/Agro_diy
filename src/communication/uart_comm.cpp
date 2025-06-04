// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: communication/uart_comm.cpp
// Descrição: Implementação do módulo de comunicação UART com o ESP32 Secundário.
// Responsável por receber dados do Secundário, parsear e disponibilizar.
// ========================================================

// === Includes Necessários ===
#include "communication/uart_comm.h" // Inclui o header deste modulo
#include "main.h"       // Para handles globais (dataMutex, systemEvents2), defines de bits (BIT_NOVO_DADO_SECUNDARIO, BIT_ERRO_UART_SECUNDARIO)
#include "config.h"      // Para defines de pinos (PIN_UART_RX_SECUNDARIO, PIN_UART_TX_SECUNDARIO), UART_NUM, etc.

#include <Arduino.h>     // Para Serial.print, etc.
#include <HardwareSerial.h>  // Biblioteca para usar as UARTs do ESP32



// === Variáveis Globais para Dados Recebidos (DEFINICOES) ===
// DEFINICOES das variaveis declaradas como 'extern volatile' em uart_comm.h
// Estas variaveis serao atualizadas por esta tarefa e lidas por outras tarefas (ex: Calculos).
// Devem ser protegidas pelo dataMutex.

// Variaveis para armazenar os dados DECODIFICADOS recebidos do Secundario
volatile uint8_t received_mpu_status_byte = 0;
volatile float  received_analog_level_semente = 0.0;
volatile float  received_ultrasonic_level_inoculante_cm = 0.0;
volatile float  received_temp_inoculante_celsius = 0.0;
volatile uint64_t received_rtc_timestamp_seconds = 0;
volatile long  received_adubo_esq_pulses = 0;
volatile long  received_adubo_dir_pulses = 0;
volatile long  received_semente_esq_pulses = 0;
volatile long  received_semente_dir_pulses = 0;
// TODO: Adicionar definicoes para outras variaveis globais para dados futuros se necessario


// === Variáveis Internas do Módulo UART_Comm ===
// Objeto HardwareSerial para a UART selecionada
HardwareSerial* uart_port = NULL;

// Buffer para montar o pacote recebido byte a byte
uint8_t uart_rx_buffer[MAX_PACKET_SIZE];
size_t uart_rx_buffer_pos = 0;

// Estado da maquina de estados para montagem do pacote UART
typedef enum {
 UART_STATE_IDLE,       // Esperando pelo primeiro byte de inicio
 UART_STATE_WAITING_START2,  // Recebeu o primeiro byte, esperando o segundo
 UART_STATE_WAITING_TYPE,   // Recebeu bytes de inicio, esperando o tipo
 UART_STATE_WAITING_LENGTH,  // Recebeu tipo, esperando o comprimento
 UART_STATE_WAITING_PAYLOAD, // Recebeu comprimento, esperando o payload
 UART_STATE_WAITING_CHECKSUM // Recebeu payload, esperando o checksum
} uart_rx_state_t;

uart_rx_state_t current_uart_rx_state = UART_STATE_IDLE;

// Variaveis temporarias durante a montagem do pacote
uint8_t received_packet_type = PACKET_TYPE_NONE;
uint8_t received_payload_length = 0;
uint8_t calculated_checksum = 0;


// === Protótipos de Funções Internas ===
/**
 * @brief Calcula o checksum simples (XOR) de um buffer de dados.
 * @param data Ponteiro para o buffer.
 * @param len Tamanho do buffer.
 * @return O valor do checksum calculado.
 */
uint8_t calculateChecksum(const uint8_t* data, size_t len);

/**
 * @brief Tenta parsear um pacote de dados recebido via UART.
 * Assume o formato de payload para PACKET_TYPE_STATUS_DATA.
 * @param payload Ponteiro para o buffer com os dados do payload.
 * @param payload_len Tamanho dos dados no buffer (deve ser STATUS_DATA_PAYLOAD_SIZE para este tipo).
 * @return true se o parsing foi bem-sucedido e os dados sao validos, false caso contrario.
 */
bool parseStatusDataPayload(const uint8_t* payload, size_t payload_len);


// === Implementação das Funções do Módulo UART_Comm ===

// Inicializa a comunicação UART.
void uart_comm_init() {
  Serial.println("UART Comm: Initializing...");

  // Seleciona e configura a UART
  if (UART_NUM == 0) uart_port = &Serial;
  else if (UART_NUM == 1) uart_port = &Serial1;
  else if (UART_NUM == 2) uart_port = &Serial2;
  else {
    Serial.printf("UART Comm: ERROR - Invalid UART_NUM selected: %d\n", UART_NUM);
    // Sinaliza erro no SEGUNDO Event Group
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
    return; // Nao pode inicializar
  }

  // Configura os pinos e baud rate
  // Usa os defines de config.h
  if (uart_port != NULL) {
    // Nota: O HardwareSerial::begin() ja configura pinos e buffers.
    uart_port->begin(UART_BAUD_RATE, SERIAL_8N1, PIN_UART_RX_SECUNDARIO, PIN_UART_TX_SECUNDARIO, false, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE);
    Serial.printf("UART Comm: Initialized on pins RX:%d, TX:%d with baud rate %d\n", PIN_UART_RX_SECUNDARIO, PIN_UART_TX_SECUNDARIO, UART_BAUD_RATE);
    Serial.printf("UART Comm: RX buffer size %d, TX buffer size %d\n", UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE);
  } else {
    Serial.println("UART Comm: ERROR - UART port is NULL after selection.");
    // Sinaliza erro no SEGUNDO Event Group
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
  }

  // Limpa o buffer de recepcao e reseta o estado da maquina de estados
  if (uart_port != NULL) {
    while(uart_port->available()) {
      uart_port->read();
    }
  }
  uart_rx_buffer_pos = 0;
  current_uart_rx_state = UART_STATE_IDLE;

  Serial.println("UART Comm: Initialization complete.");
}

// Calcula o checksum simples (XOR) de um buffer de dados.
uint8_t calculateChecksum(const uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Tenta parsear um pacote de dados recebido via UART.
// Assume o formato de payload para PACKET_TYPE_STATUS_DATA.
bool parseStatusDataPayload(const uint8_t* payload, size_t payload_len) {
  // Verifica se o tamanho do payload corresponde ao esperado para este tipo de pacote
  if (payload_len != STATUS_DATA_PAYLOAD_SIZE) {
    Serial.printf("UART Comm: Error parsing status data - Invalid payload size (%zu bytes, expected %zu).\n", payload_len, STATUS_DATA_PAYLOAD_SIZE);
    // Sinaliza erro de parsing
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
    return false;
  }

  // TODO: Implementar validacao adicional dos dados recebidos se necessario

  // Copia os dados do payload para a estrutura temporaria
  status_data_payload_t received_data;
  memcpy(&received_data, payload, STATUS_DATA_PAYLOAD_SIZE);

  // Protege o acesso as variaveis globais antes de atualizar
  if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { // Timeout curto
    // Atualiza as variaveis globais com os dados recebidos
    received_mpu_status_byte = received_data.mpu_status_byte;
    received_analog_level_semente = received_data.analog_level_semente;
    received_ultrasonic_level_inoculante_cm = received_data.ultrasonic_level_inoculante_cm;
    received_temp_inoculante_celsius = received_data.temp_inoculante_celsius;
    received_rtc_timestamp_seconds = received_data.rtc_timestamp_seconds;
    received_adubo_esq_pulses = received_data.adubo_esq_pulses;
    received_adubo_dir_pulses = received_data.adubo_dir_pulses;
    received_semente_esq_pulses = received_data.semente_esq_pulses;
    received_semente_dir_pulses = received_data.semente_dir_pulses;
    // TODO: Atualizar outras variaveis globais se adicionadas ao payload

    xSemaphoreGive(dataMutex); // Libera o mutex

    Serial.println("UART Comm: Global sensor data updated from UART."); // Debug

    // Sinaliza que um novo dado foi recebido e processado com sucesso
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_NOVO_DADO_SECUNDARIO);

    return true; // Parsing e atualizacao bem-sucedidos
  } else {
    Serial.println("UART Comm: Warning - Could not get dataMutex to update global data."); // Debug
    // Sinaliza erro temporario (mutex nao disponivel)
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
    return false; // Falha ao obter mutex
  }
}

// Função para enviar um pacote pela UART (do Principal para o Secundario).
bool uartComm_sendPacket(packet_type_t type, const uint8_t* data, size_t data_size) {
  if (uart_port == NULL) {
    Serial.println("UART Comm: Error sending packet - UART not initialized.");
    return false;
  }
  if (data_size > MAX_PAYLOAD_SIZE) {
    Serial.printf("UART Comm: Error sending packet - Payload size (%zu) exceeds MAX_PAYLOAD_SIZE (%zu).\n", data_size, MAX_PAYLOAD_SIZE);
    return false;
  }

  // Monta o pacote na estrutura
  uart_packet_t packet_to_send;
  packet_to_send.start1 = PACKET_START_BYTE1;
  packet_to_send.start2 = PACKET_START_BYTE2;
  packet_to_send.type = (uint8_t)type;
  packet_to_send.length = (uint8_t)data_size; // Assume data_size cabe em uint8_t (max 255)

  // Copia o payload
  if (data != NULL && data_size > 0) {
    memcpy(packet_to_send.payload, data, data_size);
  } else {
    // Se nao ha payload, garantir que o campo length seja 0
    packet_to_send.length = 0;
  }

  // Calcula o checksum do header + payload
  calculated_checksum = calculateChecksum((uint8_t*)&packet_to_send, sizeof(uart_packet_t) - sizeof(uint8_t)); // XOR de tudo menos o checksum final
  packet_to_send.checksum = calculated_checksum;

  // Envia o pacote pela UART
  size_t bytes_written = uart_port->write((uint8_t*)&packet_to_send, sizeof(uart_packet_t));

  if (bytes_written == sizeof(uart_packet_t)) {
    // Serial.println("UART Comm: Packet sent successfully."); // Debug (pode ser verboso)
    return true;
  } else {
    Serial.printf("UART Comm: Error sending packet - Only %zu of %zu bytes written.\n", bytes_written, sizeof(uart_packet_t));
    // Sinaliza erro de envio UART
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
    return false;
  }
}


// === Implementação da Tarefa FreeRTOS UART Comm ===
// Esta tarefa fica em loop, lendo dados da UART, montando pacotes, parseando e atualizando variaveis globais.
void tarefaComunicacaoUART(void *pvParameters) {
  Serial.println("Task UART Comm: Started.");

  // A inicializacao (uart_comm_init) ja foi feita em setup()

  while(true) {
    // Verifica se ha dados disponiveis na UART
    if (uart_port != NULL && uart_port->available()) {
      // Le os dados byte a byte e alimenta a maquina de estados de recepcao
      while (uart_port->available()) {
        uint8_t byte_lido = uart_port->read();

        switch (current_uart_rx_state) {
          case UART_STATE_IDLE:
            if (byte_lido == PACKET_START_BYTE1) {
              uart_rx_buffer[0] = byte_lido;
              uart_rx_buffer_pos = 1;
              current_uart_rx_state = UART_STATE_WAITING_START2;
            }
            break;

          case UART_STATE_WAITING_START2:
            if (byte_lido == PACKET_START_BYTE2) {
              uart_rx_buffer[uart_rx_buffer_pos++] = byte_lido;
              current_uart_rx_state = UART_STATE_WAITING_TYPE;
            } else {
              // Byte invalido, volta para IDLE
              Serial.println("UART Comm: Invalid byte while waiting for start2."); // Debug
              current_uart_rx_state = UART_STATE_IDLE;
              uart_rx_buffer_pos = 0;
            }
            break;

          case UART_STATE_WAITING_TYPE:
            received_packet_type = byte_lido;
            uart_rx_buffer[uart_rx_buffer_pos++] = byte_lido;
            current_uart_rx_state = UART_STATE_WAITING_LENGTH;
            break;

          case UART_STATE_WAITING_LENGTH:
            received_payload_length = byte_lido;
            uart_rx_buffer[uart_rx_buffer_pos++] = byte_lido;

            // Verifica se o comprimento do payload eh valido
            if (received_payload_length > MAX_PAYLOAD_SIZE) {
              Serial.printf("UART Comm: Error - Received invalid payload length: %d\n", received_payload_length);
              // Sinaliza erro e reseta a maquina de estados
              if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
              current_uart_rx_state = UART_STATE_IDLE;
              uart_rx_buffer_pos = 0;
            } else if (received_payload_length == 0) {
              // Pacote sem payload, proximo byte eh o checksum
              current_uart_rx_state = UART_STATE_WAITING_CHECKSUM;
            } else {
              // Esperando os bytes do payload
              current_uart_rx_state = UART_STATE_WAITING_PAYLOAD;
            }
            break;

          case UART_STATE_WAITING_PAYLOAD:
            uart_rx_buffer[uart_rx_buffer_pos++] = byte_lido;
            // Verifica se recebeu todos os bytes do payload
            if (uart_rx_buffer_pos == (sizeof(uart_packet_t) - sizeof(uint8_t) - MAX_PAYLOAD_SIZE + received_payload_length)) {
              current_uart_rx_state = UART_STATE_WAITING_CHECKSUM;
            }
            // Verifica se o buffer de recepcao estourou (deveria ser evitado pelo check de length, mas eh uma seguranca)
            if (uart_rx_buffer_pos >= MAX_PACKET_SIZE) {
              Serial.println("UART Comm: Error - RX buffer overflow during payload reception.");
              if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
              current_uart_rx_state = UART_STATE_IDLE; // Reseta a maquina de estados
              uart_rx_buffer_pos = 0;
            }
            break;

          case UART_STATE_WAITING_CHECKSUM: { // Usando chaves para escopo
            uint8_t received_checksum = byte_lido;
            uart_rx_buffer[uart_rx_buffer_pos++] = byte_lido; // Adiciona o checksum ao buffer para calculo

            // Calcula o checksum dos bytes recebidos (incluindo o checksum recebido)
            // Se o checksum calculado for 0, significa que o checksum recebido estava correto
            uint8_t final_checksum_check = calculateChecksum(uart_rx_buffer, uart_rx_buffer_pos);

            if (final_checksum_check == 0) {
              // Checksum OK, pacote completo e valido
              // Processa o pacote com base no tipo
              Serial.printf("UART Comm: Received valid packet type %d, length %d.\n", received_packet_type, received_payload_length); // Debug

              switch (received_packet_type) {
                case PACKET_TYPE_STATUS_DATA:
                  // Parseia o payload dos dados de status
                  parseStatusDataPayload(uart_rx_buffer + sizeof(uart_packet_t) - MAX_PAYLOAD_SIZE, received_payload_length);
                  break;
                case PACKET_TYPE_COMMAND_PRINCIPAL:
                  // TODO: Processar comandos recebidos do Principal (se houver)
                  Serial.println("UART Comm: Received COMMAND_PRINCIPAL packet (processing TODO).");
                  break;
                case PACKET_TYPE_ACK:
                  // TODO: Processar pacotes ACK recebidos (se o Principal enviar ACK)
                  Serial.println("UART Comm: Received ACK packet (processing TODO).");
                  break;
                case PACKET_TYPE_ERROR:
                  // TODO: Processar pacotes ERROR recebidos
                  Serial.println("UART Comm: Received ERROR packet (processing TODO).");
                  break;
                default:
                  Serial.printf("UART Comm: Received packet with unknown type: %d\n", received_packet_type); // Debug
                  // Sinaliza erro de protocolo
                  if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
                  break;
              }

            } else {
              // Checksum Invalido
              Serial.printf("UART Comm: Checksum mismatch. Received: %02X, Calculated: %02X\n", received_checksum, received_checksum ^ final_checksum_check); // Debug
              // Sinaliza erro de checksum
              if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_UART_SECUNDARIO);
            }

            // Pacote processado (valido ou invalido), resetar a maquina de estados para o proximo pacote
            current_uart_rx_state = UART_STATE_IDLE;
            uart_rx_buffer_pos = 0;
            break;
          } // Fim do case UART_STATE_WAITING_CHECKSUM

          default:
            // Estado desconhecido, resetar para IDLE
            Serial.println("UART Comm: Entered unknown RX state. Resetting."); // Debug
            current_uart_rx_state = UART_STATE_IDLE;
            uart_rx_buffer_pos = 0;
            break;
        } // Fim do switch (current_uart_rx_state)

      } // Fim do while (uart_port->available())

    } // Fim do if (uart_port != NULL && uart_port->available())


    // Pequeno delay para nao ocupar o CPU desnecessariamente quando nao ha dados
    vTaskDelay(pdMS_TO_TICKS(5)); // Delay curto para reagir rapido a dados UART
  } // Fim do while(true)

  vTaskDelete(NULL); // Se a tarefa algum dia sair do loop
}
