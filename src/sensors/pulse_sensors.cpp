// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: sensors/pulse_sensors.cpp
// Descrição: Implementação do módulo de sensores de pulso.
// Gerencia a leitura dos pulsos dos sensores (roda, semente, etc.).
// Utiliza ISRs (Interrupt Service Routines) para contagem de pulsos.
// ========================================================

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "sensors/pulse_sensors.h" // Inclui o header deste modulo
#include "config.h"    // Para defines de pinos (PIN_SENSOR_RODA, etc.) e NUM_LINHAS
#include "main.h"   // Para SystemEventBits2_t e systemEvents2 (sinalizar erros) - Corrigida sintaxe do include

#include <Arduino.h>   // Para Serial.print, pinMode, attachInterrupt, digitalPinToInterrupt
#include <atomic>    // Para std::atomic
#include "soc/soc.h"   // Necessario para IRAM_ATTR (colocar ISRs na RAM) - Corrigida sintaxe do include
#include "soc/gpio_struct.h" // Necessario para manipulacao direta de GPIO em ISRs (se usado, geralmente evitado)
#include "esp_timer.h"  // Para esp_timer_get_time() se precisar de timestamps em ISRs

// === Variáveis Globais (Contadores de Pulso) ===
// DEFINICOES das variaveis globais declaradas como 'extern' em sensors/pulse_sensors.h

// Array para contagem temporária de pulsos na ISR (um contador atômico para cada sensor de pulso)
// O tamanho maximo deve acomodar todos os sensores de pulso gerenciados pelo Principal.
// Sementes (NUM_LINHAS) + Roda + Inoculante + Acionamento
// NUM_LINHAS (ex: 11) + 1 + 1 + 1 = 14. Usamos o define MAX_PULSE_COUNTERS.
// Declaradas como std::atomic<uint32_t> para garantir operacoes atomicas thread-safe em C++.
// Inicializadas com 0 na funcao pulseSensors_init() para evitar o erro de construtor deletado.
std::atomic<uint32_t> pulseCounters[MAX_PULSE_COUNTERS]; // Removida inicializacao imediata {0}

// Variaveis para mapear o tipo de sensor para o indice no array pulseCounters
#define PULSE_COUNTER_INDEX_RODA     0
#define PULSE_COUNTER_INDEX_INOCULANTE  1
#define PULSE_COUNTER_INDEX_ACIONAMENTO  2
#define PULSE_COUNTER_INDEX_SEMENTE_BASE 3 // Indice base para os sensores de semente por linha


// === Implementação das Funções de ISR (Interrupt Service Routine) ===
// Devem ter o atributo IRAM_ATTR para rodar de forma confiavel durante operacoes de flash/WiFi
// Nao precisam de extern "C" aqui nas DEFINICOES se o header as declara com extern "C".

// ISR para o sensor da roda
IRAM_ATTR void isr_roda() { // Removido extern "C"
  pulseCounters[PULSE_COUNTER_INDEX_RODA].fetch_add(1, std::memory_order_relaxed); // Incrementa o contador atomicamente
  // esp_timer_get_time() pode ser usado aqui se precisar registrar o timestamp
}

// ISR genérica para os sensores de semente por linha
// O argumento 'arg' contem o indice da linha (0 a NUM_LINHAS-1)
IRAM_ATTR void isr_semente_linha(void* arg) { // Removido extern "C"
  int linha_index = (int)arg;
  if (linha_index >= 0 && linha_index < NUM_LINHAS) {
    pulseCounters[PULSE_COUNTER_INDEX_SEMENTE_BASE + linha_index].fetch_add(1, std::memory_order_relaxed);
    // esp_timer_get_time() pode ser usado aqui
  } else {
    // Tratar erro: indice de linha invalido na ISR
    // Nao faca Serial.print() ou outras operacoes lentas/bloqueantes dentro da ISR!
    // Sinalizar um Event Group (se houver um) pode ser uma opcao para que uma tarefa
    // trate o erro fora da ISR.
    // if (systemEvents2 != NULL) xEventGroupSetBitsFromISR(systemEvents2, BIT_ERRO_SENSOR_SEMENTE_ISR, NULL); // Exemplo
  }
}

// ISR para o sensor de inoculante
IRAM_ATTR void isr_inoculante() { // Removido extern "C"
  pulseCounters[PULSE_COUNTER_INDEX_INOCULANTE].fetch_add(1, std::memory_order_relaxed);
  // esp_timer_get_time() pode ser usado aqui
}

// ISR para o sensor de acionamento
IRAM_ATTR void isr_acionamento() { // Removido extern "C"
  pulseCounters[PULSE_COUNTER_INDEX_ACIONAMENTO].fetch_add(1, std::memory_order_relaxed);
  // esp_timer_get_time() pode ser usado aqui
}

// A ISR de Rotação foi removida do ESP32 Principal.


// === Implementação das Funções do Módulo de Sensores de Pulso ===

/**
 * @brief Função de inicialização do módulo (configura pinos, anexa ISRs).
 */
void pulseSensors_init() {
  Serial.println("Module Pulse_Sensors: Initializing.");

  // Inicializa todos os contadores com zero antes de configurar ISRs
  for(int i=0; i < MAX_PULSE_COUNTERS; ++i) {
    pulseCounters[i].store(0, std::memory_order_relaxed);
  }


  // Configura o pino do sensor da roda e anexa a ISR
#ifdef PIN_SENSOR_RODA // Verifica se o pino esta definido em config.h
  pinMode(PIN_SENSOR_RODA, INPUT_PULLUP); // Configura o pino (ajuste PULLUP/DOWN/INPUT)
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_RODA), isr_roda, RISING); // Anexa a ISR (ajuste RISING/FALLING/CHANGE)
  Serial.printf("Pulse_Sensors: Roda sensor on pin %d initialized.\n", PIN_SENSOR_RODA);
#else
  Serial.println("Pulse_Sensors: PIN_SENSOR_RODA not defined.");
#endif

  // Configura os pinos dos sensores de semente por linha e anexa a ISR
  // Usamos a mesma ISR 'isr_semente_linha' para todas as linhas, passando o indice da linha como argumento.
  Serial.printf("Pulse_Sensors: Initializing %d semente sensors...\n", NUM_LINHAS);
  for (int i = 0; i < NUM_LINHAS; i++) {
#ifdef PIN_SENSOR_SEMENTE // Verifica se o array de pinos esta definido em config.h
    // Verifica se o pino eh valido antes de configurar e anexar ISR
    if (PIN_SENSOR_SEMENTE[i] != -1) { // Use -1 ou outro valor invalido em config.h para pinos nao usados
      // Corrigido typo no nome do array de pinos: PIN_SENSOR_SEMENTE
      pinMode(PIN_SENSOR_SEMENTE[i], INPUT_PULLUP); // Configura o pino (ajuste PULLUP/DOWN/INPUT)
      // attachInterrupt com argumento precisa de um wrapper ou a API do ESP-IDF gpio_isr_handler_add
      // Usando a API do Arduino com argumento (pode variar entre versoes/plataformas)
      // attachInterruptArg(digitalPinToInterrupt(PIN_SENSOR_SEMENTE[i]), isr_semente_linha, (void*)i, RISING); // Exemplo com attachInterruptArg
      // Nota: attachInterruptArg pode nao estar disponivel em todas as versoes do core.
      // Se attachInterruptArg nao compilar, usar gpio_isr_handler_add do ESP-IDF diretamente.
      // Por simplicidade, vamos usar attachInterrupt(pin, handler, mode) sem arg por enquanto e tratar a linha na ISR se o pino for lido,
      // MAS isso exige ler o pino DENTRO da ISR, o que eh mais lento.
      // A abordagem correta com argumento eh preferivel. Vamos ASSUMIR attachInterruptArg existe.
      // Se attachInterruptArg nao compilar, precisaremos migrar para gpio_isr_handler_add.
      attachInterruptArg(digitalPinToInterrupt(PIN_SENSOR_SEMENTE[i]), isr_semente_linha, (void*)i, RISING); // Usando attachInterruptArg
      Serial.printf("Pulse_Sensors: Semente sensor linha %d on pin %d initialized.\n", i, PIN_SENSOR_SEMENTE[i]);
    } else {
      Serial.printf("Pulse_Sensors: Semente sensor linha %d on pin -1 (skipped).\n", i);
    }
#else
    Serial.println("Pulse_Sensors: PIN_SENSOR_SEMENTE array not defined.");
    break; // Sai do loop se o array de pinos nao esta definido
#endif
  }


  // Configura o pino do sensor de inoculante e anexa a ISR
#ifdef PIN_SENSOR_INOCULANTE // Verifica se o pino esta definido
  pinMode(PIN_SENSOR_INOCULANTE, INPUT_PULLUP); // Configura o pino (ajuste PULLUP/DOWN/INPUT)
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_INOCULANTE), isr_inoculante, RISING); // Anexa a ISR (ajuste)
  Serial.printf("Pulse_Sensors: Inoculante sensor on pin %d initialized.\n", PIN_SENSOR_INOCULANTE);
#else
  Serial.println("Pulse_Sensors: PIN_SENSOR_INOCULANTE not defined.");
#endif

  // Configura o pino do sensor de acionamento e anexa a ISR
#ifdef PIN_SENSOR_ACIONAMENTO // Verifica se o pino esta definido
  pinMode(PIN_SENSOR_ACIONAMENTO, INPUT_PULLUP); // Configura o pino (ajuste)
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_ACIONAMENTO), isr_acionamento, CHANGE); // Anexa a ISR (CHANGE para detectar subida e descida)
  Serial.printf("Pulse_Sensors: Acionamento sensor on pin %d initialized.\n", PIN_SENSOR_ACIONAMENTO);
#else
  Serial.println("Pulse_Sensors: PIN_SENSOR_ACIONAMENTO not defined.");
#endif


  Serial.println("Module Pulse_Sensors: Initialization complete.");
}

/**
 * @brief Funcao para obter e resetar o contador de pulsos de um sensor especifico.
 * Deve ser chamada por tarefas para ler os pulsos acumulados desde a ultima leitura.
 * @param sensor_type O tipo do sensor (SENSOR_TYPE_... de config.h).
 * @return O numero de pulsos acumulados desde a ultima chamada.
 */
uint32_t getAndResetPulseCount(int sensor_type) {
  int counter_index = -1;

  // Mapeia o tipo de sensor para o indice no array pulseCounters
  switch (sensor_type) {
    case SENSOR_TYPE_RODA:
      counter_index = PULSE_COUNTER_INDEX_RODA;
      break;
    case SENSOR_TYPE_INOCULANTE:
      counter_index = PULSE_COUNTER_INDEX_INOCULANTE;
      break;
    case SENSOR_TYPE_ACIONAMENTO:
      counter_index = PULSE_COUNTER_INDEX_ACIONAMENTO;
      break;
    default:
      // Verifica se eh um sensor de semente por linha
      if (sensor_type >= SENSOR_TYPE_SEMENTE_BASE && sensor_type < SENSOR_TYPE_SEMENTE_BASE + NUM_LINHAS) {
        counter_index = PULSE_COUNTER_INDEX_SEMENTE_BASE + (sensor_type - SENSOR_TYPE_SEMENTE_BASE);
      } else {
        // Tipo de sensor invalido
        Serial.printf("Pulse_Sensors: WARNING - Invalid sensor type requested: %d\n", sensor_type);
        return 0; // Retorna 0 para tipo invalido
      }
      break;
  }

  // Obtem o valor atual do contador e reseta-o para zero atomicamente
  // exchange(0) retorna o valor antigo ANTES de setar para 0
  return pulseCounters[counter_index].exchange(0, std::memory_order_relaxed);
}

// A tarefa tarefaLeituraProcessamentoSensoresPulso foi removida,
// pois a tarefa de Calculos le os pulsos diretamente.
// void tarefaLeituraProcessamentoSensoresPulso(void *pvParameters);