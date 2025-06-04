// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: sensors/pulse_sensors.h
// Descrição: Definições e protótipos para o módulo de sensores de pulso.
// Gerencia a leitura dos pulsos dos sensores (roda, semente, etc.).
// Utiliza ISRs (Interrupt Service Routines) para contagem de pulsos.
// ========================================================

#ifndef PULSE_SENSORS_H
#define PULSE_SENSORS_H

// === Includes Necessários ===
#include <Arduino.h>   // Para IRAM_ATTR (definido pelo core do ESP32)
#include <atomic>    // Para std::atomic (thread-safe counters)
#include "config.h"   // Para defines de pinos, NUM_LINHAS, SENSOR_TYPE_...
#include "main.h"   // Inclui main.h para Event Groups (usado em ISR se sinalizar erro)


// === Variáveis Globais (Contadores de Pulso) ===
// Contadores de pulso para cada sensor. Usamos std::atomic para garantir
// operacoes thread-safe ao serem incrementados pelas ISRs e lidos por tarefas.
// Declarados como 'extern' aqui, definidos em sensors/pulse_sensors.cpp

#define MAX_PULSE_COUNTERS (NUM_LINHAS + 3) // Roda, Inoculante, Acionamento + Sementes por linha
extern std::atomic<uint32_t> pulseCounters[MAX_PULSE_COUNTERS];


// === Protótipos das Funções de ISR (Interrupt Service Routine) ===
// Estas funções são as rotinas de serviço de interrupção acionadas pelos sensores.
// Precisam do atributo IRAM_ATTR para serem colocadas na IRAM e rodarem de forma confiável.
// Precisam de extern "C" para serem anexadas usando a API C-style attachInterrupt do Arduino/ESP-IDF.

extern "C" { // Adicionado extern "C" para as declarações das ISRs
  IRAM_ATTR void isr_roda();
  // ISR generica que recebe o numero da linha como argumento
  IRAM_ATTR void isr_semente_linha(void* arg);
  IRAM_ATTR void isr_inoculante();
  IRAM_ATTR void isr_acionamento();
}


// === Protótipos das Funções do Módulo de Sensores de Pulso ===

/**
 * @brief Função de inicialização do módulo (configura pinos, anexa ISRs).
 */
void pulseSensors_init();

/**
 * @brief Funcao para obter e resetar o contador de pulsos de um sensor especifico.
 * Deve ser chamada por tarefas para ler os pulsos acumulados desde a ultima leitura.
 * @param sensor_type O tipo do sensor (SENSOR_TYPE_... de config.h).
 * @return O numero de pulsos acumulados desde a ultima chamada.
 */
uint32_t getAndResetPulseCount(int sensor_type);


// A tarefa tarefaLeituraProcessamentoSensoresPulso foi removida,
// pois a tarefa de Calculos le os pulsos diretamente.
// extern "C" { void tarefaLeituraProcessamentoSensoresPulso(void *pvParameters); }


#endif // PULSE_SENSORS_H