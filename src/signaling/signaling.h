// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: signaling/signaling.h
// Descrição: Header do modulo de Sinalização.
// Declara funcoes de inicializacao e a tarefa FreeRTOS de sinalizacao.
// ========================================================

#ifndef SIGNALING_H
#define SIGNALING_H

// Inclui headers necessarios se funcoes declaradas aqui os utilizam em seus parametros, etc.
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Necessario para TaskFunction_t
#include "freertos/semphr.h" // Necessario para SemaphoreHandle_t
#include "freertos/event_groups.h" // Necessario para EventGroupHandle_t


#include "main.h" // Para SystemEventBits_t, SystemEventBits2_t, systemEvents, systemEvents2
#include "config.h" // Para NUM_LINHAS e definicoes de pinos de sinalizacao (se movidos para ca ou incluidos aqui)


// === Declarações de Funções ===

/**
 * @brief Inicializa os pinos e perifericos relacionados a sinalizacao (LEDs, Buzzer, etc.).
 */
void signaling_init();


// === Declaração da Tarefa FreeRTOS ===

/**
 * @brief Tarefa responsavel por monitorar os Event Groups do sistema
 * e acionar as sinalizacoes visuais e sonoras correspondentes.
 * @param pvParameters Parâmetros da tarefa (não utilizado).
 */
extern "C" { // Adicionado extern "C" para compatibilidade com C/FreeRTOS
    void tarefaSinalizacao(void *pvParameters);
}


#endif // SIGNALING_H