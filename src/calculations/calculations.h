// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: calculations/calculations.h
// Descrição: Definições e protótipos para o módulo de Cálculos.
// Processa dados de sensores e outros inputs para calcular métricas da plantadeira.
// ========================================================

#ifndef CALCULATIONS_H
#define CALCULATIONS_H

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Necessario para TaskFunction_t
#include "freertos/semphr.h" // Necessario para SemaphoreHandle_t
#include "freertos/event_groups.h" // Necessario para EventGroupHandle_t

#include <Arduino.h> // Para tipos basicos (uint32_t, float), millis()
#include "main.h"    // Para handles globais (dataMutex, configMutex, systemEvents, systemEvents2)
#include "config.h"  // Para defines de pinos, constantes de calibracao padrao, NUM_LINHAS, etc.
#include "sensors/pulse_sensors.h" // Para usar getAndResetPulseCount e defines SENSOR_TYPE_...


// === Variáveis Globais Calculadas (Declaradas como extern volatile) ===
// Estas variaveis sao atualizadas pela tarefaCalculos e lidas por outras tarefas (UI, Comunicacao).
// Devem ser declaradas como 'volatile' pois podem ser modificadas por uma tarefa e lidas por outra.
// Proteja o acesso a estas variaveis com o 'dataMutex'.
extern volatile float velocidade_km_h;     // Velocidade atual em km/h
extern volatile float media_sementes_por_metro;  // SPM media calculada em todas as linhas
extern volatile float sementes_por_metro_linha[NUM_LINHAS]; // SPM calculada por linha individual
extern volatile float vazao_sementes_por_hectare;  // Vazao de sementes em sementes/hectare
extern volatile float vazao_inoculante_por_hectare;  // Vazao de inoculante em Litros/hectare
extern volatile float distancia_percorrida_m;   // Distancia total percorrida em metros
extern volatile float area_plantada_ha;     // Area total plantada em hectares
extern volatile bool plantadeira_ativa;     // Flag que indica se a plantadeira esta em operacao (velocidade > 0)


// === Protótipos das Funções do Módulo de Cálculos ===
// Nenhuma função pública necessária no momento, toda a lógica está na tarefa.

// === Protótipo da Tarefa FreeRTOS do Módulo de Cálculos ===
// Esta tarefa roda em loop, lendo os contadores de pulso, realizando os calculos,
// verificando alarmes e atualizando as variaveis globais calculadas.
extern "C" { // Adicionado extern "C" para compatibilidade com C/FreeRTOS
    void tarefaCalculos(void *pvParameters);
}


#endif // CALCULATIONS_H