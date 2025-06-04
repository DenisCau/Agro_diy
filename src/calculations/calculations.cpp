// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: calculations/calculations.cpp
// Descrição: Implementação do módulo de Cálculos.
// Processa dados de sensores e outros inputs para calcular métricas da plantadeira.
// Gerencia as variaveis globais calculadas e verifica condicoes de alarme.
// ========================================================

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include <Arduino.h>   // Para Serial.print, millis(), etc.
#include <ArduinoJson.h> // Para trabalhar com JSON
#include "esp_timer.h"  // Para esp_timer_get_time()

#include "calculations/calculations.h" // Inclui o header deste modulo
#include "main.h"    // Para handles globais (dataMutex, configMutex, systemEvents, systemEvents2), defines de bits
#include "config.h"   // Para defines de pinos, constantes de calibracao padrao, NUM_LINHAS, etc.
#include "sensors/pulse_sensors.h" // Para usar getAndResetPulseCount e defines SENSOR_TYPE_...
#include "control/pump_control.h" // Para usar getPumpPwmDutyCycle() // ADICIONADO
#include "communication/rpi_comm.h" // Para enviar dados para RPi
#include "ui/ui_task.h" // Para acessar variaveis de configuracao/calibracao (cal_roda_m_por_pulso, etc.) e saveParametersToNVS() - MOVIDO MAIS PARA CIMA

// === Variáveis Globais Calculadas (DEFINICOES) ===
// DEFINICOES das variaveis globais declaradas como 'extern volatile' em calculations.h

volatile float velocidade_km_h = 0.0;
volatile float media_sementes_por_metro = 0.0;
volatile float sementes_por_metro_linha[NUM_LINHAS] = {0.0}; // Inicializa todos os elementos com 0.0
volatile float vazao_sementes_por_hectare = 0.0;
volatile float vazao_inoculante_por_hectare = 0.0;
volatile float distancia_percorrida_m = 0.0;
volatile float area_plantada_ha = 0.0;
volatile bool plantadeira_ativa = false;


// === Constantes e Variáveis Internas da Tarefa ===
// Intervalos de calculo e leitura de sensores
const TickType_t CALCULATION_INTERVAL_MS = pdMS_TO_TICKS(200); // Intervalo de 200ms para os calculos

// Variáveis para cálculo de velocidade
volatile uint32_t last_roda_pulse_count = 0;
volatile uint64_t last_calculation_time_us = 0; // Tempo em microsegundos para maior precisao

// Variáveis para cálculo de SPM e Vazao
volatile uint32_t last_semente_pulse_count_linha[NUM_LINHAS] = {0};
volatile uint32_t last_inoculante_pulse_count = 0;


// === Implementação das Funções do Módulo de Cálculos ===

// Nenhuma funcao publica alem da tarefa e getters (se houver)

// === Implementação da Tarefa FreeRTOS do Módulo de Cálculos ===
// Removido extern "C" da DEFINICAO da tarefa. Ele so fica na DECLARACAO no header.
void tarefaCalculos(void *pvParameters) {
  Serial.println("Task Calculos: Started.");

  // Inicializa o tempo da ultima calculo
  last_calculation_time_us = esp_timer_get_time();


  while(true) {
    // --- 1. Obter Pulsos dos Sensores (desde a ultima leitura) ---
    // Usa a funcao getAndResetPulseCount do modulo pulse_sensors
    uint32_t roda_pulses = getAndResetPulseCount(SENSOR_TYPE_RODA);
    uint32_t inoculante_pulses = getAndResetPulseCount(SENSOR_TYPE_INOCULANTE);
    // Removido acionamento_pulses pois nao esta sendo usado e causa warning
    // uint32_t acionamento_pulses = getAndResetPulseCount(SENSOR_TYPE_ACIONAMENTO);

    uint32_t semente_pulses_linha[NUM_LINHAS];
    for (int i = 0; i < NUM_LINHAS; i++) {
      semente_pulses_linha[i] = getAndResetPulseCount(SENSOR_TYPE_SEMENTE_BASE + i);
    }


    // --- 2. Obter o Tempo Decorrido ---
    uint64_t current_time_us = esp_timer_get_time();
    uint64_t elapsed_time_us = current_time_us - last_calculation_time_us;
    last_calculation_time_us = current_time_us; // Atualiza para o proximo ciclo

    // Evita divisao por zero se o tempo decorrido for 0 ou muito pequeno
    float elapsed_time_s = (float)elapsed_time_us / 1000000.0;
    if (elapsed_time_s < 0.001) { // Define um limite mínimo para o tempo decorrido
             vTaskDelay(pdMS_TO_TICKS(1)); // Pequeno delay para evitar loop muito rapido em caso de tempo zero
      continue; // Pula este ciclo se o tempo for insignificante
    }


    // --- 3. Realizar Cálculos (Protegido por dataMutex) ---
    // Protege o acesso as variaveis globais calculadas
    if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { // Tenta pegar o mutex com timeout curto

      // === Calculo da Velocidade (km/h) ===
      // velocidade (m/s) = (pulsos_roda * cal_roda_m_por_pulso) / tempo_decorrido_s
      // velocidade (km/h) = velocidade (m/s) * 3.6
      // Obtem a calibracao da roda (metros por pulso) - Protegido por configMutex
      float cal_roda = 0.0;
      if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        cal_roda = cal_roda_m_por_pulso; // Variavel global de ui/ui_task.h
        xSemaphoreGive(configMutex);
      } else {
        // Se nao conseguir o mutex, usa um valor padrao ou o ultimo valor conhecido
        cal_roda = 0.5; // Valor padrao de seguranca
        Serial.println("Calculos: WARNING - Could not get configMutex for cal_roda!");
      }

      float distancia_roda_neste_ciclo = (float)roda_pulses * cal_roda;
      float velocidade_m_s = distancia_roda_neste_ciclo / elapsed_time_s;
      velocidade_km_h = velocidade_m_s * 3.6;

      // Define se a plantadeira esta ativa (ex: velocidade > 0.5 km/h)
      plantadeira_ativa = (velocidade_km_h > 0.5); // AJUSTE O LIMITE DE VELOCIDADE


      // === Calculo da Distância e Área ===
      distancia_percorrida_m = distancia_roda_neste_ciclo;
      // Area (ha) = (Distancia percorrida em m) * (Largura da plantadeira em m) / 10000 (m2/ha)
      // Obtem a largura da plantadeira - Protegido por configMutex
      float largura_plantadeira = 0.0; // Em metros
      if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        largura_plantadeira = LARGURA_PLANTADEIRA_METROS; // Corrigido o nome da constante // Assume que LARGURA_PLANTADEIRA_METROS esta em config.h
        xSemaphoreGive(configMutex);
      } else {
        // Se nao conseguir o mutex, usa um valor padrao de seguranca ou do config.h
        largura_plantadeira = LARGURA_PLANTADEIRA_METROS; // Corrigido o nome da constante // Valor padrao de seguranca
        Serial.println("Calculos: WARNING - Could not get configMutex for largura_plantadeira!");
      }

      area_plantada_ha = (distancia_percorrida_m * largura_plantadeira) / 10000.0;


      // === Calculo de Sementes por Metro (SPM) por Linha e Media ===
      float total_sementes_neste_ciclo = 0;
      for (int i = 0; i < NUM_LINHAS; i++) {
        // SPM (sementes/m) = (pulsos_semente_linha_i) / (distancia_roda_neste_ciclo)
        if (distancia_roda_neste_ciclo > 0.01) { // Evita divisao por zero ou por distancia muito pequena
          sementes_por_metro_linha[i] = (float)semente_pulses_linha[i] / distancia_roda_neste_ciclo;
        } else {
          sementes_por_metro_linha[i] = 0.0; // Se nao moveu, SPM eh zero
        }
        total_sementes_neste_ciclo += semente_pulses_linha[i];
      }

      // SPM Media = (Total de sementes em todas as linhas) / (Distancia percorrida) / (Numero de linhas)
      if (distancia_roda_neste_ciclo > 0.01 && NUM_LINHAS > 0) {
        media_sementes_por_metro = total_sementes_neste_ciclo / distancia_roda_neste_ciclo / (float)NUM_LINHAS;
      } else {
        media_sementes_por_metro = 0.0;
      }


      // === Calculo de Vazão de Sementes por Hectare ===
      // Vazao (sementes/ha) = (Total de sementes neste ciclo) / (Area plantada neste ciclo em ha)
      float area_neste_ciclo_ha = (distancia_roda_neste_ciclo * largura_plantadeira) / 10000.0;
      if (area_neste_ciclo_ha > 0.000001) { // Evita divisao por zero ou area muito pequena
        vazao_sementes_por_hectare = total_sementes_neste_ciclo / area_neste_ciclo_ha;
      } else {
        vazao_sementes_por_hectare = 0.0;
      }


      // === Calculo de Vazão de Inoculante por Hectare ===
      // Vazao (L/ha) = (Pulsos inoculante) / (1000 pulsos/cal_inoculante_L) / (Area plantada neste ciclo em ha)
      // Obtem a calibracao do inoculante (Litros por 1000 pulsos) - Protegido por configMutex
      float cal_inoculante = 0.0; // Litros por 1000 pulsos
      if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        cal_inoculante = cal_inoculante_l_por_1000pulsos; // Variavel global de ui/ui_task.h
        xSemaphoreGive(configMutex);
      } else {
        // Se nao conseguir o mutex, usa um valor padrao de seguranca
        cal_inoculante = 10.0; // Valor padrao de seguranca
        Serial.println("Calculos: WARNING - Could not get configMutex for cal_inoculante!");
      }

      if (area_neste_ciclo_ha > 0.000001) {
        float litros_inoculante_neste_ciclo = ((float)inoculante_pulses / 1000.0) * cal_inoculante;
        vazao_inoculante_por_hectare = litros_inoculante_neste_ciclo / area_neste_ciclo_ha;
      } else {
        vazao_inoculante_por_hectare = 0.0;
      }


      // --- 4. Verificar Condições de Alarme (Protegido por dataMutex para leitura das variaveis calculadas) ---
      // Esta logica verifica se a SPM media esta fora da faixa alvo (min/max) e sinaliza os Event Groups.
      // A faixa alvo (setting_alarme_spm_min/max) eh obtida com configMutex.

      float alarme_spm_min = 0.0;
      float alarme_spm_max = 0.0;
      if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        alarme_spm_min = setting_alarme_spm_min; // Variavel global de ui/ui_task.h
        alarme_spm_max = setting_alarme_spm_max; // Variavel global de ui/ui_task.h
        xSemaphoreGive(configMutex);
      } else {
        // Se nao conseguir o mutex, usa valores padrao de seguranca
        alarme_spm_min = 4.0;
        alarme_spm_max = 6.0;
        Serial.println("Calculos: WARNING - Could not get configMutex for alarme_spm!");
      }

      // Verifica alarme geral de sementes SOMENTE se a plantadeira estiver ativa
      if (plantadeira_ativa) {
        if (media_sementes_por_metro < alarme_spm_min || media_sementes_por_metro > alarme_spm_max) {
          // Sinaliza alarme geral de sementes fora da faixa
          if (systemEvents != NULL) xEventGroupSetBits(systemEvents, BIT_ALERTA_SEMENTES_GERAL);
          // Serial.printf("Calculos: ALARME GERAL SEMENTES - SPM Media: %.1f\n", media_sementes_por_metro); // Debug de alarme
        } else {
          // Limpa o bit de alarme geral se a SPM estiver dentro da faixa
          if (systemEvents != NULL) xEventGroupClearBits(systemEvents, BIT_ALERTA_SEMENTES_GERAL);
        }

        // Verifica falha total (nenhum pulso em todas as linhas em um ciclo, enquanto ativo)
        if (total_sementes_neste_ciclo == 0 && distancia_roda_neste_ciclo > 0.01) { // Se moveu mas nao teve sementes
          if (systemEvents != NULL) xEventGroupSetBits(systemEvents, BIT_SEMENTES_FALHA_TOTAL);
          // Serial.println("Calculos: ALARME - Falha total de sementes!"); // Debug de alarme
        } else {
          if (systemEvents != NULL) xEventGroupClearBits(systemEvents, BIT_SEMENTES_FALHA_TOTAL);
        }

        // TODO: Verificar alarmes por linha individual (BIT_SEMENTES_LINHA_i_ABAIXO/ACIMA)
        for (int i = 0; i < NUM_LINHAS; i++) {
          bool alarme_abaixo = (sementes_por_metro_linha[i] < alarme_spm_min && distancia_roda_neste_ciclo > 0.01);
          bool alarme_acima = (sementes_por_metro_linha[i] > alarme_spm_max && distancia_roda_neste_ciclo > 0.01);

          if (systemEvents != NULL) {
            if (alarme_abaixo) xEventGroupSetBits(systemEvents, GET_BIT_SEMENTES_LINHA_ABAIXO(i));
            else xEventGroupClearBits(systemEvents, GET_BIT_SEMENTES_LINHA_ABAIXO(i));

            if (alarme_acima) xEventGroupSetBits(systemEvents, GET_BIT_SEMENTES_LINHA_ACIMA(i));
            else xEventGroupClearBits(systemEvents, GET_BIT_SEMENTES_LINHA_ACIMA(i));
          }
          // if (alarme_abaixo || alarme_acima) Serial.printf("Calculos: ALARME Linha %d - SPM: %.1f\n", i, sementes_por_metro_linha[i]); // Debug de alarme por linha
        }

        // TODO: Verificar alarme de vazao de inoculante (BIT_ALERTA_VAZAO_INOCULANTE)
        // A logica de alarme para inoculante dependeria da vazao alvo (L/ha) e da vazao medida.
        // float vazao_inoculante_alvo = ...; // Obter de configMutex (setting_vazao_alvo_lha)
        // if (vazao_inoculante_por_hectare < vazao_inoculante_alvo * 0.9 || vazao_inoculante_por_hectare > vazao_inoculante_alvo * 1.1) { // Exemplo: +-10%
        //  if (systemEvents != NULL) xEventGroupSetBits(systemEvents, BIT_ALERTA_VAZAO_INOCULANTE);
        // } else {
        //  if (systemEvents != NULL) xEventGroupClearBits(systemEvents, BIT_ALERTA_VAZAO_INOCULANTE);
        // }

      } else { // Plantadeira inativa, limpar bits de alarme relacionados a sementes/vazao
        if (systemEvents != NULL) {
          xEventGroupClearBits(systemEvents, BIT_ALERTA_SEMENTES_GERAL | BIT_SEMENTES_FALHA_TOTAL | BIT_ALERTA_VAZAO_INOCULANTE);
          for (int i = 0; i < NUM_LINHAS; i++) {
            xEventGroupClearBits(systemEvents, GET_BIT_SEMENTES_LINHA_ABAIXO(i) | GET_BIT_SEMENTES_LINHA_ACIMA(i));
          }
        }
      }


      // --- 5. Enviar Dados Calculados para RPi (em formato JSON) ---
      // Usa a funcao rpi_comm_send_data do modulo rpi_comm
      // Criar um objeto JSON estatico na stack (evita fragmentacao da heap)
      // Tamanho do buffer JSON (ajuste conforme a quantidade de dados)
      const size_t JSON_DOC_SIZE = 768; // Tamanho adequado para os dados atuais + folga
      StaticJsonDocument<JSON_DOC_SIZE> doc; // Corrigido: Adicionado argumento de template

      // Popula o objeto JSON com os dados calculados
      doc["velocidade_kmh"] = velocidade_km_h;
      doc["spm_media"] = media_sementes_por_metro;
      // Adiciona SPM por linha como um array JSON
      JsonArray sementes_linha_array = doc.createNestedArray("spm_linha");
      for(int i=0; i < NUM_LINHAS; i++){
        sementes_linha_array.add(sementes_por_metro_linha[i]);
      }
      doc["vazao_sementes_ha"] = vazao_sementes_por_hectare;
      doc["vazao_inoculante_ha"] = vazao_inoculante_por_hectare;
      doc["distancia_m"] = distancia_percorrida_m;
      doc["area_ha"] = area_plantada_ha;
      doc["plantadeira_ativa"] = plantadeira_ativa;
      doc["timestamp_us"] = current_time_us; // Incluir timestamp para referencia

      // Incluir status dos Event Groups (opcional, pode ser util para debug na RPi)
      if (systemEvents != NULL) doc["sys_events_1"] = xEventGroupGetBits(systemEvents);
      if (systemEvents2 != NULL) doc["sys_events_2"] = xEventGroupGetBits(systemEvents2);

      // Incluir duty cycle da bomba (para monitoramento)
      doc["bomba_pwm_duty"] = getPumpPwmDutyCycle(); // Chama funcao do modulo para obter o duty cycle atual // ADICIONADO

      // Serializar o objeto JSON para uma string
      String json_string;
      serializeJson(doc, json_string);

      // Enviar os dados para a RPi
      // TODO: Definir o endpoint correto na RPi para telemetria
      // rpi_comm_send_data("/telemetria", json_string);


      xSemaphoreGive(dataMutex); // Libera o mutex
    } else {
      Serial.println("Calculos: ERROR - Could not get dataMutex!");
      // Sinaliza erro no SEGUNDO Event Group (se houver um bit apropriado)
      // if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_MUTEX_DATA); // Exemplo
    }


    // Aguarda o proximo ciclo de calculo
    vTaskDelay(CALCULATION_INTERVAL_MS);

  } // Fim do while(true)

  // Esta linha teoricamente nunca sera alcancada
  vTaskDelete(NULL);
}