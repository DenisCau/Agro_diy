// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: main.h
// Descrição: Definições e declarações globais do projeto principal.
// Contém handles para Event Groups, Mutexes e defines de bits de eventos.
// ========================================================

#ifndef MAIN_H
#define MAIN_H

// === Includes Necessários ===
// Inclui headers do FreeRTOS PRIMEIRO para garantir que os tipos estejam definidos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// Inclui outros headers do sistema e bibliotecas
#include <Arduino.h> // Para tipos basicos, funcoes Arduino, Serial, etc.
#include <esp_err.h> // Para esp_err_t e outros codigos de erro do ESP-IDF
#include <esp_log.h> // Para logging do ESP-IDF


// Inclui headers de configuracao e outros modulos
#include "config.h" // Configurações de pinos, constantes, etc.


// === Event Groups do Sistema ===
// Event Groups sao usados para sinalizar eventos entre tarefas.
// Criados em main.cpp e declarados como extern aqui.
extern EventGroupHandle_t systemEvents;  // Eventos gerais do sistema (alarmes de sensores, etc.)
extern EventGroupHandle_t systemEvents2; // Eventos de comunicação/erros de hardware (WiFi, UART, NVS, etc.)

// === Definições de Bits para o Primeiro Event Group (systemEvents) ===
// Use bits individuais para sinalizar condicoes especificas.
// As tarefas interessadas esperam pelos bits relevantes.
// Bits 0 a (NUM_LINHAS-1) * 2 : Alarmes por linha de semente (ABAIXO e ACIMA do alvo)
// BIT_SEMENTES_LINHA_i_ABAIXO = 1 << (i * 2)
// BIT_SEMENTES_LINHA_i_ACIMA = 1 << (i * 2 + 1)
// Ex: Linha 0 Abaixo = 1 << 0 (0x01)
// Ex: Linha 0 Acima = 1 << 1 (0x02)
// Ex: Linha 1 Abaixo = 1 << 2 (0x04)
// Ex: Linha 1 Acima = 1 << 3 (0x08)
// ... e assim por diante.
#if NUM_LINHAS > 0
// Define os bits para a primeira linha como exemplo
#define BIT_SEMENTES_LINHA_0_ABAIXO (1 << 0)
#define BIT_SEMENTES_LINHA_0_ACIMA  (1 << 1)
// Macros para obter os bits para qualquer linha i
#define GET_BIT_SEMENTES_LINHA_ABAIXO(i) (1 << ((i) * 2))
#define GET_BIT_SEMENTES_LINHA_ACIMA(i)  (1 << ((i) * 2 + 1))
#endif

// Outros bits de evento para systemEvents (gerais, nao por linha especifica)
#define BIT_ALERTA_SEMENTES_GERAL     (1 << (NUM_LINHAS * 2))     // Alarme geral de sementes (SPM medio fora da faixa)
#define BIT_SEMENTES_FALHA_TOTAL      (1 << (NUM_LINHAS * 2 + 1)) // Falha total de sementes (nenhum pulso em todas as linhas)
#define BIT_ALERTA_VAZAO_INOCULANTE   (1 << (NUM_LINHAS * 2 + 2)) // Alarme de vazao de inoculante (fora do alvo/faixa)
// TODO: Adicionar outros bits de evento para systemEvents (ex: velocidade baixa/alta)

// === Definições de Bits para o Segundo Event Group (systemEvents2) ===
// Usado para erros de hardware, comunicacao, status do Secundario, etc.
#define BIT_ERRO_WIFI                 (1 << 0) // Erro na conexao WiFi
#define BIT_ERRO_HTTP                 (1 << 1) // Erro na comunicacao HTTP (cliente ou servidor)
#define BIT_ERRO_NVS                  (1 << 2) // Erro na leitura/escrita da NVS
#define BIT_ERRO_SENSOR_SEMENTE_ISR   (1 << 3) // Erro na ISR do sensor de semente (ex: pino sem interrupt)
#define BIT_ERRO_UART_SECUNDARIO      (1 << 4) // Erro na comunicacao UART com o Secundario (checksum, framing, etc.)
#define BIT_NOVO_DADO_SECUNDARIO      (1 << 5) // Sinaliza que um novo pacote de dados do Secundario foi recebido e processado
#define BIT_ALERTA_MPU_SECUNDARIO     (1 << 6) // Alerta do status do MPU reportado pelo Secundario
#define BIT_ALERTA_NIVEL_BAIXO_INOCULANTE (1 << 7) // Alerta de nivel baixo de inoculante reportado pelo Secundario
// TODO: Adicionar outros bits de evento para systemEvents2 (ex: erro RTC Secundario, erro no sensor analogico/ultrasonico do Secundario)


// === Mutexes Globais ===
// Mutexes para proteger o acesso a recursos/variaveis compartilhados entre tarefas.
// Criados em main.cpp e declarados como extern aqui.
extern SemaphoreHandle_t dataMutex;   // Protege variaveis globais de dados calculados (velocidade, SPM, vazao, distancia, area)
extern SemaphoreHandle_t configMutex; // Protege variaveis globais de configuracao e calibracao (calibracoes, alarmes, etc.)
extern SemaphoreHandle_t uiMutex;     // Protege o acesso ao display OLED e variaveis de estado da UI

// === Protótipos das Funções Principais ===
// Declaracao das funcoes principais (setup, loop estilo Arduino)
// A funcao app_main eh o ponto de entrada do ESP-IDF
// A funcao setup_arduino_tasks eh onde as tarefas FreeRTOS sao criadas
// A funcao loop_dummy eh uma tarefa vazia para manter o core 1 rodando se a tarefa da UI nao usar o core 1
extern "C" {
    void app_main();
}
void setup_arduino_tasks();
void loop_dummy(void *pvParameters);

#endif // MAIN_H