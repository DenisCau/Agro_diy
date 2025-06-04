// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: communication/rpi_comm.h
// Descrição: Definições e protótipos para o módulo de comunicação WiFi e HTTP com a Raspberry Pi.
// Inclui a tarefa FreeRTOS para gerenciar o servidor HTTP.
// ========================================================

#ifndef RPI_COMM_H
#define RPI_COMM_H

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Necessario para TaskFunction_t
#include "freertos/semphr.h" // Necessario para SemaphoreHandle_t
#include "freertos/event_groups.h" // Necessario para EventGroupHandle_t

#include <Arduino.h> // Para tipos basicos, String, etc.
#include "config.h"  // Para defines de WiFi (SSID, Password) e RPI_HTTP_SERVER_URL
#include "main.h"    // Para handles globais (configMutex, systemEvents2), defines de bits (BIT_ERRO_WIFI, BIT_ERRO_HTTP)


// === Definições e Constantes ===
// TODO: Definir SSID e Password da rede WiFi em config.h ou carregar da NVS

// Porta HTTP para o servidor web neste ESP32
#define HTTP_SERVER_PORT 80 // RENOMEADO para evitar confusao com porta da RPi

// Endereco base do servidor HTTP da Raspberry Pi (definido em config.h)
// #define RPI_HTTP_SERVER_URL "http://192.168.1.100:5000/" // Definido em config.h

// === Variáveis Globais do Módulo RPi_Comm (extern) ===
// Nenhuma variavel global precisa ser acessada externamente neste modulo corrigido

// === Protótipos de Funções do Módulo RPi_Comm ===

/**
 * @brief Inicializa a comunicação WiFi e o Servidor HTTP.
 * Esta função deve ser chamada DENTRO da tarefa FreeRTOS dedicada.
 */
void rpi_comm_init();

/**
 * @brief Envia dados (em formato JSON) para um endpoint específico na Raspberry Pi via HTTP POST.
 * @param endpoint O caminho do endpoint na RPi (ex: "/telemetria", "/alarme"). Deve incluir a barra inicial.
 * @param json_data Uma string contendo os dados em formato JSON.
 * @return true se o envio HTTP foi bem-sucedido (código 2xx), false caso contrário.
 */
bool rpi_comm_send_data(const char* endpoint, const String& json_data);


// === Protótipo da Tarefa FreeRTOS do Módulo RPi_Comm ===
// Esta tarefa gerencia a conexao WiFi e o servidor HTTP.
extern "C" { // Adicionado extern "C" para compatibilidade com C/FreeRTOS
    void tarefaComunicacaoRPi(void *pvParameters);
}


#endif // RPI_COMM_H