// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: nvs_rw/nvs_rw.h
// Descrição: Declarações para o módulo de leitura e escrita na NVS (Non-Volatile Storage).
// Gerencia o acesso aos dados persistentes do sistema (configuracoes, calibracoes, etc.).
// ========================================================

#ifndef NVS_RW_H
#define NVS_RW_H

#include <esp_err.h>   // Para esp_err_t
#include <nvs.h>       // Para nvs_handle_t
#include <stddef.h>   // Para size_t

// === Variáveis Globais do Módulo NVS (Declaradas como extern) ===
// Estas variaveis sao declaradas aqui e definidas em nvs_rw.cpp
extern nvs_handle_t s_nvs_handle; // Handle da NVS para o nosso namespace
extern bool nvs_initialized;    // Flag para indicar se o namespace NVS foi inicializado com sucesso


// === Protótipos das Funções do Módulo NVS ===

/**
 * @brief Inicializa o namespace NVS do projeto ('plantadeira').
 * Deve ser chamado apos nvs_flash_init() e apos a criacao do configMutex.
 * @return ESP_OK em caso de sucesso, ou um codigo de erro esp_err_t.
 */
esp_err_t nvs_init_namespace();

/**
 * @brief Le um blob (bloco de dados binarios) da NVS.
 * Usa o namespace padrao do modulo. Protegido por configMutex.
 * @param key Chave para o dado na NVS.
 * @param out_value Ponteiro para o buffer onde os dados lidos serao armazenados.
 * @param length Ponteiro para o tamanho do buffer de saida. No retorno, contem o tamanho real lido.
 * @return ESP_OK em caso de sucesso, ESP_ERR_NVS_NOT_FOUND se a chave nao existe, ou outro codigo de erro.
 */
esp_err_t nvs_read_blob(const char* key, void* out_value, size_t length);

/**
 * @brief Escreve um blob (bloco de dados binarios) na NVS.
 * Usa o namespace padrao do modulo. Protegido por configMutex.
 * @param key Chave para o dado na NVS.
 * @param value Ponteiro para os dados a serem escritos.
 * @param length Tamanho dos dados em bytes.
 * @return ESP_OK em caso de sucesso, ou um codigo de erro esp_err_t.
 */
esp_err_t nvs_write_blob(const char* key, const void* value, size_t length);

/**
 * @brief Salva os parametros de configuracao e calibracao da plantadeira na NVS.
 * Esta funcao deve ser chamada quando os parametros forem modificados e precisarem ser persistidos.
 * Protegido por configMutex.
 * @param params Ponteiro para a estrutura contendo os parametros a serem salvos.
 * @param params_size Tamanho da estrutura de parametros em bytes (use sizeof()).
 * @return ESP_OK em caso de sucesso, ou um codigo de erro esp_err_t.
 */
esp_err_t nvs_write_plant_parameters(const void* params, size_t params_size);


// TODO: Adicionar prototipos para outras funcoes de leitura/escrita NVS se necessario
// Ex: esp_err_t nvs_read_int(const char* key, int* out_value);
// Ex: esp_err_t nvs_write_int(const char* key, int value);


#endif // NVS_RW_H
