// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: src/nvs_wrapper.c
// Descrição: Wrapper C para funções NVS do ESP-IDF utilizadas em C++.
// Garante a compatibilidade de linkage para uso em .cpp.
// ========================================================

// Incluir headers C padrão essenciais explicitamente
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> // Para tipo bool em C
#include <stdint.h>  // Para tipos inteiros

// Incluir headers do driver NVS do ESP-IDF
#include "esp_err.h"     // Para esp_err_t e macros relacionadas
#include "nvs_flash.h"   // Para nvs_flash_init() e apis de inicializacao
#include "nvs.h"         // Para nvs_open, nvs_get_*, nvs_set_*, etc.

// --- DECLARACOES EXTERN MANUAIS PARA FUNCOES FLOAT NVS (WORKAROUND) ---
// O compilador C nao esta encontrando estas declaracoes em nvs.h por algum motivo na v5.1.2.
// Adicionamos manualmente para permitir a compilacao.
extern esp_err_t nvs_get_float(nvs_handle_t handle, const char* key, float* out_value);
extern esp_err_t nvs_set_float(nvs_handle_t handle, const char* key, float value);
// --- FIM DECLARACOES EXTERN MANUAIS ---


// Incluir o header do wrapper C (deve ser incluido depois dos headers que ele wrappa)
#include "nvs_wrapper.h"


// A implementacao das funcoes wrapper C (declaradas no header nvs_wrapper.h)

/**
 * @brief Wrapper C para nvs_open.
 * @param name Namespace NVS.
 * @param open_mode Modo de abertura (NVS_READONLY or NVS_READWRITE).
 * @param out_handle Handle NVS retornado.
 * @return ESP_OK em sucesso, ou codigo de erro NVS.
 */
esp_err_t nvs_wrapper_open(const char* name, nvs_open_mode open_mode, nvs_handle_t* out_handle) {
  return nvs_open(name, open_mode, out_handle);
}

/**
 * @brief Wrapper C para nvs_close.
 * @param handle Handle NVS.
 * @return ESP_OK em sucesso (ja que nvs_close original eh void).
 */
esp_err_t nvs_wrapper_close(nvs_handle_t handle) {
  nvs_close(handle);
  return ESP_OK;
}

/**
 * @brief Wrapper C para nvs_get_float.
 * @param handle Handle NVS.
 * @param key Chave NVS.
 * @param out_value Ponteiro para armazenar o valor float lido.
 * @return ESP_OK em sucesso, ou codigo de erro NVS.
 */
esp_err_t nvs_wrapper_get_float(nvs_handle_t handle, const char* key, float* out_value) {
  // O compilador C agora encontrara a declaracao extern acima.
  return nvs_get_float(handle, key, out_value);
}

/**
 * @brief Wrapper C para nvs_set_float.
 * @param handle Handle NVS.
 * @param key Chave NVS.
 * @param value Valor float a ser salvo.
 * @return ESP_OK em sucesso, ou codigo de erro NVS.
 */
esp_err_t nvs_wrapper_set_float(nvs_handle_t handle, const char* key, float value) {
  // O compilador C agora encontrara a declaracao extern acima.
  return nvs_set_float(handle, key, value);
}


// Implemente wrappers para outras funcoes NVS (get/set para outros tipos) se necessario