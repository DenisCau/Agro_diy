// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: include/nvs_wrapper.h
// Descrição: Header para o wrapper C das funcoes NVS,
//      usado para contornar problemas de linkage C++
// ========================================================

#ifndef NVS_WRAPPER_H
#define NVS_WRAPPER_H

// Inclui headers necessarios do ESP-IDF
#include "esp_err.h"   // Para esp_err_t
#include "nvs_flash.h"  // Para nvs_handle_t
#include "nvs.h"    // Para outras definicoes NVS (como nvs_open_mode)

// Declaracoes das funcoes wrapper com linkage C
// Usamos #ifdef __cplusplus para que extern "C" so seja ativado em compiladores C++.
#ifdef __cplusplus
extern "C" {
#endif

  esp_err_t nvs_wrapper_open(const char* namespace_name, nvs_open_mode open_mode, nvs_handle_t* out_handle);
  esp_err_t nvs_wrapper_close(nvs_handle_t handle); // Retorna esp_err_t para consistencia, mesmo que original seja void
  esp_err_t nvs_wrapper_commit(nvs_handle_t handle);

  // Wrappers para as funcoes float que estao dando problema
  esp_err_t nvs_wrapper_get_float(nvs_handle_t handle, const char* key, float* out_value);
  esp_err_t nvs_wrapper_set_float(nvs_handle_t handle, const char* key, float value);

  // TODO: Adicionar wrappers para outras funcoes NVS (i32, string, etc.) se forem usadas
  esp_err_t nvs_wrapper_get_i32(nvs_handle_t handle, const char* key, int32_t* out_value);
  esp_err_t nvs_wrapper_set_i32(nvs_handle_t handle, const char* key, int32_t value);
  esp_err_t nvs_wrapper_erase_key(nvs_handle_t handle, const char* key);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // NVS_WRAPPER_H