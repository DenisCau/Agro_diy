// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: nvs_rw/nvs_rw.cpp
// Descrição: Implementação do módulo de leitura e escrita na NVS (Non-Volatile Storage).
// Gerencia o acesso aos dados persistentes do sistema (configuracoes, calibracoes, etc.).
// ========================================================

// === Includes Necessários ===
#include "nvs_rw/nvs_rw.h" // Inclui o header deste modulo
#include <nvs_flash.h>   // Para funcoes da NVS Flash (nvs_open, nvs_close, nvs_get_blob, nvs_set_blob, nvs_commit)
#include <esp_err.h>     // Para codigos de erro como ESP_OK, ESP_ERR_NVS_NOT_FOUND, ESP_ERR_NVS_INVALID_SIZE, esp_err_to_name
#include <Arduino.h>     // Para Serial.println, etc.
#include "main.h"        // Para handles globais (configMutex, systemEvents2), defines de bits (BIT_ERRO_NVS)
 // FreeRTOS Event Groups


// Namespace da NVS a ser usado para o projeto
#define NVS_NAMESPACE "plantadeira"
// Chave NVS para os parametros de configuracao e calibracao da plantadeira
#define NVS_KEY_PLANT_PARAMS "plant_params"

// Handle da NVS (sera aberto na inicializacao e mantido aberto durante a execucao)
// DEFINICAO da variavel declarada como 'extern' em nvs_rw.h
nvs_handle_t s_nvs_handle;

// Flag para indicar se o nosso namespace NVS foi inicializado com sucesso
// DEFINICAO da variavel declarada como 'extern' em nvs_rw.h
bool nvs_initialized = false;


// === Implementação das Funções do Módulo NVS ===

// Inicializa o namespace NVS do projeto.
esp_err_t nvs_init_namespace() {
    esp_err_t ret = ESP_FAIL; // Inicializa com falha

    // Tenta abrir o namespace. Se nao conseguir o mutex, retorna erro.
    if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // Tenta pegar o mutex com timeout
        // Se o namespace ainda nao foi inicializado
        if (!nvs_initialized) {
            Serial.printf("NVS: Initializing namespace '%s'...\n", NVS_NAMESPACE);
            // Abre o namespace no modo de leitura/escrita
            ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs_handle);
            if (ret == ESP_OK) {
                Serial.println("NVS: Namespace opened successfully.");
                nvs_initialized = true; // Marca como inicializado
            } else {
                Serial.printf("NVS: ERROR - Failed to open namespace '%s'! (%s)\n", NVS_NAMESPACE, esp_err_to_name(ret));
                nvs_initialized = false; // Garante que a flag esteja falsa em caso de erro
                // Sinaliza erro no SEGUNDO Event Group
                if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
            }
        } else {
            // Namespace ja inicializado
            Serial.println("NVS: Namespace already initialized.");
            ret = ESP_OK; // Considera sucesso se ja estiver inicializado
        }
        xSemaphoreGive(configMutex); // Libera o mutex
    } else {
        Serial.println("NVS: ERROR - Could not get configMutex for initialization!");
        ret = ESP_ERR_TIMEOUT; // Retorna erro de timeout se nao conseguir o mutex
        // Sinaliza erro no SEGUNDO Event Group
        if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
    }

    return ret;
}

// Le um blob (bloco de dados binarios) da NVS.
esp_err_t nvs_read_blob(const char* key, void* out_value, size_t length) {
    esp_err_t ret = ESP_FAIL;

    // Tenta pegar o mutex e verifica se a NVS foi inicializada
    if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (nvs_initialized) {
            // Serial.printf("NVS: Reading blob for key '%s', max size %zu bytes...\n", key, length); // Debug (pode ser verboso)
            // Le o blob da NVS
            ret = nvs_get_blob(s_nvs_handle, key, out_value, &length);
            if (ret == ESP_OK) {
                // Serial.printf("NVS: Blob key '%s' read successfully. Size: %zu bytes.\n", key, length); // Debug
            } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
                // Serial.printf("NVS: Blob key '%s' not found.\n", key); // Debug
            } else {
                Serial.printf("NVS: ERROR - Failed to read blob for key '%s'! (%s)\n", key, esp_err_to_name(ret));
                // Sinaliza erro no SEGUNDO Event Group
                if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
            }
        } else {
            Serial.println("NVS: ERROR - NVS namespace not initialized for reading!");
            ret = ESP_ERR_NVS_NOT_INITIALIZED;
            // Sinaliza erro no SEGUNDO Event Group
            if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
        }
        xSemaphoreGive(configMutex); // Libera o mutex
    } else {
        Serial.println("NVS: ERROR - Could not get configMutex for reading!");
        ret = ESP_ERR_TIMEOUT;
        // Sinaliza erro no SEGUNDO Event Group
        if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
    }

    return ret;
}

// Escreve um blob (bloco de dados binarios) na NVS.
esp_err_t nvs_write_blob(const char* key, const void* value, size_t length) {
    esp_err_t ret = ESP_FAIL;

    // Tenta pegar o mutex e verifica se a NVS foi inicializada
    if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (nvs_initialized) {
            // Serial.printf("NVS: Writing blob for key '%s', size %zu bytes...\n", key, length); // Debug (pode ser verboso)
            // Escreve o blob na NVS
            ret = nvs_set_blob(s_nvs_handle, key, value, length);
            if (ret == ESP_OK) {
                // Serial.printf("NVS: Blob key '%s' written successfully.\n", key); // Debug
            } else {
                Serial.printf("NVS: ERROR - Failed to write blob for key '%s'! (%s)\n", key, esp_err_to_name(ret));
                // Sinaliza erro no SEGUNDO Event Group
                if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
            }
        } else {
            Serial.println("NVS: ERROR - NVS namespace not initialized for writing!");
            ret = ESP_ERR_NVS_NOT_INITIALIZED;
            // Sinaliza erro no SEGUNDO Event Group
            if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
        }
        xSemaphoreGive(configMutex); // Libera o mutex
    } else {
        Serial.println("NVS: ERROR - Could not get configMutex for writing!");
        ret = ESP_ERR_TIMEOUT;
        // Sinaliza erro no SEGUNDO Event Group
        if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
    }

    return ret;
}

// Salva os parametros de configuracao e calibracao da plantadeira na NVS.
// Esta funcao deve ser chamada quando os parametros forem modificados e precisarem ser persistidos.
// Protegido por configMutex (atraves das chamadas a nvs_write_blob e nvs_commit).
esp_err_t nvs_write_plant_parameters(const void* params, size_t params_size) {
    esp_err_t ret = ESP_FAIL;

    Serial.println("NVS: Attempting to save plant parameters...");

    // Primeiro, escreve o blob com os parametros
    ret = nvs_write_blob(NVS_KEY_PLANT_PARAMS, params, params_size);

    if (ret == ESP_OK) {
        // Se a escrita do blob foi bem-sucedida, agora commita as mudancas para a flash
        // O commit tambem precisa do mutex, que ja eh gerenciado por nvs_write_blob.
        // Precisamos pegar o mutex novamente para o commit.
        if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.println("NVS: Committing changes...");
            esp_err_t commit_ret = nvs_commit(s_nvs_handle);
            if (commit_ret == ESP_OK) {
                Serial.println("NVS: Changes committed successfully.");
                ret = ESP_OK; // Retorna sucesso geral se o commit foi OK
            } else {
                Serial.printf("NVS: ERROR - Failed to commit changes! (%s)\n", esp_err_to_name(commit_ret));
                ret = commit_ret; // Retorna o erro do commit
                // Sinaliza erro no SEGUNDO Event Group
                if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
            }
            xSemaphoreGive(configMutex); // Libera o mutex
        } else {
            Serial.println("NVS: ERROR - Could not get configMutex for committing!");
            ret = ESP_ERR_TIMEOUT; // Retorna erro de timeout
            // Sinaliza erro no SEGUNDO Event Group
            if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
        }
    } else {
        // O erro ja foi reportado dentro de nvs_write_blob
        Serial.println("NVS: Failed to save plant parameters (write blob failed).");
        // O bit de erro NVS ja foi setado em nvs_write_blob
    }

    return ret;
}


// TODO: Implementar outras funcoes de leitura/escrita NVS se necessario
/*
esp_err_t nvs_read_int(const char* key, int* out_value) {
    esp_err_t ret = ESP_FAIL;
    if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (nvs_initialized) {
            ret = nvs_get_i32(s_nvs_handle, key, out_value);
            // TODO: Tratar retornos de erro e debug
        } else {
            ret = ESP_ERR_NVS_NOT_INITIALIZED;
        }
        xSemaphoreGive(configMutex);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    return ret;
}

esp_err_t nvs_write_int(const char* key, int value) {
    esp_err_t ret = ESP_FAIL;
    if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (nvs_initialized) {
            ret = nvs_set_i32(s_nvs_handle, key, value);
            if (ret == ESP_OK) {
                // Commita as mudancas imediatamente apos escrever (pode ser ineficiente se escrever varios itens)
                esp_err_t commit_ret = nvs_commit(s_nvs_handle);
                if (commit_ret != ESP_OK) {
                    // TODO: Tratar erro de commit
                    ret = commit_ret;
                }
            }
            // TODO: Tratar retornos de erro da escrita
        } else {
            ret = ESP_ERR_NVS_NOT_INITIALIZED;
        }
        xSemaphoreGive(configMutex);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    return ret;
}
*/
