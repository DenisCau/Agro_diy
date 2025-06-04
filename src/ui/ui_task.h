// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: ui/ui_task.h
// Descrição: Definições e protótipos para o módulo de Interface do Usuario (UI).
// Gerencia display OLED e botoes.
// ========================================================

#ifndef UI_TASK_H
#define UI_TASK_H

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Necessario para TaskFunction_t
#include "freertos/semphr.h" // Necessario para SemaphoreHandle_t
#include "freertos/event_groups.h" // Necessario para EventGroupHandle_t

#include "main.h" // Para SystemEventBits_t, SystemEventBits2_t, systemEvents, systemEvents2, Mutexes
#include "config.h" // Para NUM_LINHAS e definicoes de pinos


// === Definição dos Estados da Interface do Usuário ===
typedef enum {
 STATE_OPERATION,
 STATE_MENU_MAIN,
 STATE_MENU_CALIBRATION,
 STATE_MENU_SETTINGS,
 STATE_EDIT_CAL_RODA,
 STATE_EDIT_CAL_SEMENTE,
 STATE_EDIT_CAL_INOCULANTE,
 STATE_EDIT_SETTING_SPM_MIN,
 STATE_EDIT_SETTING_SPM_MAX,
 STATE_EDIT_SETTING_VAZAO_ALVO,
} ui_state_t;

// === Parâmetros Ajustáveis ===
typedef enum {
 PARAM_ADJUST_CAL_RODA = 0,
 PARAM_ADJUST_CAL_SEMENTE,
 PARAM_ADJUST_CAL_INOCULANTE,
 PARAM_SETTING_ALARME_SPM_MIN,
 PARAM_SETTING_ALARME_SPM_MAX,
 PARAM_SETTING_VAZAO_ALVO,
 PARAM_COUNT
} adjustable_parameter_t;

// === Variáveis Globais do Módulo UI ===
extern ui_state_t currentUIState;
extern int currentMenuItem;

// Variaveis de configuracao e calibracao que sao ajustadas pela UI e usadas por Calculos/RPi_Comm
// Protegidas pelo configMutex
extern volatile float cal_roda_m_por_pulso;
extern volatile float cal_semente_s_por_metro;
extern volatile float cal_inoculante_l_por_1000pulsos;

extern volatile float setting_alarme_spm_min;
extern volatile float setting_alarme_spm_max;
extern volatile float setting_vazao_alvo_lha;


// === Protótipos das Funções da UI ===
/**
 * @brief Inicializa o display OLED e os pinos dos botões.
 */
void ui_init();

/**
 * @brief Carrega os parâmetros de configuração e calibração da NVS.
 */
void loadParametersFromNVS();

/**
 * @brief Salva os parâmetros de configuração e calibração na NVS.
 */
void saveParametersToNVS();

/**
 * @brief Desenha a tela de operação principal no display.
 */
void drawOperationScreen();

/**
 * @brief Desenha o menu principal no display.
 */
void drawMainMenu();

/**
 * @brief Desenha o menu de calibração no display.
 */
void drawCalibrationMenu();

/**
 * @brief Desenha o menu de configurações no display.
 */
void drawSettingsMenu();

/**
 * @brief Desenha a tela para edição de um parâmetro.
 * @param param_name Nome do parâmetro a ser exibido.
 * @param current_value Valor atual sendo editado.
 */
void drawEditParameterScreen(const char* param_name, float current_value);

/**
 * @brief Desenha a tela atual com base no estado da UI.
 */
void drawCurrentScreen();

/**
 * @brief Processa a entrada dos botões e transiciona os estados da UI.
 */
void processButtonInput();


// === Protótipo da Tarefa FreeRTOS ===

/**
 * @brief Tarefa principal da Interface do Usuário.
 * Gerencia o display, botões e estados da UI.
 * @param pvParameters Parâmetros da tarefa (não utilizado).
 */
extern "C" { // Adicionado extern "C" para compatibilidade com C/FreeRTOS
    void tarefaUI(void *pvParameters);
}


#endif // UI_TASK_H