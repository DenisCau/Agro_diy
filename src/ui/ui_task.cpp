// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: ui/ui_task.cpp
// Descrição: Implementação do módulo de Interface do Usuario (UI).
// Gerencia display OLED e botoes.
// ========================================================

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "ui/ui_task.h" // Inclui o header deste modulo - Corrigida sintaxe do include
#include "main.h"  // Para handles globais (dataMutex, configMutex, uiMutex, systemEvents, systemEvents2), defines de bits
#include "config.h" // Para defines de pinos, constantes (NVS_NAMESPACE, etc.)
#include "calculations/calculations.h" // Para acessar as variaveis globais calculadas (velocidade, SPM, etc.) - ADICIONADO

#include <Arduino.h> // Para Serial.print, pinMode, digitalRead
#include <Wire.h>  // Para comunicacao I2C (Display OLED)

// Incluir header do wrapper C para NVS
// Usamos o wrapper para contornar problemas de visibilidade C/C++ com as funcoes NVS.
#include "nvs_wrapper.h" // Usar wrapper em vez de incluir diretamente nvs_flash.h/nvs.h


// Incluir biblioteca da NVS (Non-Volatile Storage)
// Inclui nvs_flash.h (que inclui nvs.h onde as funcoes get/set_... e typedef nvs_handle estao)
// Remover declarações explicitas extern "C" para evitar ambiguidade se nvs.h já as declara.
#include "nvs_flash.h"
#include "nvs.h"


// Incluir bibliotecas do Display OLED
// Instale via Library Manager no PlatformIO: Adafruit GFX Library, Adafruit SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// === Definições do Display OLED ===
#define OLED_SDA          21 // Pino SDA para I2C (ajuste conforme seu hardware)
#define OLED_SCL          22 // Pino SCL para I2C (ajuste conforme seu hardware)
#define OLED_ADDR         0x3C // Endereco I2C do display OLED (0x3C ou 0x3D)
#define SCREEN_WIDTH       128 // Largura do display OLED em pixels
#define SCREEN_HEIGHT       64 // Altura do display OLED em pixels
#define OLED_RESET         -1 // Pino de reset do OLED (-1 se compartilhado com RST do ESP32)


// === Definições dos Botões ===
// TODO: Definir pinos para os botoes em config.h
#ifndef PIN_BTN_UP
 #define PIN_BTN_UP         13 // Exemplo: Pino para botao UP
#endif
#ifndef PIN_BTN_DOWN
 #define PIN_BTN_DOWN        12 // Exemplo: Pino para botao DOWN
#endif
#ifndef PIN_BTN_SELECT
 #define PIN_BTN_SELECT      14 // Exemplo: Pino para botao SELECT/ENTER
#endif
#ifndef PIN_BTN_BACK
 #define PIN_BTN_BACK       27 // Exemplo: Pino para botao BACK/CANCEL
#endif


// === Variáveis Globais do Módulo UI (DEFINICOES) ===
// DEFINICOES das variaveis globais declaradas como 'extern' em ui_task.h

ui_state_t currentUIState = STATE_OPERATION; // ui_state_t e STATE_OPERATION vem de ui_task.h
int currentMenuItem = 0;

// Variaveis de configuracao e calibracao que sao ajustadas pela UI e usadas por Calculos/RPi_Comm
// Protegidas pelo configMutex. Inicializadas com valores padrao.
volatile float cal_roda_m_por_pulso = 0.5;     // Calibracao da roda (metros por pulso)
volatile float cal_semente_s_por_metro = 0.0;  // Calibracao semente (sementes por metro) - Nota: Geralmente por linha
volatile float cal_inoculante_l_por_1000pulsos = 10.0; // Calibracao inoculante (Litros por 1000 pulsos)

volatile float setting_alarme_spm_min = 4.0;   // SPM minima para alarme
volatile float setting_alarme_spm_max = 6.0;   // SPM maxima para alarme
volatile float setting_vazao_alvo_lha = 50.0;  // Vazao alvo de inoculante (L/ha)

// TODO: Adicionar outras variaveis de configuracao/calibracao conforme necessario (largura plantadeira, etc.)


// === Objetos de Perifericos ===
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Objeto Display
// Renomeado a variavel global 'nvs_handle' para 's_nvs_handle' para evitar conflito com o typedef do ESP-IDF
// static nvs_handle_t s_nvs_handle; // Handle para a NVS - Nao precisamos que seja global aqui, pode ser local nas funcoes


// === Constantes e Variáveis Internas da Tarefa UI ===
// Intervalo de atualização do display (ms)
const TickType_t DISPLAY_UPDATE_INTERVAL_MS = pdMS_TO_TICKS(100); // Atualiza 10 vezes por segundo

// Intervalo de leitura dos botoes (ms)
const TickType_t BUTTON_READ_INTERVAL_MS = pdMS_TO_TICKS(50); // Leitura 20 vezes por segundo

// Variáveis para controle de debounce dos botões
unsigned long last_button_press_time = 0;
const unsigned long BUTTON_DEBOUNCE_DELAY_MS = 200; // Atraso de debounce em ms


// === Implementação das Funções da UI ===

/**
 * @brief Inicializa o display OLED e os pinos dos botões.
 */
void ui_init() {
 Serial.println("Module UI_Task: Initializing.");

 // === Inicialização do Display OLED ===
 // Inicia a comunicacao I2C para o display
 Wire.begin(OLED_SDA, OLED_SCL);

 // Inicializa o display com o endereco I2C e pino de reset (se houver)
 if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) { // Endereco 0x3C ou 0x3D
  Serial.println("UI_Task: ERROR - SSD1306 allocation failed");
  // TODO: Sinalizar erro no Event Group (se houver um bit para erro de hardware)
  // Loop infinito ou tentar reiniciar o display
  for(;;); // Nao continua se o display falhar
 }

 Serial.println("UI_Task: Display OLED initialized.");

 // Configura texto basico
 display.clearDisplay();
 display.setTextSize(1);   // Tamanho do texto 1 (6x8 pixels)
 display.setTextColor(SSD1306_WHITE); // Cor do texto (branco)
 display.setCursor(0,0);   // Posicao inicial do cursor
 display.display();    // Atualiza o display

 // === Inicialização dos Botões ===
#ifdef PIN_BTN_UP
 pinMode(PIN_BTN_UP, INPUT_PULLUP); // Configura pino como INPUT com PULLUP interno
#endif
#ifdef PIN_BTN_DOWN
 pinMode(PIN_BTN_DOWN, INPUT_PULLUP); // Configura pino como INPUT com PULLUP interno
#endif
#ifdef PIN_BTN_SELECT
 pinMode(PIN_BTN_SELECT, INPUT_PULLUP); // Configura pino como INPUT com PULLUP interno
#endif
#ifdef PIN_BTN_BACK
 pinMode(PIN_BTN_BACK, INPUT_PULLUP); // Configura pino como INPUT com PULLUP interno
#endif

 Serial.println("Module UI_Task: Initialization complete.");
}

/**
 * @brief Carrega os parâmetros de configuração e calibração da NVS.
 */
void loadParametersFromNVS() {
 Serial.println("UI_Task: Loading parameters from NVS.");

 // Protege o acesso as variaveis globais com o configMutex
 if (configMutex != NULL && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) { // Espera indefinidamente pelo mutex

  nvs_handle_t handle; // Usar variavel local para o handle
  esp_err_t err;

  // TODO: Definir NVS_NAMESPACE em config.h
#ifdef NVS_NAMESPACE
  err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle); // Abre a NVS para leitura
#else
    Serial.println("UI_Task: ERROR - NVS_NAMESPACE not defined in config.h!");
    err = ESP_ERR_NOT_FOUND; // Simula um erro para entrar no tratamento abaixo
#endif


  if (err != ESP_OK) {
   Serial.printf("UI_Task: Error opening NVS handle for reading (%s)!\n", esp_err_to_name(err));
   // Sinaliza erro no SEGUNDO Event Group
   if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
  } else { // NVS opened successfully
   // Le os valores da NVS. Se a chave nao existir, usa o valor padrao inicializado.
   // Erro ESP_ERR_NVS_NOT_FOUND nao eh critico para leitura, apenas significa que a chave nao existe (primeiro boot ou nunca salvo).

   err = nvs_get_float(handle, "cal_roda", (float*)&cal_roda_m_por_pulso); // Casting para float* para nvs_get_float
   if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) { // Error other than NOT_FOUND
    Serial.printf("UI_Task: Error reading cal_roda from NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   } else if (err == ESP_OK) {
    Serial.printf("UI_Task: Loaded cal_roda: %.3f\n", cal_roda_m_por_pulso);
   }

   err = nvs_get_float(handle, "cal_semente", (float*)&cal_semente_s_por_metro); // Casting
   if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) { // Error other than NOT_FOUND
    Serial.printf("UI_Task: Error reading cal_semente from NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   } else if (err == ESP_OK) {
    Serial.printf("UI_Task: Loaded cal_semente: %.3f\n", cal_semente_s_por_metro);
   }

   err = nvs_get_float(handle, "cal_inoculante", (float*)&cal_inoculante_l_por_1000pulsos); // Casting
   if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) { // Error other than NOT_FOUND
    Serial.printf("UI_Task: Error reading cal_inoculante from NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   } else if (err == ESP_OK) {
    Serial.printf("UI_Task: Loaded cal_inoculante: %.3f\n", cal_inoculante_l_por_1000pulsos);
   }

   err = nvs_get_float(handle, "setting_spm_min", (float*)&setting_alarme_spm_min); // Casting
   if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) { // Error other than NOT_FOUND
    Serial.printf("UI_Task: Error reading setting_spm_min from NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   } else if (err == ESP_OK) {
    Serial.printf("UI_Task: Loaded setting_spm_min: %.3f\n", setting_alarme_spm_min);
   }

   err = nvs_get_float(handle, "setting_spm_max", (float*)&setting_alarme_spm_max); // Casting
   if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) { // Error other than NOT_FOUND
    Serial.printf("UI_Task: Error reading setting_spm_max from NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   } else if (err == ESP_OK) {
    Serial.printf("UI_Task: Loaded setting_spm_max: %.3f\n", setting_alarme_spm_max);
   }

   err = nvs_get_float(handle, "setting_vazao_alvo", (float*)&setting_vazao_alvo_lha); // Casting
   if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) { // Error other than NOT_FOUND
    Serial.printf("UI_Task: Error reading setting_vazao_alvo from NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   } else if (err == ESP_OK) {
    Serial.printf("UI_Task: Loaded setting_vazao_alvo: %.3f\n", setting_vazao_alvo_lha);
   }

   // TODO: Ler outras variaveis da NVS

   nvs_close(handle); // Fecha o handle da NVS

  } // Fim do else (NVS opened successfully)


  xSemaphoreGive(configMutex); // Libera o mutex
  Serial.println("UI_Task: Parameter loading from NVS complete.");

 } else { // Nao conseguiu o mutex
  Serial.println("UI_Task: ERROR - Could not get configMutex to load parameters from NVS!");
  // Sinaliza erro no SEGUNDO Event Group (se houver um bit apropriado)
  // if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_MUTEX_CONFIG); // Exemplo
 }
}

/**
 * @brief Salva os parâmetros de configuração e calibração na NVS.
 */
void saveParametersToNVS() {
 Serial.println("UI_Task: Saving parameters to NVS.");

 // Protege o acesso as variaveis globais com o configMutex
 if (configMutex != NULL && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) { // Espera indefinidamente pelo mutex

  nvs_handle_t handle; // Usar variavel local para o handle
  esp_err_t err;

  // TODO: Definir NVS_NAMESPACE em config.h
#ifdef NVS_NAMESPACE
  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle); // Abre a NVS para leitura/escrita
#else
    Serial.println("UI_Task: ERROR - NVS_NAMESPACE not defined in config.h!");
    err = ESP_ERR_NOT_FOUND; // Simula um erro para entrar no tratamento abaixo
#endif


  if (err != ESP_OK) {
   Serial.printf("UI_Task: Error opening NVS handle for writing (%s)!\n", esp_err_to_name(err));
   // Sinaliza erro no SEGUNDO Event Group
   if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
  } else { // NVS opened successfully
   // Escreve os valores na NVS
   err = nvs_set_float(handle, "cal_roda", cal_roda_m_por_pulso);
   if (err != ESP_OK) {
    Serial.printf("UI_Task: Error writing cal_roda to NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   }

   err = nvs_set_float(handle, "cal_semente", cal_semente_s_por_metro);
   if (err != ESP_OK) {
    Serial.printf("UI_Task: Error writing cal_semente to NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   }

   err = nvs_set_float(handle, "cal_inoculante", cal_inoculante_l_por_1000pulsos);
   if (err != ESP_OK) {
    Serial.printf("UI_Task: Error writing cal_inoculante to NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   }

   err = nvs_set_float(handle, "setting_spm_min", setting_alarme_spm_min);
   if (err != ESP_OK) {
    Serial.printf("UI_Task: Error writing setting_spm_min to NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   }

   err = nvs_set_float(handle, "setting_spm_max", setting_alarme_spm_max);
   if (err != ESP_OK) {
    Serial.printf("UI_Task: Error writing setting_spm_max to NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   }

   err = nvs_set_float(handle, "setting_vazao_alvo", setting_vazao_alvo_lha);
   if (err != ESP_OK) {
    Serial.printf("UI_Task: Error writing setting_vazao_alvo to NVS (%s)!\n", esp_err_to_name(err));
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS); // Sinaliza erro
   }

   // TODO: Escrever outras variaveis na NVS

   // Commit the written values
   err = nvs_commit(handle);
   if (err != ESP_OK) {
    Serial.printf("UI_Task: Error committing NVS changes (%s)!\n", esp_err_to_name(err));
    // Sinaliza erro no SEGUNDO Event Group
    if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_NVS);
   } else {
    Serial.println("UI_Task: NVS changes committed.");
    // Limpa o bit de erro NVS se o commit foi bem sucedido E NAO houve erro de escrita anterior
    // Se algum nvs_set_float falhou antes do commit, o BIT_ERRO_NVS ja foi setado.
    // Mantemos setado se houve erro de escrita ou commit.
    if (systemEvents2 != NULL && !(xEventGroupGetBits(systemEvents2) & BIT_ERRO_NVS)) {
     xEventGroupClearBits(systemEvents2, BIT_ERRO_NVS); // So limpa se nao houve erros
    }
   }

   nvs_close(handle); // Fecha o handle da NVS

  } // Fim do else (NVS opened successfully)


  xSemaphoreGive(configMutex); // Libera o mutex
  Serial.println("UI_Task: Parameter saving to NVS complete.");

 } else { // Nao conseguiu o mutex
  Serial.println("UI_Task: ERROR - Could not get configMutex to save parameters to NVS!");
  // Sinaliza erro no SEGUNDO Event Group (se houver um bit apropriado)
  // if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_MUTEX_CONFIG); // Exemplo
 }
}


/**
 * @brief Desenha a tela de operação principal no display.
 */
void drawOperationScreen() {
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(SSD1306_WHITE);

 // Protege o acesso as variaveis globais calculadas com o dataMutex
 if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) { // Tenta pegar o mutex com timeout curto

  // Exibe velocidade
  display.setCursor(0, 0);
  display.printf("Vel: %.1f km/h", velocidade_km_h); // Variavel global de calculations.h

  // Exibe SPM Media
  display.setCursor(0, 10);
  display.printf("SPM Med: %.1f", media_sementes_por_metro); // Variavel global de calculations.h

  // Exibe Vazao Inoculante
  display.setCursor(0, 20);
  display.printf("Vazao Inoc: %.1f L/ha", vazao_inoculante_por_hectare); // Variavel global de calculations.h

  // Exibe Distancia Percorrida
  display.setCursor(0, 30);
  display.printf("Dist: %.1f m", distancia_percorrida_m); // Variavel global de calculations.h

  // Exibe Area Plantada
  display.setCursor(0, 40);
  display.printf("Area: %.2f ha", area_plantada_ha); // Variavel global de calculations.h

  // Exibe status geral (Ok, Alarme Semente, Alarme Vazao, Erro)
  // Verifica bits nos Event Groups (sem precisar de mutex, EventBits_t eh atomico em arquiteturas modernas)
  EventBits_t current_system_events = xEventGroupGetBits(systemEvents);
  EventBits_t current_system_events2 = xEventGroupGetBits(systemEvents2);

  display.setCursor(0, 54); // Ultima linha
  if (current_system_events & (BIT_ALERTA_SEMENTES_GERAL | BIT_SEMENTES_FALHA_TOTAL)) {
   display.print("ALR SEMENTE");
  } else if (current_system_events & BIT_ALERTA_VAZAO_INOCULANTE) {
   display.print("ALR VAZAO");
  } else if (current_system_events2 & (BIT_ERRO_WIFI | BIT_ERRO_HTTP | BIT_ERRO_NVS | BIT_ERRO_SENSOR_SEMENTE_ISR | BIT_ERRO_UART_SECUNDARIO)) {
   display.print("ERRO SISTEMA");
  } else {
   display.print("OPERACAO OK");
  }


  xSemaphoreGive(dataMutex); // Libera o mutex
 } else {
  // Nao conseguiu o mutex, exibe uma mensagem de erro no display
  display.setCursor(0, 0);
  display.println("ERRO: Mutex");
  display.println("Data Bloqueada");
  Serial.println("UI_Task: WARNING - Could not get dataMutex for display update!");
 }

 display.display(); // Atualiza o display
}

/**
 * @brief Desenha o menu principal no display.
 */
void drawMainMenu() {
 // Implementação do desenho do menu principal (similar ao OperationScreen, mas com opcoes de menu)
 // Use currentMenuItem para destacar a opcao selecionada
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(SSD1306_WHITE);

 display.setCursor(0, 0);
 display.println("--- Menu Principal ---");

 // Opcoes do menu
 const char* menu_items[] = {"Operacao", "Calibracao", "Configuracoes", "Status", "Reset"};
 int num_items = sizeof(menu_items) / sizeof(menu_items[0]);

 for (int i = 0; i < num_items; i++) {
  display.setCursor(0, 15 + i * 10); // Ajuste a posicao vertical
  if (i == currentMenuItem) {
   display.print(">"); // Indica item selecionado
  } else {
   display.print(" ");
  }
  display.println(menu_items[i]);
 }

 display.display();
}

/**
 * @brief Desenha o menu de calibração no display.
 */
void drawCalibrationMenu() {
 // Implementação do desenho do menu de calibracao
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(SSD1306_WHITE);

 display.setCursor(0, 0);
 display.println("--- Calibracao ---");

 const char* menu_items[] = {"Cal Roda", "Cal Semente", "Cal Inoculante", "Voltar"};
 int num_items = sizeof(menu_items) / sizeof(menu_items[0]);

 for (int i = 0; i < num_items; i++) {
  display.setCursor(0, 15 + i * 10); // Ajuste a posicao vertical
  if (i == currentMenuItem) {
   display.print(">"); // Indica item selecionado
  } else {
   display.print(" ");
  }
  display.println(menu_items[i]);
 }

 display.display();
}

/**
 * @brief Desenha o menu de configurações no display.
 */
void drawSettingsMenu() {
 // Implementação do desenho do menu de configuracoes
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(SSD1306_WHITE);

 display.setCursor(0, 0);
 display.println("--- Configuracoes ---");

 const char* menu_items[] = {"SPM Min", "SPM Max", "Vazao Alvo", "Voltar"};
 int num_items = sizeof(menu_items) / sizeof(menu_items[0]);

 for (int i = 0; i < num_items; i++) {
  display.setCursor(0, 15 + i * 10); // Ajuste a posicao vertical
  if (i == currentMenuItem) {
   display.print(">"); // Indica item selecionado
  } else {
   display.print(" ");
  }
  display.println(menu_items[i]);
 }

 display.display();
}

/**
 * @brief Desenha a tela para edição de um parâmetro.
 * @param param_name Nome do parâmetro a ser exibido.
 * @param current_value Valor atual sendo editado.
 */
void drawEditParameterScreen(const char* param_name, float current_value) {
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(SSD1306_WHITE);

 display.setCursor(0, 0);
 display.println("--- Editar ---");

 display.setCursor(0, 15);
 display.println(param_name);

 display.setCursor(0, 30);
 display.setTextSize(2); // Tamanho maior para o valor
 display.printf("%.3f", current_value); // Exibe o valor com 3 casas decimais

 display.setTextSize(1); // Volta ao tamanho normal para as instrucoes
 display.setCursor(0, 50);
 display.println("UP/DOWN: Ajusta");
 display.setCursor(0, 58);
 display.println("SELECT: Salva | BACK: Cancela");


 display.display();
}


/**
 * @brief Desenha a tela atual com base no estado da UI.
 */
void drawCurrentScreen() {
 // Protege o acesso ao display com o uiMutex antes de desenhar
 if (uiMutex != NULL && xSemaphoreTake(uiMutex, pdMS_TO_TICKS(5)) == pdTRUE) { // Tenta pegar o mutex com timeout curto

  switch (currentUIState) { // ui_state_t e currentUIState vem de ui_task.h
   case STATE_OPERATION: // STATE_... vem de ui_task.h
    drawOperationScreen();
    break;
   case STATE_MENU_MAIN:
    drawMainMenu();
    break;
   case STATE_MENU_CALIBRATION:
    drawCalibrationMenu();
    break;
   case STATE_MENU_SETTINGS:
    drawSettingsMenu();
    break;
   case STATE_EDIT_CAL_RODA:
    drawEditParameterScreen("Cal Roda (m/pulso)", cal_roda_m_por_pulso); // Acessa variavel global (protegida por configMutex em processButtonInput)
    break;
   case STATE_EDIT_CAL_SEMENTE:
    drawEditParameterScreen("Cal Semente (s/m)", cal_semente_s_por_metro); // Acessa variavel global
    break;
   case STATE_EDIT_CAL_INOCULANTE:
    drawEditParameterScreen("Cal Inoc (L/1k pulso)", cal_inoculante_l_por_1000pulsos); // Acessa variavel global
    break;
   case STATE_EDIT_SETTING_SPM_MIN:
    drawEditParameterScreen("Alarme SPM Min", setting_alarme_spm_min); // Acessa variavel global
    break;
   case STATE_EDIT_SETTING_SPM_MAX:
    drawEditParameterScreen("Alarme SPM Max", setting_alarme_spm_max); // Acessa variavel global
    break;
   case STATE_EDIT_SETTING_VAZAO_ALVO:
    drawEditParameterScreen("Vazao Alvo (L/ha)", setting_vazao_alvo_lha); // Acessa variavel global
    break;
   default:
    // Estado desconhecido, volta para operacao
    currentUIState = STATE_OPERATION;
    break;
  }

  xSemaphoreGive(uiMutex); // Libera o mutex
 } else {
  // Nao conseguiu o mutex do display, apenas imprime um aviso
  Serial.println("UI_Task: WARNING - Could not get uiMutex for drawing!");
 }
}

/**
 * @brief Processa a entrada dos botões e transiciona os estados da UI.
 */
void processButtonInput() {
 // Verifica o tempo desde o ultimo pressionamento para debounce
 if ((millis() - last_button_press_time) < BUTTON_DEBOUNCE_DELAY_MS) {
  return; // Ainda dentro do periodo de debounce
 }

 bool buttonPressed = false;

 // Leitura dos botões (INPUT_PULLUP significa LOW quando pressionado)
#ifdef PIN_BTN_UP
 bool btnUpState = (digitalRead(PIN_BTN_UP) == LOW);
#else
 bool btnUpState = false;
#endif
#ifdef PIN_BTN_DOWN
 bool btnDownState = (digitalRead(PIN_BTN_DOWN) == LOW);
#else
 bool btnDownState = false;
#endif
#ifdef PIN_BTN_SELECT
 bool btnSelectState = (digitalRead(PIN_BTN_SELECT) == LOW);
#else
 bool btnSelectState = false;
#endif
#ifdef PIN_BTN_BACK
 bool btnBackState = (digitalRead(PIN_BTN_BACK) == LOW);
#else
 bool btnBackState = false;
#endif


 // Protege o acesso ao estado da UI e as variaveis globais de config/calib com o uiMutex e configMutex
 // O uiMutex protege o currentUIState e currentMenuItem
 // O configMutex protege as variaveis cal_... e setting_...
 if (uiMutex != NULL && xSemaphoreTake(uiMutex, pdMS_TO_TICKS(5)) == pdTRUE) { // Tenta pegar uiMutex com timeout curto

  switch (currentUIState) { // ui_state_t e currentUIState vem de ui_task.h
   case STATE_OPERATION: // STATE_... vem de ui_task.h
    if (btnSelectState) {
     currentUIState = STATE_MENU_MAIN; // Vai para o menu principal
     currentMenuItem = 0;       // Reseta a selecao do menu
     buttonPressed = true;
    }
    break;

   case STATE_MENU_MAIN: { // STATE_... vem de ui_task.h
    const int num_items = 5; // Operacao, Calibracao, Configuracoes, Status, Reset
    if (btnUpState) {
     currentMenuItem = (currentMenuItem - 1 + num_items) % num_items;
     buttonPressed = true;
    } else if (btnDownState) {
     currentMenuItem = (currentMenuItem + 1) % num_items;
     buttonPressed = true;
    } else if (btnSelectState) {
     // Acao baseada no item selecionado
     switch (currentMenuItem) {
      case 0: currentUIState = STATE_OPERATION; break; // Voltar para Operacao
      case 1: currentUIState = STATE_MENU_CALIBRATION; currentMenuItem = 0; break; // Ir para Calibracao
      case 2: currentUIState = STATE_MENU_SETTINGS; currentMenuItem = 0; break; // Ir para Configuracoes
      case 3: /* TODO: Ir para tela de Status */ break;
      case 4: /* TODO: Implementar Reset */ break;
     }
     buttonPressed = true;
    } else if (btnBackState) {
     currentUIState = STATE_OPERATION; // Voltar para Operacao
     buttonPressed = true;
    }
    break;
   } // Fim case STATE_MENU_MAIN

   case STATE_MENU_CALIBRATION: { // STATE_... vem de ui_task.h
    const int num_items = 4; // Cal Roda, Cal Semente, Cal Inoculante, Voltar
    if (btnUpState) {
     currentMenuItem = (currentMenuItem - 1 + num_items) % num_items;
     buttonPressed = true;
    } else if (btnDownState) {
     currentMenuItem = (currentMenuItem + 1) % num_items;
     buttonPressed = true;
    } else if (btnSelectState) {
     // Acao baseada no item selecionado
     switch (currentMenuItem) {
      case 0: currentUIState = STATE_EDIT_CAL_RODA; break;  // Editar Cal Roda
      case 1: currentUIState = STATE_EDIT_CAL_SEMENTE; break; // Editar Cal Semente
      case 2: currentUIState = STATE_EDIT_CAL_INOCULANTE; break; // Editar Cal Inoculante
      case 3: currentUIState = STATE_MENU_MAIN; currentMenuItem = 1; break; // Voltar para Menu Principal (item Calibracao)
     }
     buttonPressed = true;
    } else if (btnBackState) {
     currentUIState = STATE_MENU_MAIN; // Voltar para Menu Principal
     currentMenuItem = 1;       // Volta para a opcao Calibracao no menu anterior
     buttonPressed = true;
    }
    break;
   } // Fim case STATE_MENU_CALIBRATION

   case STATE_MENU_SETTINGS: { // STATE_... vem de ui_task.h
    const int num_items = 4; // SPM Min, SPM Max, Vazao Alvo, Voltar
    if (btnUpState) {
     currentMenuItem = (currentMenuItem - 1 + num_items) % num_items;
     buttonPressed = true;
    } else if (btnDownState) {
     currentMenuItem = (currentMenuItem + 1) % num_items;
     buttonPressed = true;
    } else if (btnSelectState) {
     // Acao baseada no item selecionado
     switch (currentMenuItem) {
      case 0: currentUIState = STATE_EDIT_SETTING_SPM_MIN; break; // Editar SPM Min
      case 1: currentUIState = STATE_EDIT_SETTING_SPM_MAX; break; // Editar SPM Max
      case 2: currentUIState = STATE_EDIT_SETTING_VAZAO_ALVO; break; // Editar Vazao Alvo
      case 3: currentUIState = STATE_MENU_MAIN; currentMenuItem = 2; break; // Voltar para Menu Principal (item Configuracoes)
     }
     buttonPressed = true;
    } else if (btnBackState) {
     currentUIState = STATE_MENU_MAIN; // Voltar para Menu Principal
     currentMenuItem = 2;       // Volta para a opcao Configuracoes no menu anterior
     buttonPressed = true;
    }
    break;
   } // Fim case STATE_MENU_SETTINGS

   case STATE_EDIT_CAL_RODA: // STATE_... vem de ui_task.h
   case STATE_EDIT_CAL_SEMENTE:
   case STATE_EDIT_CAL_INOCULANTE:
   case STATE_EDIT_SETTING_SPM_MIN:
   case STATE_EDIT_SETTING_SPM_MAX:
   case STATE_EDIT_SETTING_VAZAO_ALVO: {
    // Logica de edicao de parametro (ajustar valor, salvar, cancelar)
    // Use configMutex para acessar/modificar as variaveis cal_... e setting_...
    if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE) { // Tenta pegar configMutex

     float step = 0.1; // TODO: Definir steps diferentes para cada parametro (config.h ou array local)
     // Exemplo de steps:
     // if (currentUIState == STATE_EDIT_CAL_RODA) step = 0.001;
     // else if (currentUIState == STATE_EDIT_CAL_INOCULANTE) step = 0.1;
     // else step = 0.1; // Default

     // Ajusta o valor com UP/DOWN
     volatile float* param_to_edit = NULL; // Ponteiro deve ser para volatile float
     // Aponta para a variavel correta a ser editada com base no estado atual
     switch (currentUIState) {
      case STATE_EDIT_CAL_RODA:      param_to_edit = &cal_roda_m_por_pulso; break;
      case STATE_EDIT_CAL_SEMENTE:    param_to_edit = &cal_semente_s_por_metro; break; // Nota: Calibracao por linha pode ser mais complexa
      case STATE_EDIT_CAL_INOCULANTE:  param_to_edit = &cal_inoculante_l_por_1000pulsos; break;
      case STATE_EDIT_SETTING_SPM_MIN: param_to_edit = &setting_alarme_spm_min; break;
      case STATE_EDIT_SETTING_SPM_MAX: param_to_edit = &setting_alarme_spm_max; break;
      case STATE_EDIT_SETTING_VAZAO_ALVO: param_to_edit = &setting_vazao_alvo_lha; break;
      default: param_to_edit = NULL; break; // Nao deveria acontecer
     }

     if (param_to_edit != NULL) {
      if (btnUpState) {
       (*param_to_edit) += step;
       buttonPressed = true;
      } else if (btnDownState) {
       (*param_to_edit) -= step;
       // Opcional: Adicionar limite minimo (ex: 0)
       if ((*param_to_edit) < 0) (*param_to_edit) = 0; // Exemplo
       buttonPressed = true;
      }
     }


     xSemaphoreGive(configMutex); // Libera o mutex
    } else {
     Serial.println("UI_Task: WARNING - Could not get configMutex for parameter editing!");
     // TODO: Sinalizar erro no Event Group
    }


    // Salvar (SELECT) ou Cancelar (BACK)
    if (btnSelectState) {
     // Salva as configuracoes na NVS
     saveParametersToNVS(); // Esta funcao ja protege com configMutex
     // TODO: Voltar para o menu anterior (Calibracao ou Configuracoes)
     if (currentUIState >= STATE_EDIT_CAL_RODA && currentUIState <= STATE_EDIT_CAL_INOCULANTE) {
      currentUIState = STATE_MENU_CALIBRATION;
     } else { // Assume que eh um estado de edicao de Setting
      currentUIState = STATE_MENU_SETTINGS;
     }
     currentMenuItem = 0; // Opcional: voltar para o topo do menu anterior
     buttonPressed = true;

    } else if (btnBackState) {
     // Cancela a edicao (nao salva)
     // TODO: Voltar para o menu anterior (Calibracao ou Configuracoes)
     // Opcional: Recarregar o valor original da NVS ou da variavel global antes da edicao (se voce fez uma copia)
     loadParametersFromNVS(); // Recarrega da NVS para descartar edicoes nao salvas
     if (currentUIState >= STATE_EDIT_CAL_RODA && currentUIState <= STATE_EDIT_CAL_INOCULANTE) {
      currentUIState = STATE_MENU_CALIBRATION;
     } else { // Assume que eh um estado de edicao de Setting
      currentUIState = STATE_MENU_SETTINGS;
     }
     currentMenuItem = 0; // Opcional: voltar para o topo do menu anterior
     buttonPressed = true;
    }
    break;
   } // Fim case STATE_EDIT_...

   default:
    // Estado desconhecido, volta para operacao
    currentUIState = STATE_OPERATION;
    buttonPressed = true; // Para resetar o debounce
    break;
  }

  xSemaphoreGive(uiMutex); // Libera o uiMutex

  // Atualiza o tempo do ultimo pressionamento APENAS se um botao valido foi pressionado
  if (buttonPressed) {
   last_button_press_time = millis();
  }
 } else {
  Serial.println("UI_Task: WARNING - Could not get uiMutex for button processing!");
  // TODO: Sinalizar erro no Event Group
 }
}


// === Implementação da Tarefa FreeRTOS da UI ===
// Removido extern "C" da DEFINICAO da tarefa. Ele so fica na DECLARACAO no header.
void tarefaUI(void *pvParameters) {
  Serial.println("Task UI_Task: Started.");

  // Inicializa o display e botoes (esta funcao eh chamada em app_main, mas aqui eh o loop da tarefa)
  ui_init(); // Chamado em app_main() - MANTIDO AQUI POR ENQUANTO, SE DER ERRO REVERTER
  loadParametersFromNVS(); // Chamado em app_main() - MANTIDO AQUI POR ENQUANTO, SE DER ERRO REVERTER

  while(true) {
    // Processa a entrada dos botões (com debounce)
    processButtonInput();

    // Desenha a tela atual no display
    drawCurrentScreen();

    // Aguarda um curto periodo antes de atualizar o display/ler botoes novamente
    vTaskDelay(DISPLAY_UPDATE_INTERVAL_MS); // O intervalo de delay aqui controla a taxa de atualizacao da UI
  }

  // Esta linha teoricamente nunca sera alcancada
  vTaskDelete(NULL);
}