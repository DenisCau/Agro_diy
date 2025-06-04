// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: main.cpp
// Descrição: Código principal (ponto de entrada) para o ESP32 Principal.
// Configura o sistema, inicializa a NVS e cria as tarefas FreeRTOS.
// Utiliza o framework ESP-IDF com o componente Arduino.
// ========================================================

// === Headers de Projeto e Frameworks ===
#include "main.h"  // Includes globais, mutexes, event groups e FreeRTOS
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include <Arduino.h>  // Mantido fora de extern "C"

// === Headers de Módulos do Projeto ===
#include "nvs_rw/nvs_rw.h"
#include "communication/rpi_comm.h"
#include "communication/uart_comm.h"
#include "sensors/pulse_sensors.h"
#include "control/pump_control.h"
#include "signaling/signaling.h"
#include "ui/ui_task.h"
#include "calculations/calculations.h"

// === Handles Globais (Definidos aqui) ===
EventGroupHandle_t systemEvents = NULL;
EventGroupHandle_t systemEvents2 = NULL;
SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t configMutex = NULL;
SemaphoreHandle_t uiMutex = NULL;

// === app_main ===
extern "C" void app_main() {
    // Inicializa Serial após initArduino()
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Cria mutexes
    dataMutex = xSemaphoreCreateMutex();
    configMutex = xSemaphoreCreateMutex();
    uiMutex = xSemaphoreCreateMutex();
    if (!dataMutex || !configMutex || !uiMutex) {
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Cria event groups
    systemEvents = xEventGroupCreate();
    systemEvents2 = xEventGroupCreate();
    if (!systemEvents || !systemEvents2) {
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Inicializa NVS do projeto
    if (nvs_init_namespace() != ESP_OK) {
        // Aviso já é impresso pela função
    }

    // Inicializa núcleo Arduino (Serial etc.)
    initArduino();
    Serial.begin(115200);
    Serial.println("--- ESP32 Principal: Sistema Iniciando ---");

    // Inicializações de hardware
    pulseSensors_init();
    pumpControl_init();
    signaling_init();

    // Criação de tarefas
    xTaskCreatePinnedToCore(tarefaCalculos, "Calculos", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(tarefaComunicacaoUART, "UART_Comm", 3072, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(tarefaPumpControl, "Pump_Control", 2048, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(tarefaSinalizacao, "Sinalizacao", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(tarefaUI, "UI_Task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(tarefaComunicacaoRPi, "RPi_Comm_Task", 6144, NULL, 4, NULL, 1);

    Serial.println("--- Tarefas FreeRTOS criadas ---");
    Serial.println("Sistema pronto.");
}

// === setup e loop padrão do Arduino ===
void setup() {
    // Vazio — se precisar colocar inicializações que dependem do Arduino, pode usar aqui
}

void loop() {
    // Se estiver vazio, não deixar consumir CPU
    vTaskDelay(1);
}
