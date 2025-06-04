// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: signaling/signaling.cpp
// Descrição: Implementação do modulo de Sinalização.
// Monitora os Event Groups do sistema e aciona sinalizacoes visuais e sonoras.
// ========================================================

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "signaling/signaling.h" // Inclui o header deste modulo
#include "main.h"  // Para SystemEventBits_t, SystemEventBits2_t, systemEvents, systemEvents2, defines de bits
#include "config.h" // Para NUM_LINHAS e definicoes de pinos de sinalizacao (se movidos para ca)

#include <Arduino.h> // Para Serial.print, pinMode, digitalWrite


// === Definições e Constantes Internas ===
// TODO: Definir pinos para LEDs e Buzzer em config.h
#ifndef PIN_LED_STATUS
 #define PIN_LED_STATUS   2 // Exemplo: LED de status geral
#endif
#ifndef PIN_LED_ALARME_GERAL
 #define PIN_LED_ALARME_GERAL 4 // Exemplo: LED para alarmes gerais (sementes, vazao, etc.)
#endif
// TODO: Definir pinos para LEDs por linha (se houver)
// TODO: Definir pino para Buzzer

// Tempo de piscada rapida para alarmes criticos (ms)
#define BLINK_FAST_MS  100
// Tempo de piscada lenta para avisos (ms)
#define BLINK_SLOW_MS  500
// Duracao do toque sonoro para alarmes (ms)
#define BUZZ_DURATION_MS 200
// Intervalo entre toques sonoros (ms)
#define BUZZ_INTERVAL_MS 500


// === Variáveis Internas do Módulo ===
// Nenhuma variavel interna precisa de acesso externo no momento.


// === Implementação das Funções do Módulo de Sinalização ===

/**
 * @brief Inicializa os pinos e perifericos relacionados a sinalizacao (LEDs, Buzzer, etc.).
 */
void signaling_init() {
 Serial.println("Module Signaling: Initializing.");

 // Configura os pinos de sinalizacao como OUTPUT
#ifdef PIN_LED_STATUS
 pinMode(PIN_LED_STATUS, OUTPUT);
 digitalWrite(PIN_LED_STATUS, LOW); // Comeca desligado
#endif
#ifdef PIN_LED_ALARME_GERAL
 pinMode(PIN_LED_ALARME_GERAL, OUTPUT);
 digitalWrite(PIN_LED_ALARME_GERAL, LOW); // Comeca desligado
#endif
 // TODO: Configurar pinos de LEDs por linha e Buzzer

 Serial.println("Module Signaling: Initialization complete.");
}


// === Implementação da Tarefa FreeRTOS de Sinalização ===
// Removido extern "C" da DEFINICAO da tarefa. Ele so fica na DECLARACAO no header.
void tarefaSinalizacao(void *pvParameters) {
  Serial.println("Task Sinalizacao: Started.");

  // Variaveis para controlar o estado da sinalizacao
  bool led_alarme_on = false;
  uint64_t last_blink_time = esp_timer_get_time(); // Usa esp_timer para precisao
  // Removido last_buzz_time pois nao estava sendo usado e causava warning
  // uint64_t last_buzz_time = esp_timer_get_time();

  while (true) {
    // Aguarda por eventos nos Event Groups (bloqueia a tarefa ate que algum bit interesse)
    // Espera por qualquer bit nos dois Event Groups.
    // O timeout eh curto (ex: 100ms) para permitir que a tarefa "tick" periodicamente
    // mesmo que nao haja eventos, para manter a logica de piscada/buzzer por tempo.
    // Ou pode esperar indefinidamente (portMAX_DELAY) e a logica de piscada
    // seria baseada apenas nos bits setados no Event Group, nao no loop continuo.
    // Optamos por um timeout curto para o loop rodar periodicamente.

    EventBits_t currentBits1 = 0; // Inicializa com 0
    if (systemEvents != NULL) {
      // Espera por QUALQUER bit setado em systemEvents
      currentBits1 = xEventGroupWaitBits(
        systemEvents,         // O Event Group a verificar
        pdTRUE,           // WAIT_FOR_ANY_BIT - Espera que qualquer bit de interesse esteja setado
        pdFALSE,          // xClearBitsBeforeReturn - Nao limpa os bits ao retornar
        pdFALSE,          // xWaitForAllBits - Espera por TODOS os bits? Nao.
        pdMS_TO_TICKS(50)      // Tempo de espera (timeout em ticks). Ajuste conforme a necessidade de resposta.
      );
    }

    // Verifica o SEGUNDO Event Group - Nao precisa esperar, apenas ler o estado atual
    EventBits_t currentBits2 = 0; // Inicializa com 0
    if (systemEvents2 != NULL) {
      currentBits2 = xEventGroupGetBits(systemEvents2); // Apenas le os bits atuais
    }

    // --- Logica de Sinalizacao ---

    // Status geral do sistema (Ex: LED piscando rapido = erro, piscando lento = aviso, solido = ok)
    // Vamos manter simples por enquanto: LED de Status geral aceso se tudo OK, piscando em caso de algum erro.
#ifdef PIN_LED_STATUS
    if (currentBits1 == 0 && currentBits2 == 0) {
      // Nenhum evento/alarme ativo - LED de status solido ON (ou OFF, defina seu padrao)
      digitalWrite(PIN_LED_STATUS, HIGH); // Exemplo: HIGH = OK
    } else {
      // Algum evento/alarme ativo - LED de status piscando (ou OFF, defina seu padrao de ERRO/AVISO)
      // digitalWrite(PIN_LED_STATUS, LOW); // Exemplo: LOW = ERROR
      // Pisca o LED de status em caso de qualquer erro/alarme
      uint64_t current_time = esp_timer_get_time();
      uint32_t blink_interval = BLINK_SLOW_MS; // Ajuste para piscar lento ou rapido dependendo do tipo de evento
      // Exemplo: if (currentBits2 & BIT_ERRO_WIFI) blink_interval = BLINK_FAST_MS;

      if ((current_time - last_blink_time) / 1000 >= blink_interval) {
        led_alarme_on = !led_alarme_on; // Reutilizando led_alarme_on para o LED de Status
        digitalWrite(PIN_LED_STATUS, led_alarme_on ? HIGH : LOW);
        last_blink_time = current_time;
      }
    }
#endif

    // Sinalizacao de Alarme Geral de Sementes (usando PIN_LED_ALARME_GERAL)
    // Verifica o bit BIT_ALERTA_SEMENTES_GERAL ou BIT_SEMENTES_FALHA_TOTAL no systemEvents
#ifdef PIN_LED_ALARME_GERAL
    // Usa uma logica diferente para o LED ALARME GERAL, que pisca somente se houver esse alarme
    if ((currentBits1 & BIT_ALERTA_SEMENTES_GERAL) || (currentBits1 & BIT_SEMENTES_FALHA_TOTAL)) {
      // Pisca o LED de alarme geral rapido
      uint64_t current_time = esp_timer_get_time();
      if ((current_time - last_blink_time) / 1000 >= BLINK_FAST_MS) { // Usa last_blink_time do LED Status, pode causar conflito se os intervalos forem muito diferentes
        // Melhor usar outra variavel de tempo para o LED Alarme Geral
        static uint64_t last_alarme_blink_time = 0; // Variavel estatica local para este LED
        static bool alarme_led_on = false;

        if ((current_time - last_alarme_blink_time) / 1000 >= BLINK_FAST_MS) {
          alarme_led_on = !alarme_led_on;
          digitalWrite(PIN_LED_ALARME_GERAL, alarme_led_on ? HIGH : LOW);
          last_alarme_blink_time = current_time;
        }
      }


      // Ativa o Buzzer (se definido) - Exemplo: toca um bip curto periodicamente
#ifdef PIN_BUZZER // TODO: Definir PIN_BUZZER em config.h
      static uint64_t last_buzz_time = 0; // Variavel estatica local para o Buzzer
      uint64_t current_time_buzz = esp_timer_get_time();
      if ((current_time_buzz - last_buzz_time) / 1000 >= (BUZZ_DURATION_MS + BUZZ_INTERVAL_MS)) {
        digitalWrite(PIN_BUZZER, HIGH); // Liga Buzzer
        vTaskDelay(pdMS_TO_TICKS(BUZZ_DURATION_MS)); // Mantem ligado pela duracao
        digitalWrite(PIN_BUZZER, LOW); // Desliga Buzzer
        last_buzz_time = current_time_buzz; // Atualiza o tempo base para o proximo bip
      }
#endif

    } else {
      // Nao ha alarme geral de sementes - Desliga o LED de alarme geral
      digitalWrite(PIN_LED_ALARME_GERAL, LOW);
      // Desliga Buzzer se estava ativo
#ifdef PIN_BUZZER
      digitalWrite(PIN_BUZZER, LOW);
#endif
    }
#endif


    // TODO: Implementar logica para outros bits de evento (alarmes por linha, erros de hardware/comunicacao, etc.)
    // Por exemplo, verificar BIT_ERRO_WIFI e BIT_ERRO_HTTP em currentBits2 para piscar o LED de status de outra forma,
    // ou acionar LEDs de linha especificos verificando BIT_SEMENTES_LINHA_i_... em currentBits1.

    // Pequeno delay para nao sobrecarregar o loop (mesmo com xEventGroupWaitBits com timeout)
    vTaskDelay(pdMS_TO_TICKS(10)); // Ajuste conforme necessario
  }

  // Esta linha teoricamente nunca sera alcancada
  vTaskDelete(NULL);
}