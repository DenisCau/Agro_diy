// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: src/control/pump_control.cpp
// Descrição: Implementação do módulo de Controle da Bomba de Inoculante.
// Gerencia o acionamento da bomba via PWM ou digital.
// ========================================================

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include <cstddef> // Incluido para ptrdiff_t e outros tipos padrao C++
#include <stddef.h> // Incluido para ptrdiff_t e outros tipos padrao C
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "control/pump_control.h" // Inclui o header deste modulo
#include "config.h" // Para defines de pinos (PIN_BOMBA_PWM, etc.), constantes de PWM

// === INCLUSAO NECESSARIA PARA O DRIVER LEDC DO ESP-IDF ===
#include "driver/ledc.h"   // Inclui o driver LEDC do ESP-IDF
// === FIM INCLUSAO NECESSARIA ===

// #include <Arduino.h> // <-- Nao é necessario para configurar e usar LEDC com driver ESP-IDF
// #include "soc/soc.h"  // <-- Nao é sempre necessario, depende de outras funcoes usadas

// === Constantes e Variáveis Internas do Módulo ===
// Parametros do PWM (para a bomba)
// Usando defines de config.h se definidos la, caso contrario, usa defines locais
#ifndef PWM_FREQ
#define PWM_FREQ    1000 // Frequencia do PWM em Hz (ajuste conforme o motor/driver)
#endif
#ifndef PWM_RESOLUTION_BITS
#define PWM_RESOLUTION_BITS 8 // Resolucao do PWM em bits (8 bits = 0-255, 10 bits = 0-1023, etc.)
#endif
#ifndef PWM_CHANNEL
#define PWM_CHANNEL   0 // Canal PWM do ESP32 a ser usado (0 a 15)
#endif

// O valor maximo do duty cycle com a resolucao definida
#define MAX_PWM_DUTY  ((1 << PWM_RESOLUTION_BITS) - 1) // Ex: para 8 bits, 2^8 - 1 = 255

// Variavel interna para armazenar o valor atual do duty cycle
volatile uint8_t current_pwm_duty_cycle = 0;

// Usar um tipo de dado apropriado para o duty cycle com base na resolucao
// Se PWM_RESOLUTION_BITS for <= 8, uint8_t é OK para a variavel interna.
// Se for > 8 (ex: 10, 12, etc), use uint16_t ou uint32_t para a variavel e nos parametros das funcoes.
// Por exemplo, se usar 10 bits, MAX_PWM_DUTY = 1023. uint8_t so vai ate 255.
// Vamos manter uint8_t por enquanto assumindo 8 bits, mas esteja ciente dessa limitacao se mudar a resolucao.


// === Implementação das Funções do Módulo de Controle da Bomba ===

/**
 * @brief Inicializa o periférico LEDC para controle PWM da bomba.
 */
void pumpControl_init() {
  Serial.println("Module Pump_Control: Initializing.");

  // TODO: Definir PIN_BOMBA_PWM em config.h - (Ja esta em config.h)

#ifdef PIN_BOMBA_PWM // Verifica se o pino da bomba esta definido

  // === Configuracao do LEDC (PWM) usando driver do ESP-IDF ===

  // Definir a resolucao do Duty Cycle como enum do LEDC
  ledc_timer_bit_t ledc_resolution;
  switch(PWM_RESOLUTION_BITS) {
    case 1: ledc_resolution = LEDC_TIMER_1_BIT; break;
    case 2: ledc_resolution = LEDC_TIMER_2_BIT; break;
    case 3: ledc_resolution = LEDC_TIMER_3_BIT; break;
    case 4: ledc_resolution = LEDC_TIMER_4_BIT; break;
    case 5: ledc_resolution = LEDC_TIMER_5_BIT; break;
    case 6: ledc_resolution = LEDC_TIMER_6_BIT; break;
    case 7: ledc_resolution = LEDC_TIMER_7_BIT; break;
    case 8: ledc_resolution = LEDC_TIMER_8_BIT; break;
    case 9: ledc_resolution = LEDC_TIMER_9_BIT; break;
    case 10: ledc_resolution = LEDC_TIMER_10_BIT; break;
    case 11: ledc_resolution = LEDC_TIMER_11_BIT; break;
    case 12: ledc_resolution = LEDC_TIMER_12_BIT; break;
    case 13: ledc_resolution = LEDC_TIMER_13_BIT; break;
    case 14: ledc_resolution = LEDC_TIMER_14_BIT; break;
    default:
      Serial.printf("Module Pump_Control: WARNING - Invalid PWM_RESOLUTION_BITS (%d). Using 8 bits.\n", PWM_RESOLUTION_BITS);
      ledc_resolution = LEDC_TIMER_8_BIT; // Default para 8 bits se o valor for invalido
      break;
  }


  // Configuracao do Timer LEDC (geralmente Timer 0 para o canal 0)
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE, // Modo de velocidade
    .duty_resolution  = ledc_resolution,   // Resolucao do Duty Cycle
    .timer_num        = LEDC_TIMER_0,      // Numero do Timer a ser usado (REORDENADO)
    .freq_hz          = PWM_FREQ,          // Frequencia do PWM (REORDENADO)
    .clk_cfg          = LEDC_AUTO_CLK      // Configuracao do clock (REORDENADO)
  };
  // Configura o timer e checa por erros.
  esp_err_t ret = ledc_timer_config(&ledc_timer); // Declaracao de 'ret' movida para aqui
  if (ret != ESP_OK) {
    Serial.printf("Module Pump_Control: ERROR configuring LEDC timer (%s)!\n", esp_err_to_name(ret));
    // TODO: Sinalizar erro no Event Group (se houver um bit para erro de hardware)
  }


  // Configuracao do Canal LEDC (geralmente Canal 0 para o Timer 0)
  ledc_channel_config_t ledc_channel = {
    .gpio_num       = PIN_BOMBA_PWM,         // Pino GPIO para o PWM da bomba
    .speed_mode     = LEDC_LOW_SPEED_MODE,   // Modo de velocidade (deve ser o mesmo do Timer)
    .channel        = LEDC_CHANNEL_0,        // Numero do Canal a ser usado (Use seu define PWM_CHANNEL se for 0)
    .intr_type      = LEDC_INTR_DISABLE,     // Tipo de interrupcao
    .timer_sel      = LEDC_TIMER_0,          // Seleciona o Timer configurado acima
    .duty           = 0,                     // Duty cycle inicial
    .hpoint         = 0,                     // Ponto inicial
        .flags          = 0                      // Inicializar campo 'flags' (resolve warning)
  };
  // Configura o canal e checa por erros
  ret = ledc_channel_config(&ledc_channel); // Reuso de 'ret'
  if (ret != ESP_OK) {
    Serial.printf("Module Pump_Control: ERROR configuring LEDC channel (%s)!\n", esp_err_to_name(ret));
    // TODO: Sinalizar erro no Event Group
  }

  // Nao necessario instalar Fade Function se for usar apenas ledc_set_duty
  // ledc_fade_func_install(0);


  // Garante que a bomba comece desligada
  // Usa a funcao do driver LEDC do ESP-IDF para setar o duty cycle
  // Use o MESMO modo de velocidade e numero de canal configurados acima
  ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Duty inicial = 0
  if (ret != ESP_OK) {
    Serial.printf("Module Pump_Control: ERROR setting initial LEDC duty (%s)!\n", esp_err_to_name(ret));
    // TODO: Sinalizar erro
  }

  // Atualiza o duty cycle para que a mudanca tenha efeito
  // Use o MESMO modo de velocidade e numero de canal
  ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  if (ret != ESP_OK) {
    Serial.printf("Module Pump_Control: ERROR updating initial LEDC duty (%s)!\n", esp_err_to_name(ret));
    // TODO: Sinalizar erro
  }

  current_pwm_duty_cycle = 0; // Zera a variavel interna

  Serial.printf("Module Pump_Control: PWM initialized on pin %d, channel %d, freq %d Hz, resolution %d bits.\n",
  PIN_BOMBA_PWM, PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION_BITS);


#else
  Serial.println("Module Pump_Control: PIN_BOMBA_PWM not defined. Skipping pump control initialization.");
  // TODO: Sinalizar erro de configuracao se este modulo eh essencial
#endif

  Serial.println("Module Pump_Control: Initialization complete.");
}

/**
 * @brief Define o valor de PWM para controlar a velocidade/vazao da bomba.
 * @param duty Duty cycle (0 a (2^PWM_RESOLUTION_BITS)-1, ajuste conforme sua configuracao LEDC).
 */
void setPumpPwm(uint8_t duty_cycle) {
  // Note: Esta funcao assume que duty_cycle é passado como uint8_t (0-255).
  // Se a resolucao do PWM for maior que 8 bits, o parametro e a variavel interna
  // 'current_pwm_duty_cycle' precisarao ser uint16_t ou uint32_t.
  // E a mapeamento de valores (se aplicavel) precisara ser ajustado para a resolucao correta.

#ifdef PIN_BOMBA_PWM
  // Garante que o duty cycle nao exceda o valor maximo configurado pela RESOLUCAO DO LEDC.
  // Se PWM_RESOLUTION_BITS <= 8, MAX_PWM_DUTY é 255 ou menos.
  // uint8_t duty_cycle ja limita a entrada a 0-255.
  // Se a resolucao LEDC for maior que 8 bits, 'duty_cycle' (uint8_t) deve ser mapeado ou convertido para o range correto (0 a MAX_PWM_DUTY).
  // Como o parametro aqui é uint8_t, assumimos que o valor de entrada é 0-255
  // e que a resolucao LEDC é 8 bits ou que o mapeamento é feito antes de chamar esta funcao
  // ou dentro dela.

  // Usar a funcao do driver LEDC do ESP-IDF para setar o duty cycle
  // Use o MESMO modo de velocidade e numero de canal configurados em pumpControl_init()
  esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle); // Use seu define PWM_CHANNEL se nao for 0
  if (ret != ESP_OK) {
    Serial.printf("Module Pump_Control: ERROR setting LEDC duty (%s)!\n", esp_err_to_name(ret));
    // TODO: Sinalizar erro
    // Nao tentar atualizar duty se set_duty falhou
  } else {
    // Atualiza o duty cycle para que a mudanca tenha efeito
    // Use o MESMO modo de velocidade e numero de canal
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); // Use seu define PWM_CHANNEL
    if (ret != ESP_OK) {
      Serial.printf("Module Pump_Control: ERROR updating LEDC duty (%s)!\n", esp_err_to_name(ret));
      // TODO: Sinalizar erro
    } else {
      current_pwm_duty_cycle = duty_cycle; // Atualiza a variavel interna SOMENTE se o duty foi atualizado com sucesso
    }
  }

  // Serial.printf("Module Pump_Control: Set PWM duty cycle to %d\n", duty_cycle); // Debug (pode ser verboso)

#else
  // Se o pino nao esta definido, apenas imprime um aviso (ou sinaliza erro)
  // Serial.println("Module Pump_Control: WARNING - setPumpPwm called but PIN_BOMBA_PWM not defined.");
#endif
}

// Implementacao da funcao getter
// Removido extern "C" da DEFINICAO. Ele so fica na DECLARACAO no header.
uint8_t getPumpPwmDutyCycle() {
  return current_pwm_duty_cycle;
}


// Implementação da Tarefa FreeRTOS do Módulo de Controle da Bomba
// Esta tarefa eh um placeholder caso a logica de controle PID ou loop de vazao
// precise rodar em uma tarefa dedicada. Se a tarefaCalculos ou outra tarefa
// chama setPumpPwm diretamente com o valor calculado, esta tarefa pode nao ser usada.
// Removido extern "C" da DEFINICAO da tarefa. Ele so fica na DECLARACAO no header.
void tarefaPumpControl(void *pvParameters) {
  Serial.println("Task Pump_Control: Started.");

  // Exemplo de loop de controle (se a logica PID estiver aqui)
  while (true) {
    // TODO: Implementar logica de leitura de vazao medida (talvez via variavel global),
    // calculo de erro, PID, e chamada a setPumpPwm().
    // Exemplo:
    // float vazao_medida = vazao_inoculante_por_hectare; // Ler variavel global (proteger com mutex!)
    // float vazao_alvo = ...; // Ler vazao alvo (proteger com mutex!)
    // float erro = vazao_alvo - vazao_medida;
    // float output_pid = calcular_pid(erro); // Funcao PID (pode estar em outro modulo)
    // uint8_t duty_cycle = (uint8_t) map(output_pid, MIN_PID_OUT, MAX_PID_OUT, 0, MAX_PWM_DUTY); // Mapear PID para PWM
    // setPumpPwm(duty_cycle);

    // Se a logica de controle estiver em outro lugar e apenas setPumpPwm for chamado,
    // esta tarefa pode apenas ter um delay ou ser removida.
    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeno delay de controle/actualizacao
  }

  // Esta linha teoricamente nunca sera alcancada
  vTaskDelete(NULL);
}