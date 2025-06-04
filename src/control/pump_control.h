// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: control/pump_control.h
// Descrição: Definições e protótipos para o módulo de Controle da Bomba de Inoculante.
// Gerencia o acionamento da bomba via PWM ou digital.
// ========================================================

#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Necessario para TaskFunction_t
#include "freertos/semphr.h" // Necessario para SemaphoreHandle_t
#include "freertos/event_groups.h" // Necessario para EventGroupHandle_t

#include <Arduino.h>  // Para tipos basicos (uint8_t, etc.), pinMode, digitalWrite
#include "config.h"  // Para defines de pinos (PIN_BOMBA_PWM, etc.), constantes de PWM

// === Variáveis Globais do Módulo (extern) ===
// Nenhuma variavel global precisa ser acessada externamente no momento.
// O duty cycle atual eh mantido internamente no .cpp e acessado via getter.


// === Protótipos das Funções do Módulo de Controle da Bomba ===

/**
* @brief Inicializa o hardware da bomba (pinos, PWM se aplicavel).
*/
void pumpControl_init();

/**
* @brief Define o valor de PWM para controlar a velocidade/vazao da bomba.
* Assumindo controle via PWM. Se for ON/OFF digital, use 0 ou 255/MAX_DUTY.
* @param duty_cycle Valor do ciclo de trabalho PWM (0 a MAX_PWM_DUTY).
*/
void setPumpPwm(uint8_t duty_cycle);

/**
 * @brief Obtem o valor atual do ciclo de trabalho PWM da bomba.
 * @return O valor atual do ciclo de trabalho PWM (0 a MAX_PWM_DUTY).
 */
uint8_t getPumpPwmDutyCycle();


// === Protótipo da Tarefa FreeRTOS do Módulo de Controle da Bomba ===
// Esta tarefa seria responsavel por ajustar o PWM da bomba
// com base na vazao alvo e na vazao medida (logica PID ou similar).
// O PID provavelmente estaria no modulo de calculos ou controle mais alto nivel.
// Esta tarefa apenas seria o "executor" do valor de PWM calculado.
// Se o ajuste do PWM for feito diretamente na tarefa de calculos/controle principal,
// esta tarefa pode nao ser necessaria. Por enquanto, mantemos o prototipo
// caso a logica de controle PID resida aqui ou precise de uma tarefa dedicada.
extern "C" { // Adicionado extern "C" para compatibilidade com C/FreeRTOS
    void tarefaPumpControl(void *pvParameters);
}


#endif // PUMP_CONTROL_H