// ========================================================
// PROJETO PLANTADEIRA - ESP32 PRINCIPAL (MESTRE)
// Arquivo: communication/rpi_comm.cpp
// Descrição: Implementação do módulo de comunicação WiFi e Servidor HTTP com a Raspberry Pi.
// Permite que a Raspberry Pi (ou outra IHM via rede) envie configuracoes
// para o ESP32 Principal via HTTP. Inclui funcionalidade de cliente HTTP para
// enviar dados para a RPi.
// ========================================================

// === Includes Necessários ===
// Inclui headers do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "communication/rpi_comm.h" // Inclui o header deste modulo
#include "config.h"    // Para defines de WiFi (SSID, Password) e RPI_HTTP_SERVER_URL
#include "main.h"    // Para handles globais (configMutex, systemEvents2), defines de bits (BIT_ERRO_WIFI, BIT_ERRO_HTTP)
#include "nvs_rw/nvs_rw.h" // Para salvar configuracoes na NVS // Corrigido espaco
#include "ui/ui_task.h"  // Para acessar a estrutura PlantParameters_t e saveParametersToNVS() // Corrigido espaco

#include <Arduino.h>   // Para Serial.println, delay, etc.
#include <WiFi.h>    // Biblioteca WiFi para ESP32
#include <WebServer.h>  // Biblioteca para criar um Servidor Web
#include <HTTPClient.h>  // Biblioteca para fazer requisições HTTP (Cliente)


// Incluir biblioteca para parsing JSON (se os dados vierem em JSON)
// Instale via Library Manager no PlatformIO: ArduinoJson
#include <ArduinoJson.h>


// === Configuração da Rede WiFi ===
// TODO: Definir SSID e Password da rede WiFi em config.h ou carregar da NVS
// Por enquanto, definimos aqui para teste:
#ifndef WIFI_SSID
#define WIFI_SSID "Casa" // <<< SUBSTITUA pelo SSID da sua rede
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "12345678" // <<< SUBSTITUA pela senha da sua rede
#endif

// Porta HTTP para o servidor web neste ESP32
#define HTTP_SERVER_PORT 80 // RENOMEADO para evitar confusao com porta da RPi

// Endereco base do servidor HTTP da Raspberry Pi (definido em config.h)
// #define RPI_HTTP_SERVER_URL "http://192.168.1.100:5000/" // Definido em config.h


// === Variáveis Globais do Módulo RPi_Comm ===
// Objeto servidor web (para receber configs da RPi) - Agora local ao modulo
WebServer server(HTTP_SERVER_PORT); // Usando a porta renomeada


// === Protótipos de Funções Internas ===
// Funções para lidar com as requisições HTTP (Servidor)
void handleRoot();    // Handler para a raiz "/"
void handleSaveConfig();  // Handler para salvar configuracoes
void handleNotFound();   // Handler para paginas nao encontradas


// === Implementação das Funções do Módulo RPi_Comm ===

// Inicializa a comunicação WiFi e o Servidor HTTP.
// Esta função agora é chamada DENTRO da tarefa dedicada.
void rpi_comm_init() {
 Serial.println("Module RPi_Comm: Initializing.");

 // === Configuração e Conexão WiFi (Modo Station) ===
 Serial.printf("RPi_Comm: Connecting to WiFi '%s'...\n", WIFI_SSID);
 WiFi.mode(WIFI_STA); // Configura como Station (para conectar a um AP existente)
 WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Usando defines de config.h ou locais

 // Espera pela conexao WiFi
 unsigned long wifi_connect_timeout = millis();
 const unsigned long WIFI_TIMEOUT_MS = 20000; // Timeout de 20 segundos para conectar

 while (WiFi.status() != WL_CONNECTED && millis() - wifi_connect_timeout < WIFI_TIMEOUT_MS) {
  vTaskDelay(pdMS_TO_TICKS(500)); // Use vTaskDelay em tarefas FreeRTOS
  Serial.print(".");
 }

 if (WiFi.status() == WL_CONNECTED) {
  Serial.println("\nRPi_Comm: WiFi connected!");
  Serial.printf("RPi_Comm: IP Address: %s\n", WiFi.localIP().toString().c_str());

  // === Configuração do Servidor HTTP (para receber da RPi) ===
  Serial.println("RPi_Comm: Starting HTTP Server...");

  // Define os handlers para as requisições HTTP
  server.on("/", handleRoot); // Handler para a raiz
  server.on("/config/save", HTTP_POST, handleSaveConfig); // Handler para salvar config (espera POST)
  server.onNotFound(handleNotFound); // Handler para paginas nao encontradas

  // Inicia o servidor
  server.begin();
  Serial.printf("RPi_Comm: HTTP server started on port %d\n", HTTP_SERVER_PORT);

 } else {
  Serial.println("\nRPi_Comm: ERROR - WiFi connection failed!");
  // Sinaliza erro no SEGUNDO Event Group
  if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_WIFI);
  // TODO: Implementar logica de reconexao ou modo AP de fallback
 }

 Serial.println("Module RPi_Comm: Initialization complete.");
}

// Handler para a raiz "/"
void handleRoot() {
 server.send(200, "text/plain", "Plantadeira Principal - HTTP Server (RPi Comm)"); // Ajustado mensagem
}

// Handler para salvar configuracoes (endpoint /config/save)
void handleSaveConfig() {
 Serial.println("RPi_Comm: Received POST request on /config/save");

 // Verifica se a requisição é POST e tem corpo
 if (server.method() == HTTP_POST && server.hasArg("plain")) {
  String postBody = server.arg("plain");
  Serial.printf("RPi_Comm: Received body: %s\n", postBody.c_str());

  // Implementar parsing do corpo da requisição (assumindo JSON)
  const size_t JSON_DOC_SIZE_RX = 512; // Tamanho para o JSON de entrada (ajuste conforme necessario)
  StaticJsonDocument<JSON_DOC_SIZE_RX> doc; // Corrigido: Adicionado argumento de template
  DeserializationError error = deserializeJson(doc, postBody);

  if (error) {
   Serial.printf("RPi_Comm: JSON deserialization failed: %s\n", error.c_str());
   server.send(400, "text/plain", "Invalid JSON format");
   // Sinaliza erro no SEGUNDO Event Group
   if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_HTTP); // Usar BIT_ERRO_HTTP para erros no servidor
   return;
  }

  // Extrair os valores dos parametros do JSON e atualizar as variaveis globais
  // Proteja o acesso as variaveis globais com o configMutex!
  if (configMutex != NULL && xSemaphoreTake(configMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // Tenta pegar o mutex com timeout

   // As variaveis de configuracao/calibracao sao declaradas em ui/ui_task.h
   // Para acessa-las aqui, elas devem ser declaradas como 'extern volatile' em ui/ui_task.h,
   // o que ja foi feito no arquivo que voce enviou.

   if (doc.containsKey("cal_roda_m_por_pulso")) {
    cal_roda_m_por_pulso = doc["cal_roda_m_por_pulso"].as<float>();
    Serial.printf("RPi_Comm: Updated cal_roda_m_por_pulso to %.3f\n", cal_roda_m_por_pulso);
   }
   // Calibracao semente_s_por_metro provavelmente eh uma variavel local da UI/calibracao, nao global
   // if (doc.containsKey("cal_semente_s_por_metro")) {
   //  cal_semente_s_por_metro = doc["cal_semente_s_por_metro"].as<float>();
   //  Serial.printf("RPi_Comm: Updated cal_semente_s_por_metro to %.3f\n", cal_semente_s_por_metro);
   // }
      if (doc.containsKey("cal_inoculante_l_por_1000pulsos")) {
        cal_inoculante_l_por_1000pulsos = doc["cal_inoculante_l_por_1000pulsos"].as<float>();
        Serial.printf("RPi_Comm: Updated cal_inoculante_l_por_1000pulsos to %.3f\n", cal_inoculante_l_por_1000pulsos);
      }
      if (doc.containsKey("setting_alarme_spm_min")) {
        setting_alarme_spm_min = doc["setting_alarme_spm_min"].as<float>();
        Serial.printf("RPi_Comm: Updated setting_alarme_spm_min to %.3f\n", setting_alarme_spm_min);
      }
      if (doc.containsKey("setting_alarme_spm_max")) {
        setting_alarme_spm_max = doc["setting_alarme_spm_max"].as<float>();
        Serial.printf("RPi_Comm: Updated setting_alarme_spm_max to %.3f\n", setting_alarme_spm_max);
      }
       if (doc.containsKey("setting_vazao_alvo_lha")) {
        setting_vazao_alvo_lha = doc["setting_vazao_alvo_lha"].as<float>();
        Serial.printf("RPi_Comm: Updated setting_vazao_alvo_lha to %.3f\n", setting_vazao_alvo_lha);
      }
   // TODO: Adicionar logica para ler outros parametros do JSON e atualizar as variaveis globais (settings, largura plantadeira, etc.)

   xSemaphoreGive(configMutex); // Libera o mutex
   Serial.println("RPi_Comm: Global parameters updated from HTTP.");

   // Salva os parametros atualizados na NVS
   // A funcao saveParametersToNVS ja protege o acesso com configMutex internamente
   saveParametersToNVS(); // Chama a funcao da UI/NVS para salvar

   server.send(200, "text/plain", "Configuration saved successfully");
  } else {
   Serial.println("RPi_Comm: ERROR - Could not get configMutex to update parameters from HTTP!");
   server.send(500, "text/plain", "Internal Server Error (Mutex)");
   // Sinaliza erro no SEGUNDO Event Group
   if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_HTTP); // Usar BIT_ERRO_HTTP para erros no servidor
  }

 } else {
  // Método não permitido ou corpo da requisição vazio
  server.send(405, "text/plain", "Method Not Allowed or Empty Body"); // 405 Method Not Allowed
 }
}

// Handler para paginas nao encontradas
void handleNotFound() {
 server.send(404, "text/plain", "Not Found");
}


// === Implementação da Função de Envio de Dados (Cliente HTTP) ===

/**
* @brief Envia dados (em formato JSON) para um endpoint específico na Raspberry Pi via HTTP POST.
* @param endpoint O caminho do endpoint na RPi (ex: "/telemetria", "/alarme"). Deve incluir a barra inicial.
* @param json_data Uma string contendo os dados em formato JSON.
* @return true se o envio HTTP foi bem-sucedido (código 2xx), false caso contrário.
*/
bool rpi_comm_send_data(const char* endpoint, const String& json_data) {
 // TODO: Obter RPI_HTTP_SERVER_URL de config.h
 #ifndef RPI_HTTP_SERVER_URL
  #error "RPI_HTTP_SERVER_URL not defined in config.h"
 #endif


 if (WiFi.status() != WL_CONNECTED) {
  Serial.println("RPi_Comm: Cannot send data, WiFi not connected.");
  // Sinaliza erro no SEGUNDO Event Group
  if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_WIFI); // Pode ser um erro de WiFi
  return false;
 }

 // Constrói a URL completa
 String url = RPI_HTTP_SERVER_URL;
 url += endpoint;

 HTTPClient http; // Cria um objeto HTTPClient

 Serial.printf("RPi_Comm: Sending POST request to %s with data: %s\n", url.c_str(), json_data.c_str());

 // Inicia a conexao HTTP
 http.begin(url);

 // Define o cabeçalho Content-Type como application/json
 http.addHeader("Content-Type", "application/json");

 // Envia a requisição POST com os dados JSON
 int httpResponseCode = http.POST(json_data);

 bool success = false;
 if (httpResponseCode > 0) {
  Serial.printf("RPi_Comm: HTTP POST response code: %d\n", httpResponseCode);
  // String response = http.getString(); // Opcional: ler a resposta do servidor
  // Serial.println(response);

  // Considera sucesso se o código de resposta for 2xx
  if (httpResponseCode >= 200 && httpResponseCode < 300) {
   success = true;
   // Limpa o bit de erro HTTP se a comunicação foi bem-sucedida
   if (systemEvents2 != NULL) xEventGroupClearBits(systemEvents2, BIT_ERRO_HTTP);
  } else {
   // Sinaliza erro no SEGUNDO Event Group para outros códigos de resposta não 2xx
   if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_HTTP);
  }

 } else {
  Serial.printf("RPi_Comm: HTTP POST request failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  // Sinaliza erro no SEGUNDO Event Group para falhas na requisição
  if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_HTTP);
 }

 http.end(); // Fecha a conexão

 return success;
}


// === Implementação da Tarefa FreeRTOS RPi Comm ===
// Esta tarefa gerencia a conexao WiFi e o servidor HTTP.
// Removido extern "C" da DEFINICAO da tarefa. Ele so fica na DECLARACAO no header.
void tarefaComunicacaoRPi(void *pvParameters) {
    Serial.println("Task RPi_Comm: Started.");

    // Inicializa WiFi e o servidor HTTP dentro da tarefa
    rpi_comm_init();

    // Loop principal da tarefa - Mantem o servidor HTTP rodando
    while (true) {
        // O WebServer.handleClient() processa as requisicoes recebidas.
        // Nao bloqueia, entao precisamos de um pequeno delay.
        server.handleClient();

        // Verificar status do WiFi periodicamente e tentar reconectar se desconectado
        if (WiFi.status() != WL_CONNECTED) {
             Serial.println("RPi_Comm Task: WiFi disconnected, attempting to reconnect...");
             // Sinaliza erro WiFi
             if (systemEvents2 != NULL) xEventGroupSetBits(systemEvents2, BIT_ERRO_WIFI);
             WiFi.reconnect(); // Tenta reconectar
             // O loop continuará tentando handleClient, que falhará, mas a reconexao acontece em background
        } else {
             // Limpa o bit de erro WiFi se a conexão estiver OK
             if (systemEvents2 != NULL) xEventGroupClearBits(systemEvents2, BIT_ERRO_WIFI);
        }


        // Pequeno delay para nao saturar o loop e permitir que outras tarefas rodem
        vTaskDelay(pdMS_TO_TICKS(5)); // Ajuste conforme a necessidade de resposta do servidor
    }

    // Esta linha teoricamente nunca será alcançada
    vTaskDelete(NULL);
}