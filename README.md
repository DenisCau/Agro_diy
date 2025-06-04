# Agro_diy
Automação de Plantadeira
# Projeto de Monitoramento e Controle com Raspberry Pi e ESP32

## Visão Geral

Este repositório contém o código e a documentação para um sistema de monitoramento e controle distribuído. O sistema utiliza uma Raspberry Pi Zero 2 W como unidade central para coletar dados de sensores provenientes de múltiplos microcontroladores ESP32, bem como de sensores diretamente conectados à própria Raspberry Pi. Uma Interface Homem-Máquina (IHM) é implementada na Raspberry Pi para visualização e interação com os dados coletados.

## Componentes Principais

1.  **Microcontroladores ESP32 (x2):**
    * Responsáveis pela leitura de dados de sensores diversos (ex: temperatura, umidade, etc.).
    * Enviam os dados coletados para a Raspberry Pi via rede.

2.  **Raspberry Pi Zero 2 W:**
    * **Hub Central:** Recebe dados dos ESP32.
    * **Sensores Locais:** Coleta dados de sensores conectados diretamente aos seus GPIOs, I2C, SPI, etc.
    * **Processamento de Dados:** Pode realizar processamento ou agregação dos dados recebidos.
    * **Servidor da IHM:** Hospeda e executa a Interface Homem-Máquina.

3.  **Interface Homem-Máquina (IHM):**
    * Permite a visualização em tempo real (ou quase real) dos dados dos sensores.
    * Pode oferecer funcionalidades de controle ou configuração do sistema (a ser definido).

## Arquitetura do Sistema (Proposta)

* **Comunicação ESP32 -> Raspberry Pi:**
    * Os ESP32 publicam os dados dos sensores em tópicos MQTT.
    * A Raspberry Pi executa um broker MQTT (ex: Mosquitto) para receber essas publicações e um cliente MQTT para processá-las.
* **Leitura de Sensores na Raspberry Pi:**
    * Scripts Python utilizam bibliotecas apropriadas (ex: `RPi.GPIO`, `smbus2`, bibliotecas específicas de sensores) para ler dados de sensores conectados localmente.
* **Interface Homem-Máquina (IHM):**
    * **Opção 1 (Web):** Um backend Python (ex: Flask ou Django) na Raspberry Pi serve uma aplicação web (HTML, CSS, JavaScript) que pode ser acessada por um navegador na própria Pi (em modo kiosk) ou em outros dispositivos na rede.
    * **Opção 2 (GUI Nativa):** Uma aplicação desktop desenvolvida com bibliotecas como PyQt, Kivy ou Tkinter rodando diretamente na Raspberry Pi.
    * **Opção 3 (Node-RED):** Um dashboard criado com Node-RED para prototipagem rápida e visualização.
* **Armazenamento de Dados (Opcional):**
    * Os dados podem ser armazenados em arquivos (CSV), bancos de dados SQLite ou bancos de dados de séries temporais (InfluxDB) para histórico e análises futuras.

## Tecnologias Utilizadas (Potenciais)

* **Hardware:**
    * Raspberry Pi Zero 2 W
    * ESP32 (x2)
    * Diversos sensores (a serem especificados)
* **Software & Protocolos:**
    * **Raspberry Pi OS**
    * **Python:** Linguagem principal para backend, scripts de sensores e IHM.
    * **MQTT:** Protocolo de mensagens para comunicação IoT.
        * Broker: Mosquitto
        * Cliente Python: Paho-MQTT
    * **Frameworks Web (para IHM):**
        * Flask / Django (Python Backend)
        * HTML, CSS, JavaScript (Frontend)
    * **Bibliotecas GUI (para IHM):**
        * PyQt / Kivy / Tkinter
    * **Node-RED** (para prototipagem de IHM e fluxos de dados)
    * **Bibliotecas de Sensores:** `RPi.GPIO`, `gpiozero`, `Adafruit_CircuitPython` (ou equivalentes), etc.
* **Outros:**
    * Git & GitHub (Controle de versão)
    * SSH, VNC (Acesso remoto à Raspberry Pi)

## Como Configurar e Executar

*(Esta seção será preenchida com as instruções específicas de instalação de dependências, configuração dos dispositivos e como iniciar cada componente do projeto).*

1.  **Configuração dos ESP32:**
    * [Link para o diretório/documentação dos ESP32]
2.  **Configuração da Raspberry Pi:**
    * [Link para o diretório/documentação da Raspberry Pi]
3.  **Execução da IHM:**
    * [Instruções para iniciar a IHM]

## Próximos Passos / Funcionalidades Futuras

* [ ] Implementação de [Funcionalidade X]
* [ ] Adição de suporte para [Sensor Y]
* [ ] Melhorias na interface da IHM
* [ ] Implementação de sistema de alertas

---
