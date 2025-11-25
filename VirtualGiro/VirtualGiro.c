#include <stdio.h>
#include <string.h> // Adicionado para manipulação de strings e JSON
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"  // Adicionado para temperatura
#include "hardware/gpio.h" // Adicionado para botão
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h" // Adicionado para MQTT
#include "lwip/dns.h"       // Adicionado para resolver nome do broker
#include "udp_sender.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "mpu6050_i2c.h"
#include "ssd1306.h"

// --- Configurações Wi-Fi ---
#define WIFI_SSID     "ITSelf"
#define WIFI_PASSWORD "code2020"

// --- Configurações UDP (Mantido do original) ---
#define SERVER_IP   "192.168.1.186"
#define SERVER_PORT 12345

// --- Configurações MQTT (ADICIONADO) ---
#define MQTT_BROKER      "broker.emqx.io"
#define MQTT_BROKER_PORT 1883 
#define MQTT_TOPIC       "projetoAR/controle/dados"

// --- Hardware Pins ---
#define BUTTON_GPIO 5 // Botão A da BitDogLab

#define I2C_MPU i2c0
#define I2C0_SDA 0
#define I2C0_SCL 1
#define I2C_OLED i2c1
#define I2C1_SDA 14
#define I2C1_SCL 15

// --- Variáveis Globais e Flags ---
volatile bool wifi_connected = false;
static mqtt_client_t *mqtt_client; // Cliente MQTT
static ip_addr_t broker_ip;
volatile bool mqtt_connected_flag = false; // Flag para saber se MQTT está online

SemaphoreHandle_t xI2C_Mutex;
QueueHandle_t xMPU_Queue;

// ============================================================================
// === FUNÇÕES AUXILIARES (TEMPERATURA E CALLBACKS MQTT) - ADICIONADO ===
// ============================================================================

// Leitura da temperatura interna do RP2040
float read_temperature() {
    adc_select_input(4); 
    uint16_t raw = adc_read();
    return 27.0f - ((raw * 3.3f / (1 << 12)) - 0.706f) / 0.001721f;
}

// Callback de conexão MQTT
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Conectado ao broker!\n");
        mqtt_connected_flag = true;
    } else {
        printf("[MQTT] Falha na conexão: %d\n", status);
        mqtt_connected_flag = false;
    }
}

// Callback DNS (para resolver broker.emqx.io)
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    if (ipaddr != NULL) {
        broker_ip = *ipaddr;
        printf("[DNS] Broker IP resolvido: %s\n", ip4addr_ntoa(ipaddr));
    } else {
        printf("[DNS] Falha ao resolver DNS do broker\n");
    }
}

// Função auxiliar para publicar mensagem MQTT com thread-safety
void send_mqtt_msg(const char* topic, const char* payload) {
    if (mqtt_connected_flag && wifi_connected) {
        // No FreeRTOS com LwIP, devemos proteger chamadas de rede com cyw43_arch_lwip_begin/end
        cyw43_arch_lwip_begin();
        err_t err = mqtt_publish(mqtt_client, topic, payload, strlen(payload), 0, 0, NULL, NULL);
        cyw43_arch_lwip_end();
        
        if(err != ERR_OK) {
            printf("[MQTT] Erro ao publicar: %d\n", err);
        }
    }
}

// ============================================================================
// === TASKS (TAREFAS) DO SISTEMA ===
// ============================================================================

// ---- Task: leitura MPU6050 (Original) ----
void vTaskMPU6050(void *pvParameters) {
    mpu6050_data_t mpu;
    if (!mpu6050_init(I2C_MPU, I2C0_SDA, I2C0_SCL)) {
        printf("Erro inicializando MPU6050\n");
        vTaskDelete(NULL);
    }

    while (1) {
        if (xSemaphoreTake(xI2C_Mutex, portMAX_DELAY) == pdTRUE) {
            if (mpu6050_read(&mpu)) {
                mpu6050_calc_angles(&mpu);
                // Sobrescreve a fila com o dado mais recente
                xQueueOverwrite(xMPU_Queue, &mpu);
            }
            xSemaphoreGive(xI2C_Mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz de amostragem (ajustável)
    }
}

// ---- Task: Lógica MQTT (NOVA - Integração do button_temp_mqtt.c) ----
void vTaskMQTT(void *pvParameters) {
    // Inicializa ADC e Botão aqui ou no main (fiz no main)
    mqtt_client = mqtt_client_new();
    
    mpu6050_data_t mpu_data;
    bool last_button_state = false;
    char payload[256];
    int temp_counter = 0;

    // Aguarda conexão Wi-Fi inicial
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Resolve DNS
    cyw43_arch_lwip_begin();
    dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
    cyw43_arch_lwip_end();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Tempo para DNS resolver

    while (1) {
        // 1. Tenta conectar se estiver desconectado
        if (wifi_connected && !mqtt_connected_flag && broker_ip.addr != 0) {
            struct mqtt_connect_client_info_t client_info = {
                .client_id = "pico_bitdoglab_giro",
                .keep_alive = 60,
                .client_user = NULL, .client_pass = NULL,
                .will_topic = NULL, .will_msg = NULL, .will_qos = 0, .will_retain = 0
            };
            
            printf("[MQTT] Tentando conectar ao broker...\n");
            cyw43_arch_lwip_begin();
            mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_callback, NULL, &client_info);
            cyw43_arch_lwip_end();
            
            vTaskDelay(pdMS_TO_TICKS(5000)); // Espera antes de tentar de novo
        }

        if (mqtt_connected_flag) {
            // --- A. Leitura e Envio do Botão (Detecta mudança de estado) ---
            bool current_button_state = !gpio_get(BUTTON_GPIO); // Pull-up: 0 é pressionado
            if (current_button_state != last_button_state) {
                snprintf(payload, sizeof(payload), "{\"type\":\"action\",\"payload\":{\"button\":%d}}", current_button_state ? 1 : 0);
                send_mqtt_msg(MQTT_TOPIC, payload);
                printf("[MQTT] Botão: %s\n", payload);
                last_button_state = current_button_state;
                vTaskDelay(pdMS_TO_TICKS(50)); // Debounce simples
            }

            // --- B. Leitura e Envio do Giroscópio (Real do MPU6050) ---
            // Usamos xQueuePeek para ler sem remover da fila (para não roubar dados do UDP ou OLED)
            if (xQueuePeek(xMPU_Queue, &mpu_data, 0)) {
                // Formata JSON com dados REAIS do sensor
                snprintf(payload, sizeof(payload), 
                        "{\"type\":\"rotate\",\"payload\":{\"alpha\":%.2f,\"beta\":%.2f,\"gamma\":%.2f}}", 
                        mpu_data.angle_z,  // Alpha (Z)
                        mpu_data.angle_x,  // Beta (X)
                        mpu_data.angle_y); // Gamma (Y) - Ajuste conforme orientação física
                
                send_mqtt_msg(MQTT_TOPIC, payload);
                // Envia rotação a cada 200ms (evita floodar o broker)
                vTaskDelay(pdMS_TO_TICKS(200)); 
            }

            // --- C. Leitura e Envio de Temperatura (A cada aprox. 2 seg) ---
            //temp_counter++;
            if (temp_counter >= 50) { // 50 * 200ms = 10000ms
                float t = read_temperature();
                snprintf(payload, sizeof(payload), "{\"type\":\"temperatura\",\"payload\":{\"tempe_c\":%.2f}}", t);
                send_mqtt_msg(MQTT_TOPIC, payload);
                printf("[MQTT] Temp: %s\n", payload);
                temp_counter = 0;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// ---- Task: display OLED (Original) ----
void vTaskOLED(void *pvParameters) {
    ssd1306_t oled;
    ssd1306_init(&oled, I2C_OLED, I2C1_SDA, I2C1_SCL);

    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 0, "Iniciando...");
    ssd1306_show(&oled);

    mpu6050_data_t mpu;

    while (1) {
        // Recebe da fila (copia o dado)
        if (xQueuePeek(xMPU_Queue, &mpu, pdMS_TO_TICKS(500))) {
            if (xSemaphoreTake(xI2C_Mutex, portMAX_DELAY) == pdTRUE) {
                ssd1306_clear(&oled);
                char buf[32];

                snprintf(buf, sizeof(buf), "X: %.1f", mpu.angle_x);
                ssd1306_draw_string(&oled, 0, 0, buf);

                snprintf(buf, sizeof(buf), "Y: %.1f", mpu.angle_y);
                ssd1306_draw_string(&oled, 0, 16, buf);

                snprintf(buf, sizeof(buf), "Z: %.1f", mpu.angle_z);
                ssd1306_draw_string(&oled, 0, 32, buf);

                // Status Wi-Fi e MQTT
                const char *status = wifi_connected ? (mqtt_connected_flag ? "Net: MQTT OK" : "Net: Wi-Fi OK") : "Net: OFF";
                ssd1306_draw_string(&oled, 0, 48, status);

                ssd1306_show(&oled);
                xSemaphoreGive(xI2C_Mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Atualiza tela 5fps
    }
}

// ---- Task: debug serial ----
void vTaskDebug(void *pvParameters) {
    mpu6050_data_t mpu;
    while (1) {
        if (xQueueReceive(xMPU_Queue, &mpu, pdMS_TO_TICKS(1000))) {
            printf("AX=%.2f AY=%.2f AZ=%.2f | GX=%.2f GY=%.2f GZ=%.2f | X=%.2f Y=%.2f Z=%.2f\n",
                   mpu.ax, mpu.ay, mpu.az,
                   mpu.gx, mpu.gy, mpu.gz,
                   mpu.angle_x, mpu.angle_y, mpu.angle_z);
        }
    }
}

// ---- Task: reconexão Wi-Fi (Original) ----
void vTaskWiFiReconnect(void *pvParameters) {
    while (1) {
        // Proteção necessária ao acessar estado do cyw43
        cyw43_arch_lwip_begin();
        struct netif *netif = &cyw43_state.netif[0];
        bool is_ip_set = !ip4_addr_isany_val(netif->ip_addr);
        cyw43_arch_lwip_end();

        if (!is_ip_set) {
            wifi_connected = false;
            printf("Wi-Fi desconectado. Tentando reconectar...\n");
            
            // Tentativa de conexão (bloqueante, mas ok nesta task específica)
            if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                                   CYW43_AUTH_WPA2_AES_PSK, 10000)) {
                printf("Falha na reconexão.\n");
            } else {
                wifi_connected = true;
                printf("Reconectado!\n");
            }
        } else {
            wifi_connected = true;
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
}

// ---- Task: Envio UDP (Original - Mantida para compatibilidade) ----
void vTaskSendUDP(void *pvParameters) {
    struct udp_pcb *pcb = udp_sender_init();
    if (!pcb) vTaskDelete(NULL);

    mpu6050_data_t mpu;

    while (1) {
        if (wifi_connected && xQueuePeek(xMPU_Queue, &mpu, pdMS_TO_TICKS(1000))) {
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
                     mpu.ax, mpu.ay, mpu.az,
                     mpu.gx, mpu.gy, mpu.gz,
                     mpu.angle_x, mpu.angle_y, mpu.angle_z);
            
            // LwIP raw api requer lock
            cyw43_arch_lwip_begin();
            udp_sender_send(pcb, SERVER_IP, SERVER_PORT, msg);
            cyw43_arch_lwip_end();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================================
// === MAIN ===
// ============================================================================
int main() {
    stdio_init_all();

    // --- Hardware Init: I2C MPU6050 ---
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    // --- Hardware Init: I2C OLED ---
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_pull_up(14);
    gpio_pull_up(15);

    // --- Hardware Init: Botão e ADC (ADICIONADO) ---
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    adc_init();
    adc_set_temp_sensor_enabled(true);

    // --- Inicialização Wi-Fi ---
    if (cyw43_arch_init()) {
        printf("Erro inicializando Wi-Fi\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    
    printf("Conectando ao Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Falha ao conectar Wi-Fi.\n");
    } else {
        printf("Wi-Fi conectado!\n");
        wifi_connected = true;
    }

    // --- FreeRTOS Resources ---
    xI2C_Mutex = xSemaphoreCreateMutex();
    xMPU_Queue = xQueueCreate(1, sizeof(mpu6050_data_t)); // Fila tamanho 1 (mantém só o mais atual)

    if (xI2C_Mutex == NULL || xMPU_Queue == NULL) {
        printf("Erro criando mutex ou fila\n");
        return -1;
    }

    // --- Criação das Tasks ---
    xTaskCreate(vTaskMPU6050, "MPU6050", 512, NULL, 2, NULL);   // Prioridade alta para leitura
    xTaskCreate(vTaskOLED, "OLED", 1024, NULL, 1, NULL);        // Atualiza display
    xTaskCreate(vTaskDebug, "DEBUG", 512, NULL, 1, NULL);       // Debug serial 
    xTaskCreate(vTaskWiFiReconnect, "WiFiRec", 512, NULL, 1, NULL);
    xTaskCreate(vTaskSendUDP, "SendUDP", 512, NULL, 1, NULL);   // Envia UDP (Legado)
    xTaskCreate(vTaskMQTT, "TaskMQTT", 2048, NULL, 1, NULL);    // Envia MQTT (Mais pilha necessária)
    

    // --- Iniciar Scheduler ---
    vTaskStartScheduler();

    while (1);
}