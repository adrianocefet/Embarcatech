#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "udp_sender.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "mpu6050_i2c.h"
#include "ssd1306.h"

#define WIFI_SSID     "ITSelf"
#define WIFI_PASSWORD "code2020"

#define SERVER_IP   "192.168.1.186"
#define SERVER_PORT 12345

#define I2C_MPU i2c0
#define I2C0_SDA 0
#define I2C0_SCL 1
#define I2C_OLED i2c1
#define I2C1_SDA 14
#define I2C1_SCL 15

volatile bool wifi_connected = false;

SemaphoreHandle_t xI2C_Mutex;
QueueHandle_t xMPU_Queue;

// ---- Task: leitura MPU6050 ----
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
                xQueueOverwrite(xMPU_Queue, &mpu);
            }
            xSemaphoreGive(xI2C_Mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ---- Task: display OLED ----
void vTaskOLED(void *pvParameters) {
    ssd1306_t oled;
    ssd1306_init(&oled, I2C_OLED, I2C1_SDA, I2C1_SCL);

    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 0, "Iniciando...");
    ssd1306_show(&oled);

    mpu6050_data_t mpu;

    while (1) {
        if (xQueueReceive(xMPU_Queue, &mpu, pdMS_TO_TICKS(500))) {
            if (xSemaphoreTake(xI2C_Mutex, portMAX_DELAY) == pdTRUE) {
                ssd1306_clear(&oled);

                char buf[32];

                // Ângulos
                snprintf(buf, sizeof(buf), "X: %.1f", mpu.angle_x);
                ssd1306_draw_string(&oled, 0, 0, buf);

                snprintf(buf, sizeof(buf), "Y: %.1f", mpu.angle_y);
                ssd1306_draw_string(&oled, 0, 16, buf);

                snprintf(buf, sizeof(buf), "Z: %.1f", mpu.angle_z);
                ssd1306_draw_string(&oled, 0, 32, buf);

                // Status Wi-Fi
                const char *wifi_status = wifi_connected ? "Wi-Fi: OK" : "Wi-Fi: OFF";
                ssd1306_draw_string(&oled, 0, 48, wifi_status);

                ssd1306_show(&oled);
                xSemaphoreGive(xI2C_Mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
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

// ---- Task: reconexão Wi-Fi ----
void vTaskWiFiReconnect(void *pvParameters) {
    while (1) {
        struct netif *netif = &cyw43_state.netif[0];
        if (ip4_addr_isany_val(netif->ip_addr)) {
            wifi_connected = false;
            printf("Wi-Fi desconectado. Tentando reconectar...\n");

            if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                                   CYW43_AUTH_WPA2_AES_PSK, 10000)) {
                printf("Falha na reconexão.\n");
            } else {
                wifi_connected = true;
                ip4_addr_t ip = netif->ip_addr;
                printf("Reconectado! IP: %s\n", ip4addr_ntoa(&ip));
            }
        } else {
            wifi_connected = true;
        }

        vTaskDelay(pdMS_TO_TICKS(10000)); // Verifica a cada 10 segundos
    }
}

void vTaskSendUDP(void *pvParameters) {
    struct udp_pcb *pcb = udp_sender_init();
    if (!pcb) vTaskDelete(NULL);

    mpu6050_data_t mpu;

    while (1) {
        if (wifi_connected && xQueueReceive(xMPU_Queue, &mpu, pdMS_TO_TICKS(1000))) {
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
                     mpu.ax, mpu.ay, mpu.az,
                     mpu.gx, mpu.gy, mpu.gz,
                     mpu.angle_x, mpu.angle_y, mpu.angle_z);

            udp_sender_send(pcb, SERVER_IP, SERVER_PORT, msg);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---- main ----
int main() {
    stdio_init_all();

    // I2C0 - MPU6050
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    // I2C1 - OLED
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_pull_up(14);
    gpio_pull_up(15);

    // ----- Conectar Wi-Fi antes do FreeRTOS -----
    if (cyw43_arch_init()) {
        printf("Erro inicializando Wi-Fi\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Falha ao conectar.\n");
    } else {
        struct netif *netif = &cyw43_state.netif[0];
        ip4_addr_t ip = netif->ip_addr;
        printf("Wi-Fi conectado! IP: %s\n", ip4addr_ntoa(&ip));
    }

    // ----- Criar mutex e fila -----
    xI2C_Mutex = xSemaphoreCreateMutex();
    xMPU_Queue = xQueueCreate(1, sizeof(mpu6050_data_t));

    if (xI2C_Mutex == NULL || xMPU_Queue == NULL) {
        printf("Erro criando mutex ou fila\n");
        return -1;
    }

    // ----- Criar tasks -----
    xTaskCreate(vTaskMPU6050, "MPU6050", 512, NULL, 2, NULL);
    xTaskCreate(vTaskOLED, "OLED", 1024, NULL, 1, NULL);
    xTaskCreate(vTaskDebug, "DEBUG", 512, NULL, 1, NULL);
    xTaskCreate(vTaskWiFiReconnect, "WiFiReconnect", 512, NULL, 1, NULL);
    xTaskCreate(vTaskSendUDP, "SendUDP", 512, NULL, 1, NULL);

    // ----- Iniciar scheduler -----
    vTaskStartScheduler();

    while (1); // nunca chega aqui
}
