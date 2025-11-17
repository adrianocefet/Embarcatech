:::mermaid

flowchart TD
  %% =========================
  %% BOOT / MAIN
  %% =========================
  A0([Boot / Reset]) --> A1[stdio_init_all]
  A1 --> A2[I2C0 init -MPU6050<br/>I2C1 init OLED]
  A2 --> A3{cyw43_arch_init OK?}
  A3 -- Não --> A3e[printf-Erro init Wi-Fi'<br/>return -1]
  A3 -- Sim --> A4[cyw43_arch_enable_sta_mode]
  A4 --> A5[printf-'Conectando ao Wi-Fi...']
  A5 --> A6{Conectou em timeout?}
  A6 -- Não --> A6n[printf-'Falha ao conectar']
  A6 -- Sim --> A6s[printf-'Wi-Fi conectado!']

  A6n --> A7[xI2C_Mutex = xSemaphoreCreateMutex]
  A6s --> A7
  A7 --> A8[xMPU_Queue = xQueueCreate]
  A8 --> A9{xI2C_Mutex e fila OK?}
  A9 -- Não --> A9e[printf'Erro criando recursos'<br/>return -1]
  A9 -- Sim --> A10[[xTaskCreate vTaskMPU6050 - prio 2]]
  A10 --> A11[[xTaskCreate vTaskOLED - prio 1]]
  A11 --> A12[[xTaskCreate vTaskDebug - prio 1]]
  A12 --> A13[[xTaskCreate vTaskWiFiReconnect prio 1]]

  A13 --> A14[[xTaskCreate vTaskSendUDP - prio 1]]
  A14 --> A15[vTaskStartScheduler]
  A15 --> A16([RTOS em execução])

  %% =========================
  %% TASK: MPU6050 (produtora)
  %% =========================
  subgraph T1[Task vTaskMPU6050 — Lê sensor e publica na fila]
    direction TB
    M0[mpu6050_init- I2C0] --> M1{Init OK?}
    M1 -- Não --> M1e[printf'Erro MPU6050'<br/>vTaskDelete]
    M1 -- Sim --> M2[[Loop a cada ~200 ms]]
    M2 --> M3{xSemaphoreTake-xI2C_Mutex}
    M3 -- Pegou --> M4{mpu6050_read}
    M4 -- Sucesso --> M5[mpu6050_calc_angles]
    M5 --> M6[xQueueOverwrite]
    M4 -- Falha --> M6f[Sem envio]
    M6 --> M7[xSemaphoreGive-xI2C_Mutex]
    M6f --> M7
    M7 --> M2
  end

  %% =========================
  %% TASK: OLED (consumidora)
  %% =========================
  subgraph T2[Task vTaskOLED — Mostra ângulos e status Wi-Fi]
    direction TB
    O0[ssd1306_init-I2C1] --> O1[Clear + 'Iniciando...' + show]
    O1 --> O2[[Loop ~500 ms]]
    O2 --> O3{xQueueReceive xMPU_Queue, 500 ms}
    O3 -- Recebeu --> O4{xSemaphoreTake}
    O4 -- Pegou --> O5[Limpa; desenha X/Y/Z;<br/>desenha status Wi-Fi]
    O5 --> O6[ssd1306_show xSemaphoreGive]
    O4 -- Não --> O2
    O3 -- Vazio --> O2
  end


  %% =========================
  %% TASK: DEBUG (consumidora)
  %% =========================
  subgraph T3[Task vTaskDebug — Log serial]
    direction TB
    D0[[Loop ~1000 ms]] --> D1{xQueueReceive-xMPU_Queue, 1000 ms}
    D1 -- Recebeu --> D2[printf de acel/gyro/ângulos]
    D1 -- Vazio --> D0
    D2 --> D0
  end

  %% =========================
  %% TASK: Wi-Fi Reconnect
  %% =========================
  subgraph T4[Task vTaskWiFiReconnect — Monitora e reconecta]
    direction TB
    W0[[Loop ~10 s]]
    W0 --> W1[Verifica IP atual]
    W1 --> W2{Desconectado?}
    W2 -- Sim --> W3[wifi_connected=false;<br/>'Wi-Fi OFF']
    W3 --> W4{cyw43_arch_wifi_connect_timeout_ms}
    W4 -- Sim --> W5[wifi_connected=true;<br/>printf]
    W4 -- Não --> W6[printf- Falha reconexão]
    W2 -- Não --> W5
    W5 --> W0
    W6 --> W0
  end

  %% =========================
  %% TASK: Send UDP (consum.)
  %% =========================
  subgraph T5[Task vTaskSendUDP — Envia JSON por UDP]
    direction TB
    U0[pcb = udp_sender_init]
    U0 --> U1{pcb válido?}
    U1 -- Não --> U1e[vTaskDelete]
    U1 -- Sim --> U2[[Loop ~100 ms]]
    U2 --> U3{wifi_connected == true}
    U3 -- Sim --> U4{xQueueReceive}
    U4 -- Recebeu --> U5[Formata JSON<br/>-ax,ay,az,gx,gy,gz,angle_x,y,z]
    U5 --> U6[udp_sender_send]
    U4 -- Vazio --> U2
    U3 -- Não --> U2
  end

  %% Ligações do scheduler para as tasks
  A16 --> T1
  A16 --> T2
  A16 --> T3
  A16 --> T4
  A16 --> T5


  :::
