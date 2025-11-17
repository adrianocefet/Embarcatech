
:::mermaid
---
config:
  theme: redux
---
flowchart TD
    a(["Main"]) --> b["Inicializa I2C0 - MPU6050"]
    b --> c["I2C1 - OLED"]
    c --> d["Conectar Wi-Fi antes do FreeRTOS"]
    d --> e["Criar mutex e fila"]
    e --> f["Tasks (MPU6050)"]
    e --> g["Tasks (OLED)"]
    e --> h["Tasks (WiFiReconnect)"]
    e --> i["Tasks (Debug)"]
    e --> j["Tasks (SendUDP)"]
    f@{ shape: procs}
    g@{ shape: procs}
    h@{ shape: procs}
    i@{ shape: procs}
    j@{ shape: procs}

  :::  