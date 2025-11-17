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
    e --> f["Criar tasks (MPU6050)"]
    f --> g["Criar tasks (OLED)"]
    g --> h["Criar tasks (WiFiReconnect)"]
    h --> i["Criar tasks (SendUDP)"]
    i --> j["Iniciar scheduler"]
    j --> l["Loop Infinito"]
    l --> l
    j@{ shape: procs}

:::
