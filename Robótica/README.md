# Robótica — Estación 2 (SCARA RRP, dispensado de pasta de soldar)

Firmware del ESP32 (WROOM-32) que controla el manipulador. Objetivo: poder comandarlo
tanto por **micro-ROS** (WiFi, ROS 2 Jazzy) como en **vivo por USB serial** desde el PC.

## Estructura
```
Robótica/
└── firmware/            Proyecto PlatformIO (Arduino-ESP32 2.0.x)
    ├── platformio.ini
    ├── include/config.h  Constantes mecánicas, pines y parámetros de control
    └── src/main.cpp      Encoders, drivers MC33926 y PID de posición por articulación
```

## Estado
- [x] Etapa 1–2: drivers + encoders (x4) + PID de posición por articulación, sintonizable por serial.
- [ ] Verificar el mapa de pines y poner `PINS_VERIFIED true` en `config.h`.
- [ ] PID en cascada (posición → velocidad), homing con finales de carrera, Z (solenoide) con secuencia temporizada.
- [ ] Capa de comandos con arbitraje de modo (ROS / manual) y watchdog.
- [ ] Adaptador micro-ROS (WiFi) y adaptador serial en vivo.

## Compilar y cargar
```bash
cd Robótica/firmware
pio run                  # compilar
pio run -t upload        # cargar al ESP32
pio device monitor       # monitor serial, 115200
```

## Comandos serial
`j <1|2>` selecciona articulación · `e <0|1>` deshabilita/habilita (arranca DESHABILITADA) ·
`t <deg>` objetivo · `g <kp> <ki> <kd>` ganancias · `u <0..1>` duty máximo ·
`z` declara 0° en la posición actual · `v` stream para el Serial Plotter · `p` estado.

## Notas
- Resolución por articulación: 64 CPR × 50 (reductora) × 5 (poleas 20T→100T) = 16000 cuentas/vuelta (0.0225°).
- Confirmar la relación exacta de la reductora en la ficha de Pololu.
