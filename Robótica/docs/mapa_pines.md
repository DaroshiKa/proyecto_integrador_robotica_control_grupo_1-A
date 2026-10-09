# Mapa de pines — SCARA (ESP32 WROOM-32 + 2 × MC33926)

Mapeado a mano sobre la placa (no hay esquemático).

## Encoders (alimentados con Vin y GND)
| Articulación | Canal A (blanco) | Canal B (amarillo) |
|---|---|---|
| J1 | D35 | D34 |
| J2 | VN (GPIO39) | VP (GPIO36) |

## Drivers MC33926
| Pin del driver | Driver 1 (J1) | Driver 2 (J2) |
|---|---|---|
| INV (dirección) | D26 | D33 |
| D2 / PWM (velocidad) | D25 | D32 |
| IN1 | 3V3 | 3V3 |
| IN2 | GND | GND |
| EN, SLEW | 3V3 | 3V3 |
| D1 | a GND en la placa del driver (verificar) | GND |
| OUT1 / OUT2 | cable negro / rojo del motor 1 | cable negro / rojo del motor 2 |
| VIN / GND | +V / −V de la fuente | +V / −V de la fuente |

Con IN1 en alto e IN2 en bajo, el pin INV invierte el sentido y el PWM en D2 fija la velocidad.
Con D2 en bajo el driver queda en tri-state: el motor gira libre, sin freno.
FB (corriente) y SF (falla) no están conectados.

## Finales de carrera (contacto NO entre el pin y GND, activos en bajo)
| Articulación | Derecho | Izquierdo |
|---|---|---|
| J1 | D27 | D13 |
| J2 | D19 | D18 |

## Pendiente
- Z (solenoide): sin pin asignado.
- Los canales A/B de cada encoder se asumieron por color; si el signo sale al revés, se corrige con `ENC_SIGN_Jx` en `config.h`.
