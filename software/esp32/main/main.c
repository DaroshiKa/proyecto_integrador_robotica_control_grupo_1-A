#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"


/* ============================================================
 * PINES
 * ============================================================
 */

#define MOTOR_PWM_GPIO      GPIO_NUM_25
#define MOTOR_AIN1_GPIO     GPIO_NUM_26
#define MOTOR_AIN2_GPIO     GPIO_NUM_27
#define MOTOR_STBY_GPIO     GPIO_NUM_33

#define ENCODER_A_GPIO      GPIO_NUM_34
#define ENCODER_B_GPIO      GPIO_NUM_35


/* ============================================================
 * DATOS DEL MOTOR / ENCODER
 * ============================================================
 */

/*
 * Pololu:
 *
 * 64 counts/revolution del eje del motor
 * usando ambos flancos de A y B.
 */
#define ENCODER_CPR_MOTOR       64.0f


/*
 * Relacion de transmision del motor #4751:
 *
 * 19:1
 */
#define GEAR_RATIO              19.0f


/*
 * Cuentas equivalentes por revolucion
 * del eje de salida.
 */
#define ENCODER_CPR_OUTPUT      \
    (ENCODER_CPR_MOTOR * GEAR_RATIO)


/*
 * Alimentacion del motor.
 */
#define MOTOR_SUPPLY_VOLTAGE    12.0f


/* ============================================================
 * PWM
 * ============================================================
 */

#define PWM_FREQUENCY_HZ        20000

#define PWM_RESOLUTION          LEDC_TIMER_10_BIT

#define PWM_MAX_DUTY            1023

#define PWM_TIMER               LEDC_TIMER_0

#define PWM_CHANNEL             LEDC_CHANNEL_0

#define PWM_SPEED_MODE          LEDC_LOW_SPEED_MODE


/* ============================================================
 * EXPERIMENTO
 * ============================================================
 */

/*
 * Tiempo antes del escalon.
 *
 * Durante este periodo:
 *
 *     PWM = 0 %
 *     motor detenido
 *
 * Se obtiene la condicion inicial.
 */
#define PRE_STEP_TIME_MS        3000


/*
 * Duracion de la respuesta al escalon.
 *
 * Debe ser suficientemente larga para observar
 * el asentamiento de la velocidad.
 */
#define STEP_TIME_MS            6000


/*
 * Tiempo entre pruebas.
 */
#define REST_TIME_MS            3000


/*
 * Periodo de muestreo.
 *
 * 20 ms = 50 Hz
 */
#define SAMPLE_TIME_MS          20


/*
 * Tres escalones diferentes.
 *
 * La guia exige tres curvas de reaccion
 * con diferentes magnitudes.
 */
static const float TEST_STEPS[] =
{
    55.0f,
    65.0f,
    75.0f
};

#define NUM_TESTS \
    (sizeof(TEST_STEPS) / sizeof(TEST_STEPS[0]))


/* ============================================================
 * PCNT
 * ============================================================
 */

#define PCNT_HIGH_LIMIT         30000
#define PCNT_LOW_LIMIT          -30000

static pcnt_unit_handle_t pcnt_unit = NULL;

static pcnt_channel_handle_t pcnt_chan_a = NULL;

static pcnt_channel_handle_t pcnt_chan_b = NULL;


/* ============================================================
 * VARIABLES
 * ============================================================
 */

static const char *TAG = "MOTOR_ID";


/* ============================================================
 * MOTOR
 * ============================================================
 */

static void motor_init(void)
{
    /*
     * AIN1, AIN2 y STBY como salidas.
     */

    gpio_config_t gpio_conf = {
        .pin_bit_mask =
            (1ULL << MOTOR_AIN1_GPIO) |
            (1ULL << MOTOR_AIN2_GPIO) |
            (1ULL << MOTOR_STBY_GPIO),

        .mode = GPIO_MODE_OUTPUT,

        .pull_up_en = GPIO_PULLUP_DISABLE,

        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(
        gpio_config(&gpio_conf)
    );


    /*
     * Estado inicial seguro.
     */

    gpio_set_level(
        MOTOR_AIN1_GPIO,
        0
    );

    gpio_set_level(
        MOTOR_AIN2_GPIO,
        0
    );

    gpio_set_level(
        MOTOR_STBY_GPIO,
        0
    );


    /*
     * Configurar timer PWM.
     */

    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_SPEED_MODE,

        .timer_num = PWM_TIMER,

        .duty_resolution = PWM_RESOLUTION,

        .freq_hz = PWM_FREQUENCY_HZ,

        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(
        ledc_timer_config(&timer_conf)
    );


    /*
     * Configurar canal PWM.
     */

    ledc_channel_config_t channel_conf = {
        .gpio_num = MOTOR_PWM_GPIO,

        .speed_mode = PWM_SPEED_MODE,

        .channel = PWM_CHANNEL,

        .timer_sel = PWM_TIMER,

        .duty = 0,

        .hpoint = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&channel_conf)
    );


    ESP_LOGI(
        TAG,
        "TB6612FNG listo"
    );
}


/* ============================================================
 * MOTOR FORWARD
 * ============================================================
 */

static void motor_forward(void)
{
    gpio_set_level(
        MOTOR_STBY_GPIO,
        1
    );

    gpio_set_level(
        MOTOR_AIN1_GPIO,
        1
    );

    gpio_set_level(
        MOTOR_AIN2_GPIO,
        0
    );
}


/* ============================================================
 * MOTOR STOP
 * ============================================================
 */

static void motor_stop(void)
{
    /*
     * PWM = 0
     */

    ESP_ERROR_CHECK(
        ledc_set_duty(
            PWM_SPEED_MODE,
            PWM_CHANNEL,
            0
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            PWM_SPEED_MODE,
            PWM_CHANNEL
        )
    );


    /*
     * STBY = 0
     */

    gpio_set_level(
        MOTOR_STBY_GPIO,
        0
    );


    /*
     * Direccion neutra.
     */

    gpio_set_level(
        MOTOR_AIN1_GPIO,
        0
    );

    gpio_set_level(
        MOTOR_AIN2_GPIO,
        0
    );
}


/* ============================================================
 * MOTOR PWM
 * ============================================================
 */

static void motor_set_pwm(float percent)
{
    if (percent < 0.0f)
        percent = 0.0f;

    if (percent > 100.0f)
        percent = 100.0f;


    uint32_t duty =
        (uint32_t)(
            (percent / 100.0f)
            * PWM_MAX_DUTY
        );


    ESP_ERROR_CHECK(
        ledc_set_duty(
            PWM_SPEED_MODE,
            PWM_CHANNEL,
            duty
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            PWM_SPEED_MODE,
            PWM_CHANNEL
        )
    );
}


/* ============================================================
 * ENCODER INIT
 *
 * Cuadratura x4
 *
 * A:
 *   flanco subida
 *   flanco bajada
 *
 * B:
 *   flanco subida
 *   flanco bajada
 *
 * Total:
 *
 *       4 cuentas por ciclo de cuadratura
 *
 *       64 cuentas/rev
 *
 * ============================================================
 */

static void encoder_init(void)
{
    /*
     * Unidad PCNT.
     */

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,

        .low_limit = PCNT_LOW_LIMIT,

        .flags.accum_count = true
    };


    ESP_ERROR_CHECK(
        pcnt_new_unit(
            &unit_config,
            &pcnt_unit
        )
    );


    /*
     * Filtro anti-glitch.
     *
     * Las señales del encoder son limpias,
     * pero dejamos un filtro pequeño.
     */

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000
    };


    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(
            pcnt_unit,
            &filter_config
        )
    );


    /* --------------------------------------------------------
     * CANAL A
     * --------------------------------------------------------
     */

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_A_GPIO,

        .level_gpio_num = ENCODER_B_GPIO
    };


    ESP_ERROR_CHECK(
        pcnt_new_channel(
            pcnt_unit,
            &chan_a_config,
            &pcnt_chan_a
        )
    );


    /*
     * A:
     *
     * rising  -> incremento
     * falling -> decremento
     */

    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            pcnt_chan_a,

            PCNT_CHANNEL_EDGE_ACTION_INCREASE,

            PCNT_CHANNEL_EDGE_ACTION_DECREASE
        )
    );


    /*
     * B determina la direccion.
     */

    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            pcnt_chan_a,

            PCNT_CHANNEL_LEVEL_ACTION_KEEP,

            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );


    /* --------------------------------------------------------
     * CANAL B
     * --------------------------------------------------------
     */

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_B_GPIO,

        .level_gpio_num = ENCODER_A_GPIO
    };


    ESP_ERROR_CHECK(
        pcnt_new_channel(
            pcnt_unit,
            &chan_b_config,
            &pcnt_chan_b
        )
    );


    /*
     * B:
     *
     * rising  -> decremento
     * falling -> incremento
     */

    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            pcnt_chan_b,

            PCNT_CHANNEL_EDGE_ACTION_DECREASE,

            PCNT_CHANNEL_EDGE_ACTION_INCREASE
        )
    );


    /*
     * A determina la direccion.
     */

    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            pcnt_chan_b,

            PCNT_CHANNEL_LEVEL_ACTION_KEEP,

            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );


    /*
     * Activar unidad.
     */

    ESP_ERROR_CHECK(
        pcnt_unit_enable(
            pcnt_unit
        )
    );


    /*
     * Limpiar contador.
     */

    ESP_ERROR_CHECK(
        pcnt_unit_clear_count(
            pcnt_unit
        )
    );


    /*
     * Comenzar conteo.
     */

    ESP_ERROR_CHECK(
        pcnt_unit_start(
            pcnt_unit
        )
    );


    ESP_LOGI(
        TAG,
        "Encoder cuadratura x4 listo"
    );

    ESP_LOGI(
        TAG,
        "CPR motor = %.0f",
        ENCODER_CPR_MOTOR
    );

    ESP_LOGI(
        TAG,
        "Gear ratio = %.1f:1",
        GEAR_RATIO
    );

    ESP_LOGI(
        TAG,
        "CPR salida = %.0f",
        ENCODER_CPR_OUTPUT
    );
}


/* ============================================================
 * LEER CONTADOR
 * ============================================================
 */

static int32_t encoder_get_count(void)
{
    int count = 0;


    ESP_ERROR_CHECK(
        pcnt_unit_get_count(
            pcnt_unit,
            &count
        )
    );


    return (int32_t)count;
}


/* ============================================================
 * LIMPIAR CONTADOR
 * ============================================================
 */

static void encoder_clear(void)
{
    ESP_ERROR_CHECK(
        pcnt_unit_clear_count(
            pcnt_unit
        )
    );
}


/* ============================================================
 * CALCULAR RPM DEL EJE DEL MOTOR
 * ============================================================
 */

static float calculate_motor_rpm(
    int32_t delta_count,
    int64_t delta_time_us
)
{
    if (delta_time_us <= 0)
        return 0.0f;


    float revolutions =
        (float)delta_count /
        ENCODER_CPR_MOTOR;


    float rpm =
        revolutions *
        60000000.0f /
        (float)delta_time_us;


    return rpm;
}


/* ============================================================
 * CALCULAR RPM DEL EJE DE SALIDA
 * ============================================================
 */

static float calculate_output_rpm(
    float motor_rpm
)
{
    return motor_rpm /
           GEAR_RATIO;
}


/* ============================================================
 * EJECUTAR UNA CURVA DE REACCION
 * ============================================================
 */

static void run_test(
    int test_number,
    float step_percent
)
{
    ESP_LOGI(
        TAG,
        "Preparando Test %d - %.1f%%",
        test_number,
        step_percent
    );


    /*
     * --------------------------------------------------------
     * PARAR MOTOR
     * --------------------------------------------------------
     */

    motor_stop();

    encoder_clear();


    /*
     * --------------------------------------------------------
     * CABECERA DE LA PRUEBA
     * --------------------------------------------------------
     */

    printf("\n");

    printf(
        "===TEST_START,%d,%.1f===\n",
        test_number,
        step_percent
    );


    /*
     * --------------------------------------------------------
     * CONDICION INICIAL
     *
     * Tiempo negativo:
     *
     *   -3000 ms -> 0 ms
     *
     * Esto permite visualizar la condicion inicial
     * antes del escalon.
     * --------------------------------------------------------
     */

    int64_t test_start_us =
        esp_timer_get_time();


    int64_t previous_time_us =
        test_start_us;


    int32_t previous_count =
        encoder_get_count();


    while (1)
    {
        int64_t now_us =
            esp_timer_get_time();


        int64_t elapsed_us =
            now_us -
            test_start_us;


        if (
            elapsed_us >=
            ((int64_t)PRE_STEP_TIME_MS * 1000)
        )
        {
            break;
        }


        vTaskDelay(
            pdMS_TO_TICKS(
                SAMPLE_TIME_MS
            )
        );
    }


    /*
     * Limpiar el contador justo antes del escalon.
     */

    encoder_clear();


    previous_count =
        encoder_get_count();


    previous_time_us =
        esp_timer_get_time();


    /*
     * --------------------------------------------------------
     * APLICAR ESCALON
     * --------------------------------------------------------
     */

    motor_forward();

    motor_set_pwm(
        step_percent
    );


    int64_t step_start_us =
        esp_timer_get_time();


    previous_time_us =
        step_start_us;


    previous_count =
        encoder_get_count();


    /*
     * --------------------------------------------------------
     * ADQUISICION
     * --------------------------------------------------------
     */

    while (1)
    {
        vTaskDelay(
            pdMS_TO_TICKS(
                SAMPLE_TIME_MS
            )
        );


        /*
         * Tiempo actual.
         */

        int64_t now_us =
            esp_timer_get_time();


        /*
         * Tiempo desde el escalon.
         */

        int64_t t_ms =
            (now_us - step_start_us)
            / 1000;


        /*
         * Contador actual.
         */

        int32_t current_count =
            encoder_get_count();


        /*
         * Diferencia de pulsos.
         */

        int32_t delta_count =
            current_count -
            previous_count;


        /*
         * Tiempo real de muestreo.
         */

        int64_t delta_time_us =
            now_us -
            previous_time_us;


        /*
         * RPM del eje del motor.
         */

        float motor_rpm =
            calculate_motor_rpm(
                delta_count,
                delta_time_us
            );


        /*
         * RPM reales del eje de salida.
         */

        float output_rpm =
            calculate_output_rpm(
                motor_rpm
            );


        /*
         * Voltaje promedio equivalente.
         *
         * NO es la amplitud instantanea del PWM.
         *
         * Es:
         *
         * V_eq = V_motor * duty
         */

        float voltage_equivalent =
            MOTOR_SUPPLY_VOLTAGE *
            step_percent /
            100.0f;


        /*
         * CSV.
         *
         * test
         * t_ms
         * pwm_percent
         * voltage_equivalent
         * delta_count
         * cumulative_count
         * motor_rpm
         * output_rpm
         */

        printf(
            "DATA,%d,%lld,%.2f,%.3f,%ld,%ld,%.3f,%.3f\n",

            test_number,

            (long long)t_ms,

            step_percent,

            voltage_equivalent,

            (long)delta_count,

            (long)current_count,

            motor_rpm,

            output_rpm
        );


        /*
         * Actualizar variables.
         */

        previous_count =
            current_count;


        previous_time_us =
            now_us;


        /*
         * Fin del escalon.
         */

        if (
            t_ms >=
            STEP_TIME_MS
        )
        {
            break;
        }
    }


    /*
     * --------------------------------------------------------
     * QUITAR ESCALON
     * --------------------------------------------------------
     */

    motor_stop();


    printf(
        "===STEP_END,%d===\n",
        test_number
    );


    /*
     * --------------------------------------------------------
     * DESCANSO
     * --------------------------------------------------------
     */

    vTaskDelay(
        pdMS_TO_TICKS(
            REST_TIME_MS
        )
    );


    printf(
        "===TEST_END,%d===\n",
        test_number
    );
}


/* ============================================================
 * MAIN
 * ============================================================
 */

void app_main(void)
{
    printf("\n\n");

    printf(
        "====================================================\n"
    );

    printf(
        " DC MOTOR - IDENTIFICACION NO PARAMETRICA\n"
    );

    printf(
        " ESP32 + TB6612FNG + ENCODER\n"
    );

    printf(
        "====================================================\n"
    );


    printf(
        "Encoder CPR motor: %.0f\n",
        ENCODER_CPR_MOTOR
    );

    printf(
        "Gear ratio: %.1f:1\n",
        GEAR_RATIO
    );

    printf(
        "Encoder CPR salida: %.0f\n",
        ENCODER_CPR_OUTPUT
    );

    printf(
        "Sample time: %d ms\n",
        SAMPLE_TIME_MS
    );

    printf(
        "PWM frequency: %d Hz\n",
        PWM_FREQUENCY_HZ
    );

    printf(
        "Motor supply: %.1f V\n",
        MOTOR_SUPPLY_VOLTAGE
    );


    /*
     * Inicializar hardware.
     */

    motor_init();

    encoder_init();


    /*
     * Seguridad:
     * motor detenido.
     */

    motor_stop();


    /*
     * Esperar antes de comenzar.
     */

    printf(
        "\nStarting identification in 3 seconds...\n"
    );


    vTaskDelay(
        pdMS_TO_TICKS(3000)
    );


    /*
     * Cabecera CSV global.
     */

    printf(
        "\n===DATA_START===\n"
    );


    printf(
        "DATA_HEADER,test,t_ms,pwm_percent,"
        "voltage_equivalent_V,delta_count,"
        "cumulative_count,motor_rpm,output_rpm\n"
    );


    /*
     * Ejecutar las tres curvas.
     */

    for (
        int i = 0;
        i < (int)NUM_TESTS;
        i++
    )
    {
        run_test(
            i + 1,
            TEST_STEPS[i]
        );
    }


    /*
     * Seguridad final.
     */

    motor_stop();

    encoder_clear();


    printf(
        "===DATA_END===\n"
    );


    printf(
        "\n====================================================\n"
    );

    printf(
        " IDENTIFICATION FINISHED\n"
    );

    printf(
        " MOTOR STOPPED\n"
    );

    printf(
        "====================================================\n"
    );
}
