#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"

#define MQ135_PIN ADC1_CHANNEL_6 // GPIO34 corresponde a ADC1_CHANNEL_6
#define RLOAD 10.0               // Resistencia de carga (RL) en kΩ
#define WIFI_SSID "MIWIFI_2G_DbRt_EXT"
#define WIFI_PASSWORD "Seat+Ford"
#define THINGSBOARD_SERVER "mqtt://demo.thingsboard.io"
#define ACCESS_TOKEN "R4jOqiRUjRlUfgtJfeb0"
#define THINGSBOARD_PORT 1883
#define FILTER_SIZE 10
#define RELAY_PIN GPIO_NUM_4
#define PIN_NUM_MISO  19
#define PIN_NUM_MOSI  -1
#define PIN_NUM_CLK   18
#define PIN_NUM_CS    5

static const char *TAG = "MQ135_Current_Sensor";

float R0 = 6.33; // Resistencia calibrada en aire limpio (en kΩ)
float filter_buffer[FILTER_SIZE] = {0};
int filter_index = 0;

// Umbral para desactivar el relé si el nivel de gases es alto (en ppm)
#define GAS_THRESHOLD 60.0

// Funciones para cálculo del sensor MQ-135
float calcular_ratio(float Rs) {
    return Rs / R0;
}

float calcular_ppm_alcohol(float ratio) {
    const float m = -0.3;
    const float b = 0.5;
    float log_ratio = log10(ratio);
    float log_ppm = (log_ratio - b) / m;
    return pow(10, log_ppm);
}

float calcular_rs(int adc_value) {
    float voltaje = (float)adc_value * 3.3 / 4095.0;
    return ((3.3 - voltaje) / voltaje) * RLOAD;
}

// Filtro para sensor de corriente
float apply_filter(float new_value) {
    filter_buffer[filter_index] = new_value;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += filter_buffer[i];
    }
    return sum / FILTER_SIZE;
}

// Inicialización de SPI
static spi_device_handle_t spi_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    spi_device_handle_t handle;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &handle));

    return handle;
}

// Lectura del sensor de corriente
static uint16_t read_sensor(spi_device_handle_t spi) {
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};

    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    ESP_ERROR_CHECK(spi_device_transmit(spi, &trans));

    uint16_t value = (rx_data[0] << 8) | rx_data[1];
    return value;
}

// Configuración de MQTT
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Conectado a ThingsBoard");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Desconectado de ThingsBoard");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Datos MQTT recibidos");
            break;
        default:
            break;
    }
}

esp_mqtt_client_handle_t mqtt_init() {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = THINGSBOARD_SERVER,
        .credentials.username = ACCESS_TOKEN,
        .broker.address.port = THINGSBOARD_PORT,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    return client;
}

// Configuración de Wi-Fi
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Desconectado del Wi-Fi, intentando reconectar...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP asignada: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void wifi_init() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Tarea principal para lectura del sensor y envío de datos
void sensor_task(void *arg) {
    esp_mqtt_client_handle_t mqtt_client = mqtt_init();
    spi_device_handle_t spi = spi_init();

    // Configura el pin del relé
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_PIN, 1); // Activa el relé por defecto

    while (1) {
        // Sensor de gas
        int adc_value = adc1_get_raw(MQ135_PIN);
        float Rs = calcular_rs(adc_value);
        float ratio = calcular_ratio(Rs);
        float ppm_alcohol = calcular_ppm_alcohol(ratio);

        ESP_LOGI(TAG, "Gas -> ADC: %d, Rs: %.2f kΩ, Ratio: %.2f, Alcohol: %.2f ppm", 
                 adc_value, Rs, ratio, ppm_alcohol);

        // Control del relé según el nivel de gases
        if (ppm_alcohol > GAS_THRESHOLD) {
            ESP_LOGW(TAG, "Nivel de gases alto (%.2f ppm), apagando el relé", ppm_alcohol);
            gpio_set_level(RELAY_PIN, 0); // Apaga el relé
        } else {
            gpio_set_level(RELAY_PIN, 1); // Enciende el relé
        }

        // Sensor de corriente
        uint16_t raw_value = read_sensor(spi);
        const int OFFSET = 2048;
        const float SENSITIVITY = 89.95;
        float current_mA = ((float)(raw_value - OFFSET) * 1000) / SENSITIVITY;
        float current_A = fabs(current_mA / 1000.0);
        float filtered_current = apply_filter(current_A);

        ESP_LOGI(TAG, "Corriente -> Raw: 0x%04X, Filtrada: %.2f A", raw_value, filtered_current);

        // Enviar telemetría
        char telemetry[256];
        snprintf(telemetry, sizeof(telemetry), "{\"alcohol_ppm\": %.2f, \"current_A\": %.2f}", ppm_alcohol, filtered_current);
        esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", telemetry, 0, 1, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    // Inicializa Wi-Fi
    wifi_init();

    // Configura ADC para MQ-135
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MQ135_PIN, ADC_ATTEN_DB_11);

    // Espera a que Wi-Fi esté listo
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Inicia la tarea del sensor
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
