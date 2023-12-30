#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>

template <uart_port_t uart_port, size_t rx_queue_size, size_t evt_queue_len, gpio_num_t pin_tx, gpio_num_t pin_rx, gpio_num_t pin_rts, gpio_num_t pin_cts>
class base_uart_manager
{
public:
    esp_err_t init();
    esp_err_t reconfigure(uart_config_t *cfg);
    esp_err_t wait_for_event(uart_event_t *evt, uint32_t timeout_ms);
    int read(uint8_t *buf, size_t len, uint32_t timeout_ms);
    int write(uint8_t *buf, size_t len, uint32_t timeout_ms);
    esp_err_t flush();
    esp_err_t deinit();

protected:
    base_uart_manager() = default;

private:
    static const constexpr char TAG[] = "uart_mgr";
    QueueHandle_t rx_evt_queue = nullptr;
};

template<uart_port_t uart_port, size_t rx_queue_size, size_t evt_queue_len, gpio_num_t pin_tx, gpio_num_t pin_rx, gpio_num_t pin_rts, gpio_num_t pin_cts>
esp_err_t base_uart_manager<uart_port, rx_queue_size, evt_queue_len, pin_tx, pin_rx, pin_rts, pin_cts>::deinit()
{
    return uart_driver_delete(uart_port);
}

template<uart_port_t uart_port, size_t rx_queue_size, size_t evt_queue_len, gpio_num_t pin_tx, gpio_num_t pin_rx, gpio_num_t pin_rts, gpio_num_t pin_cts>
esp_err_t base_uart_manager<uart_port, rx_queue_size, evt_queue_len, pin_tx, pin_rx, pin_rts, pin_cts>::flush()
{
    return uart_flush(uart_port);
}

template<uart_port_t uart_port, size_t rx_queue_size, size_t evt_queue_len, gpio_num_t pin_tx, gpio_num_t pin_rx, gpio_num_t pin_rts, gpio_num_t pin_cts>
int base_uart_manager<uart_port, rx_queue_size, evt_queue_len, pin_tx, pin_rx, pin_rts, pin_cts>::write(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    int ret = uart_write_bytes(uart_port, buf, len);
    if (timeout_ms == 0 || ret < 1) {
        return ret;
    }

    return uart_wait_tx_done(uart_port, timeout_ms == UINT32_MAX ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms)) == ESP_OK ? ret : -1;
}

template<uart_port_t uart_port, size_t rx_queue_size, size_t evt_queue_len, gpio_num_t pin_tx, gpio_num_t pin_rx, gpio_num_t pin_rts, gpio_num_t pin_cts>
int base_uart_manager<uart_port, rx_queue_size, evt_queue_len, pin_tx, pin_rx, pin_rts, pin_cts>::read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    return uart_read_bytes(uart_port, buf, len, timeout_ms == UINT32_MAX ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms));
}

template<uart_port_t uart_port, size_t rx_queue_size, size_t evt_queue_len, gpio_num_t pin_tx, gpio_num_t pin_rx, gpio_num_t pin_rts, gpio_num_t pin_cts>
esp_err_t base_uart_manager<uart_port, rx_queue_size, evt_queue_len, pin_tx, pin_rx, pin_rts, pin_cts>::wait_for_event(uart_event_t *evt, uint32_t timeout_ms)
{
    if (evt == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return xQueueReceive(rx_evt_queue, evt, timeout_ms == UINT32_MAX ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms)) == pdTRUE ? ESP_OK : ESP_FAIL;
}

template<uart_port_t uart_port, size_t rx_queue_size, size_t evt_queue_len, gpio_num_t pin_tx, gpio_num_t pin_rx, gpio_num_t pin_rts, gpio_num_t pin_cts>
esp_err_t base_uart_manager<uart_port, rx_queue_size, evt_queue_len, pin_tx, pin_rx, pin_rts, pin_cts>::reconfigure(uart_config_t *cfg)
{
    return uart_param_config(uart_port, cfg);
}

template<uart_port_t uart_port, size_t rx_queue_size, size_t evt_queue_len, gpio_num_t pin_tx, gpio_num_t pin_rx, gpio_num_t pin_rts, gpio_num_t pin_cts>
esp_err_t base_uart_manager<uart_port, rx_queue_size, evt_queue_len, pin_tx, pin_rx, pin_rts, pin_cts>::init()
{
    ESP_LOGI(TAG, "Set up GPS at: Tx %d, Rx %d with UART# %d", pin_tx, pin_rx, uart_port);

    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;
    auto ret = uart_driver_install(uart_port, rx_queue_size, 0, CONFIG_SR_UART_RX_EVT_QUEUE_SIZE, &rx_evt_queue, 0);
    ret = ret ?: uart_param_config(uart_port, &uart_config);
    ret = ret ?: uart_set_pin(uart_port, pin_tx, pin_rx, pin_rts, pin_cts);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set up UART hardware: 0x%x", ret);
        return ret;
    }

    return ret;
}

class uart1_manager : public base_uart_manager<UART_NUM_1, CONFIG_SR_UART_RX_EVT_QUEUE_SIZE, CONFIG_SR_UART_RX_BUFFER_SIZE,
        (gpio_num_t)CONFIG_SR_UART1_TX_PIN, (gpio_num_t)CONFIG_SR_UART1_RX_PIN, (gpio_num_t)CONFIG_SR_UART1_RTS_PIN, (gpio_num_t)CONFIG_SR_UART1_CTS_PIN>
{
public:
    static uart1_manager *instance()
    {
        static uart1_manager _instance;
        return &_instance;
    }

private:
    uart1_manager() = default;
};

class uart2_manager : public base_uart_manager<UART_NUM_2, CONFIG_SR_UART_RX_EVT_QUEUE_SIZE, CONFIG_SR_UART_RX_BUFFER_SIZE,
        (gpio_num_t)CONFIG_SR_UART2_TX_PIN, (gpio_num_t)CONFIG_SR_UART2_RX_PIN, (gpio_num_t)CONFIG_SR_UART2_RTS_PIN, (gpio_num_t)CONFIG_SR_UART2_CTS_PIN>
{
public:
    static uart2_manager *instance()
    {
        static uart2_manager _instance;
        return &_instance;
    }

private:
    uart2_manager() = default;
};