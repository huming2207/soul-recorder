#pragma once

#include <esp_err.h>

class storage_manager
{
public:
    esp_err_t init(const char *path);
    esp_err_t append_data(uint8_t *buf, size_t len);
};

