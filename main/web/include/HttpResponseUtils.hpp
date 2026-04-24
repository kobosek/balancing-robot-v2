#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include <cstddef>

inline esp_err_t httpResponseResult(esp_err_t sendResult)
{
    return sendResult == ESP_OK ? ESP_OK : ESP_FAIL;
}

inline esp_err_t sendHttpError(httpd_req_t* req, httpd_err_code_t status, const char* message)
{
    return httpResponseResult(httpd_resp_send_err(req, status, message));
}

inline esp_err_t sendHttpTimeout(httpd_req_t* req)
{
    return httpResponseResult(httpd_resp_send_408(req));
}

inline esp_err_t sendHttp500(httpd_req_t* req)
{
    return httpResponseResult(httpd_resp_send_500(req));
}

inline int receiveHttpRequestBody(httpd_req_t* req, char* buffer, size_t contentLength)
{
    size_t received = 0;
    while (received < contentLength) {
        const int ret = httpd_req_recv(req, buffer + received, contentLength - received);
        if (ret <= 0) {
            return ret;
        }
        received += static_cast<size_t>(ret);
    }

    return static_cast<int>(received);
}
