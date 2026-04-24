// main/web/StaticFileHandler.cpp

#include "StaticFileHandler.hpp"
#include "HttpResponseUtils.hpp"
#include "esp_vfs.h"    // For fopen, FILE, etc.
#include "esp_check.h"  // For ESP_LOGx macros
#include <memory>       // For unique_ptr
#include <cstring>      // For strcmp, strrchr, strlen, strerror
#include <cerrno>       // For errno
#include <sys/param.h>  // For MAX/MIN (can use std::max/min too)
#include <new>          // For std::nothrow

// Define buffer sizes and max path length
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN + 10)
#define SCRATCH_BUFSIZE (8192)

// Constructor
StaticFileHandler::StaticFileHandler(const char* base_path) : m_base_path(base_path) {
    ESP_LOGI(TAG, "StaticFileHandler created with base_path: %s", m_base_path);
}

// Sets the Content-Type header based on file extension
void StaticFileHandler::set_content_type_from_file(httpd_req_t *req, const char *filepath) {
    const char *dot = strrchr(filepath, '.');
    const char *default_type = "text/plain";

    if (!dot || dot == filepath) {
        ESP_LOGD(TAG, "No extension for %s, using default type: %s", filepath, default_type);
        httpd_resp_set_type(req, default_type);
        return;
    }

    const char *ext = dot + 1;
    ESP_LOGV(TAG, "File extension: '%s'", ext);

    // Map common extensions
    if (strcasecmp(ext, "html") == 0 || strcasecmp(ext, "htm") == 0) httpd_resp_set_type(req, "text/html");
    else if (strcasecmp(ext, "css") == 0) httpd_resp_set_type(req, "text/css");
    else if (strcasecmp(ext, "js") == 0) httpd_resp_set_type(req, "application/javascript");
    else if (strcasecmp(ext, "png") == 0) httpd_resp_set_type(req, "image/png");
    else if (strcasecmp(ext, "jpg") == 0 || strcasecmp(ext, "jpeg") == 0) httpd_resp_set_type(req, "image/jpeg");
    else if (strcasecmp(ext, "gif") == 0) httpd_resp_set_type(req, "image/gif");
    else if (strcasecmp(ext, "ico") == 0) httpd_resp_set_type(req, "image/x-icon");
    else if (strcasecmp(ext, "json") == 0) httpd_resp_set_type(req, "application/json");
    else if (strcasecmp(ext, "svg") == 0) httpd_resp_set_type(req, "image/svg+xml");
    else httpd_resp_set_type(req, default_type);
}

// Main handler function
esp_err_t StaticFileHandler::handleRequest(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    const char *uri = req->uri;
    ESP_LOGD(TAG, "StaticFileHandler handling request for URI: %s", uri);

    // Prepend base path
    strlcpy(filepath, m_base_path, sizeof(filepath));

    // Append URI, handling root case to serve index.html
    if (strcmp(uri, "/") == 0) {
        strlcat(filepath, "/index.html", sizeof(filepath));
    } else {
        // Prevent directory traversal
        if (strstr(uri, "..") != NULL) {
             ESP_LOGW(TAG, "Directory traversal attempt detected: %s", uri);
             return sendHttpError(req, HTTPD_400_BAD_REQUEST, "Invalid URI");
        }
        strlcat(filepath, uri, sizeof(filepath));
    }

    // Log the final path being attempted
    ESP_LOGD(TAG, "Attempting to open file: %s", filepath);

    // --- Open File ---
    FILE* f = fopen(filepath, "rb"); // Use binary read mode
    if (f == NULL) {
        int current_errno = errno;
        ESP_LOGW(TAG, "fopen failed for '%s'. errno: %d (%s)", filepath, current_errno, strerror(current_errno));

        if (current_errno == ENOENT) {
            // Special case: If original request ended in '/', try appending index.html.
            // Root has already been mapped to /index.html above.
            if (strcmp(uri, "/") != 0 && uri[strlen(uri) - 1] == '/') {
                 strlcat(filepath, "index.html", sizeof(filepath));
                 ESP_LOGI(TAG, "Retrying with index.html: %s", filepath);
                 f = fopen(filepath, "rb");
                 if (f != NULL) {
                     // Successfully opened index.html, proceed below
                     ESP_LOGD(TAG,"Found index.html in directory.");
                 } else {
                     // Still not found even with index.html
                     httpd_resp_send_404(req);
                     return ESP_OK;
                 }
            } else {
                 // It was a specific file request that wasn't found
                 httpd_resp_send_404(req);
                 return ESP_OK;
            }
        }
         else {
            // Other file open error
            ESP_LOGE(TAG, "Unhandled file open error for %s", filepath);
            return sendHttpError(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file");
        }
    }

    // --- File Opened Successfully ---
    ESP_LOGD(TAG, "File opened successfully: %s", filepath);
    set_content_type_from_file(req, filepath);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    // --- Allocate Buffer ---
    std::unique_ptr<char[]> chunk_ptr(new (std::nothrow) char[SCRATCH_BUFSIZE]);
    char* chunk = chunk_ptr.get();
    if (!chunk) {
        ESP_LOGE(TAG, "Failed to allocate send buffer");
        fclose(f);
        return sendHttpError(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation error");
    }

    // --- Send File Content ---
    size_t chunksize;
    esp_err_t send_ret = ESP_OK;
    do {
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, f);
        if (ferror(f)) {
            ESP_LOGE(TAG, "Error reading from file: %s", filepath);
            send_ret = ESP_FAIL;
            break;
        }
        if (chunksize > 0) {
            send_ret = httpd_resp_send_chunk(req, chunk, chunksize);
            if (send_ret != ESP_OK) {
                ESP_LOGE(TAG, "httpd_resp_send_chunk failed: %s (%d)", esp_err_to_name(send_ret), send_ret);
                break;
            }
            ESP_LOGV(TAG, "Sent %d byte chunk", chunksize);
        }
    } while (chunksize > 0);

    // --- Cleanup and Finalize ---
    fclose(f);
    ESP_LOGD(TAG, "Closed file: %s", filepath);

    if (send_ret == ESP_OK) {
        // Send final chunk if no previous error
        send_ret = httpd_resp_send_chunk(req, NULL, 0);
        if (send_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed sending final chunk: %s (%d)", esp_err_to_name(send_ret), send_ret);
        }
    } else {
        // If read/send failed earlier, attempt to close session
        httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
    }

    // Log completion based on final status
    if (send_ret == ESP_OK) {
        ESP_LOGD(TAG, "File sending complete for: %s", filepath);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "File sending failed for: %s", filepath);
        return ESP_FAIL;
    }
}
