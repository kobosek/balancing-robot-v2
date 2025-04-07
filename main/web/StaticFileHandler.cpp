#include "StaticFileHandler.hpp"
#include "esp_vfs.h" // For fopen, etc.
#include "esp_check.h"
#include <memory> // For unique_ptr
#include <cstring> // For strcmp, strrchr, etc.
#include <cerrno> // For errno
#include <sys/param.h> // For MAX/MIN (optional, could use std::max/min)

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240) // Larger buffer for potentially faster sends

StaticFileHandler::StaticFileHandler(const char* base_path) : m_base_path(base_path) {}

// Sets the Content-Type header based on file extension
void StaticFileHandler::set_content_type_from_file(httpd_req_t *req, const char *filename) {
    const char *dot = strrchr(filename, '.');
    const char *default_type = "text/plain";

    if (!dot || dot == filename) {
        httpd_resp_set_type(req, default_type);
        return;
    }

    const char *ext = dot + 1;
    ESP_LOGV(TAG, "Setting content type for extension: %s", ext);

    // Map extensions to MIME types
    if (strcasecmp(ext, "html") == 0 || strcasecmp(ext, "htm") == 0) httpd_resp_set_type(req, "text/html");
    else if (strcasecmp(ext, "css") == 0) httpd_resp_set_type(req, "text/css");
    else if (strcasecmp(ext, "js") == 0) httpd_resp_set_type(req, "application/javascript");
    else if (strcasecmp(ext, "png") == 0) httpd_resp_set_type(req, "image/png");
    else if (strcasecmp(ext, "jpg") == 0) httpd_resp_set_type(req, "image/jpeg");
    else if (strcasecmp(ext, "jpeg") == 0) httpd_resp_set_type(req, "image/jpeg");
    else if (strcasecmp(ext, "gif") == 0) httpd_resp_set_type(req, "image/gif");
    else if (strcasecmp(ext, "ico") == 0) httpd_resp_set_type(req, "image/x-icon");
    else if (strcasecmp(ext, "json") == 0) httpd_resp_set_type(req, "application/json");
    else if (strcasecmp(ext, "svg") == 0) httpd_resp_set_type(req, "image/svg+xml");
    else httpd_resp_set_type(req, default_type);
}

esp_err_t StaticFileHandler::handleRequest(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    strlcpy(filepath, m_base_path, sizeof(filepath));

    // Append URI, ensuring it doesn't start with '../' or contain '..'.
    // Also handle the root case.
    if (req->uri[0] == '/' && strlen(req->uri) == 1) {
         strlcat(filepath, "/index.html", sizeof(filepath));
    } else if (strncmp(req->uri, "/../", 4) == 0 || strstr(req->uri, "/../")) {
        ESP_LOGW(TAG, "Directory traversal attempt detected: %s", req->uri);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid URI");
        return ESP_FAIL;
    } else {
         // Append the requested URI to the base path
         strlcat(filepath, req->uri, sizeof(filepath));
    }


    ESP_LOGI(TAG, "Serving file: %s", filepath);

    // Open the file
    FILE* f = fopen(filepath, "r");
    if (f == NULL) {
        if (errno == ENOENT) {
             ESP_LOGW(TAG, "File not found: %s", filepath);
             // Try serving index.html if it's a directory-like request
             if (filepath[strlen(filepath) - 1] == '/') {
                  strlcat(filepath, "index.html", sizeof(filepath));
                  f = fopen(filepath, "r");
             }
             if (f == NULL) {
                  httpd_resp_send_404(req);
                  return ESP_OK;
             }
        } else {
            ESP_LOGE(TAG, "Failed to open file %s: %s", filepath, strerror(errno));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file");
            return ESP_FAIL;
        }
    }

    set_content_type_from_file(req, filepath);

    // Use a larger buffer for potentially faster transfer
    std::unique_ptr<char[]> chunk_ptr(new (std::nothrow) char[SCRATCH_BUFSIZE]);
    char* chunk = chunk_ptr.get();
    if (!chunk) {
         ESP_LOGE(TAG, "Failed to allocate scratch buffer for file");
         fclose(f);
         httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation error");
         return ESP_FAIL;
    }

    size_t chunksize;
    esp_err_t ret = ESP_OK;
    do {
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, f);
        if (chunksize > 0) {
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                fclose(f);
                ESP_LOGE(TAG, "File sending failed!");
                // Abort sending file
                httpd_resp_send_chunk(req, NULL, 0); // Finalize with error indication? Might not work.
                httpd_resp_send_500(req); // Try sending 500
                return ESP_FAIL;
            }
        }
    } while (chunksize != 0); // Loop until EOF

    fclose(f);
    ESP_LOGD(TAG, "File sending complete");

    // Finalize the response (send zero-length chunk)
    ret = httpd_resp_send_chunk(req, NULL, 0);
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed sending final chunk");
         // Can't reliably send another error if previous chunks succeeded
    }
    return ret;
}