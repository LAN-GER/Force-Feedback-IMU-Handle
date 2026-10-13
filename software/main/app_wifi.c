/** Wi-Fi STA provisioning over USB and a single authenticated WebSocket client. */
#include "app_wifi.h"
#include "app.h"
#include "bsp/bsp_usb.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    bool enabled;
    char ssid[33];
    char password[65];
    char token[33];
} wifi_settings_t;

typedef struct { int fd; unsigned epoch; } wifi_peer_t;
typedef struct {
    wifi_peer_t peer;
    httpd_handle_t server;
    char json[BSP_USB_JSON_CAPACITY];
} wifi_frame_t;
typedef struct { wifi_settings_t settings; unsigned revision; } wifi_request_t;

static const char *TAG = "app_wifi";
static portMUX_TYPE s_guard = portMUX_INITIALIZER_UNLOCKED;
static wifi_settings_t s_settings;
static bool s_ready, s_connected, s_running, s_subscribed, s_pending, s_busy;
static unsigned s_revision;
static esp_err_t s_error;
static int s_fd = -1;
static unsigned s_epoch;
static unsigned s_reason;
static char s_ip[16];
static QueueHandle_t s_config_queue;
static httpd_handle_t s_server;
static SemaphoreHandle_t s_server_guard;
static bool s_accepting;
static wifi_frame_t *s_queued;
/* Driver/netif lifecycle belongs exclusively to wifi_worker. */
static esp_netif_t *s_netif;
static bool s_driver, s_wifi_handler, s_ip_handler, s_network_initialized;

static void make_token(char token[33])
{
    uint8_t random[16];
    esp_fill_random(random, sizeof(random));
    for (size_t i = 0; i < sizeof(random); i++) snprintf(token + i * 2, 3, "%02x", random[i]);
}

static esp_err_t save_settings(const wifi_settings_t *settings)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ffh_wifi", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_str(nvs, "ssid", settings->ssid);
    if (err == ESP_OK) err = nvs_set_str(nvs, "password", settings->password);
    if (err == ESP_OK) err = nvs_set_str(nvs, "token", settings->token);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "enabled", settings->enabled);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

static void peer_free(void *ctx)
{
    wifi_peer_t *peer = ctx;
    portENTER_CRITICAL(&s_guard);
    if (s_fd == peer->fd && s_epoch == peer->epoch) {
        s_fd = -1;
        s_epoch++;
        s_subscribed = false;
    }
    portEXIT_CRITICAL(&s_guard);
    free(peer);
}

static esp_err_t ws_authorize(httpd_req_t *req)
{
    char query[96], token[40];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "token", token, sizeof(token)) != ESP_OK || strlen(token) != 32) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "USB access key required");
        return ESP_FAIL;
    }
    unsigned difference = 0;
    portENTER_CRITICAL(&s_guard);
    for (size_t i = 0; i < 32; i++) difference |= (unsigned char)token[i] ^ (unsigned char)s_settings.token[i];
    bool occupied = s_fd >= 0;
    bool connected = s_connected && s_settings.enabled && !s_busy;
    portEXIT_CRITICAL(&s_guard);
    if (difference || !connected) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Invalid access key");
        return ESP_FAIL;
    }
    if (occupied) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "Another Wi-Fi client is connected");
        return ESP_FAIL;
    }
    wifi_peer_t *peer = calloc(1, sizeof(*peer));
    if (!peer) return ESP_ERR_NO_MEM;
    portENTER_CRITICAL(&s_guard);
    peer->fd = s_fd = httpd_req_to_sockfd(req);
    peer->epoch = ++s_epoch;
    s_subscribed = false;
    portEXIT_CRITICAL(&s_guard);
    req->sess_ctx = peer;
    req->free_ctx = peer_free;
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use WebSocket /ws");
        return ESP_FAIL;
    }
    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) return err;
    /* Only one complete text command, with the same 255-byte USB limit. */
    if (frame.type != HTTPD_WS_TYPE_TEXT || !frame.final || !frame.len || frame.len > 255) return ESP_FAIL;
    char line[256] = {0};
    frame.payload = (uint8_t *)line;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK || memchr(line, '\0', frame.len)) return ESP_FAIL;
    app_protocol_wifi_line(line);
    return ESP_OK;
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = data;
        char ip[16] = {0};
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&event->ip_info.ip));
        portENTER_CRITICAL(&s_guard);
        memcpy(s_ip, ip, sizeof(s_ip));
        s_connected = s_settings.enabled && s_running;
        s_reason = 0;
        portEXIT_CRITICAL(&s_guard);
        ESP_LOGI(TAG, "Wi-Fi ready: ws://%s/ws", ip);
    } else if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_DISCONNECTED || id == WIFI_EVENT_STA_STOP)) {
        portENTER_CRITICAL(&s_guard);
        s_connected = false;
        s_ip[0] = '\0';
        s_subscribed = false;
        if (id == WIFI_EVENT_STA_DISCONNECTED) s_reason = ((wifi_event_sta_disconnected_t *)data)->reason;
        int fd = s_fd;
        portEXIT_CRITICAL(&s_guard);
        xSemaphoreTake(s_server_guard, portMAX_DELAY);
        if (fd >= 0 && s_accepting && s_server) httpd_sess_trigger_close(s_server, fd);
        xSemaphoreGive(s_server_guard);
    }
}

static esp_err_t stop_network(void)
{
    portENTER_CRITICAL(&s_guard);
    s_running = false;
    s_connected = false;
    s_subscribed = false;
    s_ip[0] = '\0';
    s_reason = 0;
    portEXIT_CRITICAL(&s_guard);
    /* Block new queue submissions before deleting the HTTP handle. HTTP callbacks
     * never take this mutex, so stop can wait for an in-flight command safely. */
    xSemaphoreTake(s_server_guard, portMAX_DELAY);
    s_accepting = false;
    xSemaphoreGive(s_server_guard);
    if (s_server) {
        esp_err_t err = httpd_stop(s_server);
        if (err != ESP_OK) return err;
        xSemaphoreTake(s_server_guard, portMAX_DELAY);
        s_server = NULL;
        xSemaphoreGive(s_server_guard);
    }
    /* A control datagram may have been lost; no callback can run after stop. */
    portENTER_CRITICAL(&s_guard);
    wifi_frame_t *orphan = s_queued;
    s_queued = NULL;
    s_pending = false;
    s_fd = -1;
    s_epoch++;
    portEXIT_CRITICAL(&s_guard);
    free(orphan);
    if (s_wifi_handler) {
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event);
        s_wifi_handler = false;
    }
    if (s_ip_handler) {
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event);
        s_ip_handler = false;
    }
    if (s_driver) {
        esp_err_t err = esp_wifi_stop();
        if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) return err;
        err = esp_wifi_deinit();
        if (err != ESP_OK) return err;
        s_driver = false;
    }
    if (s_netif) {
        esp_netif_destroy_default_wifi(s_netif);
        s_netif = NULL;
    }
    return ESP_OK;
}

static esp_err_t start_network(const wifi_settings_t *settings)
{
    esp_err_t err;
    /* lwIP and the shared event loop are initialized lazily and kept globally;
     * esp_netif_deinit is unsupported by ESP-IDF. Wi-Fi-specific objects are freed. */
    if (!s_network_initialized) {
        err = esp_netif_init();
        if (err != ESP_OK) return err;
        err = esp_event_loop_create_default();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
        s_network_initialized = true;
    }
    /* The convenience create_default helper aborts on allocation failure. */
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
    s_netif = esp_netif_new(&netif_config);
    if (!s_netif) return ESP_ERR_NO_MEM;
    err = esp_netif_attach_wifi_station(s_netif);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_default_wifi_sta_handlers();
    if (err != ESP_OK) return err;
    (void)esp_netif_set_hostname(s_netif, "ff-handle");
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init);
    if (err != ESP_OK) return err;
    s_driver = true;
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    if (err != ESP_OK) return err;
    s_wifi_handler = true;
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL);
    if (err != ESP_OK) return err;
    s_ip_handler = true;
    httpd_config_t http = HTTPD_DEFAULT_CONFIG();
    http.stack_size = 8192;
    http.max_open_sockets = 3;
    http.send_wait_timeout = 1;
    http.recv_wait_timeout = 2;
    http.keep_alive_enable = true;
    http.keep_alive_idle = 10;
    http.keep_alive_interval = 5;
    http.keep_alive_count = 3;
    err = httpd_start(&s_server, &http);
    if (err != ESP_OK) return err;
    const httpd_uri_t ws = {.uri = "/ws", .method = HTTP_GET, .handler = ws_handler,
        .is_websocket = true, .ws_pre_handshake_cb = ws_authorize};
    err = httpd_register_uri_handler(s_server, &ws);
    if (err != ESP_OK) return err;
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, settings->ssid, strlen(settings->ssid));
    memcpy(config.sta.password, settings->password, strlen(settings->password));
    config.sta.threshold.authmode = settings->password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    err = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (err != ESP_OK) return err;
    portENTER_CRITICAL(&s_guard);
    s_running = true;
    portEXIT_CRITICAL(&s_guard);
    err = esp_wifi_start();
    if (err != ESP_OK) return err;
    (void)esp_wifi_set_ps(WIFI_PS_NONE);
    xSemaphoreTake(s_server_guard, portMAX_DELAY);
    s_accepting = true;
    xSemaphoreGive(s_server_guard);
    return ESP_OK;
}

static void wifi_worker(void *arg)
{
    (void)arg;
    wifi_request_t request;
    for (;;) {
        portENTER_CRITICAL(&s_guard);
        bool running = s_running;
        portEXIT_CRITICAL(&s_guard);
        /* With Wi-Fi off there are no periodic reconnect wakeups. */
        if (xQueueReceive(s_config_queue, &request, running ? pdMS_TO_TICKS(2000) : portMAX_DELAY) == pdTRUE) {
            esp_err_t err = stop_network();
            if (err == ESP_OK && request.settings.enabled) {
                err = start_network(&request.settings);
                if (err != ESP_OK) {
                    esp_err_t cleanup = stop_network();
                    if (cleanup != ESP_OK) ESP_LOGE(TAG, "Wi-Fi cleanup failed: %s", esp_err_to_name(cleanup));
                }
            }
            portENTER_CRITICAL(&s_guard);
            if (request.revision == s_revision) { s_busy = false; s_error = err; }
            portEXIT_CRITICAL(&s_guard);
            if (err != ESP_OK) ESP_LOGE(TAG, "Wi-Fi apply failed: %s", esp_err_to_name(err));
            else ESP_LOGI(TAG, "Wi-Fi %s", request.settings.enabled ? "enabled" : "off; driver and WebSocket released");
        }
        portENTER_CRITICAL(&s_guard);
        bool reconnect = s_settings.enabled && s_running && !s_connected && !s_busy;
        portEXIT_CRITICAL(&s_guard);
        if (reconnect) (void)esp_wifi_connect();
    }
}

static esp_err_t apply_settings(const wifi_settings_t *settings)
{
    esp_err_t err = save_settings(settings);
    if (err != ESP_OK) return err;
    portENTER_CRITICAL(&s_guard);
    s_settings = *settings;
    s_connected = false;
    s_subscribed = false;
    s_ip[0] = '\0';
    s_reason = 0;
    s_error = ESP_OK;
    s_busy = true;
    wifi_request_t request = {.settings = *settings, .revision = ++s_revision};
    portEXIT_CRITICAL(&s_guard);
    xQueueOverwrite(s_config_queue, &request);
    return ESP_OK;
}

esp_err_t app_wifi_init(void)
{
    nvs_handle_t nvs;
    if (nvs_open("ffh_wifi", NVS_READONLY, &nvs) == ESP_OK) {
        size_t size = sizeof(s_settings.ssid);
        (void)nvs_get_str(nvs, "ssid", s_settings.ssid, &size);
        size = sizeof(s_settings.password);
        (void)nvs_get_str(nvs, "password", s_settings.password, &size);
        size = sizeof(s_settings.token);
        (void)nvs_get_str(nvs, "token", s_settings.token, &size);
        uint8_t enabled = 0;
        (void)nvs_get_u8(nvs, "enabled", &enabled);
        s_settings.enabled = enabled && s_settings.ssid[0];
        nvs_close(nvs);
    }
    /* No Wi-Fi driver, netif, HTTP server or radio is created while off. */
    if (strlen(s_settings.token) != 32 || strspn(s_settings.token, "0123456789abcdef") != 32) {
        make_token(s_settings.token);
        esp_err_t err = save_settings(&s_settings);
        if (err != ESP_OK) return err;
    }
    s_server_guard = xSemaphoreCreateMutex();
    if (!s_server_guard) return ESP_ERR_NO_MEM;
    s_config_queue = xQueueCreate(1, sizeof(wifi_request_t));
    if (!s_config_queue) { vSemaphoreDelete(s_server_guard); s_server_guard = NULL; return ESP_ERR_NO_MEM; }
    s_busy = s_settings.enabled;
    wifi_request_t request = {.settings = s_settings, .revision = ++s_revision};
    if (s_settings.enabled) xQueueOverwrite(s_config_queue, &request);
    if (xTaskCreate(wifi_worker, "app_wifi", 4096, NULL, 3, NULL) != pdPASS) {
        vQueueDelete(s_config_queue); s_config_queue = NULL;
        vSemaphoreDelete(s_server_guard); s_server_guard = NULL;
        return ESP_ERR_NO_MEM;
    }
    portENTER_CRITICAL(&s_guard);
    s_ready = true;
    portEXIT_CRITICAL(&s_guard);
    return ESP_OK;
}

esp_err_t app_wifi_enable(const char *json)
{
    portENTER_CRITICAL(&s_guard);
    bool ready = s_ready;
    wifi_settings_t settings = s_settings;
    portEXIT_CRITICAL(&s_guard);
    if (!ready) return ESP_ERR_INVALID_STATE;
    cJSON *root = cJSON_Parse(json);
    if (!root) return ESP_ERR_INVALID_ARG;
    const cJSON *value = cJSON_GetObjectItemCaseSensitive(root, "v");
    bool valid = cJSON_IsNumber(value) && (value->valuedouble == 0 || value->valuedouble == 1);
    bool enabled = valid && value->valuedouble == 1;
    cJSON_Delete(root);
    if (!valid) return ESP_ERR_INVALID_ARG;
    if (enabled && !settings.ssid[0]) return ESP_ERR_INVALID_STATE;
    settings.enabled = enabled;
    return apply_settings(&settings);
}

esp_err_t app_wifi_configure(const char *json)
{
    portENTER_CRITICAL(&s_guard);
    bool ready = s_ready;
    portEXIT_CRITICAL(&s_guard);
    if (!ready) return ESP_ERR_INVALID_STATE;
    /* Reject embedded NUL escapes instead of silently truncating C credentials. */
    for (const char *p = json; *p; p++) {
        if (*p == '\\' && p[1]) {
            if (strncmp(p + 1, "u0000", 5) == 0) return ESP_ERR_INVALID_ARG;
            p++;
        }
    }
    cJSON *root = cJSON_Parse(json);
    if (!root) return ESP_ERR_INVALID_ARG;
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(root, "ssid");
    const cJSON *password = cJSON_GetObjectItemCaseSensitive(root, "password");
    bool valid = cJSON_IsString(ssid) && cJSON_IsString(password) &&
        strlen(ssid->valuestring) > 0 && strlen(ssid->valuestring) <= 32;
    if (valid) {
        size_t len = strlen(password->valuestring);
        valid = len == 0 || (len >= 8 && len <= 63);
        if (len == 64) {
            valid = strspn(password->valuestring, "0123456789abcdefABCDEF") == 64;
        }
    }
    if (!valid) { cJSON_Delete(root); return ESP_ERR_INVALID_ARG; }
    portENTER_CRITICAL(&s_guard);
    wifi_settings_t settings = s_settings;
    portEXIT_CRITICAL(&s_guard);
    snprintf(settings.ssid, sizeof(settings.ssid), "%s", ssid->valuestring);
    snprintf(settings.password, sizeof(settings.password), "%s", password->valuestring);
    cJSON_Delete(root);
    return apply_settings(&settings);
}

esp_err_t app_wifi_forget(void)
{
    portENTER_CRITICAL(&s_guard);
    bool ready = s_ready;
    portEXIT_CRITICAL(&s_guard);
    if (!ready) return ESP_ERR_INVALID_STATE;
    wifi_settings_t settings = {0};
    make_token(settings.token);
    return apply_settings(&settings);
}

void app_wifi_status_json(char *out, size_t size, const char *cmd, bool usb)
{
    portENTER_CRITICAL(&s_guard);
    wifi_settings_t settings = s_settings;
    bool ready = s_ready, connected = s_connected, running = s_running, busy = s_busy;
    esp_err_t error = s_error;
    unsigned reason = s_reason;
    char ip[sizeof(s_ip)];
    memcpy(ip, s_ip, sizeof(ip));
    portEXIT_CRITICAL(&s_guard);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "t", "ok");
    cJSON_AddStringToObject(root, "cmd", cmd);
    cJSON *wifi = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddNumberToObject(wifi, "ready", ready);
    cJSON_AddNumberToObject(wifi, "enabled", settings.enabled);
    cJSON_AddNumberToObject(wifi, "running", running);
    cJSON_AddNumberToObject(wifi, "busy", busy);
    cJSON_AddStringToObject(wifi, "error", error == ESP_OK ? "" : esp_err_to_name(error));
    cJSON_AddNumberToObject(wifi, "connected", connected);
    cJSON_AddStringToObject(wifi, "ssid", settings.ssid);
    cJSON_AddStringToObject(wifi, "ip", ip);
    cJSON_AddNumberToObject(wifi, "reason", reason);
    char url[48] = "";
    if (connected) snprintf(url, sizeof(url), "ws://%s/ws", ip);
    cJSON_AddStringToObject(wifi, "ws_url", url);
    if (usb && ready) cJSON_AddStringToObject(wifi, "token", settings.token);
    if (!cJSON_PrintPreallocated(root, out, (int)size, false)) {
        snprintf(out, size, "{\"t\":\"err\",\"cmd\":\"wifi_get\",\"msg\":\"no memory\"}");
    }
    cJSON_Delete(root);
}

bool app_wifi_subscribed(void)
{
    portENTER_CRITICAL(&s_guard);
    bool active = s_settings.enabled && !s_busy && s_connected && s_fd >= 0 && s_subscribed;
    portEXIT_CRITICAL(&s_guard);
    return active;
}

void app_wifi_subscribe(bool enabled)
{
    portENTER_CRITICAL(&s_guard);
    s_subscribed = enabled;
    portEXIT_CRITICAL(&s_guard);
}

void app_wifi_reply(const char *json)
{
    /* Command replies run only on the HTTP server task; never interleave frames. */
    portENTER_CRITICAL(&s_guard);
    int fd = s_fd;
    portEXIT_CRITICAL(&s_guard);
    if (fd < 0) return;
    httpd_ws_frame_t frame = {.type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)json, .len = strlen(json)};
    if (httpd_ws_send_frame_async(s_server, fd, &frame) != ESP_OK) httpd_sess_trigger_close(s_server, fd);
}

static void send_telem(void *arg)
{
    wifi_frame_t *queued = arg;
    portENTER_CRITICAL(&s_guard);
    bool valid = s_settings.enabled && !s_busy && s_connected && s_subscribed && queued->peer.fd == s_fd && queued->peer.epoch == s_epoch;
    portEXIT_CRITICAL(&s_guard);
    if (valid && httpd_ws_get_fd_info(queued->server, queued->peer.fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
        httpd_ws_frame_t frame = {.type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)queued->json,
            .len = strlen(queued->json)};
        if (httpd_ws_send_frame_async(queued->server, queued->peer.fd, &frame) != ESP_OK) {
            httpd_sess_trigger_close(queued->server, queued->peer.fd);
        }
    }
    free(queued);
    portENTER_CRITICAL(&s_guard);
    s_pending = false;
    s_queued = NULL;
    portEXIT_CRITICAL(&s_guard);
}

void app_wifi_push_json(const char *json)
{
    if (!s_server_guard) return;
    /* The sampler never blocks on network teardown. */
    if (xSemaphoreTake(s_server_guard, 0) != pdTRUE) return;
    portENTER_CRITICAL(&s_guard);
    bool active = s_accepting && s_settings.enabled && !s_busy && s_connected &&
        s_fd >= 0 && s_subscribed && !s_pending;
    wifi_peer_t peer = {.fd = s_fd, .epoch = s_epoch};
    if (active) s_pending = true;
    portEXIT_CRITICAL(&s_guard);
    if (!active) { xSemaphoreGive(s_server_guard); return; }
    wifi_frame_t *queued = malloc(sizeof(*queued));
    if (queued) {
        queued->peer = peer;
        queued->server = s_server;
        snprintf(queued->json, sizeof(queued->json), "%s", json);
        portENTER_CRITICAL(&s_guard);
        s_queued = queued;
        portEXIT_CRITICAL(&s_guard);
        if (httpd_queue_work(s_server, send_telem, queued) == ESP_OK) {
            xSemaphoreGive(s_server_guard);
            return;
        }
        free(queued);
    }
    portENTER_CRITICAL(&s_guard);
    s_queued = NULL;
    s_pending = false;
    portEXIT_CRITICAL(&s_guard);
    xSemaphoreGive(s_server_guard);
}
