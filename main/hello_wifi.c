#include "esp_err.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types_generic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "http_parser.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "sensor.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"

#define CONFIG_ESP_WIFI_CHAN 1

// init cam
#define CAM_PIN_PWDN -1  // power down is not used
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 19
#define CAM_PIN_D2 18
#define CAM_PIN_D1 5
#define CAM_PIN_D0 4
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    printf("STA started\n");
    esp_wifi_connect();
  }
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    printf("STA disconnected, reconnecting...\n");
    esp_wifi_connect();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    printf("Wifi connected, got ip: " IPSTR "\n", IP2STR(&event->ip_info.ip));
  }
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
    printf("Access point started\n");
  }
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP) {
    printf("Access point stop\n");
  }
}

esp_err_t root_get_handler(httpd_req_t *req) {
  const char *html = "<h1>ESP32 Config</h1>"
                     "<form action=\"/connect\" method=\"post\">"
                     "SSID: <input name=\"ssid\"><br>"
                     "Password: <input name=\"pass\"><br>"
                     "<button type=\"submit\">Connect</button>"
                     "</form>";

  httpd_resp_send(req, html, strlen(html));
  return ESP_OK;
}

esp_err_t set_sta_post_handler(httpd_req_t *req) {
  char buf[200] = {0}; // Tăng kích thước buffer và khởi tạo 0
  int total_received = 0;
  int ret, remaining = req->content_len;

  // Đọc toàn bộ body vào buffer
  while (remaining > 0 && total_received < sizeof(buf) - 1) {
    int to_read = (remaining < sizeof(buf) - total_received - 1)
                      ? remaining
                      : sizeof(buf) - total_received - 1;
    if ((ret = httpd_req_recv(req, buf + total_received, to_read)) <= 0) {
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        continue;
      }
      return ESP_FAIL;
    }
    total_received += ret;
    remaining -= ret;
  }

  buf[total_received] = '\0'; // Null-terminate
  printf("Received full body: %s\n", buf);

  char ssid[32] = {0};
  char pass[64] = {0};

  // Parse ssid và pass
  char *token = strtok(buf, "&");
  while (token != NULL) {
    if (strncmp(token, "ssid=", 5) == 0) {
      strncpy(ssid, token + 5, sizeof(ssid) - 1);
    } else if (strncmp(token, "pass=", 5) == 0) {
      strncpy(pass, token + 5, sizeof(pass) - 1);
    }
    token = strtok(NULL, "&");
  }

  printf("Parsed SSID: %s, Pass: %s\n", ssid, pass);

  wifi_config_t sta_cfg;
  memset(&sta_cfg, 0, sizeof(wifi_config_t));

  strcpy((char *)sta_cfg.sta.ssid, ssid);
  strcpy((char *)sta_cfg.sta.password, pass);
  sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  sta_cfg.sta.pmf_cfg.capable = true;
  sta_cfg.sta.pmf_cfg.required = false;
  esp_wifi_set_mode(WIFI_MODE_APSTA);
  esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
  if (err != ESP_OK) {
    printf("Set config failed: %d\n", err);
    httpd_resp_send(req, "Can not set config wifi!", HTTPD_RESP_USE_STRLEN);
    return err;
  }

  err = esp_wifi_connect();
  if (err != ESP_OK) {
    printf("Connect failed: %d\n", err);
    httpd_resp_send(req, "Can not connect to wifi!", HTTPD_RESP_USE_STRLEN);
    return err;
  }

  httpd_resp_send(req, "Connecting...", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

void startWebServer() {
  httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
  http_config.max_uri_handlers = 8;
  http_config.stack_size = 8192;
  httpd_handle_t server = NULL;
  ESP_ERROR_CHECK(httpd_start(&server, &http_config));

  httpd_uri_t root = {
      .uri = "/", .method = HTTP_GET, .handler = root_get_handler};
  httpd_uri_t postSTA = {
      .uri = "/connect", .method = HTTP_POST, .handler = set_sta_post_handler};

  httpd_register_uri_handler(server, &root);
  httpd_register_uri_handler(server, &postSTA);
}

void app_main(void) {

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event_handler, NULL));
  esp_netif_create_default_wifi_sta();

  wifi_config_t stored_config;
  memset(&stored_config, 0, sizeof(wifi_config_t));
  bool hasAPConfig = strlen((char *)stored_config.ap.ssid);
  bool hasSTAConfig = strlen((char *)stored_config.sta.ssid);

  wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&config));
  if (hasAPConfig && hasSTAConfig) {
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

  } else {
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  }

  if (!hasAPConfig) {
    wifi_config_t wifi_ap_config = {
        .ap =
            {
                .ssid = CONFIG_WIFI_SSID,
                .ssid_len = strlen(CONFIG_WIFI_SSID),
                .channel = CONFIG_ESP_WIFI_CHAN,
                .password = CONFIG_WIFI_PASSWORD,
                .max_connection = CONFIG_MAX_STA_CONN,
                .authmode = WIFI_AUTH_WPA2_PSK,
                .pmf_cfg =
                    {
                        .required = false,
                    },
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
  } else {
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &stored_config));
  }

  if (hasSTAConfig) {
    esp_netif_create_default_wifi_ap();
    printf("Found existed sta config: %s, pass: %s. Set now!",
           stored_config.sta.ssid, stored_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &stored_config));
  }

  esp_wifi_start();
  startWebServer();
}