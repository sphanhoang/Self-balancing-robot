#include "balancing_robot.h"

#define CONFIG_WIFI_SSID        "salt5g"
#define CONFIG_WIFI_PASSWORD    "68686668"


static const char *TAG_WEB = "WEB";
web_control_t web_control = {
    .kp_roll = KP_ROLL,
    .ki_roll = KI_ROLL,
    .kd_roll = KD_ROLL,
    .kp_speed = KP_SPEED,
    .ki_speed = KI_SPEED,
    .kd_speed = KD_SPEED,
    .enable_balance = false,
    .target_angle = 0.5f
};

// HTML Page with real-time charts and controls
static const char* HTML_PAGE = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Balancing Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body { font-family: Arial; margin: 20px; }
        .panel { background: #f0f0f0; padding: 15px; margin: 10px; border-radius: 5px; }
        .slider-container { margin: 10px 0; }
        .value-display { display: inline-block; width: 60px; text-align: right; }
    </style>
</head>
<body>
    <h1>🤖 Balancing Robot Control</h1>
    
    <div class="panel">
        <h3>📊 Real-time Data</h3>
        <canvas id="dataChart" width="400" height="200"></canvas>
        <div id="status">Angle: <span id="angleValue">0.0</span>° | Output: <span id="outputValue">0</span></div>
    </div>

    <div class="panel">
        <h3>⚙️ Roll PID Tuning</h3>
        <div class="slider-container">
            KP: <input type="range" id="kpRoll" min="0" max="50" step="0.1" class="pid-slider">
            <span id="kpRollValue" class="value-display">20.0</span>
        </div>
        <div class="slider-container">
            KI: <input type="range" id="kiRoll" min="0" max="10" step="0.1" class="pid-slider">
            <span id="kiRollValue" class="value-display">2.0</span>
        </div>
        <div class="slider-container">
            KD: <input type="range" id="kdRoll" min="0" max="5" step="0.1" class="pid-slider">
            <span id="kdRollValue" class="value-display">0.8</span>
        </div>
    </div>

    <div class="panel">
        <h3>🎮 Control</h3>
        <div class="slider-container">
            Tilt Angle: <input type="range" id="target_angle" min="-1" max="1" step="0.01" class="pid-slider">
            <span id="target_angleValue" class="value-display">0.0</span>
        </div>
        <button id="enableBtn" onclick="toggleBalance()">Enable Balancing</button>
        <button onclick="saveParameters()">💾 Save to Flash</button>
        <button onclick="resetParameters()">🔄 Reset Defaults</button>
    </div>

    <script>
        let chart;
        let balanceEnabled = false;
        
        // Initialize Chart
        function initChart() {
            const ctx = document.getElementById('dataChart').getContext('2d');
            chart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        { label: 'Angle (°)', borderColor: 'red', data: [] },
                        { label: 'PID Output', borderColor: 'blue', data: [] }
                    ]
                },
                options: { responsive: true, scales: { y: { min: -20, max: 20 } } }
            });
        }

        // Update PID values from sliders
        document.querySelectorAll('.pid-slider').forEach(slider => {
            slider.oninput = function() {
                document.getElementById(this.id + 'Value').textContent = this.value;
                updatePID();
            };
        });

        // Toggle balance enable/disable
        function toggleBalance() {
            balanceEnabled = !balanceEnabled;
            fetch('/control?enable=' + balanceEnabled);
            document.getElementById('enableBtn').textContent = 
                balanceEnabled ? 'Disable Balancing' : 'Enable Balancing';
        }

        // Update PID parameters
        function updatePID() {
            const params = new URLSearchParams({
                kp_r: document.getElementById('kpRoll').value,
                ki_r: document.getElementById('kiRoll').value,
                kd_r: document.getElementById('kdRoll').value,
                kp_s: document.getElementById('kpSpeed').value,
                ki_s: document.getElementById('kiSpeed').value,
                kd_s: document.getElementById('kdSpeed').value,
                target_angle: document.getElementById('target_angle').value
            });
            fetch('/pid?' + params);
        }

        // Real-time data update
        function fetchData() {
            fetch('/data')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('angleValue').textContent = data.angle.toFixed(1);
                    document.getElementById('outputValue').textContent = data.output.toFixed(0);
                    
                    // Update chart
                    if(chart.data.labels.length > 50) {
                        chart.data.labels.shift();
                        chart.data.datasets[0].data.shift();
                        chart.data.datasets[1].data.shift();
                    }
                    chart.data.labels.push(new Date().toLocaleTimeString());
                    chart.data.datasets[0].data.push(data.angle);
                    chart.data.datasets[1].data.push(data.output);
                    chart.update();
                });
        }
        setInterval(fetchData, 100); // Update every 100ms
        initChart();
    </script>
</body>
</html>
)rawliteral";

// HTTP Request Handlers
esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, HTML_PAGE, strlen(HTML_PAGE));
}

esp_err_t data_handler(httpd_req_t *req) {
    char json_response[256];
    sensor_data_t sensor_data_local;
    control_data_t control_data_local;
    
    // Get current data safely
    if (xSemaphoreTake(mutex_sensor_data, pdMS_TO_TICKS(10)) == pdTRUE) {
        sensor_data_local = sensor_data;
        xSemaphoreGive(mutex_sensor_data);
    }
    
    control_data_local = control_data; // This is relatively safe to read without mutex
    
    snprintf(json_response, sizeof(json_response),
             "{\"angle\":%.1f,\"output\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
             sensor_data_local.roll, control_data_local.roll_output,
             sensor_data_local.pitch, sensor_data_local.yaw);
    
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

esp_err_t pid_handler(httpd_req_t *req) {
    char query[100];
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    
    if (query_len > 1) {
        httpd_req_get_url_query_str(req, query, query_len);
        
        // Parse parameters
        char param[10];
        if (httpd_query_key_value(query, "kp_r", param, sizeof(param)) == ESP_OK) {
            if (xSemaphoreTake(mutex_web_data, pdMS_TO_TICKS(10)) == pdTRUE) {
                web_control.kp_roll = atof(param);
                xSemaphoreGive(mutex_web_data);
            }
        }
        // Similar parsing for ki_r, kd_r, kp_s, ki_s, kd_s...
        if (httpd_query_key_value(query, "ki_r", param, sizeof(param)) == ESP_OK) {
            if (xSemaphoreTake(mutex_web_data, pdMS_TO_TICKS(10)) == pdTRUE) {
                web_control.ki_roll = atof(param);
                xSemaphoreGive(mutex_web_data);
            }
        }
        if (httpd_query_key_value(query, "kd_r", param, sizeof(param)) == ESP_OK) {
            if (xSemaphoreTake(mutex_web_data, pdMS_TO_TICKS(10)) == pdTRUE) {
                web_control.kd_roll = atof(param);
                xSemaphoreGive(mutex_web_data);
            }
        }
        if (httpd_query_key_value(query, "target_angle", param, sizeof(param)) == ESP_OK) {
            if (xSemaphoreTake(mutex_web_data, pdMS_TO_TICKS(10)) == pdTRUE) {
                web_control.target_angle = atof(param);
                xSemaphoreGive(mutex_web_data);
            }
        }
    }
    return httpd_resp_send(req, "OK", 2);
}

esp_err_t control_handler(httpd_req_t *req) {
    char query[100];
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    
    if (query_len > 1) {
        httpd_req_get_url_query_str(req, query, query_len);
        
        char param[10];
        if (httpd_query_key_value(query, "enable", param, sizeof(param)) == ESP_OK) {
            if (xSemaphoreTake(mutex_web_data, pdMS_TO_TICKS(10)) == pdTRUE) {
                web_control.enable_balance = (atoi(param) == 1);
                xSemaphoreGive(mutex_web_data);
            }
        }
    }
    
    return httpd_resp_send(req, "OK", 2);
}

// HTTP Server Configuration
static httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_handler
};

static httpd_uri_t data_uri = {
    .uri = "/data",
    .method = HTTP_GET,
    .handler = data_handler
};

static httpd_uri_t pid_uri = {
    .uri = "/pid",
    .method = HTTP_GET,
    .handler = pid_handler
};

static httpd_uri_t control_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = control_handler
};

// WiFi Initialization
void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    
    ESP_LOGI(TAG_WEB, "WiFi initialization finished.");
}

// Web Server Task (Runs on Core 0)
void web_server_task(void *pvParameters) {
    ESP_LOGI(TAG_WEB, "Starting web server task on core %d", xPortGetCoreID());
    
    // Initialize WiFi
    wifi_init();
    
    // Wait for WiFi connection
    int retry_count = 0;
    while (esp_wifi_connect() != ESP_OK && retry_count < 10) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        retry_count++;
    }
    
    // Start HTTP Server
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.stack_size = WEB_SERVER_STACK_SIZE;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &data_uri);
        httpd_register_uri_handler(server, &pid_uri);
        httpd_register_uri_handler(server, &control_uri);
        ESP_LOGI(TAG_WEB, "Web server started on port %d", config.server_port);
    }
    
    // Print IP address
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        ESP_LOGI(TAG_WEB, "Connect to: http://" IPSTR, IP2STR(&ip_info.ip));
    }
    
    // Keep task alive
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}