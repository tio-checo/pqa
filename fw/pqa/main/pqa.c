/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "time_sync.h"
#include "mbedtls/esp_debug.h"

#include "driver/gpio.h"
#include "freertos/idf_additions.h"

#include "daq.h"
#include "mcp3913.h"

#include "esp_dsp.h"
#include "dsps_fft2r.h"

#include "esp_http_client.h"

/* global constants */
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

/* HTTPS */
#define WEB_SERVER "api.thingspeak.com"
//#define WEB_PORT "443"
#define	THINGSPEAK_API_KEY	"TBD"
//#define WEB_URL "https://api.thingspeak.com/update?api_key=G51YVQKV9BMRHF6D&field1=170"
#define THINGSPEAK_URL "https://api.thingspeak.com/update?api_key=" THINGSPEAK_API_KEY

/* Timer interval once every day (24 Hours) */
#define TIME_PERIOD (86400000000ULL)

/* heart-beat LED */
#define PQA_LED_PIN	GPIO_NUM_6
#define PQA_LED_ON	1
#define PQA_LED_OFF	0

/* external variables */
extern const char server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const char server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");
//extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
//extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");
extern int32_t daq_data[DAQ_ADC_CHANNELS][DAQ_N];

/* external functions */
esp_err_t start_rest_server(const char *);

/* global variables */
SemaphoreHandle_t send_data_semaphore;

/* local variables */
static const char *TAG = "pqa";

/* local functions */
/*
 * heart-beat task
 */
static void
hb(void *args)
{
//	static int i = 0;

	while (1) {
		/* dsps_fft2r_sc16_aes3_(NULL, 16, NULL); */

		/* heart-beat LED */
		gpio_set_level(PQA_LED_PIN, PQA_LED_ON);
		vTaskDelay(250 / portTICK_PERIOD_MS);
		gpio_set_level(PQA_LED_PIN, PQA_LED_OFF);
		vTaskDelay(750 / portTICK_PERIOD_MS);
	}
}

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define EXAMPLE_ESP_WIFI_SSID "UPC*******"
#define EXAMPLE_ESP_WIFI_PASS "**********"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

static void
event_handler(void* arg, esp_event_base_t event_base,
              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
/*
 * Init WiFi
 */
static void
wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
#if 0
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
#endif
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s",
                 EXAMPLE_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s",
                 EXAMPLE_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}


#if 0
static const char THINGSPEAK_REQUEST[] = "GET " WEB_URL " HTTP/1.1\r\n"
                             "Host: " WEB_SERVER "\r\n"
                             "User-Agent: esp-idf/1.0 esp32\r\n"
                             "\r\n";
static void
https_get_request(esp_tls_cfg_t cfg, const char *WEB_SERVER_URL, const char *REQUEST)
{
    char buf[512];
    int ret, len;

    esp_tls_t *tls = esp_tls_init();
    if (!tls) {
        ESP_LOGE(TAG, "Failed to allocate esp_tls handle!");
        goto exit;
    }

    if (esp_tls_conn_http_new_sync(WEB_SERVER_URL, &cfg, tls) == 1) {
        ESP_LOGI(TAG, "Connection established...");
    } else {
        ESP_LOGE(TAG, "Connection failed...");
        int esp_tls_code = 0, esp_tls_flags = 0;
        esp_tls_error_handle_t tls_e = NULL;
        esp_tls_get_error_handle(tls, &tls_e);
        /* Try to get TLS stack level error and certificate failure flags, if any */
        ret = esp_tls_get_and_clear_last_error(tls_e, &esp_tls_code, &esp_tls_flags);
        if (ret == ESP_OK) {
            ESP_LOGE(TAG, "TLS error = -0x%x, TLS flags = -0x%x", esp_tls_code, esp_tls_flags);
        }
        goto cleanup;
    }

    size_t written_bytes = 0;
    do {
        ret = esp_tls_conn_write(tls,
                                 REQUEST + written_bytes,
                                 strlen(REQUEST) - written_bytes);
        if (ret >= 0) {
            ESP_LOGI(TAG, "%d bytes written", ret);
            written_bytes += ret;
        } else if (ret != ESP_TLS_ERR_SSL_WANT_READ  && ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
            ESP_LOGE(TAG, "esp_tls_conn_write  returned: [0x%02X](%s)", ret, esp_err_to_name(ret));
            goto cleanup;
        }
    } while (written_bytes < strlen(REQUEST));

    ESP_LOGI(TAG, "Reading HTTP response...");

	int fd;

	ESP_ERROR_CHECK(esp_tls_get_conn_sockfd(tls, &fd));
	ESP_LOGI(TAG, "ESP-TLS socket file descriptor: %d", fd);

    do {
        len = sizeof(buf) - 1;
        memset(buf, 0x00, sizeof(buf));
        ret = esp_tls_get_bytes_avail(tls);
	ESP_LOGI(TAG, "%d bytes available", ret);

        ret = esp_tls_conn_read(tls, (char *)buf, len);

        if (ret == ESP_TLS_ERR_SSL_WANT_WRITE  || ret == ESP_TLS_ERR_SSL_WANT_READ) {
            continue;
        } else if (ret < 0) {
            ESP_LOGE(TAG, "esp_tls_conn_read  returned [-0x%02X](%s)", -ret, esp_err_to_name(ret));
            break;
        } else if (ret == 0) {
            ESP_LOGI(TAG, "connection closed");
            break;
        }

        len = ret;
        ESP_LOGD(TAG, "%d bytes read", len);
        /* Print response directly to stdout as it is read */
        for (int i = 0; i < len; i++) {
            putchar(buf[i]);
        }
        putchar('\n'); // JSON output doesn't have a newline at end
    } while (1);

#ifdef CONFIG_EXAMPLE_CLIENT_SESSION_TICKETS
    /* The TLS session is successfully established, now saving the session ctx for reuse */
    if (save_client_session) {
        esp_tls_free_client_session(tls_client_session);
        tls_client_session = esp_tls_get_client_session(tls);
    }
#endif

cleanup:
    esp_tls_conn_destroy(tls);
exit:
    for (int countdown = 10; countdown >= 0; countdown--) {
        ESP_LOGI(TAG, "%d...", countdown);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


static void
https_get_request_using_cacert_buf(void)
{
    ESP_LOGI(TAG, "https_request using cacert_buf");
    esp_tls_cfg_t cfg = {
        .cacert_buf = (const unsigned char *) server_root_cert_pem_start,
        .cacert_bytes = server_root_cert_pem_end - server_root_cert_pem_start,
//	.skip_common_name = true,
    };
    https_get_request(cfg, WEB_URL, THINGSPEAK_REQUEST);
}

static void
https_get_request_using_global_ca_store(void)
{
    esp_err_t esp_ret = ESP_FAIL;
    ESP_LOGI(TAG, "https_request using global ca_store");
    esp_ret = esp_tls_set_global_ca_store(server_root_cert_pem_start, server_root_cert_pem_end - server_root_cert_pem_start);
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Error in setting the global ca store: [%02X] (%s),could not complete the https_request using global_ca_store", esp_ret, esp_err_to_name(esp_ret));
        return;
    }
    esp_tls_cfg_t cfg = {
        .use_global_ca_store = true,
    };
    https_get_request(cfg, WEB_URL, THINGSPEAK_REQUEST);
    esp_tls_free_global_ca_store();
}
#endif /* 0 */

/*
 * http_post_data()
 */
static void
http_post_data(int ch)
{
#define POST_SIZE	100000

	char	http_body[100];
	int	body_ln;
	char	*post_data;

	esp_http_client_config_t config = {
		.url = "http://192.168.0.234/pqa/save_post.php",
		.method = HTTP_METHOD_POST,
		.transport_type = HTTP_TRANSPORT_OVER_TCP,
//		.event_handler = _http_event_handler,
//		.cert_pem = server_root_cert_pem_start,
	};

	if ((post_data = calloc(POST_SIZE, sizeof (char))) == NULL) {
		ESP_LOGE(TAG, "calloc() failed to allocate post_data");

		return;
	}

//	for (int ch = 1; ch < 2; ch++) {
		for (int i = 0; i < DAQ_N; i++) {
			snprintf(post_data + strlen(post_data),
			    POST_SIZE - strlen(post_data), "%s%ld",
			    i == 0 ? "data=" : ",", daq_data[ch][i]);
		}
		snprintf(post_data + strlen(post_data),
		    POST_SIZE - strlen(post_data), "&ch=%d&f=4096&n=4096", ch);
//	}

	ESP_LOGI(TAG, "HTTP POST request with url =>");

	esp_http_client_handle_t client = esp_http_client_init(&config);
	esp_http_client_set_header(client, "Content-Type",
	    "application/x-www-form-urlencoded");

	esp_err_t err = esp_http_client_open(client, strlen(post_data));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to open HTTP connection: %s",
		    esp_err_to_name(err));
	} else {
		int wlen = esp_http_client_write(client, post_data,
		    strlen(post_data));
		if (wlen < 0) {
			ESP_LOGE(TAG, "Write failed");
		}
		if (esp_http_client_fetch_headers(client) < 0) {
			ESP_LOGE(TAG, "HTTP client fetch headers failed");
		} else {
			body_ln = esp_http_client_read_response(client,
			    http_body, sizeof (http_body) - 1);
			if (body_ln >= 0) {
				http_body[body_ln] = '\0';

				ESP_LOGI(TAG, "HTTP POST Status = %d, "
				    "content_length = %"PRId64,
				    esp_http_client_get_status_code(client),
				    esp_http_client_get_content_length(client));

				printf("body: %s\n", http_body);
			} else {
				ESP_LOGE(TAG, "Failed to read response");
			}
		}
	}
	esp_http_client_close(client);
	esp_http_client_cleanup(client);

	free(post_data);
}

/*
 * https_with_url()
 */
static void
https_with_url(void)
{
	static char	http_get_request[1000];
	static char	http_body[100];
	int	body_ln;

	esp_http_client_config_t config = {
		.url = http_get_request,
		.method = HTTP_METHOD_GET,
//        .event_handler = _http_event_handler,
		.cert_pem = server_root_cert_pem_start,
	};

	ESP_LOGI(TAG, "HTTPS request with url =>");

	/* Assemble URL GET request */
	snprintf(http_get_request, sizeof (http_get_request),
	    "%s&field1=%0.2f&field2=%0.2f&field3=%0.2f"
	    "&field4=%0.2f&field5=%0.2f&field6=%0.2f"
	    "&field7=%0.3f&field8=%0.1f",
	    THINGSPEAK_URL, daq.L[DAQ_L1].U, daq.L[DAQ_L2].U, daq.L[DAQ_L3].U,
	    daq.L[DAQ_L1].I, daq.L[DAQ_L2].I, daq.L[DAQ_L3].I,
	    daq.freq[DAQ_CHANNEL_U3],
	    daq.L[DAQ_L1].P + daq.L[DAQ_L2].P + daq.L[DAQ_L3].P);

	esp_http_client_handle_t client = esp_http_client_init(&config);

	esp_err_t err = esp_http_client_open(client, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to open HTTP connection: %s",
		    esp_err_to_name(err));
	} else {
		if (esp_http_client_fetch_headers(client) < 0) {
			ESP_LOGE(TAG, "HTTP client fetch headers failed");
		} else {
			body_ln = esp_http_client_read_response(client,
			    http_body, sizeof (http_body) - 1);
			if (body_ln >= 0) {
				http_body[body_ln] = '\0';

				ESP_LOGI(TAG, "HTTP GET Status = %d, "
				    "content_length = %"PRId64,
				    esp_http_client_get_status_code(client),
				    esp_http_client_get_content_length(client));

				printf("body: %s\n", http_body);
			} else {
				ESP_LOGE(TAG, "Failed to read response");
			}
		}
	}
	esp_http_client_close(client);
	esp_http_client_cleanup(client);
#if 0
	esp_err_t err = esp_http_client_perform(client);

	if (err == ESP_OK) {
		ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %"PRId64,
		    esp_http_client_get_status_code(client),
		    esp_http_client_get_content_length(client));

		if (esp_http_client_is_complete_data_received(client)) {
			printf("HTTPS data completely received\n");
		} else {
			printf("HTTPS data NOT completely received\n");
		}

		body_ln = esp_http_client_read_response(client, http_body,
		    sizeof (http_body) - 1);

		http_body[body_ln] = '\0';

		ESP_LOGI(TAG, "body_ln = %d", body_ln);
		ESP_LOGI(TAG, "body: %s", http_body);
	} else {
		ESP_LOGE(TAG, "Error perform http request %s",
		    esp_err_to_name(err));
	}
	esp_http_client_cleanup(client);
#endif
}

#if 0
/*
 * Task managing HTTPS request
 */
static void
https_request_task(void *pvparameters)
{
    ESP_LOGI(TAG, "Start https_request example");

//	mbedtls_esp_enable_debug_log();
#if 1
	https_with_url();
#else
#ifdef CONFIG_EXAMPLE_CLIENT_SESSION_TICKETS
    char *server_url = NULL;
#ifdef CONFIG_EXAMPLE_LOCAL_SERVER_URL_FROM_STDIN
    char url_buf[SERVER_URL_MAX_SZ];
    if (strcmp(CONFIG_EXAMPLE_LOCAL_SERVER_URL, "FROM_STDIN") == 0) {
        example_configure_stdin_stdout();
        fgets(url_buf, SERVER_URL_MAX_SZ, stdin);
        int len = strlen(url_buf);
        url_buf[len - 1] = '\0';
        server_url = url_buf;
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: invalid url for local server");
        abort();
    }
    printf("\nServer URL obtained is %s\n", url_buf);
#else
    server_url = CONFIG_EXAMPLE_LOCAL_SERVER_URL;
#endif /* CONFIG_EXAMPLE_LOCAL_SERVER_URL_FROM_STDIN */
    https_get_request_to_local_server(server_url);
    https_get_request_using_already_saved_session(server_url);
#endif

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE && CONFIG_EXAMPLE_USING_ESP_TLS_MBEDTLS
//    https_get_request_using_crt_bundle();
#endif
    ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
    https_get_request_using_cacert_buf();

//    https_get_request_using_global_ca_store();
//    https_get_request_using_specified_ciphersuites();
#endif /* 1 */

    ESP_LOGI(TAG, "Finish https_request example");
    vTaskDelete(NULL);
}
#endif

void app_main(void)
{
	time_t	now;
	TaskHandle_t	hb_handle = NULL;
	TaskHandle_t	daq_handle = NULL;

	/* Initialize NVS */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
	    ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_LOGE(TAG, "nvs_flash_init() failed with %d error", ret);

		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	printf("Hello world :-)\n");

	/* Print chip information */
	esp_chip_info_t chip_info;
	uint32_t flash_size;
	esp_chip_info(&chip_info);
	printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
	    CONFIG_IDF_TARGET,
	    chip_info.cores,
	    (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
	    (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
	    (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
	    (chip_info.features & CHIP_FEATURE_IEEE802154) ?
	    ", 802.15.4 (Zigbee/Thread)" : "");

	unsigned major_rev = chip_info.revision / 100;
	unsigned minor_rev = chip_info.revision % 100;
	printf("silicon revision v%d.%d, ", major_rev, minor_rev);

	if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
		printf("Get flash size failed");
		return;
	}

	printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
	    (chip_info.features & CHIP_FEATURE_EMB_FLASH) ?
	    "embedded" : "external");

	printf("Minimum free heap size: %" PRIu32 " bytes\n",
	    esp_get_minimum_free_heap_size());

	/* heart-beat LED */
	gpio_set_direction(PQA_LED_PIN, GPIO_MODE_OUTPUT);

	/* create heart-beat task */
	xTaskCreatePinnedToCore(hb, "heart-beat", 1024, NULL, 10, &hb_handle,
	    tskNO_AFFINITY);

	/* Dump GPIO configuration */
//	gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

	/* Connect to WiFi */
	wifi_init_sta();

	esp_reset_reason_t reset_reason = esp_reset_reason();
	switch (reset_reason) {
	case ESP_RST_USB:
		ESP_LOGI(TAG, "ESP reset reason: Reset by USB peripheral");
		break;
	default:
		ESP_LOGI(TAG, "ESP reset reason: %d", reset_reason);
	}

//	if (esp_reset_reason() == ESP_RST_POWERON) {
		ESP_LOGI(TAG, "Updating time from NVS");
		ESP_ERROR_CHECK(update_time_from_nvs());
//	}
#if 0
	const esp_timer_create_args_t nvs_update_timer_args = {
		.callback = (void *)&fetch_and_store_time_in_nvs,
	};

	esp_timer_handle_t nvs_update_timer;
	ESP_ERROR_CHECK(esp_timer_create(&nvs_update_timer_args,
	    &nvs_update_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(nvs_update_timer,
	    TIME_PERIOD));
#endif /* 0 */

	/* print current time */
	if (time(&now) == -1) {
		ESP_LOGI(TAG, "time() failed to obtain current time");
	} else {
		printf("Current UTC time: %s\n", ctime(&now));
	}

//	xTaskCreatePinnedToCore(&https_request_task, "https_get_task", 8192,
//	    NULL, 5, NULL, tskNO_AFFINITY);

	send_data_semaphore = xSemaphoreCreateBinary();
	/*
	 * Create data acquisition task
	 * WiFi taks is running on core 0, so pin daq_task to CPU 1.
	 */
	xTaskCreatePinnedToCore(&daq_task, "daq_task", 8192, NULL, 20,
	    &daq_handle, 1);

	/* Start REST server */
	ESP_ERROR_CHECK(start_rest_server("/www"));

	/* Run forever */
	while (1) {
		xSemaphoreTake(send_data_semaphore, portMAX_DELAY);

		/* Send data to ThingSpeak server */
		https_with_url();

        	vTaskDelay(5000 / portTICK_PERIOD_MS);

		for (int ch = 0; ch < DAQ_ADC_CHANNELS; ch++) {
//			http_post_data(ch);
		}
	}

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);

    esp_restart();
}
