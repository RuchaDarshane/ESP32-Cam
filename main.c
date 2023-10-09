#include <stdio.h>
#include <stdlib.h>
#include <string.h> //Requires by memset
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_http_client.h"

#include "includes.h"
#include <stdint.h>

#include "os.h"
#include "base64.c"
#include "sendimage.pb-c.h"

#define BOARD_ESP32CAM_AITHINKER

#define  EXAMPLE_ESP_WIFI_SSID "Galaxy M11"
#define  EXAMPLE_ESP_WIFI_PASS "tusa7146"
#define MAX_RETRY 10

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
static const char *TAG = "HTTP_CLIENT";

#define BOARD_WROVER_KIT 1

// WROVER-KIT PIN Map
#ifdef BOARD_WROVER_KIT

#define CAM_PIN_PWDN -1  //power down is not used
#define CAM_PIN_RESET -1 //software reset will be performed
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

#endif

// ESP32Cam (AiThinker) PIN Map
#ifdef BOARD_ESP32CAM_AITHINKER

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 50, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
     
};

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}
#endif


static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    // 1 - Wi-Fi/LwIP Init Phase
    esp_netif_init();                    // TCP/IP initiation                   s1.1
    esp_event_loop_create_default();     // event loop                          s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station                        s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); //                                         s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;    
       
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}
typedef struct {
        //httpd_req_t *req;
        size_t len;
} jpg_chunking_t;



static void post_rest_function(){


    esp_http_client_config_t config_post = {
        .url ="http://192.168.36.195/camera_proj/test6.php",
        .auth_type = HTTP_AUTH_TYPE_BASIC,
        .method = HTTP_METHOD_POST,
        
        .event_handler = client_event_post_handler };
        
    esp_http_client_handle_t client = esp_http_client_init(&config_post);

    //strlen(encoded_data)-904
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t fb_len = 0;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
      
        //return ESP_FAIL;
    }
    fb_len = fb->len;


    

    const char *input = (const char *)fb->buf;    
    printf(input);
    long input_size = strlen(input);
    size_t *out_len;
    
    char * encoded_data = base64_encode((const char *)fb->buf, fb_len , &out_len);
    ESP_LOGE(TAG, "Encoded Data = %d", strlen(encoded_data));
    //snprintf(post_data, sizeof(post_data), "image=%s",encoded_data);
    //char post_data = strcat("image=",encoded_data);
    //esp_http_client_set_header(client, "Content-Disposition", "inline; filename=capture.jpg");
    char post_data[670];
    char chunk[650];
    int pos ;
    int len ;
    len =500;
    int a ;
    a = (strlen(encoded_data) / len) + 1 ;
    char post_data1[10];
    
    snprintf(post_data, sizeof(post_data), "size=%d",a);
    //esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
    //esp_http_client_set_post_field(client, post_data, strlen(post_data));
    
    esp_err_t err = esp_http_client_open(client, -1); // write_len=-1 sets header "Transfer-Encoding: chunked" and method to POST
    //esp_http_client_set_header(client, "Transfer-Encoding", "chunked"); 
    
    esp_http_client_write(client, post_data1, strlen(post_data1));
    
    for (int i = 0; i < a; i++)
    {

        pos = 1;
        

        strncpy(chunk,encoded_data+(pos-1),len);
        snprintf(post_data, sizeof(post_data), "i=%s",chunk);
        //esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
        esp_http_client_write(client, post_data, strlen(post_data));
        strcpy(chunk, "");
        strcpy(post_data, "");

        
        if(i== (a - 1))

        {
            pos = pos + len ;
            len = strlen(encoded_data) - pos ;
            
        }
        else{
            pos= pos+len;
        }


    }
    esp_http_client_perform(client);
    //esp_err_t err;
    while (1) {
        err = esp_http_client_perform(client);
        if (err != ESP_ERR_HTTP_EAGAIN) {
            break;
        }
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }
    
        
    
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "JPG: %luKB %lums", (uint32_t)(fb_len/1024), (uint32_t)((fr_end - fr_start)/1000));
    //return 0; 
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
  
    
}

void app_main(void)
{
    nvs_flash_init();
    init_camera();

    wifi_connection();

    
    printf("WIFI was initiated ...........\n\n");
    
    post_rest_function();
    //xTaskCreate(&Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL); 

}
