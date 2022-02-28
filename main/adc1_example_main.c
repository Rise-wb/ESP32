/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_types.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "event_source.h"
#include "esp_event_base.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_system.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "freertos/queue.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"
#include "esp_timer.h"
#include "soc/timer_group_struct.h"

#define EXAMPLE_ESP_MAXIMUM_RETRY  10 //?
#define DEFAULT_SCAN_LIST_SIZE     10//?
#define Data_Number                1
#define DEFAULT_VREF               1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES              64          //Multisampling
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1
#define GPIO_INPUT_IO_BODY       18
#define GPIO_INPUT_IO_BREATHE     19
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_BODY) | (1ULL<<GPIO_INPUT_IO_BREATHE))
#define GPIO_INPUT_IO_PILLOW_1    22
#define GPIO_INPUT_IO_PILLOW_2    23
#define GPIO_INPUT_PIN_SEL_2  ((1ULL<<GPIO_INPUT_IO_PILLOW_1) | (1ULL<<GPIO_INPUT_IO_PILLOW_2))
#define YEAR  0
#define MONTH 1
#define DAY   2
#define HOUR  3
#define MIN   4
#define SEC   5


uint16_t number = DEFAULT_SCAN_LIST_SIZE;
wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
uint16_t ap_count = 0;
const int CONNECTED_BIT = BIT0;
//memset(ap_info, 0, sizeof(ap_info));


static const char *TAG_scan = "scan";
static const char *TAG_sta = "wifi station";
static const char *TAG_HB = "HR2";
static const char *TAG= "mqtt";
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 HEART
static const adc_channel_t channel_2 = ADC_CHANNEL_7;     //GPIO35  SNOR
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;    
static const adc_atten_t atten = ADC_ATTEN_DB_11;   //11dB衰减（ADC_ATTEN_0db）提供满量程电压3.9V
//static const int RX_BUF_SIZE = 1024;
static const int uart_num = UART_NUM_1;
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static EventGroupHandle_t wifi_event_group;
static QueueHandle_t uart2_queue;
uart_event_t event;
size_t buffered_size;
//uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
esp_event_loop_handle_t loop_without_task;
esp_event_loop_handle_t loop_with_task;
ESP_EVENT_DEFINE_BASE(TASK_EVENTS);
TaskHandle_t g_task;

//定时器句柄
esp_timer_handle_t fw_timer_handle = 0;

//char INTERVENE_ON[12]= {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x17 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
//char INTERVENE_OFF[12]={0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x17 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
//char VOLUME_0[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x02 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
//char VOLUME_0[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x02 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char VOLUME_1_OFF[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x1E ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char VOLUME_1_ON[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x1E ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char VOLUME_2_OFF[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x1F ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char VOLUME_2_ON[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x1F ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char VOLUME_3_OFF[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x20 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char VOLUME_3_ON[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x20 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
int  volumeFlag=0;
char BEE_P[10] ={0xEE ,0x09 ,0xDE ,0xED ,0x13 ,0x31 ,0xFF ,0xFC ,0xFF ,0xFF };
char BEE_0[7]  ={0xEE ,0x70 ,0x21 ,0xFF ,0xFC ,0xFF ,0xFF};
char BEE_1[7]  ={0xEE ,0x70 ,0x61 ,0xFF ,0xFC ,0xFF ,0xFF};
char BED_ON[12]=    {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x02 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char BED_OFF[12]=   {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x02 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char BODY_ON[12]=   {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x03 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char BODY_OFF[12]=  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x03 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char PILLOW_ON[12]= {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x04 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char PILLOW_OFF[12]={0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x04 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
//char BODAY[] = {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x22 ,0x00 ,0x00 ,0x00 ,0x0A ,0xFF ,0xFC ,0xFF ,0xFF};
//char SCREEN_0[] ={0xEE ,0xB1 ,0x00 ,0x00 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
//char SCREEN_1[] ={0xEE ,0xB1 ,0x00 ,0x00 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
//char SCREEN_2[] ={0xEE ,0xB1 ,0x00 ,0x00 ,0x02 ,0xFF ,0xFC ,0xFF ,0xFF};
char ADC_HEAD_HEART[10]={0xEE,0xB1,0x32,0x00,0x00,0x00,0x27,0x00,0x00,0x00 };
char ADC_HEAD_BREATHE[10]={0xEE,0xB1,0x32,0x00,0x00,0x00,0x28,0x00,0x00,0x01 };
char ADC_TAILER[4]={0xFF,0xFC,0xFF,0xFF};
char SNORE_0[15] =   {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x21 ,0x00 ,0x00 ,0x00 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char SNORE_20[15] =  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x21 ,0x00 ,0x00 ,0x00 ,0x14 ,0xFF ,0xFC ,0xFF ,0xFF};
char SNORE_40[15] =  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x21 ,0x00 ,0x00 ,0x00 ,0x28 ,0xFF ,0xFC ,0xFF ,0xFF};
char SNORE_60[15] =  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x21 ,0x00 ,0x00 ,0x00 ,0x3C ,0xFF ,0xFC ,0xFF ,0xFF};
char SNORE_80[15] =  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x21 ,0x00 ,0x00 ,0x00 ,0x50 ,0xFF ,0xFC ,0xFF ,0xFF};
char SNORE_100[15] = {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x21 ,0x00 ,0x00 ,0x00 ,0x60 ,0xFF ,0xFC ,0xFF ,0xFF};
char WIFI_RSSI_1_ON[12] =  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x13 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char WIFI_RSSI_1_OFF[12] = {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x13 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char WIFI_RSSI_2_ON[12] =  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x14 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char WIFI_RSSI_2_OFF[12] = {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x14 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char WIFI_RSSI_3_ON[12] =  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x15 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char WIFI_RSSI_3_OFF[12] = {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x15 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char WIFI_RSSI_4_ON[12] =  {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x16 ,0x00 ,0xFF ,0xFC ,0xFF ,0xFF};
char WIFI_RSSI_4_OFF[12] = {0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x16 ,0x01 ,0xFF ,0xFC ,0xFF ,0xFF};
char GETTIME[6]    =  {0xEE, 0x82, 0xFF, 0xFC, 0xFF, 0xFF};
char dataHeart[Data_Number];
char dataBreathe[Data_Number];
char Byte[2]={0x00,0x00};
char Byte2[2]={0x00,0x00};
int adc_reading = 0;
char WIFI_SCAN_HEAD[6]={0xEE,0xB1,0x10,0x00,0x02,0x00};
char WIFI_SCAN_TAILER[4]={0xFF,0xFC,0xFF,0xFF};
char WIFI_PASS_GET_EN[11]={0xEE,0xB1,0x11,0x00,0x03,0x00,0x02,0xFF,0xFC,0xFF,0xFF};//获取用户输入的密码
char WIFI_PASS_CLEAR[11]={0xEE,0xB1,0x10,0x00,0x03,0x00,0x02,0xFF,0xFC,0xFF,0xFF};
char WIFI_PASS_GET[30];
char TIMEGET[13];
int  wifiPassBytes=0;
char wifiPort[0];
int wifiScanFlag=0;
int wifiP=0;
int wifiPassGetNumber=0;
char wifiSsid[30];
char wifiPassword[30];
int ssidLen[8]={0};
int passwordLen=0;
int inputBody=0;
int inputBodyLast=0;
int inputBreathe=0;
int inputPillow1=0;
int inputPillow2=0;
int bodyMove=0;
int breatheAccount=0;
char HEAD_HEART[7]={0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x29};
char HEAD_BREATHE[7]={0xEE ,0xB1 ,0x10 ,0x00 ,0x00 ,0x00 ,0x2A};
char HEART_BREATHE[3]={0x30,0x30,0x30};
int timeLastBody_Hours =0;
int timeSleepLast_Hours = 0;
int bedInFlag=0;
int bedOutFlag=0;
int pillowOutFlag=0;
int k=0;
int heartBeatADC[200] = {0};
int *pHBADC[200];
uint8_t flag_pHBADC = 0;
uint8_t flag_HBADC = 0;
int flag_HBADC_ord = 1;
int count_HBADC = 0;
int count_pHBADC = 0;
int breatheFlag=0;
int breatheAco=0;
int time_Breathe[6]={0};
int time_Breathe_last[6]={0};
void fw_timer_cb(void *arg);

static void task_iteration_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    // Two types of data can be passed in to the event handler: the handler specific data and the event-specific data.
    //  
    // The handler specific data (handler_args) is a pointer to the original data, therefore, the user should ensure that
    // the memory location it points to is still valid when the handler executes.
    //  
    // The event-specific data (event_data) is a pointer to a deep copy of the original data, and is managed automatically.
    int iteration = *((int*) event_data);

    char* loop;

    if (handler_args == loop_with_task) {
        loop = "loop_with_task";
    } else {
        loop = "loop_without_task";
    }   

    ESP_LOGI(TAG, "handling %s:%s from %s, iteration %d", base, "TASK_ITERATION_EVENT", loop, iteration);
}


//UART init
void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,                              
      //  .rx_flow_ctrl_thresh = 122,     
        .stop_bits = UART_STOP_BITS_1,
    };
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, TXD_PIN, RXD_PIN,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart2_queue, 0);
    uart_enable_pattern_det_intr(uart_num, '+', PATTERN_CHR_NUM, 10000, 10, 10);
    uart_pattern_queue_reset(uart_num, 20);
        //ADC
    adc1_config_width(width);//采集宽度
    adc1_config_channel_atten(channel, atten);//配置通道 以及衰减度
    adc1_config_channel_atten(channel_2, atten);//配置通道 以及衰减度
    //gpio
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_config_t io_confw;
    io_confw.intr_type = GPIO_INTR_DISABLE;
    io_confw.mode = GPIO_MODE_INPUT;
    io_confw.pin_bit_mask = GPIO_INPUT_PIN_SEL_2;
    io_confw.pull_up_en = 1;
    gpio_config(&io_confw);

    //定时器结构体初始化
	esp_timer_create_args_t fw_timer = 
	{ 
		.callback = &fw_timer_cb, 	//回调函数
		.arg = NULL, 				//参数
		.name = "fw_timer" 			//定时器名称
	};
	//定时器创建、启动
	esp_err_t err = esp_timer_create(&fw_timer, &fw_timer_handle);
	err = esp_timer_start_periodic(fw_timer_handle, 10 * 1000);//1秒回调
	if(err == ESP_OK)
	{
		printf("fw timer cteate and start ok!\r\n");
	}



    esp_event_loop_args_t loop_with_task_args = {
        .queue_size = 5,
        .task_name = "loop_task", // task will be created
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 10000,
        .task_core_id = tskNO_AFFINITY
    };

    esp_event_loop_args_t loop_without_task_args = {
        .queue_size = 5,
        .task_name = NULL // no task will be created
    };

    // Create the event loops
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_with_task_args, &loop_with_task));
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_without_task_args, &loop_without_task));

    // Register the handler for task iteration event. Notice that the same handler is used for handling event on diff
    // The loop handle is provided as an argument in order for this example to display the loop the handler is being 
    ESP_ERROR_CHECK(esp_event_handler_instance_register_with(loop_with_task, TASK_EVENTS, TASK_ITERATION_EVENT, task_iteration_handler, loop_with_task, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register_with(loop_without_task, TASK_EVENTS, TASK_ITERATION_EVENT, task_iteration_handler, loop_without_task, NULL));
}

void fw_timer_cb(void *arg){
//    ESP_LOGI("K","%d",k);
//    k++;
        adc_reading = adc1_get_raw((adc1_channel_t)channel);//从单个通道获取ADC1读数。                       
//        ESP_LOGI("WbadcR","%d",adc_reading);
        dataHeart[0]=(char)(adc_reading*255/4095);
    uart_write_bytes(uart_num, ADC_HEAD_HEART, 10);
    uart_write_bytes(uart_num, dataHeart,1);
    uart_write_bytes(uart_num, ADC_TAILER, 4);
}



static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)        //mqtt连接
{
        esp_mqtt_client_handle_t client = event->client;
        int msg_id;
// your_context_t *context = event->context;
        switch (event->event_id) {
            case MQTT_EVENT_CONNECTED://连接MQTT成功
                ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
                msg_id = esp_mqtt_client_publish(client, "/public/TEST/topic0", "data_3", 0, 1, 0);
                ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

                msg_id = esp_mqtt_client_subscribe(client, "/public/TEST/topic0", 0);
                ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

                msg_id = esp_mqtt_client_subscribe(client, "/public/TEST/topic0", 1);
                ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
                msg_id = esp_mqtt_client_subscribe(client, "/public/TEST/topic0", 2);
                ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
                //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
                //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
                break;
            case MQTT_EVENT_DISCONNECTED://断开MQTT
                ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
                break;

            case MQTT_EVENT_SUBSCRIBED://订阅成功
                printf("_---------订阅--------\n");
                ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
                msg_id = esp_mqtt_client_publish(client, "/public/TEST/topic0", "data", 0, 0, 0);
                ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
                break;
            case MQTT_EVENT_UNSUBSCRIBED://取消订阅
                ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
                break;
            case MQTT_EVENT_PUBLISHED://发布成功
                printf("_--------发布----------\n");
                ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
                break;
            case MQTT_EVENT_DATA://数据接收
                ESP_LOGI(TAG, "MQTT_EVENT_DATA");
                printf("主题长度:%d 数据长度:%d\n",event->topic_len,event->data_len);
                printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
                printf("DATA=%.*s\r\n", event->data_len, event->data);
                break;
            case MQTT_EVENT_ERROR://MQTT错误
                ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
                break;
            default:
                ESP_LOGI(TAG, "Other event id:%d", event->event_id);
                break;
            }
        return ESP_OK;
}




static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
        ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
        mqtt_event_handler_cb(event_data);   
}
 
static void mqtt_app_start(void)    //mqtt初始化
{
    esp_mqtt_client_config_t mqtt_cfg = {
            .host = "mq.tongxinmao.com",//MQTT 地址
            .port = 18830,   //MQTT端口
 //           .username = "NYNORhsWF",//用户名
 //           .password = "92pjknkfeucfmyqjhmqhapu2fcq4amyvegkvpscnfwxgug4l9zarwuqcgs9u5z3q",//密码                                                
    };         
        esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);//初始化MQTT
        esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);//注册事件
        esp_mqtt_client_start(client);//启动mQTT
 //       while(1){ 
            vTaskDelay(1000 / portTICK_PERIOD_MS);                                   
//        }                        
}
/* Initialize Wi-Fi as sta and set scan method */
static void wifi_scan(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
//    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
//    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_LOGI(TAG_scan, "Total APs scanned = %u", ap_count);
    for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {//这里要加个信号排序显示
        ESP_LOGI(TAG_scan, "SSID \t\t%s", ap_info[i].ssid);
        ESP_LOGI(TAG_scan, "RSSI \t\t%d", ap_info[i].rssi);
        if(i<8){
            ESP_LOGI(TAG_scan, "wbtest i :%d", i);
            Byte2[0]=(char)(i+1);
            int len = strlen((char *)ap_info[i].ssid);
            ssidLen[i]=len;
            uart_write_bytes(uart_num, WIFI_SCAN_HEAD, 6);            //把扫描的wifi名称显示在屏幕上
            uart_write_bytes(uart_num, Byte2, 1);
            uart_write_bytes(uart_num, ap_info[i].ssid,len);
            ESP_LOGI(TAG_scan, "wbtest len :%d", len);
            uart_write_bytes(uart_num, WIFI_SCAN_TAILER, 4); 
        }
//        print_auth_mode(ap_info[i].authmode);
//        if (ap_info[i].authmode != WIFI_AUTH_WEP) {
//            print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
//        }   
        ESP_LOGI(TAG_scan, "Channel \t\t%d\n", ap_info[i].primary);
    }
//    ESP_ERROR_CHECK(esp_wifi_stop());
}

static void event_handler(void* arg, esp_event_base_t event_base,                         //用来判断wifi是否连接成功
                        int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {           
        esp_wifi_connect(); 
        ESP_LOGI(TAG_sta, "handle connect");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_sta, "retry to connect to the AP");                                            
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);                        
        }
        ESP_LOGI(TAG_sta,"connect to the AP fail");            
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_sta, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);                                       
    }   
}
//wifi sta init
void wifi_init_sta(void)                                                  //连接wifi
{
//    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
//    ESP_ERROR_CHECK(esp_netif_init());
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
//    esp_netif_create_default_wifi_sta();
//    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&event_handler,NULL,&instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&event_handler,NULL,&instance_got_ip));
    wifi_config_t wifi_config = {
        .sta = {
//            .ssid = "Ris",
//            .password = "11111111",
    /* Setting a password implies station will connect to all security modes including WEP/WPA.
    *              * However these modes are deprecated and not advisable to be used. Incase your Access point
    *                           * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
                },
        },
    };
    memcpy(wifi_config.sta.ssid,ap_info[wifiP].ssid,ssidLen[wifiP]);                          //获取要连接的wifi密码
    memcpy(wifi_config.sta.password,(WIFI_PASS_GET+8),(wifiPassGetNumber-13));
//    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//    ESP_ERROR_CHECK( esp_wifi_disconnect()  );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_connect()  );
//    ESP_ERROR_CHECK(esp_wifi_start());    
    ESP_LOGI(TAG_sta, "wifi_init_sta finished.");
    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
    *      * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            portMAX_DELAY);
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
    *      * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_sta, "connected to ap SSID:%s password:%s",
        wifi_config.sta.ssid, wifi_config.sta.password);
        mqtt_app_start();                                     //mqtt连接
        if(ap_info[wifiP].rssi<-100){                                                //判断连接成功后显示wifi信号强度
            uart_write_bytes(uart_num, WIFI_RSSI_1_OFF, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_2_OFF, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_3_OFF, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_4_OFF, 12);
        }else if(ap_info[wifiP].rssi<-88&&ap_info[wifiP].rssi>-100){
            uart_write_bytes(uart_num, WIFI_RSSI_1_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_2_OFF, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_3_OFF, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_4_OFF, 12);
        }else if(ap_info[wifiP].rssi<-77&&ap_info[wifiP].rssi>-88){
            uart_write_bytes(uart_num, WIFI_RSSI_1_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_2_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_3_OFF, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_4_OFF, 12);
        }else if(ap_info[wifiP].rssi<-55&&ap_info[wifiP].rssi>-66){
            uart_write_bytes(uart_num, WIFI_RSSI_1_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_2_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_3_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_4_OFF, 12);
        }else if(ap_info[wifiP].rssi>-55){
            uart_write_bytes(uart_num, WIFI_RSSI_1_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_2_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_3_ON, 12);
            uart_write_bytes(uart_num, WIFI_RSSI_4_ON, 12);
        }
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_sta, "Failed to connect to SSID:%s, password:%s",
        wifi_config.sta.ssid, wifi_config.sta.password);
    } else {
        ESP_LOGE(TAG_sta, "UNEXPECTED EVENT");
    }
    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

}

//void app_wifi_wait_connected(){
//        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
//}


//return the time interval(Min)  计算时间间隔
int TimeInterval(int *current,int *past)
{
    int t1=0,t2=0;
    if(*(current+DAY)>*(past+DAY)){
        t1= (*(current+HOUR) +24)*60*60 + *(current+MIN)*60+*(current+SEC);
        t2= *(past+HOUR) * 60*60 + *(past+MIN)*60 + *(past+SEC);
        return (t1-t2);
    }else if(*(current+DAY)==*(past+DAY)){
        t1= (*(current+HOUR))*60*60 + *(current+MIN)*60+*(current+SEC);
        t2= *(past+HOUR) * 60*60 + *(past+MIN)*60 + *(past+SEC);
        return (t1-t2);
    }else{
        return -1;
    }
}
 
//get time获取串口屏上的时间
void TimeGet(int *Time)
{
    uart_write_bytes(uart_num, GETTIME, 6);
//    ESP_LOGI("Wbend","%d",TIME[0]);
    if(uart_read_bytes(uart_num, TIMEGET, 13, 20 / portTICK_RATE_MS)==13){
        int j=0;
        for(int i=2;i<=8;i++){
            if(i!=4){
                *(Time+j)=((int)TIMEGET[i])/16;
                *(Time+j)=*(Time+j)*10+((int)TIMEGET[i])%16;
                j++;
            }
        }
  //      printf("year: %d  month: %d  day: %d  hour: %d  min: %d  sec: %d\n" , *(Time+YEAR),*(Time+MONTH),*(Time+DAY),*(Time+HOUR),*(Time+MIN),*(Time+SEC));
    }
}

void InformationCollection()//adc、gpio的信息获取与处理
{
 //   if(timeMinFlag==1)
 //       timeMinFlag=0;
//    TimeGet(timeMin);
//    if(timeMinFlag==0&&TimeInterval(timeMin,timeMin_last)>=1){
//        timeMinFlag=1;
//        TimeGet(timeMin_last);
//        if(bodyMove)
//    }
    inputPillow1 = gpio_get_level(GPIO_INPUT_IO_PILLOW_1);
    inputPillow2 = gpio_get_level(GPIO_INPUT_IO_PILLOW_2);
    inputBody = gpio_get_level(GPIO_INPUT_IO_BODY);
    inputBreathe = gpio_get_level(GPIO_INPUT_IO_BREATHE);
//    printf("1:%d\t2:%d\n",inputPillow1,inputPillow2);
    if(inputBody==0){
        bodyMove++;                                   //体动累计
        uart_write_bytes(uart_num, BODY_ON, 12);
    }else{
        uart_write_bytes(uart_num, BODY_OFF, 12);
    }

    if(inputPillow1==1||inputPillow2==1){
        uart_write_bytes(uart_num, BED_ON, 12);
//        if(bedInFlag==0){
 //           TimeGet(timeBody_last);
//        }
        bedInFlag=1;
//        bedOutFlag=0;
    }else if(inputPillow1==0&&inputPillow2==0){
        uart_write_bytes(uart_num, BED_OFF, 12);
        bedInFlag=0;
//        if(bedInFlag==1&&bedOutFlag==0){                //忽略一分钟的体动
//            bedOutFlag=1;
//            TimeGet(timeBodyOut_last);
//        }if(bedInFlag==1&&bedOutFlag==1){
//            int min = TimeInterval(timeMin,timeBodyOut_last);
//            if(min>=1){                                  //无法忽略离床
//                bedInFlag=0;
//                bedOutFlag=0;
//                min = TimeInterval(timeMin,timeBody_last);
                //SleepTime +=min-1;                      //睡眠时间累加
//            }
//        }
    }
    if(inputPillow1==0&&inputPillow2==0){
        uart_write_bytes(uart_num, PILLOW_ON, 12);
//        if(pillowOutFlag==0)
//            TimeGet(timePillowOut_last);
//        pillowOutFlag=1;
//        TimeGet(timePillowOut);
//        int min= TimeInterval(timeMin,timePillowOut_last);
//        if(min>=15){

            //有效落枕
//        }
    }else{
//        pillowOutFlag=0;
        uart_write_bytes(uart_num, PILLOW_OFF, 12);
    }


    /************************************Beginning of Heart Rate detection*********************************/
    if(inputPillow1==0 || inputPillow2==0)
    {
//        printf("if\n");
    	for (uint8_t s = 0; s < 200; s++)
    		heartBeatADC[s] = 0;
    	flag_pHBADC = 0;
    	flag_HBADC = 0;
    	flag_HBADC_ord = 1;
    	count_HBADC = 0;
    	count_pHBADC = 0;
    }
    else
    {
//        printf("else\n");
    	count_pHBADC = count_pHBADC + 1;
		heartBeatADC[count_pHBADC-1] = adc_reading;
		if (count_pHBADC == 200)
		{
			flag_pHBADC = 1;
			count_pHBADC = 0;
		}
		count_HBADC = count_HBADC + 1;
		if (count_HBADC == 100)
		{
			flag_HBADC = 1;
			flag_HBADC_ord = -flag_HBADC_ord;
			count_HBADC = 0;
		}
		if (flag_pHBADC*flag_HBADC == 1)
		{
  //          printf("flag_pHBADC*flag_HBADC == 1\n");
//			 data in the range of "heartBeatADC[0]-heartBeatADC[199]" 
			if (flag_pHBADC*flag_HBADC*flag_HBADC_ord == 1)
			{
				// estimate the rising or the falling slopes
				uint8_t slope[200] = {0};   //the first element is not used here
				int tmp = heartBeatADC[0];
				uint8_t i;
				for (i = 1; i <= 199; i++)  //the first element is not used here
				{
					if( heartBeatADC[i] >= tmp )
					{
						slope[i] = 1;
						if(i<199 && slope[i-1]==0 && heartBeatADC[i+1]<heartBeatADC[i-1] && (heartBeatADC[i]-heartBeatADC[i-1])/heartBeatADC[i-1]<=0.01)
							slope[i] = 0;
					}else if(heartBeatADC[i] < tmp)
					{
						slope[i] = 0;
						if(i<199 && slope[i-1]==1 && heartBeatADC[i+1]>heartBeatADC[i-1] && (heartBeatADC[i-1]-heartBeatADC[i])/heartBeatADC[i-1]<=0.01)
							slope[i] = 1;
					}
					tmp = heartBeatADC[i];
				}
				// counting the number of the turning points
				uint8_t tot_turnPoint = 0;
				uint8_t j;
				for (j = 1; j < 199; j++)
				{
					if (slope[j+1] != slope[j])
						tot_turnPoint = tot_turnPoint + 1;
				}

				// obtaining the index of the turning points and the two-ending points
				uint8_t tot_slope = tot_turnPoint + 1; // plus the ending point
				uint8_t tot_endPoint = tot_slope + 1;  // plus the beginning point
				uint8_t *idx_slopeEnd =(uint8_t *) calloc(tot_endPoint, sizeof(uint8_t));
				uint8_t idx_s = 0;
				for (j = 1; j < 199; j++)
				{
					if (slope[j+1] != slope[j])
					{
						idx_s = idx_s + 1;
						idx_slopeEnd[idx_s] = j;
					}
				}
				idx_slopeEnd[tot_endPoint-1] = 199;  //The ending point
				idx_slopeEnd[0] = 0;            //The beginning point

				// obtaining the length of the slopes
				int *len_idx_slope =(int *) calloc(tot_slope, sizeof(int));
				for (j = 1; j < tot_endPoint; j++)
					len_idx_slope[j-1] = heartBeatADC[idx_slopeEnd[j]] - heartBeatADC[idx_slopeEnd[j-1]];

				// the second way to estimate heart rate by finding the longest down-to-up slope (i.e., K point) 
				uint8_t step_f = 7;  // the search length for the next K point
				uint8_t refg = 1;    // flag for indicating the end of the len_idx_slope
				uint8_t K[100] = {0};
				uint8_t kc = 0;
				uint8_t idx_le, tmp_idx;
				if (len_idx_slope[0] < 0)  //len_idx_slope(0) < 0 for the first slope is falling slope
					idx_le = 0;
				else                       //len_idx_slope(0) >= 0 for the first slope is rising slope
					idx_le = 1;
				while (refg == 1)
				{
					tmp_idx = idx_le;
					for (j = idx_le+2; j <= idx_le+2*(step_f-1); j = j+2)
					{
						if ((len_idx_slope[j+1]-len_idx_slope[j]) >= (len_idx_slope[tmp_idx+1]-len_idx_slope[tmp_idx]))
							tmp_idx = j;
					}
					K[kc] = idx_slopeEnd[tmp_idx+1];    // the low turning point --> K point
					idx_le = tmp_idx+2;
					if (idx_le + 2*(step_f-1) >= tot_slope-1)
					{
						refg = 0;
					}
					else
						kc = kc + 1;
				}
//                printf("kc:%d\n",kc);
				uint8_t sum_tSlot = 0;
                if(kc!=0){
                    for (uint8_t q = 0; q <= kc-1; q++)
                    {
                        sum_tSlot = sum_tSlot + K[q+1] - K[q];
                    }
                    double ave_tSlot = sum_tSlot / (kc) * 0.01;
                    double HR2 = 1 / ave_tSlot * 60;    // Heart beats per min
                    ESP_LOGI(TAG_HB, "The heart beat rate is %f.", HR2);
                    int len_H=0,len_H2=0,acc_H=HR2,acc_H2=HR2;
                    while(acc_H>0){
                        acc_H=acc_H/10;
                        len_H++;
                    }
  //                  printf("len_H%d\n",len_H);
                    len_H2=len_H;
    //                printf("acc%d %d\n",acc_H,acc_H2);
                    while(len_H>=1){
                        HEART_BREATHE[len_H-1]=0x30+(char)(acc_H2%10);
                        acc_H2=acc_H2/10;
                        len_H--;
                    }
      //              printf("acc%x %x %x\n",HEART_BREATHE[0],HEART_BREATHE[1],HEART_BREATHE[2]);
                    uart_write_bytes(uart_num,HEAD_HEART ,7);
                    uart_write_bytes(uart_num, HEART_BREATHE, len_H2);
                    uart_write_bytes(uart_num, ADC_TAILER, 4);
                }
                free(idx_slopeEnd);
                free(len_idx_slope);
            }
			// data in the range of "heartBeatADC[100]-heartBeatADC[199],heartBeatADC[0]-heartBeatADC[99]" 

			else if (flag_pHBADC*flag_HBADC*flag_HBADC_ord == -1)
			{
//            printf("flag_pHBADC*flag_HBADC == -1\n");
				// estimate the rising or the falling slopes
				uint8_t slope[200] = {0};   //the first element is not used here
				int tmp = heartBeatADC[0];
				uint8_t i;
				for (i = 1; i <= 199; i++)  //the first element is not used here
				{
					if( heartBeatADC[(i+100)%200] >= tmp )
					{
						slope[i] = 1;
						if(i<199 && slope[i-1]==0 && heartBeatADC[(i+1+100)%200]<heartBeatADC[(i-1+100)%200] && (heartBeatADC[(i+100)%200]-heartBeatADC[(i-1+100)%200])/heartBeatADC[(i-1+100)%200]<=0.01)
							slope[i] = 0;
					}else if(heartBeatADC[(i+100)%200] < tmp)
					{
						slope[i] = 0;
						if(i<199 && slope[i-1]==1 && heartBeatADC[(i+1+100)%200]>heartBeatADC[(i-1+100)%200] && (heartBeatADC[(i-1+100)%200]-heartBeatADC[(i+100)%200])/heartBeatADC[(i-1+100)%200]<=0.01)
							slope[i] = 1;
					}
					tmp = heartBeatADC[(i+100)%200];
				}

				// counting the number of the turning points
				uint8_t tot_turnPoint = 0;
				uint8_t j;
				for (j = 1; j < 199; j++)
				{
					if (slope[j+1] != slope[j])
						tot_turnPoint = tot_turnPoint + 1;
				}

				// obtaining the index of the turning points and the two-ending points
				uint8_t tot_slope = tot_turnPoint + 1; // plus the ending point
				uint8_t tot_endPoint = tot_slope + 1;  // plus the beginning point
				uint8_t *idx_slopeEnd = (uint8_t *)calloc(tot_endPoint, sizeof(uint8_t));
				uint8_t idx_s = 0;
				for (j = 1; j < 199; j++)
				{
					if (slope[j+1] != slope[j])
					{
						idx_s = idx_s + 1;
						idx_slopeEnd[idx_s] = j;
					}
				}
				idx_slopeEnd[tot_endPoint-1] = 199;  //The ending point
				idx_slopeEnd[0] = 0;            //The beginning point
				// obtaining the length of the slopes
				int *len_idx_slope =(uint8_t *) calloc(tot_slope, sizeof(int));
				for (j = 1; j < tot_endPoint; j++)
					len_idx_slope[j-1] = heartBeatADC[(idx_slopeEnd[j]+100)%200] - heartBeatADC[(idx_slopeEnd[j-1]+100)%200];
				// the second way to estimate heart rate by finding the longest down-to-up slope (i.e., K point) 
				uint8_t step_f = 7;  // the search length for the next K point
				uint8_t refg = 1;    // flag for indicating the end of the len_idx_slope
				uint8_t K[100] = {0};
				uint8_t kc = 0;
				uint8_t idx_le, tmp_idx;
				if (len_idx_slope[0] < 0)  //len_idx_slope(0) < 0 for the first slope is falling slope
					idx_le = 0;
				else                       //len_idx_slope(0) >= 0 for the first slope is rising slope
					idx_le = 1;
				while (refg == 1)
				{
					tmp_idx = idx_le;
					for (j = idx_le+2; j <= idx_le+2*(step_f-1); j = j+2)
					{
						if ((len_idx_slope[j+1]-len_idx_slope[j]) >= (len_idx_slope[tmp_idx+1]-len_idx_slope[tmp_idx]))
							tmp_idx = j;
					}

					K[kc] = idx_slopeEnd[tmp_idx+1];    // the low turning point --> K point
					idx_le = tmp_idx+2;
					if (idx_le + 2*(step_f-1) >= tot_slope-1)
					{
						refg = 0;
					}
					else
						kc = kc + 1;
				}
          //      printf("kc:%d\n",kc);
				uint8_t sum_tSlot = 0;
                if(kc!=0){
                    for (uint8_t q = 0; q <= kc-1; q++)
                    {
                        sum_tSlot = sum_tSlot + K[q+1] - K[q];
                    }
                    double ave_tSlot = sum_tSlot / (kc) * 0.01;
                    double HR2 = 1 / ave_tSlot * 60;    // Heart beats per min
                    ESP_LOGI(TAG_HB, "The heart beat rate is %f.", HR2);
                    int len_H=0,len_H2=0,acc_H=HR2,acc_H2=HR2;
                    while(acc_H>0){
                        acc_H=acc_H/10;
                        len_H++;
                    }
        //            printf("len_H%d\n",len_H);
                    len_H2=len_H;
        //            printf("acc%d %d\n",acc_H,acc_H2);
                    while(len_H>=1){
                        HEART_BREATHE[len_H-1]=0x30+(char)(acc_H2%10);
                        acc_H2=acc_H2/10;
                        len_H--;
                    }
   //                 printf("acc%x %x %x\n",HEART_BREATHE[0],HEART_BREATHE[1],HEART_BREATHE[2]);
                    uart_write_bytes(uart_num,HEAD_HEART ,7);
                    uart_write_bytes(uart_num, HEART_BREATHE, len_H2);
                    uart_write_bytes(uart_num, ADC_TAILER, 4);
                }
                free(idx_slopeEnd);
                free(len_idx_slope);
			}else
			{
  //          printf("NULL\n");
				ESP_LOGI(TAG_HB, "The flags for heart rate detection are wrong !");
			}
			flag_HBADC = 0;  // when count_HBADC==100, set flag_HBADC = 1
		}
    }

/************************************Ending of Heart Rate detection***************************************/

/************************************Starting of Breathe times detection***************************************/
    if(bedInFlag==1&&inputBreathe==0){
        if(breatheFlag==1)
            breatheAccount++;
        breatheAco=0;
        breatheFlag=0;
    }else if(bedInFlag==1&&inputBreathe==1){
        breatheAco++;
        if(breatheAco>50)
            breatheFlag=1;
    }
  //  ESP_LOGI(TAG_HB,"inputBreathe %d.", inputBreathe);
    dataBreathe[0]=(char)(inputBreathe*255);
    uart_write_bytes(uart_num, ADC_HEAD_BREATHE, 10);
    uart_write_bytes(uart_num, dataBreathe,1);
    uart_write_bytes(uart_num, ADC_TAILER, 4);
    TimeGet(time_Breathe);
    int time_B=TimeInterval(time_Breathe,time_Breathe_last);
   // printf("time_B%d\n",time_B);
    if(time_B>=60){
        TimeGet(time_Breathe_last);
        printf("1min\n");
        printf("1minBreathe%d\n",breatheAccount);
        int len_B=0,len_B2,acc=breatheAccount;
        while(acc>0){
            acc=acc/10;
            len_B++;
        }
        printf("len_B%d\n",len_B);
        len_B2=len_B;
        while(len_B>=1){
            HEART_BREATHE[len_B-1]=0x30+(char)(breatheAccount%10);
            breatheAccount=breatheAccount/10;
            len_B--;
        }
        uart_write_bytes(uart_num,HEAD_BREATHE ,7);
        uart_write_bytes(uart_num, HEART_BREATHE, len_B2);
        uart_write_bytes(uart_num, ADC_TAILER, 4);
        breatheAccount=0;
    }
/************************************Ending of  Breathe times detection***************************************/

//    printf("P1: %d P2: %d Body: %d Breathe: %d\n",inputPillow1,inputPillow2,inputBody,inputBreathe);
//    ESP_LOGI("Wbstart", " %d ",k);
//    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &task_counter_value);
//    print_timer_counter(task_counter_value);
    int adc_reading2 = adc1_get_raw((adc1_channel_t)channel_2);//从单个通道获取ADC1读数。                       

    int snore =(char)(adc_reading2*100/4095);
//    ESP_LOGI("WbadcS","%d",snore);
    if(snore<10){
        uart_write_bytes(uart_num, SNORE_0, 15);
    }else if(snore>10&&snore<30){
        uart_write_bytes(uart_num, SNORE_20, 15);
    }else if(snore>30&&snore<50){
        uart_write_bytes(uart_num, SNORE_40, 15);            
    }else if(snore>50&&snore<70){
        uart_write_bytes(uart_num, SNORE_60, 15);                                                                          
    }else if(snore>70&&snore<90){
        uart_write_bytes(uart_num, SNORE_80, 15);                                                                      
    }else if(snore>90){
        uart_write_bytes(uart_num, SNORE_100, 15);                                                                      
    }
}

static void application_task(void* args)
{
   // init();
    while(1) {
 //       ESP_LOGI(TAG, "application_task: running application task");
        if(xQueueReceive(uart2_queue, (void *) &event, 20 / portTICK_RATE_MS)) {
            //UART_DATA
            if(event.type == UART_DATA)
            {
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                if(uart_read_bytes(uart_num, wifiPort, event.size, 5 / portTICK_RATE_MS)==14){
//                    ESP_LOGI("Wbend","%d",wifiP);
    //                if(uart_read_bytes(uart_num, wifiPort, 14, 1000 / portTICK_RATE_MS)==14){//确认按键按下
                    if(wifiPort[6]==0x0B&&wifiPort[4]==0x02)     //wifi连接端口
                        wifiP=0;
                    else if(wifiPort[6]==0x0C&&wifiPort[4]==0x02)
                        wifiP=1;
                    else if(wifiPort[6]==0x0D&&wifiPort[4]==0x02)
                        wifiP=2;
                    else if(wifiPort[6]==0x0E&&wifiPort[4]==0x02)
                        wifiP=3;
                    else if(wifiPort[6]==0x0F&&wifiPort[4]==0x02)
                        wifiP=4;
                    else if(wifiPort[6]==0x10&&wifiPort[4]==0x02)
                        wifiP=5;
                    else if(wifiPort[6]==0x11&&wifiPort[4]==0x02)
                        wifiP=6;
                    else if(wifiPort[6]==0x12&&wifiPort[4]==0x02)
                        wifiP=7;
                    else if(wifiPort[4]==0x03&&wifiPort[6]==0x03){      //获取wifi的密码
                        uart_write_bytes(uart_num, WIFI_PASS_GET_EN, 11);
                        wifiPassGetNumber=0;
                        while(uart_read_bytes(uart_num, (WIFI_PASS_GET+wifiPassGetNumber), 1, 10 / portTICK_RATE_MS)){  //一个字节一个字节的获取密码
                            wifiPassGetNumber++;
                            if(wifiPassGetNumber>=30){
                                break;
                            }
      //                      ESP_LOGI("WbpassRT","%d",wifiPassGetNumber);
                        }
     //                   for(int i=0;i<wifiPassGetNumber;i++){
     //                       ESP_LOGI("Wbpass","%x",WIFI_PASS_GET[i]);
     //                   }
     //                   ESP_LOGI("WbpassR","%d",wifiPassGetNumber);
                        uart_write_bytes(uart_num, WIFI_PASS_CLEAR, 11);
                        wifi_init_sta();                                      //wifi连接
                    }//else if(wifiPort[4]==0x00&&wifiPort[6]==0x1A){    //干预按钮检测
                     //   uart_write_bytes(uart_num, INTERVENE_OFF, 12);
                   // }//else if(wifiPort[4]==0x00&&wifiPort[6]==0x1B){    //干预按钮检测
                     //   uart_write_bytes(uart_num, INTERVENE_ON, 12);
                   // }
                    else if(wifiPort[4]==0x00&&wifiPort[6]==0x25){    //音量按钮检测
                        if(volumeFlag==0){
                            uart_write_bytes(uart_num, BEE_P, 10);
                            uart_write_bytes(uart_num, BEE_0, 7);
                            uart_write_bytes(uart_num, VOLUME_1_OFF, 12);
                            uart_write_bytes(uart_num, VOLUME_2_OFF, 12);
                            uart_write_bytes(uart_num, VOLUME_3_OFF, 12);
                            volumeFlag=1;
                        }else{
                            uart_write_bytes(uart_num, BEE_P, 10);
                            uart_write_bytes(uart_num, BEE_1, 7);
                            uart_write_bytes(uart_num, VOLUME_1_ON, 12);
                            uart_write_bytes(uart_num, VOLUME_2_ON, 12);
                            uart_write_bytes(uart_num, VOLUME_3_ON, 12);
                            volumeFlag=0;
                        }
                        ESP_LOGI(TAG_scan, "volumeFlag = %d",volumeFlag );

                    }else if(wifiPort[4]==0x00&&wifiPort[6]==0x26){
                        ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
                        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
                        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
//                        ESP_LOGI(TAG_scan, "Total APs scanned = %u", ap_count);
                        for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {//这里要加个信号排序显示
                            ESP_LOGI(TAG_scan, "SSID \t\t%s", ap_info[i].ssid);
                            ESP_LOGI(TAG_scan, "RSSI \t\t%d", ap_info[i].rssi);
                            if(i<8){
                                ESP_LOGI(TAG_scan, "wbtest i :%d", i);
                                Byte2[0]=(char)(i+1);
                                int len = strlen((char *)ap_info[i].ssid);
                                ssidLen[i]=len;
                                uart_write_bytes(uart_num, WIFI_SCAN_HEAD, 6);            //把扫描的wifi名称显示在屏幕上
                                uart_write_bytes(uart_num, Byte2, 1);
                                uart_write_bytes(uart_num, ap_info[i].ssid,len);
                                ESP_LOGI(TAG_scan, "wbtest len :%d", len);
                                uart_write_bytes(uart_num, WIFI_SCAN_TAILER, 4); 
                            }
                    //        print_auth_mode(ap_info[i].authmode);
                    //        if (ap_info[i].authmode != WIFI_AUTH_WEP) {
                    //            print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
                    //        }   
//                            ESP_LOGI(TAG_scan, "Channel \t\t%d\n", ap_info[i].primary);
                        }

                    }
//                    ESP_LOGI("Wbend","%d",wifiP);
                }
            }
        }
        esp_event_loop_run(loop_without_task, 100);
    }   
}

void app_main(void)
{
    uint64_t task_counter_value;
    double Sec=0,LastSec=0;
    ADC_HEAD_HEART[9] = (char)(Data_Number%256);
    ADC_HEAD_HEART[8] = (char)(Data_Number/256); 
//    ADC_HEAD_BREATHE[9] = (char)(Data_Number%256);
//    ADC_HEAD_BREATHE[8] = (char)(Data_Number/256); 
    init();
    xTaskCreatePinnedToCore(application_task, "application_task", 10000, NULL, uxTaskPriorityGet(NULL), NULL,1);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();                        
    }
    ESP_ERROR_CHECK( ret  );
    wifi_scan();
//    TimeGet(time_Breathe_last);
//    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD);
    while (1) {
//        ESP_LOGI("K","%d",k);
//        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &task_counter_value);
 //       print_timer_counter(task_counter_value);
        InformationCollection();
//        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &task_counter_value);
//        print_timer_counter(task_counter_value);
//        k++;
    }
    
}
