/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This demo showcases creating a GATT database using a predefined attribute table.
* It acts as a GATT server and can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
* Client demo will enable GATT server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"
#include "led-controller.h"

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "LedController"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define BLE_PASSKEY                 456789

static uint8_t adv_config_done       = 0;

uint16_t led_controller_ble_handle_table[IDX_END];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

// static uint8_t service_uuid[16] = {
//     /* LSB <--------------------------------------------------------------------------------> MSB */
//     //first uuid, 16bit, [12],[13] is the value
//     0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
// };

#define GATT_SIZE 16 // 16 bytes/128bits. 

// Generated GUID: c8cbff07-899c-4bb1-a9f6-3949555be68a
// Characteristic first 12 bytes
#define BASE_UUID 0xc8, 0xcb, 0xff, 0x07, 0x89, 0x9c, 0x4b, 0xb1, 0xa9, 0xf6, 0x39, 0x49
// Last 2 bytes
#define TAIL_UUID 0x00, 0x00

// Service
static uint8_t service_uuid[GATT_SIZE]        = {BASE_UUID, 0xFF, 0x00, TAIL_UUID};

// Characteristics
const uint8_t uuid_led_pattern[GATT_SIZE]      = {BASE_UUID, 0x02, 0xFF, TAIL_UUID};
const uint8_t uuid_led_color[GATT_SIZE]        = {BASE_UUID, 0x03, 0xFF, TAIL_UUID};
const uint8_t uuid_led_brightness[GATT_SIZE]   = {BASE_UUID, 0x04, 0xFF, TAIL_UUID};
const uint8_t uuid_led_speed[GATT_SIZE]        = {BASE_UUID, 0x05, 0xFF, TAIL_UUID};

const uint8_t uuid_led_pattern_list[GATT_SIZE] = {BASE_UUID, 0x10, 0xFF, TAIL_UUID};
const uint8_t uuid_led_color_list[GATT_SIZE]   = {BASE_UUID, 0x11, 0xFF, TAIL_UUID};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
// static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x00FF;
// static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0xFF01;
// static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0xFF02;
// static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0xFF03;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
// static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

// Characteristics permissions
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

// static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
// static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};
static const uint8_t char_value[2]                 = {0x2D, 0x00};

// #define MY_GATT_PERM_READ         ESP_GATT_PERM_READ
// #define MY_GATT_PERM_READWRITE    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE
#define MY_GATT_PERM_READ         ESP_GATT_PERM_READ_ENCRYPTED
#define MY_GATT_PERM_READWRITE    ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[IDX_END] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, MY_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(service_uuid), (uint8_t *)&service_uuid}},

    // Characteristic: Pattern
    /* Characteristic Declaration */
    [IDX_CHAR_PATTERN]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, MY_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_PATTERN] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&uuid_led_pattern, MY_GATT_PERM_READWRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    // /* Client Characteristic Configuration Descriptor */
    // [IDX_CHAR_CFG_A]  =
    // {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, MY_GATT_PERM_READWRITE,
    //   sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

    // Characteristic: Color
    /* Characteristic Declaration */
    [IDX_CHAR_COLOR]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, MY_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_COLOR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&uuid_led_color, MY_GATT_PERM_READWRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    // Characteristic: Brightness
    /* Characteristic Declaration */
    [IDX_CHAR_BRIGHTNESS]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, MY_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BRIGHTNESS]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&uuid_led_brightness, MY_GATT_PERM_READWRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    // Characteristic: Speed
    /* Characteristic Declaration */
    [IDX_CHAR_SPEED]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, MY_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_SPEED]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&uuid_led_speed, MY_GATT_PERM_READWRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    // Characteristic: Pattern List
    /* Characteristic Declaration */
    [IDX_CHAR_PATTERN_LIST]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, MY_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_PATTERN_LIST] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&uuid_led_pattern_list, MY_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    // Characteristic: Color List
    /* Characteristic Declaration */
    [IDX_CHAR_COLOR_LIST]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, MY_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_COLOR_LIST] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&uuid_led_color_list, MY_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

};

static void register_led_callbacks()
{
    // printf("register_led_callbacks\n");

    // Setup callback functions to be notified when values change
    SetPatternChangedCallback(PatternChangedCallback);
    SetColorChangedCallback(ColorChangedCallback);
    SetBrightnessChangedCallback(BrightnessChangedCallback);
    SetSpeedChangedCallback(SpeedChangedCallback);
}

static void load_static_led_controller_data()
{
    // printf("load_static_led_controller_data\n");
    char * temp = (char*)malloc(GATTS_DEMO_CHAR_VAL_LEN_MAX);
    if (temp != NULL)
    {
        int len = GATTS_DEMO_CHAR_VAL_LEN_MAX;
        GetPatterns(temp, &len);
        // printf("Got patterns:\n%s", temp);
        esp_err_t ret = esp_ble_gatts_set_attr_value(led_controller_ble_handle_table[IDX_CHAR_VAL_PATTERN_LIST], len, (const uint8_t*)temp);
        if (ret != ESP_OK) ESP_LOGE(GATTS_TABLE_TAG, "Error setting pattern list: %s", esp_err_to_name(ret));

        len = GATTS_DEMO_CHAR_VAL_LEN_MAX;
        GetColors(temp, &len);
        // printf("Got colors:\n%s", temp);
        ret = esp_ble_gatts_set_attr_value(led_controller_ble_handle_table[IDX_CHAR_VAL_COLOR_LIST], len, (const uint8_t*)temp);
        if (ret != ESP_OK) ESP_LOGE(GATTS_TABLE_TAG, "Error setting color list: %s", esp_err_to_name(ret));

        free(temp);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }

            // This needs to be called late-enough that the table is already loaded and ready
            // Starting advertising is the last thing done at startup
            load_static_led_controller_data();
            register_led_callbacks();

            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_OOB_REQ_EVT:
        {
            uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
            esp_ble_oob_req_reply( param->ble_security.ble_req.bd_addr, tk, sizeof( tk ) );
            break;
        }
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;

        case ESP_GAP_BLE_NC_REQ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT");
            esp_ble_confirm_reply( param->ble_security.ble_req.bd_addr, true );
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "Pairing request - passkey: %d", BLE_PASSKEY);
            break;
        case ESP_GAP_BLE_KEY_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_KEY_EVT");
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
        {
            esp_bd_addr_t bd_addr;
            memcpy( bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof( esp_bd_addr_t ) );
            ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x", ( bd_addr[0] << 24 ) + ( bd_addr[1] << 16 ) + ( bd_addr[2] << 8 ) + bd_addr[3], ( bd_addr[4] << 8 ) + bd_addr[5] );
            ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type );
            ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail" );

            if ( !param->ble_security.auth_cmpl.success )
            {
                ESP_LOGI( GATTS_TABLE_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason );
            }
            else
            {
                ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %u", param->ble_security.auth_cmpl.auth_mode);
                esp_ble_set_encryption(bd_addr, ESP_BLE_SEC_ENCRYPT_MITM);
            }
            break;
        }
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_END, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT, handle = %d", param->read.handle);
       	    break;
        case ESP_GATTS_WRITE_EVT:
            // ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT");
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                // if (led_controller_ble_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
                //     uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                //     if (descr_value == 0x0001){
                //         ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                //         uint8_t notify_data[15];
                //         for (int i = 0; i < sizeof(notify_data); ++i)
                //         {
                //             notify_data[i] = i % 0xff;
                //         }
                //         //the size of notify_data[] need less than MTU size
                //         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, led_controller_ble_handle_table[IDX_CHAR_VAL_PATTERN],
                //                                 sizeof(notify_data), notify_data, false);
                //     }else if (descr_value == 0x0002){
                //         ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                //         uint8_t indicate_data[15];
                //         for (int i = 0; i < sizeof(indicate_data); ++i)
                //         {
                //             indicate_data[i] = i % 0xff;
                //         }
                //         //the size of indicate_data[] need less than MTU size
                //         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, led_controller_ble_handle_table[IDX_CHAR_VAL_PATTERN],
                //                             sizeof(indicate_data), indicate_data, true);
                //     }
                //     else if (descr_value == 0x0000){
                //         ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                //     }else{
                //         ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                //         esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                //     }

                // }

                if (led_controller_ble_handle_table[IDX_CHAR_VAL_PATTERN] == param->write.handle && param->write.len == 1)
                {
                    set_pattern_index(*param->write.value);
                }
                else if (led_controller_ble_handle_table[IDX_CHAR_VAL_COLOR] == param->write.handle && param->write.len == 1)
                {
                    set_color_index(*param->write.value);
                }
                else if (led_controller_ble_handle_table[IDX_CHAR_VAL_BRIGHTNESS] == param->write.handle && param->write.len == 1)
                {
                    set_brightness(*param->write.value);
                }
                else if (led_controller_ble_handle_table[IDX_CHAR_VAL_SPEED] == param->write.handle && param->write.len == 1)
                {
                    set_speed(*param->write.value);
                }

                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            
            // esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != IDX_END){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to IDX_END(%d)", param->add_attr_tab.num_handle, IDX_END);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(led_controller_ble_handle_table, param->add_attr_tab.handles, sizeof(led_controller_ble_handle_table));
                esp_ble_gatts_start_service(led_controller_ble_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_STOP_EVT");
            break;
        case ESP_GATTS_OPEN_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_OPEN_EVT");
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
            break;
        case ESP_GATTS_CLOSE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CLOSE_EVT");
            break;
        case ESP_GATTS_LISTEN_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_LISTEN_EVT");
            break;
        case ESP_GATTS_CONGEST_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONGEST_EVT");
            break;
        case ESP_GATTS_UNREG_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_UNREG_EVT");
            break;
        case ESP_GATTS_DELETE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DELETE_EVT");
            break;
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void PatternChangedCallback(uint8_t value)
{
    esp_ble_gatts_set_attr_value(led_controller_ble_handle_table[IDX_CHAR_VAL_PATTERN], 1, &value);
}

void ColorChangedCallback(uint8_t value)
{
    esp_ble_gatts_set_attr_value(led_controller_ble_handle_table[IDX_CHAR_VAL_COLOR], 1, &value);
}

void BrightnessChangedCallback(uint8_t value)
{
    esp_ble_gatts_set_attr_value(led_controller_ble_handle_table[IDX_CHAR_VAL_BRIGHTNESS], 1, &value);
}

void SpeedChangedCallback(uint8_t value)
{
    esp_ble_gatts_set_attr_value(led_controller_ble_handle_table[IDX_CHAR_VAL_SPEED], 1, &value);
}

void setup_GATTS(void)
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    // Setup BLE authentication and bonding (pairing)
    //esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;     // BLE "just-works" method of bonding
    // esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;  // require authentication and bonding (uses passkey if this device can show it and the other can allow a user to enter it)
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;  // require authentication and bonding (uses passkey if this device can show it and the other can allow a user to enter it)
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;                     // this device needs to be able to display the passkey

    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;                   // don't support out of band key excahnge (we're just doing simple entry of static passkey anyway)

    uint32_t temp_pass = BLE_PASSKEY;
    ESP_ERROR_CHECK( esp_ble_gap_set_security_param( ESP_BLE_SM_SET_STATIC_PASSKEY, &temp_pass, sizeof( uint32_t ) ) );

    ESP_ERROR_CHECK( esp_ble_gap_set_security_param( ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof( uint8_t ) ) );
    ESP_ERROR_CHECK( esp_ble_gap_set_security_param( ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof( uint8_t ) ) );
    ESP_ERROR_CHECK( esp_ble_gap_set_security_param( ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof( uint8_t ) ) );
    ESP_ERROR_CHECK( esp_ble_gap_set_security_param( ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof( uint8_t ) ) );
    ESP_ERROR_CHECK( esp_ble_gap_set_security_param( ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof( uint8_t ) ) );
    ESP_ERROR_CHECK( esp_ble_gap_set_security_param( ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof( uint8_t ) ) );
    ESP_ERROR_CHECK( esp_ble_gap_set_security_param( ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof( uint8_t ) ) );

}
