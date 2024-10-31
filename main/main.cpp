/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "led-display-manager.hpp"
#include "serial_console.h"
#include "cmd_system.h"
#include "cmd_nvs.h"
#include "gatts_table_creat_demo.h"
#include "cmd_ble.h"

#include "ESPAsyncE131.h"
#include "FastLED.h"
#include "AllLedPatterns.hpp"

#define INCLUDE_TEST_PATTERNS 0  // 1 to include special test patterns
#define DESKTOP_TEST          0  // 1 to configure for a desktop test with 32-pixel strip and no ethernet

// Macro to find array length
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#if DESKTOP_TEST
  #define DATA_PIN_A 4
  #define STRIP_A_COUNT 8
  #define STRIP_B_COUNT 8
  #define STRIP_C_COUNT 8
  #define STRIP_D_COUNT 8
  #define ROCKER_PIXEL_COUNT 4
  #define UPRIGHT_PIXEL_COUNT 4
#else
  #define DATA_PIN_A 4 
  #define DATA_PIN_B 12
  #define DATA_PIN_C 13
  #define DATA_PIN_D 16
  #define STRIP_A_COUNT 300
  #define STRIP_B_COUNT 240
  #define STRIP_C_COUNT 109
  #define STRIP_D_COUNT 109
  #define ROCKER_PIXEL_COUNT 52
  #define UPRIGHT_PIXEL_COUNT 57
#endif

#define ALL_LEDS_COUNT (STRIP_A_COUNT+STRIP_B_COUNT+STRIP_C_COUNT+STRIP_D_COUNT)

#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

#if INCLUDE_TEST_PATTERNS
const CRGB PALETTE_RGBW[] = { CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::White };
#endif
const CRGB PALETTE_XMAS[] = { CRGB::Red, CRGB::Green, CRGB::White };
const CRGB PALETTE_PATRIOT[] = { CRGB::Red, CRGB::White, CRGB::Blue };

static const char *TAG = "eth_example";

// .pixels NULL for now - will be allocated during startup
led_strip_t allLEDs = { .pixel_count = ALL_LEDS_COUNT,   .pixels = nullptr };     // Single array for all LEDs
led_strip_t strip_A = { .pixel_count = STRIP_A_COUNT,    .pixels = nullptr };     // Subsets of allLEDs for sending to outputs
led_strip_t strip_B = { .pixel_count = STRIP_B_COUNT,    .pixels = nullptr };
led_strip_t strip_C = { .pixel_count = STRIP_C_COUNT,    .pixels = nullptr };
led_strip_t strip_D = { .pixel_count = STRIP_D_COUNT,    .pixels = nullptr };

#define strip_roof      strip_A       // Roof - starts back-right, wraps around front to back-left
#define strip_underbody strip_B       // Underbody

// Sides: 52 LEDs on rocker panel daisy-chained to 57 LEDs on upright
#define strip_right     strip_C
#define strip_left      strip_D

// Special strips to refer to segments of the real strips
// Rockers: back to front, uprights: bottom to top
led_strip_t strip_rightRocker =  { .pixel_count = ROCKER_PIXEL_COUNT,  .pixels = nullptr };
led_strip_t strip_rightUpright = { .pixel_count = UPRIGHT_PIXEL_COUNT, .pixels = nullptr };
led_strip_t strip_leftRocker =   { .pixel_count = ROCKER_PIXEL_COUNT,  .pixels = nullptr };
led_strip_t strip_leftUpright =  { .pixel_count = UPRIGHT_PIXEL_COUNT, .pixels = nullptr };
led_strip_t strip_rightRoof =    { .pixel_count = 0,                   .pixels = nullptr };
led_strip_t strip_leftRoof =     { .pixel_count = 0,                   .pixels = nullptr };

ESPAsyncE131 e131(5);  // Buffers for 5 universes
LedSpecialPatternFPP specialPatternFPP(&allLEDs);

// #define EXAMPLE_STATIC_IP_ADDR        CONFIG_EXAMPLE_STATIC_IP_ADDR
// #define EXAMPLE_STATIC_NETMASK_ADDR   CONFIG_EXAMPLE_STATIC_NETMASK_ADDR
// #define EXAMPLE_STATIC_GW_ADDR        CONFIG_EXAMPLE_STATIC_GW_ADDR
#define EXAMPLE_STATIC_IP_ADDR        "192.168.1.206"
#define EXAMPLE_STATIC_NETMASK_ADDR   "255.255.255.0"
#define EXAMPLE_STATIC_GW_ADDR        "192.168.1.1"
#define EXAMPLE_MAIN_DNS_SERVER       "192.168.1.1"
#define EXAMPLE_BACKUP_DNS_SERVER     "192.168.1.1"

bool e131Callback(e131_packet_t* ReceivedData, void* UserInfo)
{
  return specialPatternFPP.E131Callback(ReceivedData, UserInfo);
}

static void SetupLeds()
{
  printf("Setting up LEDs\n");
  // All LEDs in a single array for simplicity of pattern calculation and E131 support
  allLEDs.pixels = (CRGB*)malloc(allLEDs.pixel_count * sizeof(CRGB));

  if (allLEDs.pixels == NULL) ESP_LOGE(__func__, "Failed to initialize pixels");
  else
  {
    // Now set the pointers in the subset strips
    strip_A.pixels = allLEDs.pixels;
    strip_B.pixels = strip_A.pixels + strip_A.pixel_count;  // B follows A in the large array
    strip_C.pixels = strip_B.pixels + strip_B.pixel_count;  // C follows B in the large array
    strip_D.pixels = strip_C.pixels + strip_C.pixel_count;  // D follows C in the large array

    // the WS2811 family uses the RMT driver
#if DESKTOP_TEST
    FastLED.addLeds<LED_TYPE, DATA_PIN_A, COLOR_ORDER>(allLEDs.pixels, allLEDs.pixel_count);
#else
    FastLED.addLeds<LED_TYPE, DATA_PIN_A, COLOR_ORDER>(strip_A.pixels, strip_A.pixel_count); // TODO: should I add .setCorrection(TypicalLEDStrip)  ??
    FastLED.addLeds<LED_TYPE, DATA_PIN_B, COLOR_ORDER>(strip_B.pixels, strip_B.pixel_count);
    FastLED.addLeds<LED_TYPE, DATA_PIN_C, COLOR_ORDER>(strip_C.pixels, strip_C.pixel_count);
    FastLED.addLeds<LED_TYPE, DATA_PIN_D, COLOR_ORDER>(strip_D.pixels, strip_D.pixel_count);
#endif
  }

  // Upright strip is daisy-chained to rocker panel on each side 
  strip_rightRocker.pixels = strip_C.pixels;
  strip_rightUpright.pixels = strip_rightRocker.pixels + strip_rightRocker.pixel_count;
  strip_leftRocker.pixels = strip_D.pixels;
  strip_leftUpright.pixels = strip_leftRocker.pixels + strip_leftRocker.pixel_count;

  // Split the roof into left and right sides 
  strip_leftRoof.pixel_count = strip_rightRoof.pixel_count = strip_roof.pixel_count / 2;
  strip_rightRoof.pixels = strip_roof.pixels;
  strip_leftRoof.pixels = strip_rightRoof.pixels + strip_rightRoof.pixel_count;

  // Add the allLEDs strip to LedDisplayManager so it can clear everything between patterns
  LedDisplayManager::AddLedStrip(&allLEDs);

  // Cart has a 12v->5v converter with 40A capacity
  FastLED.setMaxPowerInVoltsAndMilliamps(5,40000);
}

static void SetupPatterns()
{
  LedDisplayManager::AddPattern(new LedPatternRainbow({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternGlitter({&allLEDs}, 80));
  LedDisplayManager::AddPattern(new LedPatternConfetti({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternSinelon({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternJuggle({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternBPM({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternKnightRider({&strip_A}));
  LedDisplayManager::AddPattern(new LedPatternMarquee({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternFadingMarquee({&strip_leftRoof, &strip_leftRocker}, {&strip_rightRoof, &strip_rightRocker}));
  LedDisplayManager::AddPattern(new LedPatternPolice({&strip_leftRoof, &strip_leftRocker}, {&strip_leftUpright}, {&strip_rightRoof, &strip_rightRocker}, {&strip_rightUpright}));
  LedDisplayManager::AddPattern(new LedPatternSolid({&allLEDs}, 1000));
  LedDisplayManager::AddPattern(new LedPatternPulse({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternStrobe({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternFire({&strip_leftRoof, &strip_leftRocker, &strip_rightRocker}, {&strip_leftUpright, &strip_rightUpright}));
  LedDisplayManager::AddPattern(new LedPatternRainbowStripe({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternParty({&allLEDs}));
  LedDisplayManager::AddPattern(new LedPatternPride({&allLEDs}));
#if INCLUDE_TEST_PATTERNS
// Special test patterns to check each strip
  LedDisplayManager::AddPattern(new LedPatternSolid({&strip_roof}, 1000, "TEST Roof"));
  LedDisplayManager::AddPattern(new LedPatternSolid({&strip_underbody}, 1000, "TEST Underbody"));
  LedDisplayManager::AddPattern(new LedPatternSolid({&strip_rightRocker}, 1000, "TEST R Rocker"));
  LedDisplayManager::AddPattern(new LedPatternSolid({&strip_rightUpright}, 1000, "TEST R Upright"));
  LedDisplayManager::AddPattern(new LedPatternSolid({&strip_leftRocker}, 1000, "TEST L Rocker"));
  LedDisplayManager::AddPattern(new LedPatternSolid({&strip_leftUpright}, 1000, "TEST L Upright"));
#endif
  LedDisplayManager::AddPattern(&specialPatternFPP);
}

static void SetupColors()
{
  LedDisplayManager::AddColor(new PatternColor("White", CRGB::White));
  LedDisplayManager::AddColor(new PatternColor("Warm White", CRGB(0xE1A024)));
  LedDisplayManager::AddColor(new PatternColor("Red", CRGB::Red));
  LedDisplayManager::AddColor(new PatternColor("Green", CRGB::Green));
  LedDisplayManager::AddColor(new PatternColor("Blue", CRGB::Blue));
  LedDisplayManager::AddColor(new PatternColor("Violet", CRGB::Violet));
  LedDisplayManager::AddColor(new PatternColor_Random());
  LedDisplayManager::AddColor(new PatternColor_Rainbow());
  LedDisplayManager::AddColor(new PatternColor_PaletteManual("[Xmas]", PALETTE_XMAS, ARRAY_SIZE(PALETTE_XMAS)));
  LedDisplayManager::AddColor(new PatternColor_PaletteManual("[Patriot]", PALETTE_PATRIOT, ARRAY_SIZE(PALETTE_PATRIOT)));
#if INCLUDE_TEST_PATTERNS
  LedDisplayManager::AddColor(new PatternColor_PaletteManual("[RGBW]", PALETTE_RGBW, ARRAY_SIZE(PALETTE_RGBW)));
#endif
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    //const tcpip_adapter_ip_info_t *ip_info = &event->ip_info;
    esp_netif_ip_info_t * ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

extern "C" {
  void app_main();
}

static void start_leds()
{
    SetupLeds();
    SetupPatterns();
    SetupColors();
    e131.registerCallback(NULL, e131Callback);
    LedDisplayManager::Start();
}

static esp_err_t example_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

static void example_set_static_ip(esp_netif_t *netif)
{
    esp_err_t result;
    if ((result = esp_netif_dhcpc_stop(netif)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp client - %d: %s", result, esp_err_to_name(result));
        // return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(EXAMPLE_STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info - %d: %s", result, esp_err_to_name(result));
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    ESP_LOGI(TAG, "eth_event_handler - arg=%p, event_base=%d, event_id=%d", arg, (uint32_t)event_base, (uint32_t)event_id);

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

        // Now start up the E1.31 receiver
        e131.begin(E131_UNICAST);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

void setupEthernet() {
    printf("Setting up Ethernet\n");
    esp_netif_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(tcpip_adapter_set_default_eth_handlers());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Create new default instance of esp-netif for Ethernet
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_EXAMPLE_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;
    vTaskDelay(pdMS_TO_TICKS(10));
#if CONFIG_EXAMPLE_USE_INTERNAL_ETHERNET
    mac_config.smi_mdc_gpio_num = CONFIG_EXAMPLE_ETH_MDC_GPIO;
    mac_config.smi_mdio_gpio_num = CONFIG_EXAMPLE_ETH_MDIO_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);
#if CONFIG_EXAMPLE_ETH_PHY_IP101
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_RTL8201
    esp_eth_phy_t *phy = esp_eth_phy_new_rtl8201(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_LAN8720
    esp_eth_phy_t *phy = esp_eth_phy_new_lan8720(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_DP83848
    esp_eth_phy_t *phy = esp_eth_phy_new_dp83848(&phy_config);
#endif
#elif CONFIG_EXAMPLE_USE_DM9051
    gpio_install_isr_service(0);
    spi_device_handle_t spi_handle = NULL;
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_EXAMPLE_DM9051_MISO_GPIO,
        .mosi_io_num = CONFIG_EXAMPLE_DM9051_MOSI_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_DM9051_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_EXAMPLE_DM9051_SPI_HOST, &buscfg, 1));
    spi_device_interface_config_t devcfg = {
        .command_bits = 1,
        .address_bits = 7,
        .mode = 0,
        .clock_speed_hz = CONFIG_EXAMPLE_DM9051_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_EXAMPLE_DM9051_CS_GPIO,
        .queue_size = 20
    };
    ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_EXAMPLE_DM9051_SPI_HOST, &devcfg, &spi_handle));
    /* dm9051 ethernet driver is based on spi driver */
    eth_dm9051_config_t dm9051_config = ETH_DM9051_DEFAULT_CONFIG(spi_handle);
    dm9051_config.int_gpio_num = CONFIG_EXAMPLE_DM9051_INT_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_dm9051(&phy_config);
#endif
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

example_set_static_ip(eth_netif);

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

/*
xLights expects the following:
192.168.1.206
Universes 1-5 (chan 1-2550)

                Chan   Chan
                Start  End
Roof:           1      900
Underbody:      901    1620
Left Rocker:    1621   1776
Left Upright:   1777   1947
Right Rocker:   1948   2103
Right Upright:  2104   2274

* Note that led-display-manager has following:
Roof:      Output A
Underbody: Output B
Right:     Output C
Left:      Output D

So first pixel of strip:
A: chans 1-3
B: chans 901-903
C: chans 1948-1951
D: chans 1621-1623

*/

void app_main() {

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_LOGE(__func__, "Error initializing flash: %s, wiping", esp_err_to_name(ret));
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // We want the LEDs running ASAP after power-up
    start_leds();

    // Give the LED task time to start
    vTaskDelay(1000 / portTICK_PERIOD_MS);

#if !DESKTOP_TEST
    // Start Ethernet with static IP to be able to listed for E1.31 packets
    setupEthernet();

    // BLE used for user's remote control - this can be last because the user can wait a few seconds
    setup_GATTS();
#endif

    // Setup and launch the serial console last
    init_serial_console();
    register_system();
    register_nvs();
    register_led_controller_cmds();
    register_ble_cmds();

    // This should never return
    run_serial_console();
}
