#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "lwip/api.h"
#include "lwip/tcp.h"

#include "e131.c"

#define TX_PIN 17 // The DMX transmit pin.
// #define RX_PIN 16 // The DMX receive pin.
#define RX_PIN 5  // The DMX receive pin.
#define EN_PIN 15 // The DMX transmit enable pin.

#define DMX_RED 225
#define DMX_GREEN 226
#define DMX_BLUE 227
#define DMX_WHITE 228

static const char *TAG = "main"; // The log tagline.
#define ETC_PACKET_SIZE 513

static uint8_t data[ETC_PACKET_SIZE] = {}; // Buffer to store DMX data

// udp stuff
#define PORT 5568
struct netconn *udpConn;
#define UNIVERSE 1
const ip_addr_t multicastAddr = IPADDR4_INIT_BYTES(239, 255, 0, UNIVERSE);
static struct netbuf *buf;

// byte array that contains values for rgbw
char rgbw[] = {0x00, 0x00, 0x00, 0x00};

// sequence number for e1.31 packet
int SequenceNumber = 0;

void send_udp_packet()
{
  char dmxData[UDP_DATA_SIZE];
  createPacket(dmxData, (unsigned char)SequenceNumber, UNIVERSE, rgbw);

  buf = netbuf_new();                      // Create a new netbuf
  netbuf_ref(buf, dmxData, UDP_DATA_SIZE); // refer the netbuf to the data to be sent

  err_t err = netconn_sendto(udpConn, buf, &multicastAddr, PORT);
  netbuf_delete(buf); // delete the netbuf
  if (err < 0)
  {
    ESP_LOGE(TAG, "Error occurred during sending to %s: errno %s (%d) (%d)", ipaddr_ntoa(&multicastAddr), lwip_strerr(err), err, uxTaskGetStackHighWaterMark(NULL));
  }

  if (SequenceNumber == 255)
  {
    SequenceNumber = 0;
  }
  else
  {
    SequenceNumber++;
  }
}

void dmx_task()
{
  const dmx_port_t dmx_num = DMX_NUM_1;

  // Set communication pins and install the driver
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmx_num, &config, DMX_INTR_FLAGS_DEFAULT);
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);
  dmx_set_baud_rate(dmx_num, DMX_BAUD_RATE); // Set DMX baud rate.
  dmx_set_break_len(dmx_num, 94);            // Set DMX break length.
  dmx_set_mab_len(dmx_num, 26);              // Set DMX MAB length.

  dmx_packet_t packet;
  bool is_connected = false;

  TickType_t last_update = xTaskGetTickCount();

  ESP_LOGI(TAG, "waiting for dmx connection");

  while (true)
  {
    // Block until a packet is received
    if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK))
    {
      const TickType_t now = xTaskGetTickCount();
      // ESP_LOGI(TAG, "dmx: %lu", pdTICKS_TO_MS(xTaskGetTickCount()));

      if (!is_connected)
      {
        // Log when we first connect
        ESP_LOGI(TAG, "DMX is connected.");
        is_connected = true;
      }

      {
        dmx_read(dmx_num, data, ETC_PACKET_SIZE);
        // ESP_LOGI(TAG, "error: %i", packet.err);
        // ESP_LOGI(TAG, "Start code: %02x, Size: %i", packet.sc, packet.size);
        // ESP_LOG_BUFFER_HEX(TAG, data, 16); // Log first 16 bytes

        if (packet.err == 0 && packet.sc == 0x00 && packet.size == ETC_PACKET_SIZE)
        {
          // ESP_LOGI(TAG, "Start code: %02x, Size: %i", packet.sc, packet.size);
          // ESP_LOG_BUFFER_HEX(TAG, data, ETC_PACKET_SIZE); // Log first 16 bytes

          bool changed = false;
          if (rgbw[0] != data[DMX_RED] ||
              rgbw[1] != data[DMX_GREEN] ||
              rgbw[2] != data[DMX_BLUE] ||
              rgbw[3] != data[DMX_WHITE])
          {
            changed = true;
            rgbw[0] = data[DMX_RED];
            rgbw[1] = data[DMX_GREEN];
            rgbw[2] = data[DMX_BLUE];
            rgbw[3] = data[DMX_WHITE];
          }

          // if dmx data changed, send a udp packet
          // if it's been 500ms since the last udp packet, send another one
          if (changed || (now - last_update >= pdMS_TO_TICKS(500)))
          {
            // ESP_LOGI(TAG, "%lu", pdTICKS_TO_MS(now));
            send_udp_packet();
            last_update = now;
          }
        }
        // else
        // {
        //   // ESP_LOGI(TAG, "drop: %lu", pdTICKS_TO_MS(now));
        // }
      }
    }
    // else if (is_connected)
    // {
    //   // DMX timed out after having been previously connected
    //   ESP_LOGI(TAG, "DMX was disconnected.");
    // }
  }

  ESP_LOGI(TAG, "Uninstalling the DMX driver.");
  dmx_driver_delete(dmx_num);
}

err_t setup_conn()
{
  struct netconn *conn = netconn_new(NETCONN_UDP);

  ESP_LOGI(TAG, "binding to port 56145");
  err_t err = netconn_bind(conn, NULL, 56145);
  if (err != ERR_OK)
  {
    ESP_LOGE(TAG, "bind failed: %d", err);
    return err;
  }

  tcp_nagle_disable(conn->pcb.tcp); /* HACK */

  ip_addr_t localIP;
  u16_t localPort;

  err = netconn_getaddr(conn, &localIP, &localPort, 1);
  if (err != ERR_OK)
  {
    ESP_LOGE(TAG, "getaddr failed: %d", err);
    return err;
  }

  ESP_LOGI(TAG, "using local ip: %s:%i", ipaddr_ntoa(&localIP), localPort);

  err = netconn_join_leave_group(conn, &multicastAddr, &localIP, NETCONN_JOIN);
  if (err != ERR_OK)
  {
    ESP_LOGE(TAG, "multicast join failed: %d", err);
    return err;
  }

  udpConn = conn;

  return ERR_OK;
}

void delete_conn()
{
  ESP_LOGI(TAG, "Shutting down connection");
  netconn_close(udpConn);
  netconn_delete(udpConn);
}

void app_main()
{
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
   * Read "Establishing Wi-Fi or Ethernet Connection" section in
   * examples/protocols/README.md for more information about this function.
   */
  ESP_ERROR_CHECK(example_connect());

  ESP_LOGI(TAG, "setting up udp connections");
  err_t err = setup_conn();
  if (err != ERR_OK)
  {
    ESP_LOGE(TAG, "setup failed");
    esp_restart();
  }

  ESP_LOGI(TAG, "generating default udp packet");
  createDefaultPacket();

  ESP_LOGI(TAG, "starting dmx task");
  xTaskCreate(
      dmx_task,   /* Task function. */
      "dmx_task", /* String with name of task. */
      4096,       /* Stack size in words. */
      NULL,       /* Parameter passed as input of the task */
      1,          /* Priority of the task. */
      NULL);      /* Task handle. */

  // xTaskCreatePinnedToCore(
  //     dmx_task,   /* Task function. */
  //     "dmx_task", /* String with name of task. */
  //     4096,       /* Stack size in words. */
  //     NULL,       /* Parameter passed as input of the task */
  //     1,          /* Priority of the task. */
  //     NULL,       /* Task handle. */
  //     1);         /* Core ID */
}