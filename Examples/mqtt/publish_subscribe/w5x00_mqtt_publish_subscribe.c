/**
 * Copyright (c) 2023 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * modified by rudals
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "main.h"
#include "stm32f4xx_hal.h"

#include "ili9844.h"

#include "wizchip_conf.h"
#include "w5x00_network.h"
#include "w5x00_timer.h"
#include "w5x00_demo.h"
#include "w5x00_mqtt_info.h"

#include "mqtt_interface.h"
#include "MQTTClient.h"

extern uint8_t g_dns_target_ip[];

extern const unsigned char img_fire_0[];
extern const unsigned char img_fire_1[];
extern const unsigned char img_gas_0[];
extern const unsigned char img_gas_1[];
extern const unsigned char img_door_0[];
extern const unsigned char img_door_1[];

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
#define SOCKET_DNS 0
#define DNS_RETRY_COUNT 5


/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
uint8_t g_mqtt_send_buf[ETHERNET_BUF_MAX_SIZE] = { 0, };
uint8_t g_mqtt_recv_buf[ETHERNET_BUF_MAX_SIZE] = { 0, };
Network g_mqtt_network;
MQTTClient g_mqtt_client;
MQTTPacket_connectData g_mqtt_packet_connect_data = MQTTPacket_connectData_initializer;
MQTTMessage g_mqtt_message;

char g_pub_payload_buf[MQTT_PUBLISH_PAYLOAD_SIZE];

struct ALARM_TYPE {
  int fire;
  int gas;
  int door;
};

struct ALARM_TYPE  g_alarm_type = { 1, 1, 0 };
struct ALARM_TYPE  g_alarm_type_old = { 1, 1, 0 };

char device_id[2][20];
int device_count = 0;
char mqtt_data[4][13];

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
static void repeating_timer_callback(void)
{
  MilliTimer_Handler();
}

void update_event_data(int index, int on)
{
  int pos[3][2] = { {24, 65}, {24, 152}, {24, 239} };
  const unsigned char* p_icon[3][2] = {
      {img_fire_0, img_fire_1},
      {img_gas_0, img_gas_1},
      {img_door_0, img_door_1}
  };

  LCD_DrawImage(pos[index][0], pos[index][1], p_icon[index][on], 110, 60);
}

void update_gps_data(int index, char *p_id, char *p_lang, char *p_long)
{
  int pos_id[2][2]   = { {185, 48}, {347, 48} };
  int pos_lang[2][2] = { {185, 72}, {347, 72} };
  int pos_long[2][2] = { {175, 98}, {337, 98} };

  LCD_Fill(pos_id[index][0]-27, pos_id[index][1]-3, pos_id[index][0]-27 + 158, pos_id[index][1]-3 + 24, 0xfec0);
  LCD_Fill(pos_lang[index][0]-27, pos_lang[index][1]-1, pos_lang[index][0]-27 + 158, pos_lang[index][1]-1 + 22, 0xffff);
  LCD_Fill(pos_long[index][0]-17, pos_long[index][1]-3, pos_long[index][0]-17 + 158, pos_long[index][1]-3 + 22, 0xffff);

  LCD_DrawText(  pos_id[index][0],   pos_id[index][1], 0, 0, (uint8_t *)p_id, 20, 1);
  LCD_DrawText(pos_lang[index][0], pos_lang[index][1], 0, 0, (uint8_t *)p_lang, 20, 1);
  LCD_DrawText(pos_long[index][0], pos_long[index][1], 0, 0, (uint8_t *)p_long, 20, 1);
}

void message_arrived(MessageData *msg_data)
{
  int i, j, found = 0, dat;
  int id_cnt;

  MQTTMessage *message = msg_data->message;

  j=0; dat=0;
  char *p = (char*)message->payload;
  
  for(i=0 ; i<=message->payloadlen ; i++) {
    if(p[i]==':'||p[i]=='\0') {
      mqtt_data[dat][j]='\0';
      dat++;
      j=0;
    } else {
      mqtt_data[dat][j] = p[i];
      j++;
    }
  }

  memset(message->payload, 0x0, (int)message->payloadlen);

  if(stricmp((const char*)mqtt_data, (const char*)"GPS") == 0){
    for (id_cnt = 0; id_cnt < 2; id_cnt++) {
      if (strcmp(device_id[id_cnt], mqtt_data[1]) == 0) {
        found = 1;
        break;
      }
    }

    if (!found) {
      if(device_count < 2){
        update_gps_data(device_count, mqtt_data[1], mqtt_data[2], mqtt_data[3]);
        strcpy(device_id[device_count], mqtt_data[1]);
        device_count++;
      }
    } else {
      update_gps_data(id_cnt, mqtt_data[1], mqtt_data[2], mqtt_data[3]);
    }
  } else if(stricmp((const char*)mqtt_data, (const char*)"ALARM") == 0){
    update_event_data(0, atoi(mqtt_data[1]));
    update_event_data(1, atoi(mqtt_data[2]));
    update_event_data(2, atoi(mqtt_data[3]));
  }
}

void check_event()
{
  int retval = -1;
  int fire_change = 0, gas_change = 0, door_change = 0;
  GPIO_PinState fire_state, gas_state, door_state;

  fire_state = HAL_GPIO_ReadPin(GPIOC, SENSOR1_Pin);
  gas_state = HAL_GPIO_ReadPin(GPIOC, SENSOR2_Pin);
  door_state = HAL_GPIO_ReadPin(GPIOA, SENSOR0_Pin);

  g_alarm_type.fire = (int)fire_state;
  g_alarm_type.gas = (int)gas_state;
  g_alarm_type.door = (int)door_state;

  if(g_alarm_type.fire != g_alarm_type_old.fire) fire_change = 1;
  if(g_alarm_type.gas != g_alarm_type_old.gas) gas_change = 1;
  if(g_alarm_type.door != g_alarm_type_old.door) door_change = 1;

  if(!(fire_change|gas_change|door_change)) return;

  snprintf((char*)g_pub_payload_buf, MQTT_PUBLISH_PAYLOAD_SIZE, "ALARM:%d:%d:%d",
      (g_alarm_type.fire == 1)?0:1, (g_alarm_type.gas==1)?0:1, g_alarm_type.door?1:0);

  g_mqtt_message.qos = QOS0;
  g_mqtt_message.retained = 0;
  g_mqtt_message.dup = 0;
  g_mqtt_message.payload = g_pub_payload_buf;
  g_mqtt_message.payloadlen = strlen((const char *)g_pub_payload_buf);

  retval = MQTTPublish(&g_mqtt_client, MQTT_SUBSCRIBE_TOPIC, &g_mqtt_message);
  if (retval < 0) {
    printf(" Publish failed : %d\r\n", retval);
    while (1);
  }

  memcpy(&g_alarm_type_old, &g_alarm_type, sizeof(struct ALARM_TYPE));
}

void mqtt_publish_subscribe_demo(wiz_NetInfo *net_info)
{
  int retval = 0;
  uint32_t start_msec = 0;
  uint32_t end_msec = 0;

  wizchip_1msec_timer_initialize(repeating_timer_callback);

  NewNetwork(&g_mqtt_network, SOCKET_MQTT);

  //retval = ConnectNetwork(&g_mqtt_network, g_mqtt_broker_ip, PORT_MQTT);
  retval = ConnectNetwork(&g_mqtt_network, g_dns_target_ip, PORT_MQTT);
  if (retval != 1) {
    printf(" Network connect failed\r\n");
    while (1) ;
  }

  MQTTClientInit(&g_mqtt_client, &g_mqtt_network, MQTT_TIMEOUT, g_mqtt_send_buf, ETHERNET_BUF_MAX_SIZE, g_mqtt_recv_buf, ETHERNET_BUF_MAX_SIZE);

  g_mqtt_packet_connect_data.MQTTVersion = 3;
  g_mqtt_packet_connect_data.cleansession = 1;
  g_mqtt_packet_connect_data.willFlag = 0;
  g_mqtt_packet_connect_data.keepAliveInterval = MQTT_KEEP_ALIVE;
  g_mqtt_packet_connect_data.clientID.cstring = MQTT_CLIENT_ID;
  g_mqtt_packet_connect_data.username.cstring = MQTT_USERNAME;
  g_mqtt_packet_connect_data.password.cstring = MQTT_PASSWORD;

  retval = MQTTConnect(&g_mqtt_client, &g_mqtt_packet_connect_data);

  if (retval < 0) {
    printf(" MQTT connect failed : %d\r\n", retval);
    while (1) ;
  }

  g_mqtt_message.qos = QOS0;
  g_mqtt_message.retained = 0;
  g_mqtt_message.dup = 0;
  g_mqtt_message.payload = MQTT_PUBLISH_PAYLOAD;
  g_mqtt_message.payloadlen = strlen(g_mqtt_message.payload);

  MQTTPublish(&g_mqtt_client, MQTT_PUBLISH_TOPIC, &g_mqtt_message);

  /* Subscribe */
  retval = MQTTSubscribe(&g_mqtt_client, MQTT_SUBSCRIBE_TOPIC, QOS0, message_arrived);
  if (retval < 0) {
    printf(" Subscribe failed : %d\r\n", retval);
    while (1) ;
  }

  /* Infinite loop */
  while (1) {
    if ((retval = MQTTYield(&g_mqtt_client, g_mqtt_packet_connect_data.keepAliveInterval)) < 0) {
      printf(" Yield error : %d\r\n", retval);
      while (1);
    }

    end_msec = wizchip_get_msec_tick();
    if (end_msec > start_msec + CHECK_SENSOR_PERIOD) {
      check_event();
      start_msec = wizchip_get_msec_tick();
    }
  }
}
