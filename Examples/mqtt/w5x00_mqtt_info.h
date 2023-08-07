/**
 * Copyright (c) 2023 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _W5X00_MQTT_INFO_H_
#define _W5X00_MQTT_INFO_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
#define SOCKET_MQTT 0
#define PORT_MQTT 1883

#define CHECK_SENSOR_PERIOD (1000 * 1) // unit : millisecond

#define MQTT_CLIENT_ID "IOT_GUARD_CLIENT"
#define MQTT_USERNAME "IOT_GUARD_USER"
#define MQTT_PASSWORD "IOT_GUARD_PASS"

#define MQTT_PUBLISH_TOPIC "ig_pub"
#define MQTT_PUBLISH_PAYLOAD_SIZE 50
#define MQTT_PUBLISH_PAYLOAD "IoTGuard Start"

#define MQTT_SUBSCRIBE_TOPIC "ig_sub"
#define MQTT_KEEP_ALIVE 60      // unit : second
#define MQTT_TIMEOUT (1000 * 1) // unit : millisecond

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif /* _W5X00_MQTT_INFO_H_ */
