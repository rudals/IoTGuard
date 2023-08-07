/**
 * Copyright (c) 2023 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * modified by rudals
 */

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "ili9844.h"

#include "socket.h"
#include "wizchip_conf.h"
#include "w5x00_network.h"

#define DATA_BUF_SIZE   (64000+8)

uint8_t g_raw_buf[DATA_BUF_SIZE] = { 0, };
uint8_t* buf = g_raw_buf;

static int packet_count = 0;
static int packet_size = 0;

extern char device_id[2][20];

void update_image(int index, char *p_img)
{
    int pos_img[2][2] = { {158, 119}, {319, 119} };
    LCD_DrawImage(pos_img[index][0], pos_img[index][1], (const uint8_t*)p_img, 160, 200);
}

int tcp_process(uint8_t sn, uint16_t port)
{
    int32_t ret;
    uint16_t size = 0, sentsize = 0;
    UNUSED(sentsize);
    uint32_t mode = 0;//SF_TCP_NODELAY;

    //TCP SOCKET status
    switch(getSn_SSR(sn)) {
        case SOCK_INIT:
            if((ret = listen(sn)) != SOCK_OK)
                return ret;

            break;

        case SOCK_CLOSED:
            if((ret = socket(sn, Sn_MR_TCP, port, mode)) != sn)
                return ret;

            break;

        case SOCK_CLOSE_WAIT :
            while((size = getSn_RX_RSR(sn)) > 0) {
                ret = recv(sn, &buf[packet_size], size);

                if(ret <= 0) {
                    printf("Error : SOCKERR_BUSY & SOCKERR_XXX(%lu)\r\n", ret);
                    return ret;
                }

                size = (uint16_t) ret;
                packet_size += size;
            }

            if(packet_size == DATA_BUF_SIZE) {
                char dev[9];
                strncpy(dev, buf, 8);
                dev[8] = '\0';
                int found = 0, cnt;

                for(cnt = 0; cnt < 2 ; cnt++) {
                    if(strcmp(device_id[cnt], dev) == 0) {
                        found = 1;
                        break;
                    }
                }

                update_image(cnt, (const uint8_t*)(buf + 8));
                packet_size = 0;
            }

            if((ret = disconnect(sn)) != SOCK_OK)
                return ret;

            break;

        case SOCK_ESTABLISHED:
            if(getSn_IR(sn) & Sn_IR_CON) {
                setSn_IR(sn, Sn_IR_CON);
                packet_count = 0;
                packet_size = 0;
            }

            if((size = getSn_RX_RSR(sn)) > 0) { // Don't need to check SOCKERR_BUSY because it doesn't not occur.
                if(size > DATA_BUF_SIZE)
                    size = DATA_BUF_SIZE;

                ret = recv(sn, &buf[packet_size], size);

                if(ret <= 0) {
                    printf("Error : SOCKERR_BUSY & SOCKERR_XXX(%lu)\r\n", ret);
                    return ret;      // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
                }

                size = (uint16_t) ret;
                packet_size += size;
            }

            break;

        default:
            break;
    }

    return ret;
}
