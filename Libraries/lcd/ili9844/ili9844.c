/*
 * BSD 3-Clause License
 * 
 * Copyright (c) 2023, rudals
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "stm32f4xx_hal.h"
#include "ili9844.h"
#include "font.h"

#define LCD_WIDTH 480
#define LCD_HEIGHT 320

#define CS_PORT GPIOA
#define CS_PIN  GPIO_PIN_4
#define DC_PORT GPIOC
#define DC_PIN  GPIO_PIN_7
#define RST_PORT GPIOC
#define RST_PIN  GPIO_PIN_6

#define LCD_CS_SET  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)
#define LCD_RS_SET  HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET)
#define LCD_RST_SET HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET)

#define LCD_CS_CLR  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)
#define LCD_RS_CLR  HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET)
#define LCD_RST_CLR HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi1;

void LCD_Cmd(uint8_t cmd)
{
  LCD_CS_CLR;
  LCD_RS_CLR;
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
  LCD_CS_SET;
}

void LCD_Data(uint8_t data)
{
  LCD_CS_CLR;
  LCD_RS_SET;
  HAL_SPI_Transmit(&hspi1, &data, 1, 10);
  LCD_CS_SET;
}

void LCD_Reset()
{
  LCD_RST_CLR;
  HAL_Delay(100);
  LCD_RST_SET;
  HAL_Delay(50);
}

void LCD_Init()
{
  LCD_Reset();

  LCD_Cmd(0XF7);
  LCD_Data(0xA9);
  LCD_Data(0x51);
  LCD_Data(0x2C);
  LCD_Data(0x82);
  LCD_Cmd(0xC0);
  LCD_Data(0x11);
  LCD_Data(0x09);
  LCD_Cmd(0xC1);
  LCD_Data(0x41);
  LCD_Cmd(0XC5);
  LCD_Data(0x00);
  LCD_Data(0x0A);
  LCD_Data(0x80);
  LCD_Cmd(0xB1);
  LCD_Data(0xB0);
  LCD_Data(0x11);
  LCD_Cmd(0xB4);
  LCD_Data(0x02);
  LCD_Cmd(0xB6);
  LCD_Data(0x02);
  LCD_Data(0x42);
  LCD_Cmd(0xB7);
  LCD_Data(0xc6);
  LCD_Cmd(0xBE);
  LCD_Data(0x00);
  LCD_Data(0x04);
  LCD_Cmd(0xE9);
  LCD_Data(0x00);
  LCD_Cmd(0x36);
  LCD_Data((1<<3)|(0<<7)|(1<<6)|(1<<5));
  LCD_Cmd(0x3A);
  LCD_Data(0x66);
  LCD_Cmd(0xE0);
  LCD_Data(0x00);
  LCD_Data(0x07);
  LCD_Data(0x10);
  LCD_Data(0x09);
  LCD_Data(0x17);
  LCD_Data(0x0B);
  LCD_Data(0x41);
  LCD_Data(0x89);
  LCD_Data(0x4B);
  LCD_Data(0x0A);
  LCD_Data(0x0C);
  LCD_Data(0x0E);
  LCD_Data(0x18);
  LCD_Data(0x1B);
  LCD_Data(0x0F);
  LCD_Cmd(0XE1);
  LCD_Data(0x00);
  LCD_Data(0x17);
  LCD_Data(0x1A);
  LCD_Data(0x04);
  LCD_Data(0x0E);
  LCD_Data(0x06);
  LCD_Data(0x2F);
  LCD_Data(0x45);
  LCD_Data(0x43);
  LCD_Data(0x02);
  LCD_Data(0x0A);
  LCD_Data(0x09);
  LCD_Data(0x32);
  LCD_Data(0x36);
  LCD_Data(0x0F);
  LCD_Cmd(0x11);

  HAL_Delay(10);

  LCD_Cmd(0x29);
}

void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  LCD_Cmd(0x2A);
  LCD_Data(x0 >> 8);
  LCD_Data(0x00FF & x0);
  LCD_Data(x1 >> 8);
  LCD_Data(0x00FF & x1);

  LCD_Cmd(0x2B);
  LCD_Data(y0 >> 8);
  LCD_Data(0x00FF & y0);
  LCD_Data(y1 >> 8);
  LCD_Data(0x00FF & y1);

  LCD_Cmd(0x2C);
}

void LCD_Clear(uint16_t color)
{
  uint32_t i;
  uint8_t data[3];

  data[0] = (color >> 8) & 0xF8;
  data[1] = (color >> 3) & 0xFC;
  data[2] = (color << 3);

  int size = LCD_WIDTH * LCD_HEIGHT;

  LCD_SetWindow(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);

  LCD_CS_CLR;
  LCD_RS_SET;
  for(i=0 ; i<size ; i++){
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 3, 10);
  }
  LCD_CS_SET;
}

void LCD_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
  uint8_t data[3];
  uint16_t i,j;
  uint16_t width = ex - sx + 1;
  uint16_t height = ey - sy + 1;

  data[0] = (color >> 8) & 0xF8;
  data[1] = (color >> 3) & 0xFC;
  data[2] = (color << 3);

  LCD_SetWindow(sx, sy, ex, ey);

  LCD_CS_CLR;
  LCD_RS_SET;
  for(i=0;i<height;i++) {
    for(j=0;j<width;j++){
      HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 3, 10);
    }
  }
  LCD_CS_SET;

  LCD_SetWindow(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
}

void LCD_DrawImage(uint16_t sx, uint16_t sy, const uint8_t *p, uint16_t img_w, uint16_t img_h)
{
  uint32_t i, size;
  uint8_t byteH, byteL;
  uint8_t data[3];
  uint16_t color;
  LCD_SetWindow(sx, sy, sx + img_w - 1, sy + img_h - 1);
  size  = img_w * img_h;
  LCD_CS_CLR;
  LCD_RS_SET;

  for(i = 0 ; i < size ; i++) {
    byteL = *(p + i * 2);
    byteH = *(p + i * 2 + 1);
    color = byteH << 8 | byteL;
    data[0] = (color >> 8) & 0xF8;
    data[1] = (color >> 3) & 0xFC;
    data[2] = (color << 3);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 3, 10);
  }

  LCD_CS_SET;
  LCD_SetWindow(0, 0, LCD_WIDTH, LCD_HEIGHT);
}

void LCD_SetPixel(uint16_t x, uint16_t y, uint16_t color)
{
  uint8_t data[3];

  LCD_Cmd(0x2A);
  LCD_Data(x>>8);
  LCD_Data(0x00FF&x);
  LCD_Data(x>>8);
  LCD_Data(0x00FF&x);

  LCD_Cmd(0x2B);
  LCD_Data(y>>8);
  LCD_Data(0x00FF&y);
  LCD_Data(y>>8);
  LCD_Data(0x00FF&y);

  LCD_Cmd(0x2C);

  data[0] = (color>>8)&0xF8;
  data[1] = (color>>3)&0xFC;
  data[2] = color<<3;

  LCD_CS_CLR;
  LCD_RS_SET;
  HAL_SPI_Transmit(&hspi1, data, 3, 10);
  LCD_CS_SET;
}

void LCD_ShowChar(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t num, uint8_t size, uint8_t mode)
{
  uint8_t i, j;
  uint16_t* pChar;
  uint16_t temp_char;
  uint32_t font_w, font_h;

  num=num-' ';

  LCD_SetWindow(x,y,x+size/2-1,y+size-1);
#if 0
  if(size > 16){
    //pChar = (uint16_t*)&font16x26[num][0];
    pChar = (uint16_t*)&font12x20[num][0];
    font_w = 16;
    font_h = 20;
  }else{
    pChar = (uint16_t*)&font10x16[num][0];
    font_w = 8;
    font_h = 16;
  }
#else
  pChar = (uint16_t*)&font12x20[num][0];
  font_w = 16;
  font_h = 20;
#endif

  for(j=0 ; j < font_h ; j++) {
    temp_char = *pChar;
    for(i=0 ; i<font_w ; i++) {
      if(temp_char & 0x01){
        LCD_SetPixel(x+i, y+j, fc);
      }
      temp_char >>= 1;
    }
    pChar++;
  }

  LCD_SetWindow(0,0,LCD_WIDTH-1,LCD_HEIGHT-1);
}

void LCD_DrawText(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *str, uint8_t size, uint8_t mode)
{
  uint16_t x0 = x;

  while(*str!=0)
  {
    if( x > (LCD_WIDTH-size/2) || y > (LCD_HEIGHT-size)){
      return;
    }

    if(*str == 0x0D) {
      y += size;
      x = x0;
    } else {
      LCD_ShowChar(x, y, fc, bc, *str, 16, mode);
      x += 12;
    }
    str++;
  }
}

void LCD_DrawHLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  LCD_SetWindow(x, y, x + length - 1, y);
  for(int i = x ; i < x + length ; i++){
    LCD_SetPixel(i, y, color);
  }
}

void LCD_DrawVLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  LCD_SetWindow(x, y, x, y + length -1);
  for(int i = y ; i < y + length ; i++){
    LCD_SetPixel(x, i, color);
  }
}
