/* drivers/video/sprdfb/lcd/lcd_ili9806e_mipi.c
 *
 * Support for ili9806e mipi LCD device
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "../sprdfb_panel.h"
#include <linux/io.h>//add by liuwei
#include <soc/sprd/hardware.h>//add by liuwei
//#define LCD_Delay(ms)  uDelay(ms*1000)

#define MAX_DATA   48

typedef struct LCM_Init_Code_tag {
	unsigned int tag;
	unsigned char data[MAX_DATA];
}LCM_Init_Code;

typedef struct LCM_force_cmd_code_tag{
	unsigned int datatype;
	LCM_Init_Code real_cmd_code;
}LCM_Force_Cmd_Code;

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

static LCM_Force_Cmd_Code rd_prep_code[]={
	{0x37, {LCM_SEND(2), {0x5, 0}}},
};

static LCM_Force_Cmd_Code rd_prep_code_1[]={
	{0x37, {LCM_SEND(2), {0x1, 0}}},
};

static u8 lcd_ili9806e_zgd367_cmd_1[]=   {0xFF,0xFF,0x98,0x06,0x04,0x01};//ChangetoPage1
static u8 lcd_ili9806e_zgd367_cmd_2[]=   {0x08,0x10};//outputSDA
static u8 lcd_ili9806e_zgd367_cmd_3[]=   {0x21,0x01};//DE=1Active
static u8 lcd_ili9806e_zgd367_cmd_4[]=   {0x30,0x02};//480X800
static u8 lcd_ili9806e_zgd367_cmd_5[]=   {0x31,0x02};//2-dotInversion
static u8 lcd_ili9806e_zgd367_cmd_6[]=   {0x40,0x15};//DDVDH/DDVDL15
static u8 lcd_ili9806e_zgd367_cmd_7[]=   {0x41,0x22};//DDVDH/DDVDLCP44
static u8 lcd_ili9806e_zgd367_cmd_8[]=   {0x42,0x03};//VGH/VGL-7.9414.895.38-5.6-2.51.629.5ma19.7ma0.06ma
static u8 lcd_ili9806e_zgd367_cmd_9[]=   {0x43,0x09};//VGHCP89
static u8 lcd_ili9806e_zgd367_cmd_10[]=  {0x44,0x04};//VGLCP82
static u8 lcd_ili9806e_zgd367_cmd_11[]=  {0x50,0x80};//VGMP4.5V
static u8 lcd_ili9806e_zgd367_cmd_12[]=  {0x51,0x80};//VGMN4.5V
static u8 lcd_ili9806e_zgd367_cmd_13[]=  {0x52,0x00};//Flicker
static u8 lcd_ili9806e_zgd367_cmd_14[]=  {0x53,0x69};//Flicker
static u8 lcd_ili9806e_zgd367_cmd_15[]=  {0x60,0x0D};//SDTI
static u8 lcd_ili9806e_zgd367_cmd_16[]=  {0x61,0x00};//CRTI
static u8 lcd_ili9806e_zgd367_cmd_17[]=  {0x62,0x07};//EQTI
static u8 lcd_ili9806e_zgd367_cmd_18[]=  {0x63,0x00};//PCTI
static u8 lcd_ili9806e_zgd367_cmd_19[]=  {0xA0,0x00};//Gamma0
static u8 lcd_ili9806e_zgd367_cmd_20[]=  {0xA1,0x04};//Gamma4
static u8 lcd_ili9806e_zgd367_cmd_21[]=  {0xA2,0x0F};//Gamma8
static u8 lcd_ili9806e_zgd367_cmd_22[]=  {0xA3,0x0E};//Gamma16
static u8 lcd_ili9806e_zgd367_cmd_23[]=  {0xA4,0x07};//Gamma24
static u8 lcd_ili9806e_zgd367_cmd_24[]=  {0xA5,0x1A};//Gamma52
static u8 lcd_ili9806e_zgd367_cmd_25[]=  {0xA6,0x0B};//Gamma80
static u8 lcd_ili9806e_zgd367_cmd_26[]=  {0xA7,0x09};//Gamma108
static u8 lcd_ili9806e_zgd367_cmd_27[]=  {0xA8,0x05};//Gamma147
static u8 lcd_ili9806e_zgd367_cmd_28[]=  {0xA9,0x0A};//Gamma175
static u8 lcd_ili9806e_zgd367_cmd_29[]=  {0xAA,0x06};//Gamma203
static u8 lcd_ili9806e_zgd367_cmd_30[]=  {0xAB,0x0A};//Gamma231
static u8 lcd_ili9806e_zgd367_cmd_31[]=  {0xAC,0x0E};//Gamma239
static u8 lcd_ili9806e_zgd367_cmd_32[]=  {0xAD,0x2C};//Gamma247
static u8 lcd_ili9806e_zgd367_cmd_33[]=  {0xAE,0x29};//Gamma251
static u8 lcd_ili9806e_zgd367_cmd_34[]=  {0xAF,0x00};//Gamma255
static u8 lcd_ili9806e_zgd367_cmd_35[]=  {0xC0,0x00};//Gamma0
static u8 lcd_ili9806e_zgd367_cmd_36[]=  {0xC1,0x05};//Gamma4
static u8 lcd_ili9806e_zgd367_cmd_37[]=  {0xC2,0x0E};//Gamma8
static u8 lcd_ili9806e_zgd367_cmd_38[]=  {0xC3,0x12};//Gamma16
static u8 lcd_ili9806e_zgd367_cmd_39[]=  {0xC4,0x0C};//Gamma24
static u8 lcd_ili9806e_zgd367_cmd_40[]=  {0xC5,0x1A};//Gamma52
static u8 lcd_ili9806e_zgd367_cmd_41[]=  {0xC6,0x08};//Gamma80
static u8 lcd_ili9806e_zgd367_cmd_42[]=  {0xC7,0x08};//Gamma108
static u8 lcd_ili9806e_zgd367_cmd_43[]=  {0xC8,0x03};//Gamma147
static u8 lcd_ili9806e_zgd367_cmd_44[]=  {0xC9,0x0A};//Gamma175
static u8 lcd_ili9806e_zgd367_cmd_45[]=  {0xCA,0x06};//Gamma203
static u8 lcd_ili9806e_zgd367_cmd_46[]=  {0xCB,0x08};//Gamma231
static u8 lcd_ili9806e_zgd367_cmd_47[]=  {0xCC,0x0F};//Gamma239
static u8 lcd_ili9806e_zgd367_cmd_48[]=  {0xCD,0x2B};//Gamma247
static u8 lcd_ili9806e_zgd367_cmd_49[]=  {0xCE,0x29};//Gamma251
static u8 lcd_ili9806e_zgd367_cmd_50[]=  {0xCF,0x00};//Gamma255
static u8 lcd_ili9806e_zgd367_cmd_51[]=  {0xFF,0xFF,0x98,0x06,0x04,0x06};//ChangetoPage6
static u8 lcd_ili9806e_zgd367_cmd_52[]=  {0x00,0x3C};
static u8 lcd_ili9806e_zgd367_cmd_53[]=  {0x01,0x06};
static u8 lcd_ili9806e_zgd367_cmd_54[]=  {0x02,0x00};
static u8 lcd_ili9806e_zgd367_cmd_55[]=  {0x03,0x00};
static u8 lcd_ili9806e_zgd367_cmd_56[]=  {0x04,0x0D};
static u8 lcd_ili9806e_zgd367_cmd_57[]=  {0x05,0x0D};
static u8 lcd_ili9806e_zgd367_cmd_58[]=  {0x06,0x80};
static u8 lcd_ili9806e_zgd367_cmd_59[]=  {0x07,0x04};
static u8 lcd_ili9806e_zgd367_cmd_60[]=  {0x08,0x07};
static u8 lcd_ili9806e_zgd367_cmd_61[]=  {0x09,0x00};
static u8 lcd_ili9806e_zgd367_cmd_62[]=  {0x0A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_63[]=  {0x0B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_64[]=  {0x0C,0x1F};
static u8 lcd_ili9806e_zgd367_cmd_65[]=  {0x0D,0x1F};
static u8 lcd_ili9806e_zgd367_cmd_66[]=  {0x0E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_67[]=  {0x0F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_68[]=  {0x10,0x50};
static u8 lcd_ili9806e_zgd367_cmd_69[]=  {0x11,0xD0};
static u8 lcd_ili9806e_zgd367_cmd_70[]=  {0x12,0x00};
static u8 lcd_ili9806e_zgd367_cmd_71[]=  {0x13,0x00};
static u8 lcd_ili9806e_zgd367_cmd_72[]=  {0x14,0x00};
static u8 lcd_ili9806e_zgd367_cmd_73[]=  {0x15,0xC0};
static u8 lcd_ili9806e_zgd367_cmd_74[]=  {0x16,0x08};
static u8 lcd_ili9806e_zgd367_cmd_75[]=  {0x17,0x00};
static u8 lcd_ili9806e_zgd367_cmd_76[]=  {0x18,0x00};
static u8 lcd_ili9806e_zgd367_cmd_77[]=  {0x19,0x00};
static u8 lcd_ili9806e_zgd367_cmd_78[]=  {0x1A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_79[]=  {0x1B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_80[]=  {0x1C,0x40};
static u8 lcd_ili9806e_zgd367_cmd_81[]=  {0x1D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_82[]=  {0x20,0x01};
static u8 lcd_ili9806e_zgd367_cmd_83[]=  {0x21,0x23};
static u8 lcd_ili9806e_zgd367_cmd_84[]=  {0x22,0x45};
static u8 lcd_ili9806e_zgd367_cmd_85[]=  {0x23,0x67};
static u8 lcd_ili9806e_zgd367_cmd_86[]=  {0x24,0x01};
static u8 lcd_ili9806e_zgd367_cmd_87[]=  {0x25,0x23};
static u8 lcd_ili9806e_zgd367_cmd_88[]=  {0x26,0x45};
static u8 lcd_ili9806e_zgd367_cmd_89[]=  {0x27,0x67};
static u8 lcd_ili9806e_zgd367_cmd_90[]=  {0x30,0x13};
static u8 lcd_ili9806e_zgd367_cmd_91[]=  {0x31,0x22};
static u8 lcd_ili9806e_zgd367_cmd_92[]=  {0x32,0xDD};
static u8 lcd_ili9806e_zgd367_cmd_93[]=  {0x33,0xCC};
static u8 lcd_ili9806e_zgd367_cmd_94[]=  {0x34,0xBB};
static u8 lcd_ili9806e_zgd367_cmd_95[]=  {0x35,0xAA};
static u8 lcd_ili9806e_zgd367_cmd_96[]=  {0x36,0x22};
static u8 lcd_ili9806e_zgd367_cmd_97[]=  {0x37,0x26};
static u8 lcd_ili9806e_zgd367_cmd_98[]=  {0x38,0x72};
static u8 lcd_ili9806e_zgd367_cmd_99[]=  {0x39,0xFF};
static u8 lcd_ili9806e_zgd367_cmd_100[]= {0x3A,0x22};
static u8 lcd_ili9806e_zgd367_cmd_101[]= {0x3B,0xEE};
static u8 lcd_ili9806e_zgd367_cmd_102[]= {0x3C,0x22};
static u8 lcd_ili9806e_zgd367_cmd_103[]= {0x3D,0x22};
static u8 lcd_ili9806e_zgd367_cmd_104[]= {0x3E,0x22};
static u8 lcd_ili9806e_zgd367_cmd_105[]= {0x3F,0x22};
static u8 lcd_ili9806e_zgd367_cmd_106[]= {0x40,0x22};
static u8 lcd_ili9806e_zgd367_cmd_107[]= {0x52,0x10};
static u8 lcd_ili9806e_zgd367_cmd_108[]= {0x53,0x10};//0x10:VGLOtieVGL;0x12:VGLOtieVGL_REG
static u8 lcd_ili9806e_zgd367_cmd_109[]= {0xFF,0xFF,0x98,0x06,0x04,0x07};//ChangetoPage7
static u8 lcd_ili9806e_zgd367_cmd_110[]= {0x02,0x77};
static u8 lcd_ili9806e_zgd367_cmd_111[]= {0x17,0x22};//0x22:VGLOtieVGL;0x32:VGLOtieVGL_REG
static u8 lcd_ili9806e_zgd367_cmd_112[]= {0xE1,0x79};
static u8 lcd_ili9806e_zgd367_cmd_113[]= {0xFF,0xFF,0x98,0x06,0x04,0x02};//ChangetoPage2
static u8 lcd_ili9806e_zgd367_cmd_114[]= {0x40,0x01};
static u8 lcd_ili9806e_zgd367_cmd_115[]= {0x00,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_116[]= {0x01,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_117[]= {0x02,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_118[]= {0x03,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_119[]= {0x04,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_120[]= {0x05,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_121[]= {0x06,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_122[]= {0x07,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_123[]= {0x08,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_124[]= {0x09,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_125[]= {0x0A,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_126[]= {0x0B,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_127[]= {0x0C,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_128[]= {0x0D,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_129[]= {0x0E,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_130[]= {0x0F,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_131[]= {0x10,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_132[]= {0x11,0x4B};
static u8 lcd_ili9806e_zgd367_cmd_133[]= {0x12,0x4B};
static u8 lcd_ili9806e_zgd367_cmd_134[]= {0x13,0x4B};
static u8 lcd_ili9806e_zgd367_cmd_135[]= {0x14,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_136[]= {0x15,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_137[]= {0x16,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_138[]= {0x17,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_139[]= {0x18,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_140[]= {0x19,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_141[]= {0x1A,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_142[]= {0x1B,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_143[]= {0x1C,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_144[]= {0x1D,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_145[]= {0x1E,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_146[]= {0x1F,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_147[]= {0x20,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_148[]= {0x21,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_149[]= {0x22,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_150[]= {0x23,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_151[]= {0x24,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_152[]= {0x25,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_153[]= {0x26,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_154[]= {0x27,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_155[]= {0x28,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_156[]= {0x29,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_157[]= {0x2A,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_158[]= {0x2B,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_159[]= {0x2C,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_160[]= {0x2D,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_161[]= {0x2E,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_162[]= {0x2F,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_163[]= {0x30,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_164[]= {0x31,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_165[]= {0x32,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_166[]= {0x33,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_167[]= {0x34,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_168[]= {0x35,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_169[]= {0x36,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_170[]= {0x37,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_171[]= {0x38,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_172[]= {0x39,0x5D};
static u8 lcd_ili9806e_zgd367_cmd_173[]= {0x3A,0x4C};
static u8 lcd_ili9806e_zgd367_cmd_174[]= {0x3B,0x3B};
static u8 lcd_ili9806e_zgd367_cmd_175[]= {0x3C,0x1A};
static u8 lcd_ili9806e_zgd367_cmd_176[]= {0x3D,0x0A};
static u8 lcd_ili9806e_zgd367_cmd_177[]= {0x3E,0x2A};
static u8 lcd_ili9806e_zgd367_cmd_178[]= {0x3F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_179[]= {0xFF,0xFF,0x98,0x06,0x04,0x03};//ChangetoPage3
static u8 lcd_ili9806e_zgd367_cmd_180[]= {0x00,0x00};
static u8 lcd_ili9806e_zgd367_cmd_181[]= {0x01,0x00};
static u8 lcd_ili9806e_zgd367_cmd_182[]= {0x02,0x00};
static u8 lcd_ili9806e_zgd367_cmd_183[]= {0x03,0x00};
static u8 lcd_ili9806e_zgd367_cmd_184[]= {0x04,0x00};
static u8 lcd_ili9806e_zgd367_cmd_185[]= {0x05,0x00};
static u8 lcd_ili9806e_zgd367_cmd_186[]= {0x06,0x00};
static u8 lcd_ili9806e_zgd367_cmd_187[]= {0x07,0x00};
static u8 lcd_ili9806e_zgd367_cmd_188[]= {0x08,0x00};
static u8 lcd_ili9806e_zgd367_cmd_189[]= {0x09,0x00};
static u8 lcd_ili9806e_zgd367_cmd_190[]= {0x0A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_191[]= {0x0B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_192[]= {0x0C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_193[]= {0x0D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_194[]= {0x0E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_195[]= {0x0F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_196[]= {0x10,0x00};
static u8 lcd_ili9806e_zgd367_cmd_197[]= {0x11,0x00};
static u8 lcd_ili9806e_zgd367_cmd_198[]= {0x12,0x00};
static u8 lcd_ili9806e_zgd367_cmd_199[]= {0x13,0x00};
static u8 lcd_ili9806e_zgd367_cmd_200[]= {0x14,0x00};
static u8 lcd_ili9806e_zgd367_cmd_201[]= {0x15,0x00};
static u8 lcd_ili9806e_zgd367_cmd_202[]= {0x16,0x00};
static u8 lcd_ili9806e_zgd367_cmd_203[]= {0x17,0x00};
static u8 lcd_ili9806e_zgd367_cmd_204[]= {0x18,0x00};
static u8 lcd_ili9806e_zgd367_cmd_205[]= {0x19,0x00};
static u8 lcd_ili9806e_zgd367_cmd_206[]= {0x1A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_207[]= {0x1B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_208[]= {0x1C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_209[]= {0x1D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_210[]= {0x1E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_211[]= {0x1F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_212[]= {0x20,0x00};
static u8 lcd_ili9806e_zgd367_cmd_213[]= {0x21,0x00};
static u8 lcd_ili9806e_zgd367_cmd_214[]= {0x22,0x00};
static u8 lcd_ili9806e_zgd367_cmd_215[]= {0x23,0x00};
static u8 lcd_ili9806e_zgd367_cmd_216[]= {0x24,0x00};
static u8 lcd_ili9806e_zgd367_cmd_217[]= {0x25,0x00};
static u8 lcd_ili9806e_zgd367_cmd_218[]= {0x26,0x00};
static u8 lcd_ili9806e_zgd367_cmd_219[]= {0x27,0x00};
static u8 lcd_ili9806e_zgd367_cmd_220[]= {0x28,0x00};
static u8 lcd_ili9806e_zgd367_cmd_221[]= {0x29,0x00};
static u8 lcd_ili9806e_zgd367_cmd_222[]= {0x2A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_223[]= {0x2B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_224[]= {0x2C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_225[]= {0x2D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_226[]= {0x2E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_227[]= {0x2F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_228[]= {0x30,0x00};
static u8 lcd_ili9806e_zgd367_cmd_229[]= {0x31,0x00};
static u8 lcd_ili9806e_zgd367_cmd_230[]= {0x32,0x00};
static u8 lcd_ili9806e_zgd367_cmd_231[]= {0x33,0x00};
static u8 lcd_ili9806e_zgd367_cmd_232[]= {0x34,0x00};
static u8 lcd_ili9806e_zgd367_cmd_233[]= {0x35,0x00};
static u8 lcd_ili9806e_zgd367_cmd_234[]= {0x36,0x00};
static u8 lcd_ili9806e_zgd367_cmd_235[]= {0x37,0x00};
static u8 lcd_ili9806e_zgd367_cmd_236[]= {0x38,0x00};
static u8 lcd_ili9806e_zgd367_cmd_237[]= {0x39,0x00};
static u8 lcd_ili9806e_zgd367_cmd_238[]= {0x3A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_239[]= {0x3B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_240[]= {0x3C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_241[]= {0x3D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_242[]= {0x3E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_243[]= {0x3F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_244[]= {0x40,0x00};
static u8 lcd_ili9806e_zgd367_cmd_245[]= {0x41,0x00};
static u8 lcd_ili9806e_zgd367_cmd_246[]= {0x42,0x00};
static u8 lcd_ili9806e_zgd367_cmd_247[]= {0x43,0x00};
static u8 lcd_ili9806e_zgd367_cmd_248[]= {0x44,0x00};
static u8 lcd_ili9806e_zgd367_cmd_249[]= {0x45,0x00};
static u8 lcd_ili9806e_zgd367_cmd_250[]= {0x46,0x00};
static u8 lcd_ili9806e_zgd367_cmd_251[]= {0x47,0x00};
static u8 lcd_ili9806e_zgd367_cmd_252[]= {0x48,0x00};
static u8 lcd_ili9806e_zgd367_cmd_253[]= {0x49,0x00};
static u8 lcd_ili9806e_zgd367_cmd_254[]= {0x4A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_255[]= {0x4B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_256[]= {0x4C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_257[]= {0x4D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_258[]= {0x4E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_259[]= {0x4F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_260[]= {0x50,0x00};
static u8 lcd_ili9806e_zgd367_cmd_261[]= {0x51,0x00};
static u8 lcd_ili9806e_zgd367_cmd_262[]= {0x52,0x00};
static u8 lcd_ili9806e_zgd367_cmd_263[]= {0x53,0x00};
static u8 lcd_ili9806e_zgd367_cmd_264[]= {0x54,0x00};
static u8 lcd_ili9806e_zgd367_cmd_265[]= {0x55,0x00};
static u8 lcd_ili9806e_zgd367_cmd_266[]= {0x56,0x00};
static u8 lcd_ili9806e_zgd367_cmd_267[]= {0x57,0x00};
static u8 lcd_ili9806e_zgd367_cmd_268[]= {0x58,0x00};
static u8 lcd_ili9806e_zgd367_cmd_269[]= {0x59,0x00};
static u8 lcd_ili9806e_zgd367_cmd_270[]= {0x5A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_271[]= {0x5B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_272[]= {0x5C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_273[]= {0x5D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_274[]= {0x5E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_275[]= {0x5F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_276[]= {0x60,0x00};
static u8 lcd_ili9806e_zgd367_cmd_277[]= {0x61,0x00};
static u8 lcd_ili9806e_zgd367_cmd_278[]= {0x62,0x00};
static u8 lcd_ili9806e_zgd367_cmd_279[]= {0x63,0x00};
static u8 lcd_ili9806e_zgd367_cmd_280[]= {0x64,0x00};
static u8 lcd_ili9806e_zgd367_cmd_281[]= {0x65,0x00};
static u8 lcd_ili9806e_zgd367_cmd_282[]= {0x66,0x00};
static u8 lcd_ili9806e_zgd367_cmd_283[]= {0x67,0x00};
static u8 lcd_ili9806e_zgd367_cmd_284[]= {0x68,0x00};
static u8 lcd_ili9806e_zgd367_cmd_285[]= {0x69,0x00};
static u8 lcd_ili9806e_zgd367_cmd_286[]= {0x6A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_287[]= {0x6B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_288[]= {0x6C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_289[]= {0x6D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_290[]= {0x6E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_291[]= {0x6F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_292[]= {0x70,0x00};
static u8 lcd_ili9806e_zgd367_cmd_293[]= {0x71,0x00};
static u8 lcd_ili9806e_zgd367_cmd_294[]= {0x72,0x00};
static u8 lcd_ili9806e_zgd367_cmd_295[]= {0x73,0x00};
static u8 lcd_ili9806e_zgd367_cmd_296[]= {0x74,0x00};
static u8 lcd_ili9806e_zgd367_cmd_297[]= {0x75,0x00};
static u8 lcd_ili9806e_zgd367_cmd_298[]= {0x76,0x00};
static u8 lcd_ili9806e_zgd367_cmd_299[]= {0x77,0x00};
static u8 lcd_ili9806e_zgd367_cmd_300[]= {0x78,0x00};
static u8 lcd_ili9806e_zgd367_cmd_301[]= {0x79,0x00};
static u8 lcd_ili9806e_zgd367_cmd_302[]= {0x7A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_303[]= {0x7B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_304[]= {0x7C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_305[]= {0x7D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_306[]= {0x7E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_307[]= {0x7F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_308[]= {0xFF,0xFF,0x98,0x06,0x04,0x04};//ChangetoPage4
static u8 lcd_ili9806e_zgd367_cmd_309[]= {0x00,0x00};
static u8 lcd_ili9806e_zgd367_cmd_310[]= {0x01,0x00};
static u8 lcd_ili9806e_zgd367_cmd_311[]= {0x02,0x00};
static u8 lcd_ili9806e_zgd367_cmd_312[]= {0x03,0x00};
static u8 lcd_ili9806e_zgd367_cmd_313[]= {0x04,0x00};
static u8 lcd_ili9806e_zgd367_cmd_314[]= {0x05,0x00};
static u8 lcd_ili9806e_zgd367_cmd_315[]= {0x06,0x00};
static u8 lcd_ili9806e_zgd367_cmd_316[]= {0x07,0x00};
static u8 lcd_ili9806e_zgd367_cmd_317[]= {0x08,0x00};
static u8 lcd_ili9806e_zgd367_cmd_318[]= {0x09,0x00};
static u8 lcd_ili9806e_zgd367_cmd_319[]= {0x0A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_320[]= {0x0B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_321[]= {0x0C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_322[]= {0x0D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_323[]= {0x0E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_324[]= {0x0F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_325[]= {0x10,0x00};
static u8 lcd_ili9806e_zgd367_cmd_326[]= {0x11,0x00};
static u8 lcd_ili9806e_zgd367_cmd_327[]= {0x12,0x00};
static u8 lcd_ili9806e_zgd367_cmd_328[]= {0x13,0x00};
static u8 lcd_ili9806e_zgd367_cmd_329[]= {0x14,0x00};
static u8 lcd_ili9806e_zgd367_cmd_330[]= {0x15,0x00};
static u8 lcd_ili9806e_zgd367_cmd_331[]= {0x16,0x00};
static u8 lcd_ili9806e_zgd367_cmd_332[]= {0x17,0x00};
static u8 lcd_ili9806e_zgd367_cmd_333[]= {0x18,0x00};
static u8 lcd_ili9806e_zgd367_cmd_334[]= {0x19,0x00};
static u8 lcd_ili9806e_zgd367_cmd_335[]= {0x1A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_336[]= {0x1B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_337[]= {0x1C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_338[]= {0x1D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_339[]= {0x1E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_340[]= {0x1F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_341[]= {0x20,0x00};
static u8 lcd_ili9806e_zgd367_cmd_342[]= {0x21,0x00};
static u8 lcd_ili9806e_zgd367_cmd_343[]= {0x22,0x00};
static u8 lcd_ili9806e_zgd367_cmd_344[]= {0x23,0x00};
static u8 lcd_ili9806e_zgd367_cmd_345[]= {0x24,0x00};
static u8 lcd_ili9806e_zgd367_cmd_346[]= {0x25,0x00};
static u8 lcd_ili9806e_zgd367_cmd_347[]= {0x26,0x00};
static u8 lcd_ili9806e_zgd367_cmd_348[]= {0x27,0x00};
static u8 lcd_ili9806e_zgd367_cmd_349[]= {0x28,0x00};
static u8 lcd_ili9806e_zgd367_cmd_350[]= {0x29,0x00};
static u8 lcd_ili9806e_zgd367_cmd_351[]= {0x2A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_352[]= {0x2B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_353[]= {0x2C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_354[]= {0x2D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_355[]= {0x2E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_356[]= {0x2F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_357[]= {0x30,0x00};
static u8 lcd_ili9806e_zgd367_cmd_358[]= {0x31,0x00};
static u8 lcd_ili9806e_zgd367_cmd_359[]= {0x32,0x00};
static u8 lcd_ili9806e_zgd367_cmd_360[]= {0x33,0x00};
static u8 lcd_ili9806e_zgd367_cmd_361[]= {0x34,0x00};
static u8 lcd_ili9806e_zgd367_cmd_362[]= {0x35,0x00};
static u8 lcd_ili9806e_zgd367_cmd_363[]= {0x36,0x00};
static u8 lcd_ili9806e_zgd367_cmd_364[]= {0x37,0x00};
static u8 lcd_ili9806e_zgd367_cmd_365[]= {0x38,0x00};
static u8 lcd_ili9806e_zgd367_cmd_366[]= {0x39,0x00};
static u8 lcd_ili9806e_zgd367_cmd_367[]= {0x3A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_368[]= {0x3B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_369[]= {0x3C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_370[]= {0x3D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_371[]= {0x3E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_372[]= {0x3F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_373[]= {0x40,0x00};
static u8 lcd_ili9806e_zgd367_cmd_374[]= {0x41,0x00};
static u8 lcd_ili9806e_zgd367_cmd_375[]= {0x42,0x00};
static u8 lcd_ili9806e_zgd367_cmd_376[]= {0x43,0x00};
static u8 lcd_ili9806e_zgd367_cmd_377[]= {0x44,0x00};
static u8 lcd_ili9806e_zgd367_cmd_378[]= {0x45,0x00};
static u8 lcd_ili9806e_zgd367_cmd_379[]= {0x46,0x00};
static u8 lcd_ili9806e_zgd367_cmd_380[]= {0x47,0x00};
static u8 lcd_ili9806e_zgd367_cmd_381[]= {0x48,0x00};
static u8 lcd_ili9806e_zgd367_cmd_382[]= {0x49,0x00};
static u8 lcd_ili9806e_zgd367_cmd_383[]= {0x4A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_384[]= {0x4B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_385[]= {0x4C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_386[]= {0x4D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_387[]= {0x4E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_388[]= {0x4F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_389[]= {0x50,0x00};
static u8 lcd_ili9806e_zgd367_cmd_390[]= {0x51,0x00};
static u8 lcd_ili9806e_zgd367_cmd_391[]= {0x52,0x00};
static u8 lcd_ili9806e_zgd367_cmd_392[]= {0x53,0x00};
static u8 lcd_ili9806e_zgd367_cmd_393[]= {0x54,0x00};
static u8 lcd_ili9806e_zgd367_cmd_394[]= {0x55,0x00};
static u8 lcd_ili9806e_zgd367_cmd_395[]= {0x56,0x00};
static u8 lcd_ili9806e_zgd367_cmd_396[]= {0x57,0x00};
static u8 lcd_ili9806e_zgd367_cmd_397[]= {0x58,0x00};
static u8 lcd_ili9806e_zgd367_cmd_398[]= {0x59,0x00};
static u8 lcd_ili9806e_zgd367_cmd_399[]= {0x5A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_400[]= {0x5B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_401[]= {0x5C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_402[]= {0x5D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_403[]= {0x5E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_404[]= {0x5F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_405[]= {0x60,0x00};
static u8 lcd_ili9806e_zgd367_cmd_406[]= {0x61,0x00};
static u8 lcd_ili9806e_zgd367_cmd_407[]= {0x62,0x00};
static u8 lcd_ili9806e_zgd367_cmd_408[]= {0x63,0x00};
static u8 lcd_ili9806e_zgd367_cmd_409[]= {0x64,0x00};
static u8 lcd_ili9806e_zgd367_cmd_410[]= {0x65,0x00};
static u8 lcd_ili9806e_zgd367_cmd_411[]= {0x66,0x00};
static u8 lcd_ili9806e_zgd367_cmd_412[]= {0x67,0x00};
static u8 lcd_ili9806e_zgd367_cmd_413[]= {0x68,0x00};
static u8 lcd_ili9806e_zgd367_cmd_414[]= {0x69,0x00};
static u8 lcd_ili9806e_zgd367_cmd_415[]= {0x6A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_416[]= {0x6B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_417[]= {0x6C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_418[]= {0x6D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_419[]= {0x6E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_420[]= {0x6F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_421[]= {0x70,0x00};
static u8 lcd_ili9806e_zgd367_cmd_422[]= {0x71,0x00};
static u8 lcd_ili9806e_zgd367_cmd_423[]= {0x72,0x00};
static u8 lcd_ili9806e_zgd367_cmd_424[]= {0x73,0x00};
static u8 lcd_ili9806e_zgd367_cmd_425[]= {0x74,0x00};
static u8 lcd_ili9806e_zgd367_cmd_426[]= {0x75,0x00};
static u8 lcd_ili9806e_zgd367_cmd_427[]= {0x76,0x00};
static u8 lcd_ili9806e_zgd367_cmd_428[]= {0x77,0x00};
static u8 lcd_ili9806e_zgd367_cmd_429[]= {0x78,0x00};
static u8 lcd_ili9806e_zgd367_cmd_430[]= {0x79,0x00};
static u8 lcd_ili9806e_zgd367_cmd_431[]= {0x7A,0x00};
static u8 lcd_ili9806e_zgd367_cmd_432[]= {0x7B,0x00};
static u8 lcd_ili9806e_zgd367_cmd_433[]= {0x7C,0x00};
static u8 lcd_ili9806e_zgd367_cmd_434[]= {0x7D,0x00};
static u8 lcd_ili9806e_zgd367_cmd_435[]= {0x7E,0x00};
static u8 lcd_ili9806e_zgd367_cmd_436[]= {0x7F,0x00};
static u8 lcd_ili9806e_zgd367_cmd_437[]= {0xFF,0xFF,0x98,0x06,0x04,0x00};//ChangetoPage0
static u8 lcd_ili9806e_zgd367_cmd_438[]= {0x35,0x00};//TE
static u8 lcd_ili9806e_zgd367_cmd_439[]= {0x11,0x00};//Sleep-Out
static u8 lcd_ili9806e_zgd367_cmd_440[]= {0x29,0x00};//DisplayOn

static sprdfb_dev_dsi_cmds lcd_ili9806e_zgd367_cmds_init[] =
{
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_1), .p_cmd=lcd_ili9806e_zgd367_cmd_1,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_2), .p_cmd=lcd_ili9806e_zgd367_cmd_2,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_3), .p_cmd=lcd_ili9806e_zgd367_cmd_3,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_4), .p_cmd=lcd_ili9806e_zgd367_cmd_4,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_5), .p_cmd=lcd_ili9806e_zgd367_cmd_5,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_6), .p_cmd=lcd_ili9806e_zgd367_cmd_6,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_7), .p_cmd=lcd_ili9806e_zgd367_cmd_7,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_8), .p_cmd=lcd_ili9806e_zgd367_cmd_8,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_9), .p_cmd=lcd_ili9806e_zgd367_cmd_9,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_10), .p_cmd=lcd_ili9806e_zgd367_cmd_10,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_11), .p_cmd=lcd_ili9806e_zgd367_cmd_11,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_12), .p_cmd=lcd_ili9806e_zgd367_cmd_12,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_13), .p_cmd=lcd_ili9806e_zgd367_cmd_13,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_14), .p_cmd=lcd_ili9806e_zgd367_cmd_14,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_15), .p_cmd=lcd_ili9806e_zgd367_cmd_15,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_16), .p_cmd=lcd_ili9806e_zgd367_cmd_16,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_17), .p_cmd=lcd_ili9806e_zgd367_cmd_17,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_18), .p_cmd=lcd_ili9806e_zgd367_cmd_18,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_19), .p_cmd=lcd_ili9806e_zgd367_cmd_19,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_20), .p_cmd=lcd_ili9806e_zgd367_cmd_20,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_21), .p_cmd=lcd_ili9806e_zgd367_cmd_21,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_22), .p_cmd=lcd_ili9806e_zgd367_cmd_22,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_23), .p_cmd=lcd_ili9806e_zgd367_cmd_23,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_24), .p_cmd=lcd_ili9806e_zgd367_cmd_24,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_25), .p_cmd=lcd_ili9806e_zgd367_cmd_25,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_26), .p_cmd=lcd_ili9806e_zgd367_cmd_26,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_27), .p_cmd=lcd_ili9806e_zgd367_cmd_27,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_28), .p_cmd=lcd_ili9806e_zgd367_cmd_28,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_29), .p_cmd=lcd_ili9806e_zgd367_cmd_29,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_30), .p_cmd=lcd_ili9806e_zgd367_cmd_30,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_31), .p_cmd=lcd_ili9806e_zgd367_cmd_31,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_32), .p_cmd=lcd_ili9806e_zgd367_cmd_32,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_33), .p_cmd=lcd_ili9806e_zgd367_cmd_33,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_34), .p_cmd=lcd_ili9806e_zgd367_cmd_34,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_35), .p_cmd=lcd_ili9806e_zgd367_cmd_35,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_36), .p_cmd=lcd_ili9806e_zgd367_cmd_36,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_37), .p_cmd=lcd_ili9806e_zgd367_cmd_37,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_38), .p_cmd=lcd_ili9806e_zgd367_cmd_38,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_39), .p_cmd=lcd_ili9806e_zgd367_cmd_39,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_40), .p_cmd=lcd_ili9806e_zgd367_cmd_40,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_41), .p_cmd=lcd_ili9806e_zgd367_cmd_41,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_42), .p_cmd=lcd_ili9806e_zgd367_cmd_42,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_43), .p_cmd=lcd_ili9806e_zgd367_cmd_43,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_44), .p_cmd=lcd_ili9806e_zgd367_cmd_44,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_45), .p_cmd=lcd_ili9806e_zgd367_cmd_45,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_46), .p_cmd=lcd_ili9806e_zgd367_cmd_46,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_47), .p_cmd=lcd_ili9806e_zgd367_cmd_47,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_48), .p_cmd=lcd_ili9806e_zgd367_cmd_48,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_49), .p_cmd=lcd_ili9806e_zgd367_cmd_49,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_50), .p_cmd=lcd_ili9806e_zgd367_cmd_50,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_51), .p_cmd=lcd_ili9806e_zgd367_cmd_51,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_52), .p_cmd=lcd_ili9806e_zgd367_cmd_52,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_53), .p_cmd=lcd_ili9806e_zgd367_cmd_53,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_54), .p_cmd=lcd_ili9806e_zgd367_cmd_54,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_55), .p_cmd=lcd_ili9806e_zgd367_cmd_55,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_56), .p_cmd=lcd_ili9806e_zgd367_cmd_56,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_57), .p_cmd=lcd_ili9806e_zgd367_cmd_57,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_58), .p_cmd=lcd_ili9806e_zgd367_cmd_58,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_59), .p_cmd=lcd_ili9806e_zgd367_cmd_59,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_60), .p_cmd=lcd_ili9806e_zgd367_cmd_60,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_61), .p_cmd=lcd_ili9806e_zgd367_cmd_61,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_62), .p_cmd=lcd_ili9806e_zgd367_cmd_62,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_63), .p_cmd=lcd_ili9806e_zgd367_cmd_63,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_64), .p_cmd=lcd_ili9806e_zgd367_cmd_64,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_65), .p_cmd=lcd_ili9806e_zgd367_cmd_65,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_66), .p_cmd=lcd_ili9806e_zgd367_cmd_66,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_67), .p_cmd=lcd_ili9806e_zgd367_cmd_67,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_68), .p_cmd=lcd_ili9806e_zgd367_cmd_68,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_69), .p_cmd=lcd_ili9806e_zgd367_cmd_69,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_70), .p_cmd=lcd_ili9806e_zgd367_cmd_70,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_71), .p_cmd=lcd_ili9806e_zgd367_cmd_71,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_72), .p_cmd=lcd_ili9806e_zgd367_cmd_72,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_73), .p_cmd=lcd_ili9806e_zgd367_cmd_73,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_74), .p_cmd=lcd_ili9806e_zgd367_cmd_74,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_75), .p_cmd=lcd_ili9806e_zgd367_cmd_75,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_76), .p_cmd=lcd_ili9806e_zgd367_cmd_76,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_77), .p_cmd=lcd_ili9806e_zgd367_cmd_77,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_78), .p_cmd=lcd_ili9806e_zgd367_cmd_78,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_79), .p_cmd=lcd_ili9806e_zgd367_cmd_79,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_80), .p_cmd=lcd_ili9806e_zgd367_cmd_80,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_81), .p_cmd=lcd_ili9806e_zgd367_cmd_81,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_82), .p_cmd=lcd_ili9806e_zgd367_cmd_82,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_83), .p_cmd=lcd_ili9806e_zgd367_cmd_83,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_84), .p_cmd=lcd_ili9806e_zgd367_cmd_84,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_85), .p_cmd=lcd_ili9806e_zgd367_cmd_85,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_86), .p_cmd=lcd_ili9806e_zgd367_cmd_86,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_87), .p_cmd=lcd_ili9806e_zgd367_cmd_87,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_88), .p_cmd=lcd_ili9806e_zgd367_cmd_88,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_89), .p_cmd=lcd_ili9806e_zgd367_cmd_89,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_90), .p_cmd=lcd_ili9806e_zgd367_cmd_90,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_91), .p_cmd=lcd_ili9806e_zgd367_cmd_91,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_92), .p_cmd=lcd_ili9806e_zgd367_cmd_92,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_93), .p_cmd=lcd_ili9806e_zgd367_cmd_93,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_94), .p_cmd=lcd_ili9806e_zgd367_cmd_94,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_95), .p_cmd=lcd_ili9806e_zgd367_cmd_95,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_96), .p_cmd=lcd_ili9806e_zgd367_cmd_96,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_97), .p_cmd=lcd_ili9806e_zgd367_cmd_97,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_98), .p_cmd=lcd_ili9806e_zgd367_cmd_98,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_99), .p_cmd=lcd_ili9806e_zgd367_cmd_99,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_100), .p_cmd=lcd_ili9806e_zgd367_cmd_100,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_101), .p_cmd=lcd_ili9806e_zgd367_cmd_101,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_102), .p_cmd=lcd_ili9806e_zgd367_cmd_102,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_103), .p_cmd=lcd_ili9806e_zgd367_cmd_103,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_104), .p_cmd=lcd_ili9806e_zgd367_cmd_104,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_105), .p_cmd=lcd_ili9806e_zgd367_cmd_105,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_106), .p_cmd=lcd_ili9806e_zgd367_cmd_106,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_107), .p_cmd=lcd_ili9806e_zgd367_cmd_107,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_108), .p_cmd=lcd_ili9806e_zgd367_cmd_108,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_109), .p_cmd=lcd_ili9806e_zgd367_cmd_109,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_110), .p_cmd=lcd_ili9806e_zgd367_cmd_110,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_111), .p_cmd=lcd_ili9806e_zgd367_cmd_111,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_112), .p_cmd=lcd_ili9806e_zgd367_cmd_112,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_113), .p_cmd=lcd_ili9806e_zgd367_cmd_113,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_114), .p_cmd=lcd_ili9806e_zgd367_cmd_114,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_115), .p_cmd=lcd_ili9806e_zgd367_cmd_115,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_116), .p_cmd=lcd_ili9806e_zgd367_cmd_116,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_117), .p_cmd=lcd_ili9806e_zgd367_cmd_117,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_118), .p_cmd=lcd_ili9806e_zgd367_cmd_118,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_119), .p_cmd=lcd_ili9806e_zgd367_cmd_119,},
    
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_120), .p_cmd=lcd_ili9806e_zgd367_cmd_120,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_121), .p_cmd=lcd_ili9806e_zgd367_cmd_121,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_122), .p_cmd=lcd_ili9806e_zgd367_cmd_122,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_123), .p_cmd=lcd_ili9806e_zgd367_cmd_123,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_124), .p_cmd=lcd_ili9806e_zgd367_cmd_124,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_125), .p_cmd=lcd_ili9806e_zgd367_cmd_125,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_126), .p_cmd=lcd_ili9806e_zgd367_cmd_126,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_127), .p_cmd=lcd_ili9806e_zgd367_cmd_127,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_128), .p_cmd=lcd_ili9806e_zgd367_cmd_128,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_129), .p_cmd=lcd_ili9806e_zgd367_cmd_129,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_130), .p_cmd=lcd_ili9806e_zgd367_cmd_130,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_131), .p_cmd=lcd_ili9806e_zgd367_cmd_131,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_132), .p_cmd=lcd_ili9806e_zgd367_cmd_132,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_133), .p_cmd=lcd_ili9806e_zgd367_cmd_133,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_134), .p_cmd=lcd_ili9806e_zgd367_cmd_134,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_135), .p_cmd=lcd_ili9806e_zgd367_cmd_135,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_136), .p_cmd=lcd_ili9806e_zgd367_cmd_136,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_137), .p_cmd=lcd_ili9806e_zgd367_cmd_137,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_138), .p_cmd=lcd_ili9806e_zgd367_cmd_138,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_139), .p_cmd=lcd_ili9806e_zgd367_cmd_139,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_140), .p_cmd=lcd_ili9806e_zgd367_cmd_140,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_141), .p_cmd=lcd_ili9806e_zgd367_cmd_141,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_142), .p_cmd=lcd_ili9806e_zgd367_cmd_142,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_143), .p_cmd=lcd_ili9806e_zgd367_cmd_143,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_144), .p_cmd=lcd_ili9806e_zgd367_cmd_144,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_145), .p_cmd=lcd_ili9806e_zgd367_cmd_145,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_146), .p_cmd=lcd_ili9806e_zgd367_cmd_146,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_147), .p_cmd=lcd_ili9806e_zgd367_cmd_147,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_148), .p_cmd=lcd_ili9806e_zgd367_cmd_148,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_149), .p_cmd=lcd_ili9806e_zgd367_cmd_149,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_150), .p_cmd=lcd_ili9806e_zgd367_cmd_150,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_151), .p_cmd=lcd_ili9806e_zgd367_cmd_151,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_152), .p_cmd=lcd_ili9806e_zgd367_cmd_152,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_153), .p_cmd=lcd_ili9806e_zgd367_cmd_153,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_154), .p_cmd=lcd_ili9806e_zgd367_cmd_154,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_155), .p_cmd=lcd_ili9806e_zgd367_cmd_155,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_156), .p_cmd=lcd_ili9806e_zgd367_cmd_156,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_157), .p_cmd=lcd_ili9806e_zgd367_cmd_157,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_158), .p_cmd=lcd_ili9806e_zgd367_cmd_158,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_159), .p_cmd=lcd_ili9806e_zgd367_cmd_159,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_160), .p_cmd=lcd_ili9806e_zgd367_cmd_160,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_161), .p_cmd=lcd_ili9806e_zgd367_cmd_161,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_162), .p_cmd=lcd_ili9806e_zgd367_cmd_162,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_163), .p_cmd=lcd_ili9806e_zgd367_cmd_163,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_164), .p_cmd=lcd_ili9806e_zgd367_cmd_164,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_165), .p_cmd=lcd_ili9806e_zgd367_cmd_165,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_166), .p_cmd=lcd_ili9806e_zgd367_cmd_166,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_167), .p_cmd=lcd_ili9806e_zgd367_cmd_167,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_168), .p_cmd=lcd_ili9806e_zgd367_cmd_168,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_169), .p_cmd=lcd_ili9806e_zgd367_cmd_169,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_170), .p_cmd=lcd_ili9806e_zgd367_cmd_170,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_171), .p_cmd=lcd_ili9806e_zgd367_cmd_171,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_172), .p_cmd=lcd_ili9806e_zgd367_cmd_172,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_173), .p_cmd=lcd_ili9806e_zgd367_cmd_173,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_174), .p_cmd=lcd_ili9806e_zgd367_cmd_174,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_175), .p_cmd=lcd_ili9806e_zgd367_cmd_175,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_176), .p_cmd=lcd_ili9806e_zgd367_cmd_176,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_177), .p_cmd=lcd_ili9806e_zgd367_cmd_177,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_178), .p_cmd=lcd_ili9806e_zgd367_cmd_178,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_179), .p_cmd=lcd_ili9806e_zgd367_cmd_179,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_180), .p_cmd=lcd_ili9806e_zgd367_cmd_110,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_181), .p_cmd=lcd_ili9806e_zgd367_cmd_181,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_182), .p_cmd=lcd_ili9806e_zgd367_cmd_182,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_183), .p_cmd=lcd_ili9806e_zgd367_cmd_183,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_184), .p_cmd=lcd_ili9806e_zgd367_cmd_184,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_185), .p_cmd=lcd_ili9806e_zgd367_cmd_185,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_186), .p_cmd=lcd_ili9806e_zgd367_cmd_186,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_187), .p_cmd=lcd_ili9806e_zgd367_cmd_187,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_188), .p_cmd=lcd_ili9806e_zgd367_cmd_188,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_189), .p_cmd=lcd_ili9806e_zgd367_cmd_189,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_190), .p_cmd=lcd_ili9806e_zgd367_cmd_190,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_191), .p_cmd=lcd_ili9806e_zgd367_cmd_191,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_192), .p_cmd=lcd_ili9806e_zgd367_cmd_192,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_193), .p_cmd=lcd_ili9806e_zgd367_cmd_193,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_194), .p_cmd=lcd_ili9806e_zgd367_cmd_194,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_195), .p_cmd=lcd_ili9806e_zgd367_cmd_195,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_196), .p_cmd=lcd_ili9806e_zgd367_cmd_196,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_197), .p_cmd=lcd_ili9806e_zgd367_cmd_197,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_198), .p_cmd=lcd_ili9806e_zgd367_cmd_198,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_199), .p_cmd=lcd_ili9806e_zgd367_cmd_199,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_200), .p_cmd=lcd_ili9806e_zgd367_cmd_200,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_201), .p_cmd=lcd_ili9806e_zgd367_cmd_201,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_202), .p_cmd=lcd_ili9806e_zgd367_cmd_202,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_203), .p_cmd=lcd_ili9806e_zgd367_cmd_203,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_204), .p_cmd=lcd_ili9806e_zgd367_cmd_204,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_205), .p_cmd=lcd_ili9806e_zgd367_cmd_205,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_206), .p_cmd=lcd_ili9806e_zgd367_cmd_206,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_207), .p_cmd=lcd_ili9806e_zgd367_cmd_207,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_208), .p_cmd=lcd_ili9806e_zgd367_cmd_208,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_209), .p_cmd=lcd_ili9806e_zgd367_cmd_209,},
    
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_210), .p_cmd=lcd_ili9806e_zgd367_cmd_210,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_211), .p_cmd=lcd_ili9806e_zgd367_cmd_211,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_212), .p_cmd=lcd_ili9806e_zgd367_cmd_212,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_213), .p_cmd=lcd_ili9806e_zgd367_cmd_213,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_214), .p_cmd=lcd_ili9806e_zgd367_cmd_214,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_215), .p_cmd=lcd_ili9806e_zgd367_cmd_215,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_216), .p_cmd=lcd_ili9806e_zgd367_cmd_216,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_217), .p_cmd=lcd_ili9806e_zgd367_cmd_217,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_218), .p_cmd=lcd_ili9806e_zgd367_cmd_218,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_219), .p_cmd=lcd_ili9806e_zgd367_cmd_219,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_220), .p_cmd=lcd_ili9806e_zgd367_cmd_220,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_221), .p_cmd=lcd_ili9806e_zgd367_cmd_221,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_222), .p_cmd=lcd_ili9806e_zgd367_cmd_222,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_223), .p_cmd=lcd_ili9806e_zgd367_cmd_223,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_224), .p_cmd=lcd_ili9806e_zgd367_cmd_224,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_225), .p_cmd=lcd_ili9806e_zgd367_cmd_225,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_226), .p_cmd=lcd_ili9806e_zgd367_cmd_226,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_227), .p_cmd=lcd_ili9806e_zgd367_cmd_227,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_228), .p_cmd=lcd_ili9806e_zgd367_cmd_228,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_229), .p_cmd=lcd_ili9806e_zgd367_cmd_229,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_230), .p_cmd=lcd_ili9806e_zgd367_cmd_230,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_231), .p_cmd=lcd_ili9806e_zgd367_cmd_231,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_232), .p_cmd=lcd_ili9806e_zgd367_cmd_232,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_233), .p_cmd=lcd_ili9806e_zgd367_cmd_233,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_234), .p_cmd=lcd_ili9806e_zgd367_cmd_234,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_235), .p_cmd=lcd_ili9806e_zgd367_cmd_235,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_236), .p_cmd=lcd_ili9806e_zgd367_cmd_236,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_237), .p_cmd=lcd_ili9806e_zgd367_cmd_237,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_238), .p_cmd=lcd_ili9806e_zgd367_cmd_238,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_239), .p_cmd=lcd_ili9806e_zgd367_cmd_239,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_240), .p_cmd=lcd_ili9806e_zgd367_cmd_240,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_241), .p_cmd=lcd_ili9806e_zgd367_cmd_241,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_242), .p_cmd=lcd_ili9806e_zgd367_cmd_242,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_243), .p_cmd=lcd_ili9806e_zgd367_cmd_243,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_244), .p_cmd=lcd_ili9806e_zgd367_cmd_244,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_245), .p_cmd=lcd_ili9806e_zgd367_cmd_245,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_246), .p_cmd=lcd_ili9806e_zgd367_cmd_246,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_247), .p_cmd=lcd_ili9806e_zgd367_cmd_247,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_248), .p_cmd=lcd_ili9806e_zgd367_cmd_248,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_249), .p_cmd=lcd_ili9806e_zgd367_cmd_249,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_250), .p_cmd=lcd_ili9806e_zgd367_cmd_250,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_251), .p_cmd=lcd_ili9806e_zgd367_cmd_251,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_252), .p_cmd=lcd_ili9806e_zgd367_cmd_252,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_253), .p_cmd=lcd_ili9806e_zgd367_cmd_253,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_254), .p_cmd=lcd_ili9806e_zgd367_cmd_254,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_255), .p_cmd=lcd_ili9806e_zgd367_cmd_255,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_256), .p_cmd=lcd_ili9806e_zgd367_cmd_256,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_257), .p_cmd=lcd_ili9806e_zgd367_cmd_257,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_258), .p_cmd=lcd_ili9806e_zgd367_cmd_258,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_259), .p_cmd=lcd_ili9806e_zgd367_cmd_259,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_260), .p_cmd=lcd_ili9806e_zgd367_cmd_260,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_261), .p_cmd=lcd_ili9806e_zgd367_cmd_261,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_262), .p_cmd=lcd_ili9806e_zgd367_cmd_262,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_263), .p_cmd=lcd_ili9806e_zgd367_cmd_263,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_264), .p_cmd=lcd_ili9806e_zgd367_cmd_264,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_265), .p_cmd=lcd_ili9806e_zgd367_cmd_265,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_266), .p_cmd=lcd_ili9806e_zgd367_cmd_266,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_267), .p_cmd=lcd_ili9806e_zgd367_cmd_267,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_268), .p_cmd=lcd_ili9806e_zgd367_cmd_268,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_269), .p_cmd=lcd_ili9806e_zgd367_cmd_269,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_270), .p_cmd=lcd_ili9806e_zgd367_cmd_270,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_271), .p_cmd=lcd_ili9806e_zgd367_cmd_271,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_272), .p_cmd=lcd_ili9806e_zgd367_cmd_272,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_273), .p_cmd=lcd_ili9806e_zgd367_cmd_273,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_274), .p_cmd=lcd_ili9806e_zgd367_cmd_274,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_275), .p_cmd=lcd_ili9806e_zgd367_cmd_275,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_276), .p_cmd=lcd_ili9806e_zgd367_cmd_276,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_277), .p_cmd=lcd_ili9806e_zgd367_cmd_277,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_278), .p_cmd=lcd_ili9806e_zgd367_cmd_278,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_279), .p_cmd=lcd_ili9806e_zgd367_cmd_279,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_280), .p_cmd=lcd_ili9806e_zgd367_cmd_280,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_281), .p_cmd=lcd_ili9806e_zgd367_cmd_281,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_282), .p_cmd=lcd_ili9806e_zgd367_cmd_282,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_283), .p_cmd=lcd_ili9806e_zgd367_cmd_283,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_284), .p_cmd=lcd_ili9806e_zgd367_cmd_284,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_285), .p_cmd=lcd_ili9806e_zgd367_cmd_285,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_286), .p_cmd=lcd_ili9806e_zgd367_cmd_286,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_287), .p_cmd=lcd_ili9806e_zgd367_cmd_287,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_288), .p_cmd=lcd_ili9806e_zgd367_cmd_288,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_289), .p_cmd=lcd_ili9806e_zgd367_cmd_289,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_290), .p_cmd=lcd_ili9806e_zgd367_cmd_290,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_291), .p_cmd=lcd_ili9806e_zgd367_cmd_291,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_292), .p_cmd=lcd_ili9806e_zgd367_cmd_292,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_293), .p_cmd=lcd_ili9806e_zgd367_cmd_293,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_294), .p_cmd=lcd_ili9806e_zgd367_cmd_294,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_295), .p_cmd=lcd_ili9806e_zgd367_cmd_295,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_296), .p_cmd=lcd_ili9806e_zgd367_cmd_296,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_297), .p_cmd=lcd_ili9806e_zgd367_cmd_297,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_298), .p_cmd=lcd_ili9806e_zgd367_cmd_298,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_299), .p_cmd=lcd_ili9806e_zgd367_cmd_299,},
    
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_300), .p_cmd=lcd_ili9806e_zgd367_cmd_300,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_301), .p_cmd=lcd_ili9806e_zgd367_cmd_301,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_302), .p_cmd=lcd_ili9806e_zgd367_cmd_302,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_303), .p_cmd=lcd_ili9806e_zgd367_cmd_303,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_304), .p_cmd=lcd_ili9806e_zgd367_cmd_304,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_305), .p_cmd=lcd_ili9806e_zgd367_cmd_305,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_306), .p_cmd=lcd_ili9806e_zgd367_cmd_306,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_307), .p_cmd=lcd_ili9806e_zgd367_cmd_307,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_308), .p_cmd=lcd_ili9806e_zgd367_cmd_308,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_309), .p_cmd=lcd_ili9806e_zgd367_cmd_309,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_310), .p_cmd=lcd_ili9806e_zgd367_cmd_310,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_311), .p_cmd=lcd_ili9806e_zgd367_cmd_311,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_312), .p_cmd=lcd_ili9806e_zgd367_cmd_312,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_313), .p_cmd=lcd_ili9806e_zgd367_cmd_313,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_314), .p_cmd=lcd_ili9806e_zgd367_cmd_314,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_315), .p_cmd=lcd_ili9806e_zgd367_cmd_315,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_316), .p_cmd=lcd_ili9806e_zgd367_cmd_316,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_317), .p_cmd=lcd_ili9806e_zgd367_cmd_317,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_318), .p_cmd=lcd_ili9806e_zgd367_cmd_318,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_319), .p_cmd=lcd_ili9806e_zgd367_cmd_319,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_320), .p_cmd=lcd_ili9806e_zgd367_cmd_320,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_321), .p_cmd=lcd_ili9806e_zgd367_cmd_321,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_322), .p_cmd=lcd_ili9806e_zgd367_cmd_322,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_323), .p_cmd=lcd_ili9806e_zgd367_cmd_323,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_324), .p_cmd=lcd_ili9806e_zgd367_cmd_324,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_325), .p_cmd=lcd_ili9806e_zgd367_cmd_325,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_326), .p_cmd=lcd_ili9806e_zgd367_cmd_326,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_327), .p_cmd=lcd_ili9806e_zgd367_cmd_327,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_328), .p_cmd=lcd_ili9806e_zgd367_cmd_328,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_329), .p_cmd=lcd_ili9806e_zgd367_cmd_329,},    
    
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_330), .p_cmd=lcd_ili9806e_zgd367_cmd_330,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_331), .p_cmd=lcd_ili9806e_zgd367_cmd_331,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_332), .p_cmd=lcd_ili9806e_zgd367_cmd_332,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_333), .p_cmd=lcd_ili9806e_zgd367_cmd_333,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_334), .p_cmd=lcd_ili9806e_zgd367_cmd_334,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_335), .p_cmd=lcd_ili9806e_zgd367_cmd_335,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_336), .p_cmd=lcd_ili9806e_zgd367_cmd_336,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_337), .p_cmd=lcd_ili9806e_zgd367_cmd_337,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_338), .p_cmd=lcd_ili9806e_zgd367_cmd_338,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_339), .p_cmd=lcd_ili9806e_zgd367_cmd_339,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_340), .p_cmd=lcd_ili9806e_zgd367_cmd_340,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_341), .p_cmd=lcd_ili9806e_zgd367_cmd_341,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_342), .p_cmd=lcd_ili9806e_zgd367_cmd_342,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_343), .p_cmd=lcd_ili9806e_zgd367_cmd_343,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_344), .p_cmd=lcd_ili9806e_zgd367_cmd_344,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_345), .p_cmd=lcd_ili9806e_zgd367_cmd_345,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_346), .p_cmd=lcd_ili9806e_zgd367_cmd_346,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_347), .p_cmd=lcd_ili9806e_zgd367_cmd_347,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_348), .p_cmd=lcd_ili9806e_zgd367_cmd_348,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_349), .p_cmd=lcd_ili9806e_zgd367_cmd_349,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_350), .p_cmd=lcd_ili9806e_zgd367_cmd_350,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_351), .p_cmd=lcd_ili9806e_zgd367_cmd_351,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_352), .p_cmd=lcd_ili9806e_zgd367_cmd_352,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_353), .p_cmd=lcd_ili9806e_zgd367_cmd_353,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_354), .p_cmd=lcd_ili9806e_zgd367_cmd_354,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_355), .p_cmd=lcd_ili9806e_zgd367_cmd_355,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_356), .p_cmd=lcd_ili9806e_zgd367_cmd_356,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_357), .p_cmd=lcd_ili9806e_zgd367_cmd_357,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_358), .p_cmd=lcd_ili9806e_zgd367_cmd_358,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_359), .p_cmd=lcd_ili9806e_zgd367_cmd_359,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_360), .p_cmd=lcd_ili9806e_zgd367_cmd_360,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_361), .p_cmd=lcd_ili9806e_zgd367_cmd_361,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_362), .p_cmd=lcd_ili9806e_zgd367_cmd_362,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_363), .p_cmd=lcd_ili9806e_zgd367_cmd_363,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_364), .p_cmd=lcd_ili9806e_zgd367_cmd_364,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_365), .p_cmd=lcd_ili9806e_zgd367_cmd_365,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_366), .p_cmd=lcd_ili9806e_zgd367_cmd_366,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_367), .p_cmd=lcd_ili9806e_zgd367_cmd_367,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_368), .p_cmd=lcd_ili9806e_zgd367_cmd_368,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_369), .p_cmd=lcd_ili9806e_zgd367_cmd_369,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_370), .p_cmd=lcd_ili9806e_zgd367_cmd_370,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_371), .p_cmd=lcd_ili9806e_zgd367_cmd_371,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_372), .p_cmd=lcd_ili9806e_zgd367_cmd_372,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_373), .p_cmd=lcd_ili9806e_zgd367_cmd_373,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_374), .p_cmd=lcd_ili9806e_zgd367_cmd_374,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_375), .p_cmd=lcd_ili9806e_zgd367_cmd_375,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_376), .p_cmd=lcd_ili9806e_zgd367_cmd_376,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_377), .p_cmd=lcd_ili9806e_zgd367_cmd_377,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_378), .p_cmd=lcd_ili9806e_zgd367_cmd_378,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_379), .p_cmd=lcd_ili9806e_zgd367_cmd_379,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_380), .p_cmd=lcd_ili9806e_zgd367_cmd_380,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_381), .p_cmd=lcd_ili9806e_zgd367_cmd_381,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_382), .p_cmd=lcd_ili9806e_zgd367_cmd_382,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_383), .p_cmd=lcd_ili9806e_zgd367_cmd_383,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_384), .p_cmd=lcd_ili9806e_zgd367_cmd_384,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_385), .p_cmd=lcd_ili9806e_zgd367_cmd_385,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_386), .p_cmd=lcd_ili9806e_zgd367_cmd_386,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_387), .p_cmd=lcd_ili9806e_zgd367_cmd_387,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_388), .p_cmd=lcd_ili9806e_zgd367_cmd_388,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_389), .p_cmd=lcd_ili9806e_zgd367_cmd_389,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_390), .p_cmd=lcd_ili9806e_zgd367_cmd_390,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_391), .p_cmd=lcd_ili9806e_zgd367_cmd_391,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_392), .p_cmd=lcd_ili9806e_zgd367_cmd_392,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_393), .p_cmd=lcd_ili9806e_zgd367_cmd_393,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_394), .p_cmd=lcd_ili9806e_zgd367_cmd_394,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_395), .p_cmd=lcd_ili9806e_zgd367_cmd_395,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_396), .p_cmd=lcd_ili9806e_zgd367_cmd_396,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_397), .p_cmd=lcd_ili9806e_zgd367_cmd_397,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_398), .p_cmd=lcd_ili9806e_zgd367_cmd_398,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_399), .p_cmd=lcd_ili9806e_zgd367_cmd_399,}, 
    
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_400), .p_cmd=lcd_ili9806e_zgd367_cmd_400,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_401), .p_cmd=lcd_ili9806e_zgd367_cmd_401,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_402), .p_cmd=lcd_ili9806e_zgd367_cmd_402,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_403), .p_cmd=lcd_ili9806e_zgd367_cmd_403,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_404), .p_cmd=lcd_ili9806e_zgd367_cmd_404,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_405), .p_cmd=lcd_ili9806e_zgd367_cmd_405,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_406), .p_cmd=lcd_ili9806e_zgd367_cmd_406,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_407), .p_cmd=lcd_ili9806e_zgd367_cmd_407,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_408), .p_cmd=lcd_ili9806e_zgd367_cmd_408,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_409), .p_cmd=lcd_ili9806e_zgd367_cmd_409,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_410), .p_cmd=lcd_ili9806e_zgd367_cmd_410,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_411), .p_cmd=lcd_ili9806e_zgd367_cmd_411,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_412), .p_cmd=lcd_ili9806e_zgd367_cmd_412,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_413), .p_cmd=lcd_ili9806e_zgd367_cmd_413,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_414), .p_cmd=lcd_ili9806e_zgd367_cmd_414,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_415), .p_cmd=lcd_ili9806e_zgd367_cmd_415,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_416), .p_cmd=lcd_ili9806e_zgd367_cmd_416,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_417), .p_cmd=lcd_ili9806e_zgd367_cmd_417,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_418), .p_cmd=lcd_ili9806e_zgd367_cmd_418,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_419), .p_cmd=lcd_ili9806e_zgd367_cmd_419,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_420), .p_cmd=lcd_ili9806e_zgd367_cmd_420,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_421), .p_cmd=lcd_ili9806e_zgd367_cmd_421,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_422), .p_cmd=lcd_ili9806e_zgd367_cmd_422,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_423), .p_cmd=lcd_ili9806e_zgd367_cmd_423,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_424), .p_cmd=lcd_ili9806e_zgd367_cmd_424,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_425), .p_cmd=lcd_ili9806e_zgd367_cmd_425,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_426), .p_cmd=lcd_ili9806e_zgd367_cmd_426,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_427), .p_cmd=lcd_ili9806e_zgd367_cmd_427,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_428), .p_cmd=lcd_ili9806e_zgd367_cmd_428,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_429), .p_cmd=lcd_ili9806e_zgd367_cmd_429,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_430), .p_cmd=lcd_ili9806e_zgd367_cmd_430,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_431), .p_cmd=lcd_ili9806e_zgd367_cmd_431,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_432), .p_cmd=lcd_ili9806e_zgd367_cmd_432,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_433), .p_cmd=lcd_ili9806e_zgd367_cmd_433,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_434), .p_cmd=lcd_ili9806e_zgd367_cmd_434,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_435), .p_cmd=lcd_ili9806e_zgd367_cmd_435,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_436), .p_cmd=lcd_ili9806e_zgd367_cmd_436,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_437), .p_cmd=lcd_ili9806e_zgd367_cmd_437,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_438), .p_cmd=lcd_ili9806e_zgd367_cmd_438,},
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_439), .p_cmd=lcd_ili9806e_zgd367_cmd_439,},
    { .type = DSI_CMD_DEALY,	.len = 120, },
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_ili9806e_zgd367_cmd_440), .p_cmd=lcd_ili9806e_zgd367_cmd_440,},
    { .type = DSI_CMD_DEALY,	.len = 20, },
    { .type = DSI_CMD_END,},

};


static sprdfb_dev_dsi_cmds lcd_ili9806e_zgd367_entry_sleep_cmds_init[] =
{
 
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x28},},
	{ .type = DSI_CMD_DEALY,	.len = 40, },
	{ .type = DSI_CMD_DCS,	.len = 1, .cmd={0x10},},
	{ .type = DSI_CMD_DEALY,	.len = 20, },
	{ .type = DSI_CMD_END,},
};

extern unsigned int lcd_debug_panel_on_cmds( sprdfb_dev_dsi_cmds**cmds, unsigned char** buf) ;

static int32_t ili9806e_mipi_init(struct panel_spec *self)
{
    sprdfb_dev_dsi_cmds *dbg_cmds = NULL;
    unsigned char* dbg_data= NULL ;
    unsigned int size = 0;

	sprdfb_dev_dsi_cmds *init = lcd_ili9806e_zgd367_cmds_init;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

    printk(KERN_DEBUG "[kernel]:ili9806e_zgd367_init\n");

	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);
	
    if( ( size = lcd_debug_panel_on_cmds(&dbg_cmds, &dbg_data) ) > 0 )
    {
        mipi_common_cmd(dbg_cmds);
        kfree(dbg_cmds);
        kfree(dbg_data);
        printk(KERN_INFO "[kernel]:>>>>>>>load sdcard config param success size = %d<<<<<<<<< \n",size);
    }
    else
    {
        mipi_common_cmd(init);
        printk("[kernel]:>>>>>>>>load local config param success<<<<<<<<<<<<< \n");
    }
    
    mipi_eotp_set(1,1);

    return 0;
}

static int32_t ili9806e_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	sprdfb_dev_dsi_cmds *init = lcd_ili9806e_zgd367_entry_sleep_cmds_init;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;


	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);
	
	mipi_common_cmd(init);
	
	mipi_eotp_set(1,1);
	printk(KERN_INFO "[kernel]:%s.done.is_sleep=%d.\n",__func__,is_sleep);
	
    return 0;
}
static uint32_t ili9806e_readid(struct panel_spec *self)
{
	/*Jessica TODO: need read id*/
	return 0x9806;
}

static uint32_t ili9806e_readpowermode(struct panel_spec *self)
{
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code_1;
	uint8_t read_data[1] = {0};
	int32_t read_rtn = 0;
	unsigned int tag = 0;

	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	pr_debug("lcd_ili9806e_mipi read power mode!\n");
	mipi_eotp_set(0,1);
	for(j = 0; j < 4; j++){
		rd_prepare = rd_prep_code_1;
		for(i = 0; i < ARRAY_SIZE(rd_prep_code_1); i++){
			tag = (rd_prepare->real_cmd_code.tag >> 24);
			if(tag & LCM_TAG_SEND){
				mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data, (rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			}else if(tag & LCM_TAG_SLEEP){
				msleep((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			}
			rd_prepare++;
		}
		read_rtn = mipi_force_read(0x0A, 1,(uint8_t *)read_data);
		printk("lcd_ili9806e_mipi read power mode 0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
		if((0x9c == read_data[0])  && (0 == read_rtn)){
			printk("lcd_ili9806ea_mipi read power mode success!\n");
			mipi_eotp_set(1,1);
			return 0x9c;
		}
	}

	printk("lcd_ili9806e_mipi read power mode fail!0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
	mipi_eotp_set(1,1);
	return 0x0;
}

static int32_t ili9806e_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

	mipi_set_lp_mode_t mipi_set_data_lp_mode = self->info.mipi->ops->mipi_set_data_lp_mode;
	mipi_set_hs_mode_t mipi_set_data_hs_mode = self->info.mipi->ops->mipi_set_data_hs_mode;
	mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;
	mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;
	uint16_t work_mode = self->info.mipi->work_mode;

	pr_debug("ili9806e_check_esd!\n");
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_lp_mode();
	}else{
		mipi_set_data_lp_mode();
	}
	power_mode = ili9806e_readpowermode(self);
	//power_mode = 0x0;
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_hs_mode();
	}else{
		mipi_set_data_hs_mode();
	}
	if(power_mode == 0x9c){
		pr_debug("ili9806e_check_esd OK!\n");
		return 1;
	}else{
		printk("ili9806e_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}



static u8 lcd_zgd367_ili9806e_driving_mode_cmd_1[]={0xB2,0x82,0xFE,0x09,0x03,0x00,0x50,0x11,0x42,0x1D};
static sprdfb_dev_dsi_cmds lcd_driving_mode_control[] =
{
    { .type = DSI_CMD_DCS,	.len = sizeof(lcd_zgd367_ili9806e_driving_mode_cmd_1), .p_cmd=lcd_zgd367_ili9806e_driving_mode_cmd_1,},
};
static void ili9806e_zgd367_driving_mode_set(struct panel_spec *self,bool enable)
{
    sprdfb_dev_dsi_cmds *init = lcd_driving_mode_control;
    mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
    mipi_common_cmd_t mipi_common_cmd = self->info.mipi->ops->mipi_common_cmd;
	
    if(enable)//dot mode 
    	lcd_driving_mode_control[0].p_cmd[1] = 0x82;
    else //column mode
	lcd_driving_mode_control[0].p_cmd[1] = 0x80;
	
    mipi_set_cmd_mode();

    mipi_common_cmd(init);
	
    printk(KERN_INFO "[kernel]:ili9806e_zgd367_driving_mode_set ok.dot enable=%d.\n",enable);
}
static struct panel_operations lcd_ili9806e_mipi_operations = {
	.panel_init = ili9806e_mipi_init,
	//.panel_dot_init = ili9806e_mipi_dot_init,
	.panel_readid = ili9806e_readid,
	.panel_esd_check = ili9806e_check_esd,
	.panel_enter_sleep = ili9806e_enter_sleep,
	.lcd_set_driving_mode = ili9806e_zgd367_driving_mode_set,
};

static struct timing_rgb lcd_ili9806e_mipi_timing =
{
    .hfp = 37,  /* unit: pixel */
    .hbp = 37,
    .hsync = 6,
    .vfp = 30, /*unit: line*/
    .vbp = 30,
    .vsync = 4,
};

static struct info_mipi lcd_ili9806e_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 2,
	.phy_feq = 380*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_ili9806e_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_ili9806e_zgd367_mipi_spec = {
	.width = 480,
	.height = 800,
	.fps	= 60,
	.reset_timing = {5,40,120},
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	.suspend_mode = SEND_SLEEP_CMD ,
	.info = {
		.mipi = &lcd_ili9806e_mipi_info
	},
	.ops = &lcd_ili9806e_mipi_operations,
};

struct panel_cfg lcd_ili9806e_zgd367_mipi = {
	/* this panel can only be main lcd */
    .dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x9806,
	.lcd_name = "lcd_ili9806e_zgd367_mipi",
    .lcd_voltage_max = 100,
    .lcd_voltage_min = 0,
	.panel = &lcd_ili9806e_zgd367_mipi_spec,
};

static int __init lcd_ili9806e_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_ili9806e_zgd367_mipi);
}

subsys_initcall(lcd_ili9806e_mipi_init);
