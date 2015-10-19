/**
 * \file   demoCfg.h
 * 
 * \brief  Configuration values for the Demo Application  
 *
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef _DEMOCFG_H_
#define _DEMOCFG_H_

/*
** I2C Slave Addresses
*/
#define I2C_SLAVE_AIC31_ADDR                   (0x18u)
#define I2C_SLAVE_PMIC_ADDR                    (0x48u)
#define I2C_SLAVE_EXPANDER_ADDR                (0x21u)

/*
** Configurable parameters for the Audio Tone
*/
#define TONE_SLOT_SIZE                         (16u)
#define TONE_WORD_SIZE                         (16u)
#define TONE_SAMPLE_RATE                       (44100u)

/*
** Size of the Image to be displayed on the LCD in bytes
*/
#define SIZEOF_IMAGE                           (261152u)

/*
** Number of Icons in the images
*/
#define NUM_ICON_MENU	                       (13u)
#define NUM_ICON_DFLT                          (3u)
#define NUM_ICON_BANNER                        (NUM_ICON_DFLT)
#define NUM_ICON_CHOICE                        (NUM_ICON_DFLT)
#define NUM_ICON_INTRO                         (NUM_ICON_DFLT)
#define NUM_ICON_I2C                           (NUM_ICON_DFLT)
#define NUM_ICON_TMR                           (NUM_ICON_DFLT)
#define NUM_ICON_GPIO                          (3u)
#define NUM_ICON_UART                          (NUM_ICON_DFLT)
#define NUM_ICON_SPI                           (NUM_ICON_DFLT)
#define NUM_ICON_MMCSD                         (NUM_ICON_DFLT)
#define NUM_ICON_MCASP                         (NUM_ICON_DFLT)
#define NUM_ICON_ENET                          (NUM_ICON_DFLT)
#define NUM_ICON_RTC                           (4u)
#define NUM_ICON_ECAP                          (4u)
#define NUM_ICON_PM	                           (11u)
#define NUM_ICON_DVFS	                           (7u)
#define NUM_ICON_THANK                         (NUM_ICON_DFLT)
#define NUM_ICON_USB_MOUSE	                   (7u)

/*
** The indexes of Each Icon in the html page 
*/
#define CLICK_IDX_MENU                         (1u)
#define CLICK_IDX_INTRO                        (2u)
#define CLICK_IDX_CHOICE                       (3u)
#define CLICK_IDX_MCASP                        (4u)
//#define CLICK_IDX_SPI                          (5u)
#define CLICK_IDX_MMCSD                        (5u)
#define CLICK_IDX_UART	                       (6u)
#define CLICK_IDX_RTC                          (7u)
#define CLICK_IDX_TIMER                        (8u)
#define CLICK_IDX_ETHERNET                     (9u)
#define CLICK_IDX_ECAP		                   (10u)
#define CLICK_IDX_GPIO		                   (11u)
#define CLICK_IDX_I2C		                   (12u)
//#define CLICK_IDX_LED                          (13u)
#define CLICK_IDX_PM                           (13u)
#define CLICK_IDX_DVFS                           (14u)
//#define CLICK_IDX_USB_MOUSE                    (15u)


/*
** Menu page Icons placement macros
*/
#define LEFT_ALIGNMENT	        42
#define TOP_ALIGNMENT	        53
#define ICON_WIDTH              45	
#define ICON_HEIGHT	        45
#define ICON_COL_SPACE	        25
#define ICON_ROW_SPACE	        25

#define WAKE_ICON_WIDTH		54
#define WAKE_ICON_HEIGHT	54

#define X_ALIGN_COL_1	(LEFT_ALIGNMENT)
#define X_ALIGN_COL_2	(X_ALIGN_COL_1 + ICON_WIDTH + ICON_COL_SPACE)
#define X_ALIGN_COL_3	(X_ALIGN_COL_2 + ICON_WIDTH + ICON_COL_SPACE)
#define X_ALIGN_COL_4	(X_ALIGN_COL_3 + ICON_WIDTH + ICON_COL_SPACE)
#define X_ALIGN_COL_5	(X_ALIGN_COL_4 + ICON_WIDTH + ICON_COL_SPACE)
#define X_ALIGN_COL_6	(X_ALIGN_COL_5 + ICON_WIDTH + ICON_COL_SPACE)

#define Y_ALIGN_ROW_1	(TOP_ALIGNMENT)
#define Y_ALIGN_ROW_2	(Y_ALIGN_ROW_1 + ICON_HEIGHT + ICON_ROW_SPACE)
#define Y_ALIGN_ROW_3	(Y_ALIGN_ROW_2 + ICON_HEIGHT + ICON_ROW_SPACE)

#define PM_ALIGN_ROW_1	(TOP_ALIGNMENT + ICON_ROW_SPACE)
#define PM_ALIGN_ROW_2	(PM_ALIGN_ROW_1 + 4*ICON_ROW_SPACE)


/*
** Coordinates of the 'Next' icon displayed on the LCD.
*/
#define XMIN_NEXT                              (436u)
#define XMAX_NEXT                              (474u)
#define YMIN_NEXT                              (239u)
#define YMAX_NEXT                              (266u)

/*
** Coordinates of the 'Prev' icon displayed on the LCD.
*/
#define XMIN_PREV                              (6u)
#define XMAX_PREV                              (70u)
#define YMIN_PREV                              (238u)
#define YMAX_PREV                              (266u)


/*
** Coordinates of the 'eCap' icon displayed on the LCD.
*/
#define XMIN_ECAP                              (180u)
#define XMAX_ECAP                              (XMIN_ECAP + 60)
#define YMIN_ECAP                              (196u)
#define YMAX_ECAP                              (YMIN_ECAP + 42)


/*
** Coordinates of the 'Home' icon displayed on the LCD.
*/
#define XMIN_HOME                              (5u)
#define XMAX_HOME                              (XMIN_HOME + 48)
#define YMIN_HOME                              (5u)
#define YMAX_HOME                              (YMIN_HOME + 48)


/*
** Coordinates of the 'Set Time and Date' icon displayed in the RTC slide.
*/
#define XMIN_RTC_STD                           (127u)
#define XMAX_RTC_STD                           (XMIN_RTC_STD + 225)
#define YMIN_RTC_STD                           (196u)
#define YMAX_RTC_STD                           (YMIN_RTC_STD + 22)

/*
** Coordinates of the Menu Intro icon displayed in the Menu slide.
*/
#define XMIN_INTRO								(X_ALIGN_COL_1)
#define XMAX_INTRO								(X_ALIGN_COL_1 + ICON_WIDTH)
#define YMIN_INTRO								(Y_ALIGN_ROW_1)
#define YMAX_INTRO	                            (Y_ALIGN_ROW_1 + ICON_HEIGHT)

/*
** Coordinates of the Menu web demo icon displayed in the Menu slide.
*/
#define XMIN_WEB_DEMO							(X_ALIGN_COL_2)
#define XMAX_WEB_DEMO							(X_ALIGN_COL_2 + ICON_WIDTH)
#define YMIN_WEB_DEMO							(Y_ALIGN_ROW_1)
#define YMAX_WEB_DEMO							(Y_ALIGN_ROW_1 + ICON_HEIGHT)

/*
** Coordinates of the Menu McASP icon displayed in the Menu slide.
*/
#define XMIN_MCASP								(X_ALIGN_COL_3)
#define XMAX_MCASP								(X_ALIGN_COL_3 + ICON_WIDTH)
#define YMIN_MCASP								(Y_ALIGN_ROW_1)
#define YMAX_MCASP								(Y_ALIGN_ROW_1 + ICON_HEIGHT)

/*
** Coordinates of the Menu SPI icon displayed in the Menu slide.
*/
#define XMIN_SPI	370
#define XMAX_SPI	432
#define YMIN_SPI	28
#define YMAX_SPI	90

/*
** Coordinates of the Menu UART icon displayed in the Menu slide.
*/
#define XMIN_UART								(X_ALIGN_COL_5)
#define XMAX_UART								(X_ALIGN_COL_5 + ICON_WIDTH)
#define YMIN_UART								(Y_ALIGN_ROW_1)
#define YMAX_UART								(Y_ALIGN_ROW_1 + ICON_HEIGHT)


/*
** Coordinates of the Menu RTC icon displayed in the Menu slide.
*/
#define XMIN_RTC								(X_ALIGN_COL_6)
#define XMAX_RTC								(X_ALIGN_COL_6 + ICON_WIDTH)
#define YMIN_RTC								(Y_ALIGN_ROW_1)
#define YMAX_RTC								(Y_ALIGN_ROW_1 + ICON_HEIGHT)


/*
** Coordinates of the Menu Timer icon displayed in the Menu slide.
*/
#define XMIN_TIMER								(X_ALIGN_COL_1)
#define XMAX_TIMER								(X_ALIGN_COL_1 + ICON_WIDTH)
#define YMIN_TIMER								(Y_ALIGN_ROW_2)
#define YMAX_TIMER								(Y_ALIGN_ROW_2 + ICON_HEIGHT)
							   
/*
** Coordinates of the Menu Ethernet icon displayed in the Menu slide.
*/
#define	XMIN_ETHERNET							(X_ALIGN_COL_2)
#define	XMAX_ETHERNET							(X_ALIGN_COL_2 + ICON_WIDTH)
#define	YMIN_ETHERNET							(Y_ALIGN_ROW_2)
#define	YMAX_ETHERNET							(Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the Menu eCAP icon displayed in the Menu slide.
*/
#define	XMIN_ECAP_MENU							(X_ALIGN_COL_3)
#define	XMAX_ECAP_MENU							(X_ALIGN_COL_3 + ICON_WIDTH)
#define	YMIN_ECAP_MENU							(Y_ALIGN_ROW_2)
#define	YMAX_ECAP_MENU							(Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the Menu GPIO icon displayed in the Menu slide.
*/
#define	XMIN_GPIO_MENU							(X_ALIGN_COL_4)
#define	XMAX_GPIO_MENU							(X_ALIGN_COL_4 + ICON_WIDTH)
#define	YMIN_GPIO_MENU							(Y_ALIGN_ROW_2)
#define	YMAX_GPIO_MENU							(Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the Menu I2C icon displayed in the Menu slide.
*/
#define	XMIN_I2C_MENU							(X_ALIGN_COL_5)
#define	XMAX_I2C_MENU							(X_ALIGN_COL_5 + ICON_WIDTH)
#define	YMIN_I2C_MENU							(Y_ALIGN_ROW_2)
#define	YMAX_I2C_MENU							(Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the Menu PM icon displayed in the Menu slide.
*/
#define	XMIN_PM_MENU							(X_ALIGN_COL_6)
#define	XMAX_PM_MENU							(X_ALIGN_COL_6 + ICON_WIDTH)
#define	YMIN_PM_MENU							(Y_ALIGN_ROW_2)
#define	YMAX_PM_MENU							(Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the Menu PM icon displayed in the Menu slide.
*/
#define	XMIN_DVFS_MENU							(X_ALIGN_COL_1)
#define	XMAX_DVFS_MENU							(X_ALIGN_COL_1 + ICON_WIDTH)
#define	YMIN_DVFS_MENU							(Y_ALIGN_ROW_3)
#define	YMAX_DVFS_MENU							(Y_ALIGN_ROW_3 + ICON_HEIGHT)

/*
** Coordinates of the OPP50 icon displayed in the DVFS slide.
*/
#define	XMIN_DVFS_OPP50							(X_ALIGN_COL_2)
#define	XMAX_DVFS_OPP50							(X_ALIGN_COL_2 + ICON_WIDTH)
#define	YMIN_DVFS_OPP50							(Y_ALIGN_ROW_2)
#define	YMAX_DVFS_OPP50							(Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the OPP100 icon displayed in the DVFS slide.
*/
#define	XMIN_DVFS_OPP100						(X_ALIGN_COL_3)
#define	XMAX_DVFS_OPP100   					        (X_ALIGN_COL_3 + ICON_WIDTH)
#define	YMIN_DVFS_OPP100						(Y_ALIGN_ROW_2)
#define	YMAX_DVFS_OPP100						(Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the OPP120 icon displayed in the DVFS slide.
*/
#define XMIN_DVFS_OPP120                                                (X_ALIGN_COL_4)
#define XMAX_DVFS_OPP120                                                (X_ALIGN_COL_4 + ICON_WIDTH)
#define YMIN_DVFS_OPP120                                                (Y_ALIGN_ROW_2)
#define YMAX_DVFS_OPP120                                                (Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the SRTURBO icon displayed in the DVFS slide.
*/
#define XMIN_DVFS_SRTURBO                                               (X_ALIGN_COL_5)
#define XMAX_DVFS_SRTURBO                                               (X_ALIGN_COL_5 + ICON_WIDTH)
#define YMIN_DVFS_SRTURBO                                               (Y_ALIGN_ROW_2)
#define YMAX_DVFS_SRTURBO                                               (Y_ALIGN_ROW_2 + ICON_HEIGHT)

/*
** Coordinates of the MMCSD icon displayed in the Menu slide.
*/
#define	XMIN_MMCSD								(X_ALIGN_COL_4)
#define	XMAX_MMCSD								(X_ALIGN_COL_4 + ICON_WIDTH)
#define	YMIN_MMCSD								(Y_ALIGN_ROW_1)
#define	YMAX_MMCSD								(Y_ALIGN_ROW_1 + ICON_HEIGHT)

/*
** Coordinates of the PM DS0 icon displayed in the PM slide.
*/
#define XMIN_PM_DS2_DEMO						(X_ALIGN_COL_3)
#define XMAX_PM_DS2_DEMO						(X_ALIGN_COL_3 + ICON_WIDTH)
#define YMIN_PM_DS2_DEMO						(PM_ALIGN_ROW_1)
#define YMAX_PM_DS2_DEMO						(PM_ALIGN_ROW_1 + ICON_HEIGHT)

/*
** Coordinates of the PM DS0 icon displayed in the PM slide.
*/
#define XMIN_PM_DS1_DEMO						(X_ALIGN_COL_4)
#define XMAX_PM_DS1_DEMO						(X_ALIGN_COL_4 + ICON_WIDTH)
#define YMIN_PM_DS1_DEMO						(PM_ALIGN_ROW_1)
#define YMAX_PM_DS1_DEMO						(PM_ALIGN_ROW_1 + ICON_HEIGHT)

/*
** Coordinates of the PM DS1_DEV icon displayed in the PM slide.
*/
#define XMIN_PM_DS1DEV_DEMO						(X_ALIGN_COL_2)
#define XMAX_PM_DS1DEV_DEMO						(X_ALIGN_COL_2 + ICON_WIDTH)
#define YMIN_PM_DS1DEV_DEMO						(PM_ALIGN_ROW_1)
#define YMAX_PM_DS1DEV_DEMO						(PM_ALIGN_ROW_1 + ICON_HEIGHT)

/*
** Coordinates of the PM DS0 icon displayed in the PM slide.
*/
#define XMIN_PM_DS0_DEMO						(X_ALIGN_COL_3 + X_ALIGN_COL_4)/2
#define XMAX_PM_DS0_DEMO						((X_ALIGN_COL_3 + X_ALIGN_COL_4)/2 + ICON_WIDTH)
#define YMIN_PM_DS0_DEMO						(PM_ALIGN_ROW_1)
#define YMAX_PM_DS0_DEMO						(PM_ALIGN_ROW_1 + ICON_HEIGHT)

/*
** Coordinates of the wake TSC icon displayed in the PM slide.
*/
#define XMIN_PM_WAKE_TSC						(X_ALIGN_COL_2)
#define XMAX_PM_WAKE_TSC						(X_ALIGN_COL_2 + WAKE_ICON_WIDTH)
#define YMIN_PM_WAKE_TSC						(PM_ALIGN_ROW_2)
#define YMAX_PM_WAKE_TSC						(PM_ALIGN_ROW_2 + WAKE_ICON_HEIGHT)

/*
** Coordinates of the wake Timer icon displayed in the PM slide.
*/
#define XMIN_PM_WAKE_TMR						(X_ALIGN_COL_3)
#define XMAX_PM_WAKE_TMR						(X_ALIGN_COL_3 + WAKE_ICON_WIDTH)
#define YMIN_PM_WAKE_TMR						(PM_ALIGN_ROW_2)
#define YMAX_PM_WAKE_TMR						(PM_ALIGN_ROW_2 + WAKE_ICON_HEIGHT)

/*
** Coordinates of the wake Uart icon displayed in the PM slide.
*/
#define XMIN_PM_WAKE_UART						(X_ALIGN_COL_4)
#define XMAX_PM_WAKE_UART						(X_ALIGN_COL_4 + WAKE_ICON_WIDTH)
#define YMIN_PM_WAKE_UART						(PM_ALIGN_ROW_2)
#define YMAX_PM_WAKE_UART						(PM_ALIGN_ROW_2 + WAKE_ICON_HEIGHT)

/*
** Coordinates of the wake GPIO icon displayed in the PM slide.
*/
#define XMIN_PM_WAKE_GPIO						(X_ALIGN_COL_5)
#define XMAX_PM_WAKE_GPIO						(X_ALIGN_COL_5 + WAKE_ICON_WIDTH)
#define YMIN_PM_WAKE_GPIO						(PM_ALIGN_ROW_2)
#define YMAX_PM_WAKE_GPIO						(PM_ALIGN_ROW_2 + WAKE_ICON_HEIGHT)


/*
** Coordinates of the Mouse left button in the USB HID slide.
*/
#define XMIN_MOUSE_LEFT	54
#define XMAX_MOUSE_LEFT	175
#define YMIN_MOUSE_LEFT	235
#define YMAX_MOUSE_LEFT	272

/*
** Coordinates of the Mouse middle button in the USB HID slide.
*/
#define XMIN_MOUSE_MIDDLE	177
#define XMAX_MOUSE_MIDDLE	297
#define YMIN_MOUSE_MIDDLE	235
#define YMAX_MOUSE_MIDDLE	272

/*
** Coordinates of the Mouse right button in the USB HID slide.
*/
#define XMIN_MOUSE_RIGHT	300
#define XMAX_MOUSE_RIGHT	422
#define YMIN_MOUSE_RIGHT	235
#define YMAX_MOUSE_RIGHT	272

/*
** Coordinates of the Touch pad in the USB HID slide.
*/
#define	XMIN_TOUCH_PAD	54
#define	XMAX_TOUCH_PAD	423
#define	YMIN_TOUCH_PAD	31
#define	YMAX_TOUCH_PAD	229


#endif
