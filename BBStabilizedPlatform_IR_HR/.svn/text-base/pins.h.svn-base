//*****************************************************************************
//
// pins.h - Defines the board-level pin connections.
//
// Copyright (c) 2008-2009 Luminary Micro, Inc.  All rights reserved.
// Software License Agreement
// 
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
// 
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  You may not combine
// this software with "viral" open-source software in order to form a larger
// program.  Any use in violation of the foregoing restrictions may subject
// the user to criminal sanctions under applicable laws, as well as to civil
// liability for the breach of the terms and conditions of this license.
// 
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 4652 of the RDK-BDC Firmware Package.
//
//******************************************************

#ifndef __PINS_H__
#define __PINS_H__

//******************************************************
//
// Defines for the board connections to the Stellaris microcontroller.
//
//******************************************************

/*****************************************************/
/*		For LM4F232H5QD						   		  */
/*****************************************************/
#define QEI_PITCH_INDEX_PORT          	GPIO_PORTD_BASE
#define QEI_PITCH_INDEX_PIN           	GPIO_PIN_3
#define QEI_PITCH_INDEX_INT           	INT_GPIOD

#define QEI_PITCH_PHA_PORT            	GPIO_PORTD_BASE
#define QEI_PITCH_PHA_PIN             	GPIO_PIN_6
#define QEI_PITCH_PHA_INT          	 	INT_GPIOD

#define QEI_PITCH_PHB_PORT           	GPIO_PORTD_BASE
#define QEI_PITCH_PHB_PIN             	GPIO_PIN_7

//*****************************************************

#define QEI_ROLL_INDEX_PORT          	GPIO_PORTC_BASE
#define QEI_ROLL_INDEX_PIN           	GPIO_PIN_4
#define QEI_ROLL_INDEX_INT           	INT_GPIOC

#define QEI_ROLL_PHA_PORT            	GPIO_PORTC_BASE
#define QEI_ROLL_PHA_PIN             	GPIO_PIN_5
#define QEI_ROLL_PHA_INT             	INT_GPIOC

#define QEI_ROLL_PHB_PORT            	GPIO_PORTC_BASE
#define QEI_ROLL_PHB_PIN             	GPIO_PIN_6

//*****************************************************


#define BMA180_INTERRUPT_PORT 'A'
#define BMA180_INTERRUPT_PIN   3  

#define ITG3200_INTERRUPT_PORT 'A'
#define ITG3200_INTERRUPT_PIN   4 

#define MAG3110_INTERRUPT_PORT 'A'
#define MAG3110_INTERRUPT_PIN   5 

#define A3906_OC1_INTERRUPT_PORT 'E'
#define A3906_OC1_INTERRUPT_PIN   0 

#define A3906_OC2_INTERRUPT_PORT 'E'
#define A3906_OC2_INTERRUPT_PIN   1 

#define ENCODER_PITCH_RESET_PORT 'D'
#define ENCODER_PITCH_RESET_PIN 6

#define ENCODER_ROLL_RESET_PORT 'D'
#define ENCODER_ROLL_RESET_PIN 7

#define ENCODER_CHANNEL_PITCH 			ADC_CTL_CH1
#define ENCODER_CHANNEL_ROLL  			ADC_CTL_CH0
#define PITCH_FLOW_ENCODER_CHANNEL2 	ADC_CTL_CH2
#define ROLL_FLOW_ENCODER_CHANNEL3 		ADC_CTL_CH3

//*****************************************************


#define SERVO_PORT              GPIO_PORTA_BASE
#define SERVO_PIN               GPIO_PIN_0
#define SERVO_INT               INT_GPIOA

#define LED_GREEN_PORT          GPIO_PORTA_BASE
#define LED_GREEN_PIN           GPIO_PIN_2
#define LED_GREEN_ON            LED_GREEN_PIN
#define LED_GREEN_OFF           0

#define CAN_RX_PORT             GPIO_PORTA_BASE
#define CAN_RX_PIN              GPIO_PIN_4

#define CAN_TX_PORT             GPIO_PORTA_BASE
#define CAN_TX_PIN              GPIO_PIN_5

#define HBRIDGE_PWMA_PORT       GPIO_PORTA_BASE
#define HBRIDGE_PWMA_PIN        GPIO_PIN_6

#define HBRIDGE_CTRLA_PORT      GPIO_PORTA_BASE
#define HBRIDGE_CTRLA_PIN       GPIO_PIN_7

#define HBRIDGE_PWMB_PORT       GPIO_PORTB_BASE
#define HBRIDGE_PWMB_PIN        GPIO_PIN_6

#define HBRIDGE_CTRLB_PORT      GPIO_PORTB_BASE
#define HBRIDGE_CTRLB_PIN       GPIO_PIN_7

#define I2C_SCL_PORT            GPIO_PORTB_BASE
#define I2C_SCL_PIN             GPIO_PIN_2

#define I2C_SDA_PORT            GPIO_PORTB_BASE
#define I2C_SDA_PIN             GPIO_PIN_3

#define CAN_SENSE_PORT          GPIO_PORTB_BASE
#define CAN_SENSE_PIN           GPIO_PIN_4

#define FAN_PORT                GPIO_PORTB_BASE
#define FAN_PIN                 GPIO_PIN_5
#define FAN_ON                  FAN_PIN
#define FAN_OFF                 0

#define LIMIT_REV_PORT          GPIO_PORTB_BASE
#define LIMIT_REV_PIN           GPIO_PIN_6
#define LIMIT_REV_OK            0
#define LIMIT_REV_ERR           LIMIT_REV_PIN

#define LIMIT_FWD_PORT          GPIO_PORTB_BASE
#define LIMIT_FWD_PIN           GPIO_PIN_7
#define LIMIT_FWD_OK            0
#define LIMIT_FWD_ERR           LIMIT_FWD_PIN

#define LED_RED_PORT            GPIO_PORTC_BASE
#define LED_RED_PIN             GPIO_PIN_7
#define LED_RED_ON              LED_RED_PIN
#define LED_RED_OFF             0


#define BRAKECOAST_PORT         GPIO_PORTD_BASE
#define BRAKECOAST_PIN          GPIO_PIN_2
#define BRAKECOAST_COAST        BRAKECOAST_PIN
#define BRAKECOAST_BRAKE        0

#define ADC_VBOOTB_PORT         GPIO_PORTD_BASE
#define ADC_VBOOTB_PIN          GPIO_PIN_3
#define ADC_VBOOTB_CH           ADC_CTL_CH4

#define ADC_VBOOTA_PORT         GPIO_PORTE_BASE
#define ADC_VBOOTA_PIN          GPIO_PIN_0
#define ADC_VBOOTA_CH           ADC_CTL_CH3

#define ADC_POSITION_PORT       GPIO_PORTE_BASE
#define ADC_POSITION_PIN        GPIO_PIN_1
#define ADC_POSITION_CH         ADC_CTL_CH2

#define ADC_CURRENT_PORT        GPIO_PORTE_BASE
#define ADC_CURRENT_PIN         GPIO_PIN_2
#define ADC_CURRENT_CH          ADC_CTL_CH1

#define ADC_VBUS_PORT           GPIO_PORTE_BASE
#define ADC_VBUS_PIN            GPIO_PIN_3
#define ADC_VBUS_CH             ADC_CTL_CH0

#define BUTTON_PORT             GPIO_PORTE_BASE
#define BUTTON_PIN              GPIO_PIN_4
#define BUTTON_DOWN             0
#define BUTTON_UP               BUTTON_PIN

#endif // __PINS_H__
