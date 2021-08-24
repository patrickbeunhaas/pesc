/*
	Copyright 2016 - 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_PESC_H_
#define HW_PESC_H_

#define HW_NAME					"PESC"


// HW properties
#define HW_USE_LINE_TO_LINE
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
#define HW_HAS_SIN_COS_ENCODER

#define ENCODER_SIN_VOLTS				ADC_VOLTS(ADC_IND_EXT)
#define ENCODER_COS_VOLTS				ADC_VOLTS(ADC_IND_EXT2)

//#if !defined(HW60_IS_MK3) && !defined(HW60_IS_MK4) && !defined(HW60_IS_MK5)
//#define HW_HAS_PERMANENT_NRF
//#endif

// Macros
#define ENABLE_GATE()			palSetPad(GPIOC, 14)
#define DISABLE_GATE()			palClearPad(GPIOC, 14)
#define DCCAL_ON()
#define DCCAL_OFF()
#define IS_DRV_FAULT()			(!palReadPad(GPIOB, 12))

#define LED_GREEN_ON()			palSetPad(GPIOB, 2)
#define LED_GREEN_OFF()		palClearPad(GPIOB, 2)
#define LED_RED_ON()			palSetPad(GPIOB, 11)
#define LED_RED_OFF()			palClearPad(GPIOB, 11)


//#define CURRENT_FILTER_ON()		palSetPad(GPIOD, 2)
//#define CURRENT_FILTER_OFF()	palClearPad(GPIOD, 2)

//#define PHASE_FILTER_GPIO		GPIOC
//#define PHASE_FILTER_PIN		13
//#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
//#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)


//#if defined(HW60_IS_MK3) || defined(HW60_IS_MK4) || defined(HW60_IS_MK5)
// Shutdown pin
//#define HW_SHUTDOWN_GPIO		GPIOC
//#define HW_SHUTDOWN_PIN			5
//#define HW_SHUTDOWN_HOLD_ON()	palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
//#define HW_SHUTDOWN_HOLD_OFF()	palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
//#define HW_SAMPLE_SHUTDOWN()	hw_sample_shutdown_button()




/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):  IN8		ADC_IND_EXT2
 * 7  (2):	IN6		TEMP_IGBT_2
 * 8  (3):	IN3		TEMP_PCB
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	ADC_IND_EXT
 * 11 (3):	IN13	AN_IN
 * 12 (1):	IN9		V_GATE_DRIVER
 * 13 (2):	IN4		TEMP_IGBT_1
 * 14 (3)	IN3		UNUSED
 * 15 (1):	IN5		TEMP_IGBT_3
 * 16 (2):	VREFINT	ADC_IND_VREFINT
 * 17 (3):	VREFINT	UNUSED
 */

#define HW_ADC_CHANNELS					18
#define HW_ADC_INJ_CHANNELS				3
#define HW_ADC_NBR_CONV					6

// ADC Indexes
#define ADC_IND_SENS1					0
#define ADC_IND_SENS2					1
#define ADC_IND_SENS3					2
#define ADC_IND_CURR1					3
#define ADC_IND_CURR2					4
#define ADC_IND_CURR3					5
#define ADC_IND_VIN_SENS				11
#define ADC_IND_VOUT_GATE_DRV				12
#define ADC_IND_EXT					10
#define ADC_IND_EXT2					6
#define ADC_IND_TEMP_PCB				8
#define ADC_IND_TEMP_MOTOR				9
#define ADC_IND_TEMP_IGBT_1				13
#define ADC_IND_TEMP_IGBT_2				7
#define ADC_IND_TEMP_IGBT_3				15
#define ADC_IND_VREFINT				16

// When reading switch temperature, return the center IGBT temp
// because it will be the hotter one.
#define ADC_IND_TEMP_MOS				ADC_IND_TEMP_IGBT_2


// Component parameters (can be overridden)

#define VIN_GAIN		30.01
#define PHASE_GAIN		47.2

#ifndef V_REG
#define V_REG			3.3
#endif
#ifndef VIN_R1
#define VIN_R1			(PHASE_GAIN-1.0)
#endif
#ifndef VIN_R2
#define VIN_R2			1.0
#endif

#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN	(50.0*0.9) //was 30.0. Migh also be (50.0*1.1))
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES	0.0002
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * VIN_GAIN)

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO		GPIOC
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOB
#define HW_ADC_EXT2_PIN		0

// UART Peripheral
#define HW_UART_DEV			SD3
#define HW_UART_GPIO_AF		GPIO_AF_USART3
#define HW_UART_TX_PORT		GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT		GPIOD
#define HW_UART_RX_PIN			9          //Freeing B11 for fault LED. PD9 not available in LQFT64 package

//#if defined(HW60_IS_MK3) || defined(HW60_IS_MK4) || defined(HW60_IS_MK5)
// Permanent UART Peripheral (for NRF51)
//#define HW_UART_P_BAUD			115200
//#define HW_UART_P_DEV			SD4
//#define HW_UART_P_DEV_TX		SD5 // UART for TX, due to mistake below
//#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
//#define HW_UART_P_TX_PORT		GPIOC
//#define HW_UART_P_TX_PIN		12 // This is a mistake in the HW. We have to use a hack to use UART5.
//#define HW_UART_P_RX_PORT		GPIOC
//#define HW_UART_P_RX_PIN		11
//#endif

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV			ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO			GPIOB
#define HW_ICU_PIN			6

// I2C Peripheral
#define HW_I2C_DEV			I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT		GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT		GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM			TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

//#if !defined(HW60_IS_MK3) && !defined(HW60_IS_MK4) && !defined(HW60_IS_MK5)
// NRF pins
#define NRF_PORT_CSN			GPIOB
#define NRF_PIN_CSN			12
#define NRF_PORT_SCK			GPIOB
#define NRF_PIN_SCK			4
#define NRF_PORT_MOSI			GPIOB
#define NRF_PIN_MOSI			3
#define NRF_PORT_MISO			GPIOD
#define NRF_PIN_MISO			2
//#endif

// SPI pins
#define HW_SPI_DEV			SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS		GPIOA
#define HW_SPI_PIN_NSS			15
#define HW_SPI_PORT_SCK		GPIOC
#define HW_SPI_PIN_SCK			10
#define HW_SPI_PORT_MOSI		GPIOC
#define HW_SPI_PIN_MOSI		12
#define HW_SPI_PORT_MISO		GPIOC
#define HW_SPI_PIN_MISO		11

// MPU9250
//#if !defined(HW60_IS_MK4) && !defined(HW60_IS_MK5)
//#define MPU9X50_SDA_GPIO		GPIOB
//#define MPU9X50_SDA_PIN			2
//#define MPU9X50_SCL_GPIO		GPIOA
//#define MPU9X50_SCL_PIN			15
//#define IMU_FLIP
//#else
//#define BMI160_SDA_GPIO			GPIOB
//#define BMI160_SDA_PIN			2
//#define BMI160_SCL_GPIO			GPIOA
//#define BMI160_SCL_PIN			15
//#define IMU_FLIP
//#define IMU_ROT_180
//#endif

//#if defined(HW60_IS_MK3) || defined(HW60_IS_MK4) || defined(HW60_IS_MK5)
// NRF SWD
//#define NRF5x_SWDIO_GPIO		GPIOB
//#define NRF5x_SWDIO_PIN			12
//#define NRF5x_SWCLK_GPIO		GPIOA
//#define NRF5x_SWCLK_PIN			4
//#endif

// Measurement macros
#define ADC_V_L1				(ADC_Value[ADC_IND_SENS1]-2048)
#define ADC_V_L2				(ADC_Value[ADC_IND_SENS2]-2048)
#define ADC_V_L3				(ADC_Value[ADC_IND_SENS3]-2048)
//#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)
#define ADC_V_ZERO				0

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		150.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			true	// Run control loop in both v0 and v7 (requires phase shunts)
#endif

// Setting limits
#define HW_LIM_CURRENT			-300.0, 300.0
#define HW_LIM_CURRENT_IN		-300.0, 300.0
#define HW_LIM_CURRENT_ABS		0.0, 160.0
#define HW_LIM_VIN				0.0, 85.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

//disable temperature degrading
#define MC_DISABLE_TEMPDEGRAD

// Functions
//#if defined(HW60_IS_MK3) || defined(HW60_IS_MK4) || defined(HW60_IS_MK5)
//bool hw_sample_shutdown_button(void);
//#endif

#endif /* HW_PESC_H_ */
