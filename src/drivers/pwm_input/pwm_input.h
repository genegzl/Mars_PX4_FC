/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>

#include <px4_platform_common/module.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/pwm_captures.h>

#if HRT_TIMER == PWMIN_TIMER
#error cannot share timer between HRT and PWMIN
#endif

#if !defined(GPIO_PWM_IN) || !defined(PWMIN_TIMER) || !defined(PWMIN_TIMER_CHANNEL)
#error PWMIN defines are needed in board_config.h for this board
#endif

/* Get the timer defines */
#define INPUT_TIMER PWMIN_TIMER
#include "timer_registers.h"
#define PWMIN_TIMER_BASE	TIMER_BASE
#define PWMIN_TIMER_CLOCK	TIMER_CLOCK
#define PWMIN_TIMER_POWER_REG	TIMER_CLOCK_POWER_REG
#define PWMIN_TIMER_POWER_BIT	TIMER_CLOCK_POWER_BIT
#define PWMIN_TIMER_VECTOR	TIMER_IRQ_REG

//add by cn for G2 capture
#define PWMIN_TIMER_BASE_G2	        TIMER_BASE_G2
#define PWMIN_TIMER_CLOCK_G2	    TIMER_CLOCK_G2
#define PWMIN_TIMER_POWER_REG_G2	TIMER_CLOCK_POWER_REG_G2
#define PWMIN_TIMER_POWER_BIT_G2	TIMER_CLOCK_POWER_BIT_G2
#define PWMIN_TIMER_VECTOR_G2	    TIMER_IRQ_REG_G2
//end by cn for G2 capture

/*
 * HRT clock must be at least 1MHz
 */
#if PWMIN_TIMER_CLOCK <= 1000000
# error PWMIN_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(PWMIN_TIMER_BASE + _reg))

#define rCR1		REG(STM32_GTIM_CR1_OFFSET)
#define rCR2		REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR		REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER		REG(STM32_GTIM_DIER_OFFSET)
#define rSR		    REG(STM32_GTIM_SR_OFFSET)
#define rEGR		REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1		REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2		REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER		REG(STM32_GTIM_CCER_OFFSET)
#define rCNT		REG(STM32_GTIM_CNT_OFFSET)
#define rPSC		REG(STM32_GTIM_PSC_OFFSET)
#define rARR		REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1		REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2		REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3		REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4		REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR		REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR		REG(STM32_GTIM_DMAR_OFFSET)

#define REG_G2(_reg)	(*(volatile uint32_t *)(PWMIN_TIMER_BASE_G2 + _reg))

#define rCR1_G2		REG_G2(STM32_GTIM_CR1_OFFSET)
#define rCR2_G2		REG_G2(STM32_GTIM_CR2_OFFSET)
#define rSMCR_G2		REG_G2(STM32_GTIM_SMCR_OFFSET)
#define rDIER_G2		REG_G2(STM32_GTIM_DIER_OFFSET)
#define rSR_G2		    REG_G2(STM32_GTIM_SR_OFFSET)
#define rEGR_G2		REG_G2(STM32_GTIM_EGR_OFFSET)
#define rCCMR1_G2		REG_G2(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2_G2		REG_G2(STM32_GTIM_CCMR2_OFFSET)
#define rCCER_G2		REG_G2(STM32_GTIM_CCER_OFFSET)
#define rCNT_G2		REG_G2(STM32_GTIM_CNT_OFFSET)
#define rPSC_G2		REG_G2(STM32_GTIM_PSC_OFFSET)
#define rARR_G2		REG_G2(STM32_GTIM_ARR_OFFSET)
#define rCCR1_G2		REG_G2(STM32_GTIM_CCR1_OFFSET)
#define rCCR2_G2		REG_G2(STM32_GTIM_CCR2_OFFSET)
#define rCCR3_G2		REG_G2(STM32_GTIM_CCR3_OFFSET)
#define rCCR4_G2		REG_G2(STM32_GTIM_CCR4_OFFSET)
#define rDCR_G2		REG_G2(STM32_GTIM_DCR_OFFSET)
#define rDMAR_G2		REG_G2(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if PWMIN_TIMER_CHANNEL == INPUT_CAP1

#define rCCR_PWMIN_A		rCCR1			    /* compare register for PWMIN */
#define DIER_PWMIN_A		GTIM_DIER_CC1IE 	/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_A		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
#define rCCR_PWMIN_B		rCCR2 			    /* compare register for PWMIN */
#define SR_INT_PWMIN_B		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
#define CCMR1_PWMIN		    ((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT))
#define CCMR2_PWMIN		    0
#define CCER_PWMIN		    (GTIM_CCER_CC2P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
#define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
#define SMCR_PWMIN_1		(0x05 << GTIM_SMCR_TS_SHIFT)//set TS: use TI1FP1 as CNT reset trigger
#define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)//set SMS: reset mode

#elif PWMIN_TIMER_CHANNEL == INPUT_CAP2

#define rCCR_PWMIN_A		rCCR2			    /* compare register for PWMIN */
#define DIER_PWMIN_A		GTIM_DIER_CC2IE  	/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_A		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
#define rCCR_PWMIN_B		rCCR1			    /* compare register for PWMIN */
#define DIER_PWMIN_B		GTIM_DIER_CC1IE		/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_B		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
#define CCMR1_PWMIN		    ((0x01 << GTIM_CCMR1_CC2S_SHIFT) | (0x02 << GTIM_CCMR1_CC1S_SHIFT))
#define CCMR2_PWMIN		    0
#define CCER_PWMIN		    (GTIM_CCER_CC1P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
#define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
#define SMCR_PWMIN_1		(0x06 << GTIM_SMCR_TS_SHIFT)
#define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)

#elif PWMIN_TIMER_CHANNEL == INPUT_CAP1_CAP3//cn, for dual pwn_input
//set value
//Filtered Timer Input 2 (TI2FP2)
#define SMCR_PWMIN_1		(0x05 << GTIM_SMCR_TS_SHIFT)
//Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and generates an update of the registers.
#define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#define DIER_PWMIN_A		(GTIM_DIER_CC1IE | GTIM_DIER_CC4IE | GTIM_DIER_UIE)//interrupt enable for CH1 and CH4 and overflow
#define CCMR1_PWMIN		    ((0x01 << GTIM_CCMR1_CC1S_SHIFT) | (0x02 << GTIM_CCMR1_CC2S_SHIFT))//IC1 and IC2 map to TI1
#define CCMR2_PWMIN		    ((0x02 << GTIM_CCMR2_CC3S_SHIFT) | (0x01 << GTIM_CCMR2_CC4S_SHIFT))//IC3 and IC4 map to TI4
//Enable all 4 channel, ch1 and ch4 are set to falling edge trigger
#define CCER_PWMIN		    (GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E | GTIM_CCER_CC1P | GTIM_CCER_CC4P)


#elif PWMIN_TIMER_CHANNEL == INPUT_TIM2_CH1_CH4_TIM4_CH2_CH3//cn, for dual pwn_input

//Filtered Timer Input 2 (TI2FP2)
#define SMCR_PWMIN_1		(0x05 << GTIM_SMCR_TS_SHIFT)
//Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and generates an update of the registers.
#define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#define DIER_PWMIN_A		(GTIM_DIER_CC1IE | GTIM_DIER_CC4IE | GTIM_DIER_UIE)//interrupt enable for cap1 and cap4 and overflow
#define CCMR1_PWMIN		    ((0x01 << GTIM_CCMR1_CC1S_SHIFT) | (0x02 << GTIM_CCMR1_CC2S_SHIFT))//IC1 and IC2 map to TI1
#define CCMR2_PWMIN		    ((0x02 << GTIM_CCMR2_CC3S_SHIFT) | (0x01 << GTIM_CCMR2_CC4S_SHIFT))//IC3 and IC4 map to TI4
#define CCER_PWMIN		    (GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E | GTIM_CCER_CC1P | GTIM_CCER_CC4P)
//CCMR1
//CC1S:
//01: CC1 channel is configured as input, IC1 is mapped on TI1.
//10: CC1 channel is configured as input, IC1 is mapped on TI2.
//CC2S:
//01: CC2 channel is configured as input, IC2 is mapped on TI2
//10: CC2 channel is configured as input, IC2 is mapped on TI1

//CCMR2
//CC3S:
//01: CC3 channel is configured as input, IC3 is mapped on TI3
//10: CC3 channel is configured as input, IC3 is mapped on TI4
//CC4S:
//01: CC4 channel is configured as input, IC3 is mapped on TI4
//10: CC4 channel is configured as input, IC3 is mapped on TI3

#define SMCR_PWMIN_1_G2		(0x05 << GTIM_SMCR_TS_SHIFT)
//Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and generates an update of the registers.
#define SMCR_PWMIN_2_G2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#define DIER_PWMIN_A_G2		(GTIM_DIER_CC2IE | GTIM_DIER_CC3IE | GTIM_DIER_UIE)//interrupt enable for CH2 and CH3 and overflow
#define CCMR1_PWMIN_G2		((0x02 << GTIM_CCMR1_CC1S_SHIFT) | (0x01 << GTIM_CCMR1_CC2S_SHIFT))//IC1 and IC2 map to TI2
#define CCMR2_PWMIN_G2      ((0x01 << GTIM_CCMR2_CC3S_SHIFT) | (0x02 << GTIM_CCMR2_CC4S_SHIFT))//IC3 and IC4 map to TI3
#define CCER_PWMIN_G2		(GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E | GTIM_CCER_CC2P | GTIM_CCER_CC3P)
//enable both and cap falling edge

//get value
#define rCCR_PWMIN_B		rCCR1			    /* compare register for PWMIN */
#define rCCR_PWMIN_A		rCCR2			    /* compare register for PWMIN */
//current none used
#define SR_INT_PWMIN_A		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
#define SR_INT_PWMIN_B		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
//judged
#define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)

#else
#error PWMIN_TIMER_CHANNEL definition is wrong.
#endif

class PWMIN : public ModuleBase<PWMIN>
{
public:
	void start();
	void publish(uint16_t status, uint32_t period, uint32_t pulse_width, uint8_t orb_select);
	void print_info(void);

	static int pwmin_tim_isr(int irq, void *context, void *arg);
    static int pwmin_multi_tim_isr(int irq, void *context, void *arg);
    static int pwmin_multi_tim_isr_G2(int irq, void *context, void *arg);

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);


private:
	void timer_init(void);
    void timer_init_G2(void);

	uint32_t _error_count {};
	uint32_t _pulses_captured {};
	uint32_t _last_period {};
	uint32_t _last_width {};

	bool _timer_started {};

	uint8_t ch1_state{0};
    uint8_t ch2_state{0};
    uint8_t ch3_state{0};
    uint8_t ch4_state{0};

	pwm_input_s _pwm {};
    pwm_captures_s _pwm_cap {};//cn

	uORB::PublicationData<pwm_input_s> _pwm_input_pub{ORB_ID(pwm_input)};

    uORB::PublicationData<pwm_captures_s> _pwm_captures_pub_0{ORB_ID(pwm_captures_0)};//cn
    uORB::PublicationData<pwm_captures_s> _pwm_captures_pub_1{ORB_ID(pwm_captures_1)};//cn

    uORB::PublicationData<pwm_captures_s> _pwm_captures_pub_2{ORB_ID(pwm_captures_2)};//cn
    uORB::PublicationData<pwm_captures_s> _pwm_captures_pub_3{ORB_ID(pwm_captures_3)};//cn
};
