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

#include "pwm_input.h"
#include <math.h>

//new include
#include <px4_log.h>

int
PWMIN::task_spawn(int argc, char *argv[])
{
    PX4_INFO("pwm input task_spawn **********************");
	auto *pwmin = new PWMIN();

	if (!pwmin) {
		PX4_ERR("driver allocation failed");
        PX4_INFO("pwm input error **********************");
		return PX4_ERROR;
	}

	_object.store(pwmin);
	_task_id = task_id_is_work_queue;

	pwmin->start();
    PX4_INFO("pwm input start ok **********************");

	return PX4_OK;
}

void
PWMIN::start()
{
	// NOTE: must first publish here, first publication cannot be in interrupt context
	_pwm_input_pub.update();
    _pwm_captures_pub_0.update();
    _pwm_captures_pub_1.update();

    _pwm_captures_pub_2.update();
    _pwm_captures_pub_3.update();

	// Initialize the timer isr for measuring pulse widths. Publishing is done inside the isr.
	timer_init();
    #if PWMIN_TIMER_CHANNEL == INPUT_TIM2_CH1_CH4_TIM4_CH2_CH3
    timer_init_G2();//for another pwm input group, using main output 5 and 6
    #endif
}


void PWMIN::timer_init(void)
{
	/* run with interrupts disabled in case the timer is already
	 * setup. We don't want it firing while we are doing the setup */
	irqstate_t flags = px4_enter_critical_section();

    /* Clear no bits, set timer enable bit.*/
    modifyreg32(PWMIN_TIMER_POWER_REG, 0, PWMIN_TIMER_POWER_BIT);

//multi-channel capture
#if PWMIN_TIMER_CHANNEL == INPUT_CAP1_CAP3 || \
    PWMIN_TIMER_CHANNEL == INPUT_CAP1_CAP2_CAP3 || \
    PWMIN_TIMER_CHANNEL == INPUT_TIM2_CH1_CH4_TIM4_CH2_CH3
    /* configure input pin */
    px4_arch_configgpio(GPIO_PWM_IN1);//cn
    px4_arch_configgpio(GPIO_PWM_IN2);//cn
    px4_arch_configgpio(GPIO_PWM_IN3);//cn

    /* claim our interrupt vector */
	irq_attach(PWMIN_TIMER_VECTOR, PWMIN::pwmin_multi_tim_isr, NULL);

    /* disable and configure the timer */
    rCR1 = 0;//Edge-aligned mode, upcounter
    rCR2 = 0;
    rSMCR = 0;
    rDIER = DIER_PWMIN_A;
    rCCER = 0;		/* unlock CCMR* registers */
    rCCMR1 = CCMR1_PWMIN;
    rCCMR2 = CCMR2_PWMIN;
    rCCER = CCER_PWMIN;
    rDCR = 0;
//single channel capture
#else
    /* configure input pin */
	px4_arch_configgpio(GPIO_PWM_IN);

    /* claim our interrupt vector */
    irq_attach(PWMIN_TIMER_VECTOR, PWMIN::pwmin_tim_isr, NULL);

    /* disable and configure the timer */
	rCR1 = 0;//Edge-aligned mode, upcounter
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_PWMIN_A;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PWMIN;
	rCCMR2 = CCMR2_PWMIN;
	rSMCR = SMCR_PWMIN_1;	/* Set up mode */
	rSMCR = SMCR_PWMIN_2;	/* Enable slave mode controller */
	rCCER = CCER_PWMIN;
	rDCR = 0;
#endif

	/* for simplicity scale by the clock in MHz. This gives us
	 * readings in microseconds which is typically what is needed
	 * for a PWM input driver */
    //uint32_t prescaler = PWMIN_TIMER_CLOCK / 1000000UL;

    //PWMIN_TIMER_CLOCK = 108000000
	//default is 1MHz, changed to 9MHz by cn, prescaler should be a integer
	uint32_t prescaler = PWMIN_TIMER_CLOCK / 9000000UL;
	PX4_INFO("PWMIN_TIMER_CLOCK = %ld, prescaler = %d, CAP FREQ = %ld",
             PWMIN_TIMER_CLOCK, prescaler, PWMIN_TIMER_CLOCK / prescaler);

	/*
	 * define the clock speed. We want the highest possible clock
	 * speed that avoids overflows.
	 */
	rPSC = prescaler - 1;

	/* run the full span of the counter. All timers can handle
	 * uint16 */
	rARR = UINT16_MAX; //upcounter to this value, and overflow to zero

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	px4_leave_critical_section(flags);

	/* enable interrupts */
	up_enable_irq(PWMIN_TIMER_VECTOR);
}

void PWMIN::timer_init_G2(void)
{
    /* run with interrupts disabled in case the timer is already
     * setup. We don't want it firing while we are doing the setup */
    irqstate_t flags = px4_enter_critical_section();

    /* Clear no bits, set timer enable bit.*/
    modifyreg32(PWMIN_TIMER_POWER_REG_G2, 0, PWMIN_TIMER_POWER_BIT_G2);

    /* configure input pin */
    px4_arch_configgpio(GPIO_PWM_IN1_G2);//cn
    px4_arch_configgpio(GPIO_PWM_IN2_G2);//cn

    /* claim our interrupt vector */
    irq_attach(PWMIN_TIMER_VECTOR_G2, PWMIN::pwmin_multi_tim_isr_G2, NULL);

    /* disable and configure the timer */
    rCR1_G2 = 0;//Edge-aligned mode, upcounter
    rCR2_G2 = 0;
    rSMCR_G2 = 0;
    rDIER_G2 = DIER_PWMIN_A_G2;
    rCCER_G2 = 0;		/* unlock CCMR* registers */
    rCCMR1_G2 = CCMR1_PWMIN_G2;
    rCCMR2_G2 = CCMR2_PWMIN_G2;
    rCCER_G2 = CCER_PWMIN_G2;
    rDCR_G2 = 0;


    /* for simplicity scale by the clock in MHz. This gives us
     * readings in microseconds which is typically what is needed
     * for a PWM input driver */
    //uint32_t prescaler = PWMIN_TIMER_CLOCK / 1000000UL;

    //PWMIN_TIMER_CLOCK = 108000000
    //default is 1MHz, changed to 9MHz by cn, prescaler should be a integer
    uint32_t prescaler = PWMIN_TIMER_CLOCK_G2 / 9000000UL;
    PX4_INFO("PWMIN_TIMER_CLOCK_G2 = %ld, prescaler = %d, CAP FREQ = %ld",
             PWMIN_TIMER_CLOCK_G2, prescaler, PWMIN_TIMER_CLOCK_G2 / prescaler);

    /*
     * define the clock speed. We want the highest possible clock
     * speed that avoids overflows.
     */
    rPSC_G2 = prescaler - 1;

    /* run the full span of the counter. All timers can handle
     * uint16 */
    rARR_G2 = UINT16_MAX; //upcounter to this value, and overflow to zero

    /* generate an update event; reloads the counter, all registers */
    rEGR_G2 = GTIM_EGR_UG;

    /* enable the timer */
    rCR1_G2 = GTIM_CR1_CEN;

    px4_leave_critical_section(flags);

    /* enable interrupts */
    up_enable_irq(PWMIN_TIMER_VECTOR_G2);
}

int PWMIN::pwmin_tim_isr(int irq, void *context, void *arg)
{
    uint16_t status = rSR;
    uint32_t period = rCCR_PWMIN_A;
    uint32_t pulse_width = rCCR_PWMIN_B;

    /* ack the interrupts we just read */
    rSR = 0;
    auto *obj = get_instance();
    if (obj != nullptr)
    {
        obj->publish(status, period, pulse_width, 255);
    }
    return PX4_OK;
}

int PWMIN::pwmin_multi_tim_isr(int irq, void *context, void *arg)
{
	uint16_t status = rSR;
	int32_t period_0{0}, period_1{0}, pulse_width_0{0}, pulse_width_1{0};
    static int32_t last_falling_time_0{0}, last_falling_time_1{0};
    static uint32_t overflow_flag_0 = 0,  overflow_flag_1 = 0;
    //irqstate_t flags = px4_enter_critical_section();//cn

	/* ack the interrupts we just read */
    //rSR = 0;

    if (status & GTIM_SR_UIF)
    {
        overflow_flag_0++;
        overflow_flag_1++;
        rSR &= ~(uint32_t)(GTIM_SR_UIF);//clear timer overflow flag
    }
    if(overflow_flag_0 > 100 || overflow_flag_1 > 100)
    {
        overflow_flag_0 = overflow_flag_1 = 0;
    }

	if(status & GTIM_SR_CC1IF) //ch1 interrupted
    {
        int32_t falling_time = rCCR1;
        int32_t rising_time = rCCR2;
        if(falling_time > last_falling_time_0)
            period_0 = UINT16_MAX * overflow_flag_0 + (falling_time - last_falling_time_0);
        else
            period_0 = UINT16_MAX * overflow_flag_0 - (last_falling_time_0 - falling_time);

        if (falling_time < rising_time && overflow_flag_0 > 0)//overflow between rising and falling
            overflow_flag_0 --;
        if(rising_time > last_falling_time_0)
            pulse_width_0 = UINT16_MAX * overflow_flag_0 + (rising_time - last_falling_time_0);
        else
            pulse_width_0 = UINT16_MAX * overflow_flag_0 - (last_falling_time_0 - rising_time);

        last_falling_time_0 = falling_time;
        overflow_flag_0 = 0;
        if (period_0 < 0) period_0 += UINT16_MAX;
        if (pulse_width_0 < 0) pulse_width_0 += UINT16_MAX;

        auto *obj = get_instance();
        if (obj != nullptr)
        {
            obj->publish(status, period_0, pulse_width_0, 0);
        }
        rSR &= ~(uint32_t)(GTIM_SR_CC1IF | GTIM_SR_CC2IF);//clear interrupt flag
        rSR &= ~(uint32_t)(GTIM_SR_CC1OF | GTIM_SR_CC2OF);//clear over capture flag
    }
    if(status & GTIM_SR_CC4IF) //ch4 interrupted
    {
        int32_t falling_time = rCCR4;
        int32_t rising_time = rCCR3;
        if(falling_time > last_falling_time_1)
            period_1 = UINT16_MAX * overflow_flag_1 + (falling_time - last_falling_time_1);
        else
            period_1 = UINT16_MAX * overflow_flag_1 - (last_falling_time_1 - falling_time);

        if (falling_time < rising_time && overflow_flag_1 > 0)//overflow between rising and falling
            overflow_flag_1 --;
        if(rising_time > last_falling_time_1)
            pulse_width_1 = UINT16_MAX * overflow_flag_1 + (rising_time - last_falling_time_1);
        else
            pulse_width_1 = UINT16_MAX * overflow_flag_1 - (last_falling_time_1 - rising_time);

        last_falling_time_1 = falling_time;
        overflow_flag_1 = 0;
        if (period_1 < 0) period_1 += UINT16_MAX;
        if (pulse_width_1 < 0) pulse_width_1 += UINT16_MAX;

        auto *obj = get_instance();
        if (obj != nullptr)
        {
            obj->publish(status, period_1, pulse_width_1, 1);
        }
        rSR &= ~(uint32_t)(GTIM_SR_CC3IF | GTIM_SR_CC4IF);//clear interrupt flag
        rSR &= ~(uint32_t)(GTIM_SR_CC3OF | GTIM_SR_CC4OF);//clear over capture flag
    }

    //px4_leave_critical_section(flags);//cn
	return PX4_OK;
}

int PWMIN::pwmin_multi_tim_isr_G2(int irq, void *context, void *arg)
{
    uint16_t status = rSR_G2;
    int32_t period_0{0}, period_1{0}, pulse_width_0{0}, pulse_width_1{0};
    static int32_t last_falling_time_0{0}, last_falling_time_1{0};
    static uint32_t overflow_flag_0 = 0,  overflow_flag_1 = 0;
    //irqstate_t flags = px4_enter_critical_section();//cn

    /* ack the interrupts we just read */
    //rSR = 0;

    if (status & GTIM_SR_UIF)
    {
        overflow_flag_0++;
        overflow_flag_1++;
        rSR_G2 &= ~(uint32_t)(GTIM_SR_UIF);//clear timer overflow flag
    }
    if(overflow_flag_0 > 100 || overflow_flag_1 > 100)
    {
        overflow_flag_0 = overflow_flag_1 = 0;
    }

    if(status & GTIM_SR_CC2IF) //ch2 interrupted
    {
        int32_t falling_time = rCCR2_G2;
        int32_t rising_time = rCCR1_G2;
        if(falling_time > last_falling_time_0)
            period_0 = UINT16_MAX * overflow_flag_0 + (falling_time - last_falling_time_0);
        else
            period_0 = UINT16_MAX * overflow_flag_0 - (last_falling_time_0 - falling_time);

        if (falling_time < rising_time && overflow_flag_0 > 0)//overflow between rising and falling
            overflow_flag_0 --;
        if(rising_time > last_falling_time_0)
            pulse_width_0 = UINT16_MAX * overflow_flag_0 + (rising_time - last_falling_time_0);
        else
            pulse_width_0 = UINT16_MAX * overflow_flag_0 - (last_falling_time_0 - rising_time);

        last_falling_time_0 = falling_time;
        overflow_flag_0 = 0;
        if (period_0 < 0) period_0 += UINT16_MAX;
        if (pulse_width_0 < 0) pulse_width_0 += UINT16_MAX;

        auto *obj = get_instance();
        if (obj != nullptr)
        {
            obj->publish(status, period_0, pulse_width_0, 2);
        }
        rSR_G2 &= ~(uint32_t)(GTIM_SR_CC1IF | GTIM_SR_CC2IF);//clear interrupt flag
        rSR_G2 &= ~(uint32_t)(GTIM_SR_CC1OF | GTIM_SR_CC2OF);//clear over capture flag
    }
    if(status & GTIM_SR_CC3IF) //ch3 interrupted
    {
        int32_t falling_time = rCCR3_G2;
        int32_t rising_time = rCCR4_G2;
        if(falling_time > last_falling_time_1)
            period_1 = UINT16_MAX * overflow_flag_1 + (falling_time - last_falling_time_1);
        else
            period_1 = UINT16_MAX * overflow_flag_1 - (last_falling_time_1 - falling_time);

        if (falling_time < rising_time && overflow_flag_1 > 0)//overflow between rising and falling
            overflow_flag_1 --;
        if(rising_time > last_falling_time_1)
            pulse_width_1 = UINT16_MAX * overflow_flag_1 + (rising_time - last_falling_time_1);
        else
            pulse_width_1 = UINT16_MAX * overflow_flag_1 - (last_falling_time_1 - rising_time);

        last_falling_time_1 = falling_time;
        overflow_flag_1 = 0;
        if (period_1 < 0) period_1 += UINT16_MAX;
        if (pulse_width_1 < 0) pulse_width_1 += UINT16_MAX;

        auto *obj = get_instance();
        if (obj != nullptr)
        {
            obj->publish(status, period_1, pulse_width_1, 3);
        }
        rSR_G2 &= ~(uint32_t)(GTIM_SR_CC3OF | GTIM_SR_CC4OF);//clear interrupt flag
        rSR_G2 &= ~(uint32_t)(GTIM_SR_CC3OF | GTIM_SR_CC4OF);//clear over capture flag
    }
    //px4_leave_critical_section(flags);//cn
    return PX4_OK;
}

void PWMIN::publish(uint16_t status, uint32_t period, uint32_t pulse_width, uint8_t orb_select)
{
	double freq_fs = 9000000;
	// if we missed an edge, we have to give up
	if (status & SR_OVF_PWMIN)
	{
		_error_count++;
		return;
	}

	//begin cn
  	_pwm_cap.timestamp = hrt_absolute_time();
	double freq = (double)(freq_fs / (double)period * 5);
	if (freq < 0 || freq > 10000) return;
        _pwm_cap.period = freq;
        _pwm_cap.pulse_width = pulse_width;
	if(orb_select == 0)
    {
        _pwm_captures_pub_0.publish(_pwm_cap);
    }
	else if(orb_select == 1)
    {
        _pwm_captures_pub_1.publish(_pwm_cap);
    }
    else if(orb_select == 2)
    {
        _pwm_captures_pub_2.publish(_pwm_cap);
    }
    else if(orb_select == 3)
    {
        _pwm_captures_pub_3.publish(_pwm_cap);
    }
    else
    {
        _pwm.timestamp = hrt_absolute_time();
        _pwm.error_count = _error_count;
        _pwm.period = period;
        _pwm.pulse_width = pulse_width;
        _pwm_input_pub.publish(_pwm);
    }
    //end cn

	// update statistics
	_last_period = period;
	_last_width = pulse_width;
	_pulses_captured++;
}

void PWMIN::print_info(void)
{
	PX4_INFO("count=%u period=%u width=%u\n",
		 static_cast<unsigned>(_pulses_captured),
		 static_cast<unsigned>(_last_period),
		 static_cast<unsigned>(_last_width));
}

int
PWMIN::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Measures the PWM input on AUX5 (or MAIN5) via a timer capture ISR and publishes via the uORB 'pwm_input` message.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_input", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "prints PWM capture info.");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

int
PWMIN::custom_command(int argc, char *argv[])
{
	const char *input = argv[0];
	auto *obj = get_instance();

	if (!is_running() || !obj) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	if (!strcmp(input, "test")) {
		obj->print_info();
		return PX4_OK;
	}

	print_usage();
	return PX4_ERROR;
}

extern "C" __EXPORT int pwm_input_main(int argc, char *argv[])
{
    PX4_INFO("pwm input main **********************");
	return PWMIN::main(argc, argv);
}
