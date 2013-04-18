#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 */

#ifdef CONFIG_FEATURE_KCC_F45
#define ACTIVE_STATE_CNT 1
#endif
struct gpio_keys_button {
	/* Configuration parameters */
	int code;		/* input event code (KEY_*, SW_*) */
	int gpio;
	int active_low;
	char *desc;
	int type;		/* input event type (EV_KEY, EV_SW) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	bool can_disable;
#ifdef CONFIG_FEATURE_KCC_F45
	unsigned int active_cnt:ACTIVE_STATE_CNT;
#endif
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
	unsigned int rep:1;		/* enable input subsystem auto repeat */
};

#endif
