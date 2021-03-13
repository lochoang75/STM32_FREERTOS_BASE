#ifndef _MAIN_H_ 
#define _MAIN_H_
typedef enum{
	NONE,
	SINGLE_PRESS,
	DOUBLE_PRESS
}buttonType;

typedef enum{
	HALF_SEC,
	ONE_SEC,
	ONE_HALF_SEC,
	TWO_SEC
}toneLength;

typedef enum system_state_enum {
    sys_state_idle = 0,
    sys_state_wait_first_pressed_done,
    sys_state_wait_timeout,
} system_state_t;

typedef enum button_action {
    button_no_action = 0,
    button_single_pressed,
    button_double_pressed
} button_action_t;

//user state
typedef enum{
	user_select_tone = 0,
	user_confirm_tone,
	user_select_duration,
	user_confirm_duration
} user_action_t;
#endif