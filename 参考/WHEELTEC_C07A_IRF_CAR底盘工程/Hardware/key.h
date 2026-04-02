#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"

extern uint8_t page_number;		//²Ëµ¥Ò³Ãæ

typedef enum {
	USEKEY_stateless,
	USEKEY_single_click,	
	USEKEY_double_click,
	USEKEY_long_click,
	
}UserKeyState_t;
UserKeyState_t key_scan(uint16_t freq);
UserKeyState_t key_scan_2(uint16_t freq);

#endif
