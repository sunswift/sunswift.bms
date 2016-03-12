/* ------------------------------------------------------------------------
   Scandal Obligations
   Functions which the application writer must provide.

   File name: scandal_obligations.c
   Author: Etienne Le Sueur
   Date: 28/08/2011
   ------------------------------------------------------------------------- */

#include <scandal/obligations.h>
#include <scandal/error.h>
#include <scandal/devices.h>

void scandal_user_do_first_run(void) {
	return;
}

u08 scandal_user_do_config(u08 param, s32 value, s32 value2) {
	return NO_ERR;
}

u08 scandal_user_handle_message(can_msg* msg) {
	return NO_ERR;
}

u08 scandal_user_handle_command(u08 command, u08* data) {
	return NO_ERR; 
}
