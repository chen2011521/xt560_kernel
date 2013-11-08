#include <linux/module.h>
#include <linux/kernel.h>

/*******************************************
 * Report panel info for FQC
 *  0 : Toshiba with Novatek NT35560
 *  1 : Truly with Himax HX8363-A
 * -1 : Other
 *******************************************/
extern int g_panel_id;

module_param_named(
    type, g_panel_id, int, S_IRUGO | S_IRUSR | S_IRGRP
);

#define MAX_PANEL_LENGTH 30
char * panel_name_arary[] = {
	"Toshiba, LT040MDT9000",
	"Truly, TFT3P3446"
};
static char panel_name[MAX_PANEL_LENGTH] = "unknow";
module_param_string(name, (char * const)panel_name, MAX_PANEL_LENGTH, S_IRUGO | S_IRUSR | S_IRGRP);

void cpy_panel_name(char * src)
{
	param_set_copystring(src, &__param_name);
}
