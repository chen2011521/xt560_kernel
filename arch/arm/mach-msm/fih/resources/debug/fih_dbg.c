/** 
* Debug Configuration Tool
********************************************************/
#include "fih_dbg.h"


//FIHTDC-DerrickDRLiu-DbgCfgTool-00+[ 
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define RAM_CONSOLE_PHYS 0x2D400000
#define RAM_CONSOLE_SIZE 0x00020000
static struct resource ram_console_resources[1] = {
	[0] = {
		.start  = RAM_CONSOLE_PHYS,
		.end    = RAM_CONSOLE_PHYS + RAM_CONSOLE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name   = "ram_console",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,

};

int __init ram_console_init(void)
{
	platform_device_register(&ram_console_device);
	return 0;	
}
#else	/* !CONFIG_ANDROID_RAM_CONSOLE */
int __init ram_console_init(void)
{

	return 0;	
}
#endif	/* !CONFIG_ANDROID_RAM_CONSOLE */



#ifdef CONFIG_FIH_LAST_ALOG
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define ALOG_RAM_CONSOLE_PHYS_MAIN (RAM_CONSOLE_PHYS + RAM_CONSOLE_SIZE)
#else
#define ALOG_RAM_CONSOLE_PHYS_MAIN 0x2D420000
#endif 
 
#define ALOG_RAM_CONSOLE_SIZE_MAIN 0x00020000 //128KB
#define ALOG_RAM_CONSOLE_PHYS_RADIO (ALOG_RAM_CONSOLE_PHYS_MAIN +  ALOG_RAM_CONSOLE_SIZE_MAIN)
#define ALOG_RAM_CONSOLE_SIZE_RADIO 0x00020000 //128KB
#define ALOG_RAM_CONSOLE_PHYS_EVENTS (ALOG_RAM_CONSOLE_PHYS_RADIO + ALOG_RAM_CONSOLE_SIZE_RADIO) 
#define ALOG_RAM_CONSOLE_SIZE_EVENTS 0x00020000 //128KB
#define ALOG_RAM_CONSOLE_PHYS_SYSTEM (ALOG_RAM_CONSOLE_PHYS_EVENTS + ALOG_RAM_CONSOLE_SIZE_EVENTS) 
#define ALOG_RAM_CONSOLE_SIZE_SYSTEM 0x00020000 //128KB

static struct resource alog_ram_console_resources[4] = {
	[0] = {
		.name = "alog_main_buffer",
		.start  = ALOG_RAM_CONSOLE_PHYS_MAIN,
		.end    = ALOG_RAM_CONSOLE_PHYS_MAIN + ALOG_RAM_CONSOLE_SIZE_MAIN - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.name = "alog_radio_buffer",
		.start  = ALOG_RAM_CONSOLE_PHYS_RADIO,
		.end    = ALOG_RAM_CONSOLE_PHYS_RADIO + ALOG_RAM_CONSOLE_SIZE_RADIO - 1,
		.flags  = IORESOURCE_MEM,
	},
	[2] = {
		.name = "alog_events_buffer",
		.start  = ALOG_RAM_CONSOLE_PHYS_EVENTS,
		.end    = ALOG_RAM_CONSOLE_PHYS_EVENTS + ALOG_RAM_CONSOLE_SIZE_EVENTS - 1,
		.flags  = IORESOURCE_MEM,
	},
	[3] = {
		.name = "alog_system_buffer",
		.start  = ALOG_RAM_CONSOLE_PHYS_SYSTEM,
		.end    = ALOG_RAM_CONSOLE_PHYS_SYSTEM + ALOG_RAM_CONSOLE_SIZE_SYSTEM - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device alog_ram_console_device = {
	.name = "alog_ram_console",
	.id = 0,
	.num_resources = ARRAY_SIZE(alog_ram_console_resources),
	.resource = alog_ram_console_resources,
};

int __init last_alog_init(void)
{
	platform_device_register(&alog_ram_console_device);
	return 0;	
}
#else	/* !CONFIG_FIH_LAST_ALOG */
int __init last_alog_init(void)
{
	return 0;	
}
#endif	/* !CONFIG_FIH_LAST_ALOG */
//FIHTDC-DerrickDRLiu-DbgCfgTool-00+]
