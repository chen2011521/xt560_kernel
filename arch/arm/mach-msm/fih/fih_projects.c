#include "fih_projects.h"


#include "irm/irm_proj.h"	



init_project_s fih_projects[] __initdata = {
	{
		Project_IRM,
		IRM_init,
	},
	
	{
		Project_IRE,
		IRM_init,
	},
	
	{
		Project_IMN,
		IRM_init,
	},
	
	{
		Project_ITV,
		IRM_init,
	},
	
	{
		Project_IT2,
		IRM_init,
	},
	
	{
		Project_TBP,
		IRM_init,
	},
	
	{
		Project_TBE,
		IRM_init,	
	},
	
	{
		Project_TNQ,	//Modified By SW4-BSP.Jiahao,2011-10-13
		IRM_init,	
	},
	
	{
		Project_TQ2,
		IRM_init,	
	},
	
	{
		Project_IRQ,
		IRM_init,
	},
	
	{
		Project_NPM,
		IRM_init,
	},
	
	{
		Project_IMP,
		IRM_init,
	},
	
	{
		Project_IPD,
		IRM_init,
	},
	
	{
		Project_TPP,
		IRM_init,
	},
};

int __init initialize_fih_projects(void)
{
	call_project_init(fih_projects, sizeof(fih_projects)/sizeof(init_project_s));	
	
	return 0;
}
