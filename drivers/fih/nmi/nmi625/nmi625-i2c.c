/*****************************************************************************
 Copyright(c) 2010 NMI Inc. All Rights Reserved
 
 File name : nmi625-i2c.c
 
 Description :  Generic I2C driver for NM625
 
 History : 
 ----------------------------------------------------------------------
 2010/05/17 	ssw		initial
*******************************************************************************/

#include "nmi625-i2c.h"
#include <linux/pm.h>  //chandler_dtv:Added for kernel suspend 
#include <mach/gpio.h>

#include <linux/slab.h>	// Lewis 11.08.02

static struct i2c_driver nmi625_i2c_driver;
static struct i2c_client *nmi625_i2c_client = NULL;

int nmi625_init = 0;

struct nmi625_state{
	struct i2c_client	*client;	
};
struct nmi625_state *nmi625_state;

extern int nmi625_i2c_suspend(struct device *client);
int nmi625_i2c_init(void)
{
	int res;

	printk("nmi625_i2c_init ENTER...\n");
	nmi625_i2c_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	
	if(nmi625_i2c_client== NULL) {
		printk("nmi625_i2c_client NULL...\n");
		return -ENOMEM;
	}

	if ((res=i2c_add_driver(&nmi625_i2c_driver)))
		pr_err("%s: Can't add nmi625 i2c drv\n", __func__);
	else
		pr_info("%s: Added nmi625 i2c drv\n", __func__);

	//chandler: for DTV to enter deep sleep.
#ifndef NMI_TEST // Lewis for testing
	nmi625_i2c_suspend(NULL);
#endif

	return res;
}


int nmi625_i2c_deinit(void)
{
	printk("nmi625_i2c_deinit ENTER...\n");

	i2c_del_driver(&nmi625_i2c_driver);

	return 0;
}


int nmi625_i2c_read(void *hDevice, unsigned short addr, unsigned char *data, unsigned short length) 
{
	int res;
	struct i2c_msg rmsg;

	rmsg.addr = addr;
	rmsg.flags = I2C_M_RD;
	rmsg.len = length;
	rmsg.buf = data;
		
	res = i2c_transfer(nmi625_i2c_client->adapter, &rmsg, NMI_I2C_NUM);

	return 0;
}

int nmi625_i2c_write(void *hDevice, unsigned short addr, unsigned char *data, unsigned short length)
{
	int res;
	struct i2c_msg wmsg;

	if(length+1>I2C_MAX_SEND_LENGTH)
	{
		printk(".......error,%s", __FUNCTION__);
		return -ENODEV;
	}
	wmsg.addr = addr;
	wmsg.flags = I2C_M_WR;
	wmsg.len = length;
	wmsg.buf = data;
	
	res = i2c_transfer(nmi625_i2c_client->adapter, &wmsg, NMI_I2C_NUM);

	return 0;
}


void nmi625_i2c_reg_read_cmd(u8 *reg,int *ret_val)
{
	u8 cmd[16] = {0,};
	//u8 cmd1[16] = {0,};
	struct i2c_msg rmsg;
	struct i2c_msg wmsg;
	u8 hibyte=*reg;
	u8 lowbyte=*(++reg);
	
	cmd[0] = 0x80;
	cmd[1] = 0x00;
	cmd[2] = hibyte;
	cmd[3] = lowbyte;
	cmd[4] = 0x00;
	cmd[5] = 0x04;

	wmsg.addr = NMI_I2C_ADDR;
	wmsg.flags = I2C_M_WR;
	wmsg.len = 6;
	wmsg.buf = cmd;

	i2c_transfer(nmi625_i2c_client->adapter, &wmsg, NMI_I2C_NUM);

	rmsg.addr = NMI_I2C_ADDR;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 4;
	rmsg.buf = (unsigned char *)ret_val;	// Lewis 11.08.02

	i2c_transfer(nmi625_i2c_client->adapter, &rmsg, NMI_I2C_NUM);
	//printk("%s():(%x,%x): (%02x)(%02x)(%02x)(%02x)\n",__func__, hibyte, lowbyte, rmsg.buf[0],rmsg.buf[1],rmsg.buf[2],rmsg.buf[3]);
}

void nmi625_i2c_reg_write_cmd(u8 *reg, int val)
{
	u8 cmd[16] = {0,};
	u8 hibyte=*reg;
	u8 lowbyte=*(++reg);

	struct i2c_msg wmsg;
	cmd[0] = 0x90;
	cmd[1] = 0x00;
	cmd[2] = hibyte;
	cmd[3] = lowbyte;
	cmd[4] = 0x00;
	cmd[5] = 0x04;
    cmd[6] = (uint8_t)val;
    cmd[7] = (uint8_t)(val >> 8);
    cmd[8] = (uint8_t)(val >> 16);
    cmd[9] = (uint8_t)(val >> 24);
 	
	wmsg.addr = NMI_I2C_ADDR;
	wmsg.flags = I2C_M_WR;
	wmsg.len = 10;
	wmsg.buf = cmd;

	i2c_transfer(nmi625_i2c_client->adapter, &wmsg, NMI_I2C_NUM);

	printk("%s():(%x,%x): (0x%x)\n",__func__, hibyte, lowbyte,val);

}


void nmi625_i2c_read_chip_id(void)
{
	u8 cmd[16] = {0,};
	u8 cmd1[16] = {0,};
	struct i2c_msg rmsg;
	struct i2c_msg wmsg;

	cmd[0] = 0x80;
	cmd[1] = 0x00;
	cmd[2] = 0x64;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x04;

	wmsg.addr = NMI_I2C_ADDR;
	wmsg.flags = I2C_M_WR;
	wmsg.len = 6;
	wmsg.buf = cmd;

	printk("nmi625_i2c_read_chip_id()\n");
	i2c_transfer(nmi625_i2c_client->adapter, &wmsg, NMI_I2C_NUM);

	printk("nmi625_i2c_client->addr (%08x)\n", (unsigned int)nmi625_i2c_client->addr);

	rmsg.addr = NMI_I2C_ADDR;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 4;
	rmsg.buf = cmd1;
	printk("nmi625_i2c_client->adapter (%08x)\n", (unsigned int)nmi625_i2c_client->adapter);
	i2c_transfer(nmi625_i2c_client->adapter, &rmsg, NMI_I2C_NUM);
	printk("Nmi 325 Chip Id (%02x)(%02x)(%02x)(%02x)\n", cmd1[0],cmd1[1],cmd1[2],cmd1[3]);
}
/* FIH, Chandler, add for ftm support { */
void nmi625_i2c_ftm_chip_id(u8 *buf, int size)
{
	u8 cmd[16] = {0,};
//	u8 cmd1[16] = {0,};
	u8 *cmd1=buf;
	
	struct i2c_msg rmsg;
	struct i2c_msg wmsg;

	cmd[0] = 0x80;
	cmd[1] = 0x00;
	cmd[2] = 0x64;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x04;

	wmsg.addr = NMI_I2C_ADDR;
	wmsg.flags = I2C_M_WR;
	wmsg.len = 6;
	wmsg.buf = cmd;

	printk("nmi625_i2c_read_chip_id()\n");
	i2c_transfer(nmi625_i2c_client->adapter, &wmsg, NMI_I2C_NUM);

	printk("nmi625_i2c_client->addr (%08x)\n", (unsigned int)nmi625_i2c_client->addr);

	rmsg.addr = NMI_I2C_ADDR;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 4;
	rmsg.buf = cmd1;
	printk("nmi625_i2c_client->adapter (%08x)\n", (unsigned int)nmi625_i2c_client->adapter);
	i2c_transfer(nmi625_i2c_client->adapter, &rmsg, NMI_I2C_NUM);
	printk("Nmi 325 Chip Id (%02x)(%02x)(%02x)(%02x)\n", cmd1[0],cmd1[1],cmd1[2],cmd1[3]);
}
/* FIH, Chandler, add for ftm support } */

static int nmi625_i2c_remove(struct i2c_client *client)
{
	struct nmi625_state *nmi625 = i2c_get_clientdata(client);

	kfree(nmi625);
	return 0;
}

static int nmi625_i2c_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct nmi625_state *nmi625;

	//printk("nmi625_i2c_probe: %d\n");

	nmi625 = kzalloc(sizeof(struct nmi625_state), GFP_KERNEL);
	if (nmi625 == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	nmi625->client = client;
	i2c_set_clientdata(client, nmi625);
	
	/* rest of the initialisation goes here. */
	
	printk("nmi625 attach success!!!\n");
	dev_err(&client->dev, "nmi625 attach success!!!\n");

	nmi625_i2c_client = client;

	nmi625_init = 1;
	
	return 0;
}


static const struct i2c_device_id nmi625_device_id[] = {
	{"nmi625", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, nmi625_device_id);


void nmi625_i2c_sleep(int sleep)
{
	u8 cmd[16] = {0,};

	struct i2c_msg wmsg;

	cmd[0] = 0x38;
	cmd[1] = 0x00;
	cmd[2] = 0x70;
	cmd[3] = 0x01;
	cmd[4] = 0x00;
	cmd[5] = 0x01;
	cmd[6] = (sleep ? 0 : 1 ); //0: sleep mode, 1: normal mode
	
	wmsg.addr = NMI_I2C_ADDR;
	wmsg.flags = I2C_M_WR;
	wmsg.len = 7;
	wmsg.buf = cmd;

	i2c_transfer(nmi625_i2c_client->adapter, &wmsg, NMI_I2C_NUM);

	printk(KERN_INFO "nmi625_i2c_client->addr (%08x)\n", (unsigned int)nmi625_i2c_client->addr);

}
#if 0 //vincent 2011-8-26
static void nmi625_i2c_reset(void)
{
	gpio_direction_output(34,0);
	mdelay(50);
	gpio_direction_output(34,1);
	mdelay(10);
}
#endif
//--------------------------------------------------------
int nmi625_i2c_suspend(struct device *client)
{
	int ret_val;
	u8 reg1[2]={0x64,0x48};
	u8 reg2[2]={0x64,0x58};
	u8 reg3[2]={0x64,0x4c};

	printk(KERN_ERR "%s()\n", __func__);
	
	//nmi625_i2c_reset(); vincent 2011-8-26
	
//	nmi625_i2c_read_chip_id();//for chandler_test

	nmi625_i2c_reg_read_cmd(reg1,&ret_val);
	ret_val |= 0x10;
	nmi625_i2c_reg_write_cmd(reg1,ret_val);
//	nmi625_i2c_reg_read_cmd(reg1,&ret_val);
	
//	mdelay(10);
	
	nmi625_i2c_reg_read_cmd(reg2,&ret_val);
	ret_val &= ~(0x80);
	nmi625_i2c_reg_write_cmd(reg2,ret_val);
//	nmi625_i2c_reg_read_cmd(reg2,&ret_val);
	
//	mdelay(10);
	nmi625_i2c_reg_read_cmd(reg3,&ret_val);
	ret_val &= ~(0x900);
	nmi625_i2c_reg_write_cmd(reg3,ret_val);
//	nmi625_i2c_reg_read_cmd(reg3,&ret_val);

	nmi625_i2c_sleep(1);
	printk(KERN_ERR "%s() suspend with get chip id\n", __func__);
	return 0;
}

//--------------------------------------------------------
static int nmi625_i2c_resume(struct device *client)
{
//	printk(KERN_ERR "%s()\n", __func__);
	return 0;	
}
//--------------------------------------------------------
static struct dev_pm_ops nmi625_pm_ops = 
{   .suspend = nmi625_i2c_suspend,
    .resume  = nmi625_i2c_resume,
};

static struct i2c_driver nmi625_i2c_driver = {
	.driver = {
		.name = "nmi625",
		.owner = THIS_MODULE,
		//chandler_dtv_suspend
	    .pm = &nmi625_pm_ops,
	},
	.probe	= nmi625_i2c_probe,
	.remove	= nmi625_i2c_remove,
	.id_table	= nmi625_device_id,
};


