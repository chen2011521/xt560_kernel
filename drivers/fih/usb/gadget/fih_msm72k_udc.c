/*
fih 1090_usb driver porting,add for HWID control
*/


static inline enum chg_type usb_get_chg_type(struct usb_info *ui)
{
	if ((readl(USB_PORTSC) & PORTSC_LS) == PORTSC_LS)

		return USB_CHG_TYPE__WALLCHARGER;
	else
		return USB_CHG_TYPE__SDP;

	USB_Connect = 1;//FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch


}

void usb_chg_pid(bool usb_switch) 
{
	//struct usb_info *ui = the_usb_info;
	//int i;
	//struct file			*gMD_filp = NULL;               
	//char temp[10];
	switch_enable = true;
	backup_USB_Connect = USB_Connect;
	if(usb_switch){
		//Dynamic_switch = true;
	 	//if (fih_read_usb_id_from_smem() == 0xc000 || fih_enable == 1){
      		if ((fih_host_usb_id == 0xc000) ||( fih_enable == 1)){
         		//upid = 0xc000;
         		USB_Connect = 5;
//StevenCPHuang_20110823,disable it suspendly ++
//		 	g_ps_usb->changed = 1;
//         		power_supply_changed(g_ps_usb);
//StevenCPHuang_20110823,disable it suspendly --
         	}else if (fih_host_usb_id == 0xc001 || fih_enable == 2){
      		//}else if( fih_enable == 2){
         		//upid = 0xc001;
         		USB_Connect = 6;
//StevenCPHuang_20110823,disable it suspendly ++
//		 	g_ps_usb->changed = 1;
//         		power_supply_changed(g_ps_usb);
//StevenCPHuang_20110823,disable it suspendly --
      		}
      		/* FIH, WilsonWHLee, 2010/06/15 { */
      		/*current protection indication*/
      		//else if(fih_enable == 3){
    		//     upid = 0xc003;
   		//      USB_Connect = 7;
   		//      power_supply_changed(g_ps_usb);
   		//   }
    		/* }FIH, WilsonWHLee, 2010/06/15 */
  	}
   	fih_enable = 0;
   	switch_enable = false;
}

void cable_status(bool status)
{
/* FIH, WilsonWHLee, 2010/06/21 { */
/* [FXX_CR], cannot enter suspend after plug & out */	
//	struct usb_info *ui = the_usb_info;
/* } FIH, WilsonWHLee, 2010/06/21 */
	if(status)
		printk("usb cable Connected\n");
	else 
		printk(KERN_INFO "usb cable disconnected\n");

//	Dynamic_switch = false;
//	OS_Type = 0;            
//	check_USB_type =0; //add for charging status

	Linux_Mass = 0;
	is_switch = false;
	USB_Connect = 0;
/* FIH, WilsonWHLee, 2010/06/21 { */

/* [FXX_CR], cannot enter suspend after plug & out */		
  		//if (ui->chg_type == USB_CHG_TYPE__WALLCHARGER)
    	//	power_supply_changed(g_ps_ac);
  		//else
  		//	power_supply_changed(g_ps_usb);
/* } FIH, WilsonWHLee, 2010/06/21 */
}

