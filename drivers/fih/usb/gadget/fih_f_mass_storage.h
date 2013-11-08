/*
fih 1090_usb driver porting,add for HWID control
*/

//StevenCPHuang 2011/08/12 porting base on 1050 ++
/* FIHTDC, PCtool { */
#define SC_READ_NV              0xf0
#define SC_SWITCH_STATUS        0xf1
#define SC_SWITCH_PORT          0xf2
#define SC_MODEM_STATUS         0xf4
#define SC_SHOW_PORT            0xf5
#define SC_MODEM_DISCONNECT     0xf6
#define SC_MODEM_CONNECT        0xf7
#define SC_DIAG_RUT             0xf8
#define SC_READ_BATTERY         0xf9
#define SC_READ_IMAGE           0xfa
#define SC_ENABLE_ALL_PORT      0xfd
#define SC_MASS_STORGE          0xfe
#define SC_ENTER_DOWNLOADMODE   0xff
#define SC_ENTER_FTMMODE        0xe0
#define SC_SWITCH_ROOT          0xe1	//Div2-5-3-Peripheral-LL-ADB_ROOT-00+
/* } FIHTDC, PCtool */
//StevenCPHuang 2011/08/12 porting base on 1050 --
//StevenCPHuang_20110820,add Moto's mode switch cmd to support PID switch function ++
#define SC_MODE_SWITCH      0xD6
//StevenCPHuang_20110820,add Moto's mode switch cmd to support PID switch function --

//StevenCPHuang 2011/08/12 porting base on 1050 ++
/* FIHTDC, PCtool { */
#include <linux/reboot.h>
/* } FIHTDC, PCtool */
//StevenCPHuang 2011/08/12 porting base on 1050 --

extern unsigned int fih_host_usb_id;
extern int fih_enable;
extern unsigned short orig_pid;//FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch
extern int fih_enable_Diag(struct device * pdev);
extern struct device * get_f_diag_dev(void);
#define write_buf_size 1

struct fsg_common {
	struct usb_gadget	*gadget;
	struct fsg_dev		*fsg, *new_fsg;
	wait_queue_head_t	fsg_wait;

	/* filesem protects: backing files in use */
	struct rw_semaphore	filesem;

	/* lock protects: state, all the req_busy's */
	spinlock_t		lock;

	struct usb_ep		*ep0;		/* Copy of gadget->ep0 */
	struct usb_request	*ep0req;	/* Copy of cdev->req */
	unsigned int		ep0_req_tag;
	const char		*ep0req_name;//FIH-StevenCPHuang_20110819-IRM1070_USB PID Switch
	struct fsg_buffhd	*next_buffhd_to_fill;
	struct fsg_buffhd	*next_buffhd_to_drain;
	struct fsg_buffhd	buffhds[FSG_NUM_BUFFERS];

	int			cmnd_size;
	u8			cmnd[MAX_COMMAND_SIZE];

	unsigned int		nluns;
	unsigned int		lun;
	struct fsg_lun		*luns;
	struct fsg_lun		*curlun;

	unsigned int		bulk_out_maxpacket;
	enum fsg_state		state;		/* For exception handling */
	unsigned int		exception_req_tag;

	enum data_direction	data_dir;
	u32			data_size;
	u32			data_size_from_cmnd;
	u32			tag;
	u32			residue;
	u32			usb_amount_left;

	unsigned int		can_stall:1;
	unsigned int		free_storage_on_release:1;
	unsigned int		phase_error:1;
	unsigned int		short_packet_received:1;
	unsigned int		bad_lun_okay:1;
	unsigned int		running:1;

	int			thread_wakeup_needed;
	struct completion	thread_notifier;
	struct task_struct	*thread_task;

	/* Callback functions. */
	int			(*thread_exits)(struct fsg_common *common);
	/* Gadget's private data. */
	void			*private_data;

	/*
	 * Vendor (8 chars), product (16 chars), release (4
	 * hexadecimal digits) and NUL byte
	 */
	char inquiry_string[8 + 16 + 4 + 1];

	struct kref		ref;
};

struct fsg_config {
	unsigned nluns;
	struct fsg_lun_config {
		const char *filename;
		char ro;
		char removable;
		char cdrom;
		char nofua;
	} luns[FSG_MAX_LUNS];

	const char		*lun_name_format;
	const char		*thread_name;

	/* Callback functions. */

	int	(*thread_exits)(struct fsg_common *common);
	/* Gadget's private data. */
	void			*private_data;

	const char *vendor_name;		/*  8 characters or less */
	const char *product_name;		/* 16 characters or less */
	u16 release;

	char			can_stall;

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	struct platform_device *pdev;
#endif
};