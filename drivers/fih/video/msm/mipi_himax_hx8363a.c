
static char hx8363a_sleep_out[1] = {0x11}; /* DTYPE_DCS_WRITE */
static char hx8363a_set_password[4] = {0xB9, 0xFF, 0x83, 0x63}; /* DTYPE_DCS_LWRITE */
static char hx8363a_set_power[13] = {0xB1, 0x78, 0x34, 0x08, 0x34, 0x02, 0x13, 0x11, 0x11, 0x1D,
									0x25, 0x3F, 0x3F}; /* DTYPE_DCS_LWRITE */
static char hx8363a_set_disp[5] = {0xB2, 0x33, 0x33, 0x22, 0xFF}; /* DTYPE_DCS_LWRITE */
static char hx8363a_set_mipi[14] = {0xBA, 0x80, 0x00, 0x10, 0x08, 0x08, 0x10, 0x7C, 0x6E, 0x6D,
									0x0A, 0x01, 0x84, 0x43}; /* DTYPE_DCS_LWRITE */
static char hx8363a_set_pixel_format[2] = {0x3A, 0x70}; /* DTYPE_DCS_WRITE1 */
static char hx8363a_set_rgb_if[2] = {0xB3, 0x00}; /* DTYPE_DCS_WRITE1 */
static char hx8363a_set_cyc[10] = {0xB4, 0x08, 0x12, 0x72, 0x12, 0x06, 0x03, 0x54, 0x03, 0x4E}; /* DTYPE_DCS_LWRITE */
static char hx8363a_set_vcom_b6[2] = {0xB6, 0x3D}; /* DTYPE_DCS_WRITE1 */
static char hx8363a_set_vcom_bf[3] = {0xBF, 0x00, 0x10}; /* DTYPE_DCS_LWRITE */
static char hx8363a_set_panel[2] = {0xCC, 0x0B}; /* DTYPE_DCS_WRITE1 */
static char hx8363a_set_gamma[31] = {0xE0, 0x01, 0x1D, 0x22, 0x33, 0x3A, 0x3F, 0x06, 0x8A, 0x8C,
									0x8F, 0x54, 0x12, 0x16, 0x4F, 0x18, 0x01, 0x1D, 0x22, 0x33,
									0x3A, 0x3F, 0x06, 0x8A, 0x8C, 0x8F, 0x54, 0x12, 0x16, 0x4F,
									0x18}; /* DTYPE_DCS_LWRITE */
static char hx8363a_display_on[1] = {0x29}; /* DTYPE_DCS_WRITE */
static char hx8363a_display_off[1] = {0x28}; /* DTYPE_DCS_WRITE */
static char hx8363a_sleep_in[1] = {0x10}; /* DTYPE_DCS_WRITE */

static struct dsi_cmd_desc hx8363a_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 70,
		sizeof(hx8363a_sleep_out), hx8363a_sleep_out},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(hx8363a_set_password), hx8363a_set_password},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(hx8363a_set_power), hx8363a_set_power},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(hx8363a_set_disp), hx8363a_set_disp},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(hx8363a_set_mipi), hx8363a_set_mipi},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(hx8363a_set_pixel_format), hx8363a_set_pixel_format},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(hx8363a_set_rgb_if), hx8363a_set_rgb_if},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(hx8363a_set_cyc), hx8363a_set_cyc},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(hx8363a_set_vcom_b6), hx8363a_set_vcom_b6},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(hx8363a_set_vcom_bf), hx8363a_set_vcom_bf},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(hx8363a_set_panel), hx8363a_set_panel},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 50,
		sizeof(hx8363a_set_gamma), hx8363a_set_gamma},
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(hx8363a_display_on), hx8363a_display_on},
};

static struct dsi_cmd_desc hx8363a_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(hx8363a_display_off), hx8363a_display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 40,
		sizeof(hx8363a_sleep_in), hx8363a_sleep_in},
};
