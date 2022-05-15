/* Copyright (c) 2012-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/string.h>

#include "mdss_dsi.h"
#include "mdss_debug.h"
#ifdef TARGET_HW_MDSS_HDMI
#include "mdss_dba_utils.h"
#endif
#include "mdss_debug.h"

#define DT_CMD_HDR 6
#define DEFAULT_MDP_TRANSFER_TIME 14000

#define VSYNC_DELAY msecs_to_jiffies(17)

DEFINE_LED_TRIGGER(bl_led_trigger);
char project_name[MDSS_MAX_PANEL_LEN];
extern int lcm_software_id;
extern unsigned int is_atboot;
int lcm_cabc_state  = 1;
int pre_lcm_cabc = 1;
int is_cabc_recover = 1;
int wled_cabc_enable_level = 801;
struct mdss_dsi_ctrl_pdata *g_ctrl_pdata;
static int pre_bklt_level;
static int dimming_enable;
static int dimming_for_camera = 1;

static char bbk_board_version[BBK_LCM_BL_ID_LEN] = "version_invalid";
static char *bbk_board_version_str = "bbk_board_version=";

extern unsigned int panel_off_state;
extern unsigned int phone_shutdown_state;  /* 0: no shutdown, 1: shutdown*/
int vivo_esd_check_ps_status;
int vivo_esd_check_enable_status;
extern uint32_t interval;
int lcm_sre_state = 0;
int pre_lcm_sre = 0;
int panel_id;
int bl_hw_id;	//for B version backlight ic
int composite_id;	// bl_hw_id << 24 | panel_id
extern unsigned int allGestureSwitchIntoBitmap(void);     /* is gesture mode or not*/
static void dsi_panel_update_sre(unsigned int level);
static void dsi_panel_update_cabc(u32 bl_lvl, u32 cabc_lvl);
static void vivo_adjust_cabc(u32 bl_lvl);
int panel_max_bl_level = 4095;
int lcmpr_debug_enabled;
void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	if (ctrl->pwm_pmi)
		return;

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: Error: lpg_chan=%d pwm request failed",
				__func__, ctrl->pwm_lpg_chan);
		ctrl->pwm_bl = NULL;
	}
	ctrl->pwm_enabled = 0;
}

bool mdss_dsi_panel_pwm_enable(struct mdss_dsi_ctrl_pdata *ctrl)
{
	bool status = true;

	if (!ctrl->pwm_enabled)
		goto end;

	if (pwm_enable(ctrl->pwm_bl)) {
		pr_err("%s: pwm_enable() failed\n", __func__);
		status = false;
	}

	ctrl->pwm_enabled = 1;

end:
	return status;
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;
	u32 period_ns;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	if (level == 0) {
		if (ctrl->pwm_enabled) {
			ret = pwm_config(ctrl->pwm_bl, 0,
					ctrl->pwm_period * NSEC_PER_USEC);
			if (ret)
				pr_err("%s: pwm_config() failed err=%d.\n",
						__func__, ret);
			pwm_disable(ctrl->pwm_bl);
		}
		ctrl->pwm_enabled = 0;
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	if (ctrl->pwm_period >= USEC_PER_SEC) {
		ret = pwm_config_us(ctrl->pwm_bl, duty, ctrl->pwm_period);
		if (ret) {
			pr_err("%s: pwm_config_us() failed err=%d.\n",
					__func__, ret);
			return;
		}
	} else {
		period_ns = ctrl->pwm_period * NSEC_PER_USEC;
		ret = pwm_config(ctrl->pwm_bl,
				level * period_ns / ctrl->bklt_max,
				period_ns);
		if (ret) {
			pr_err("%s: pwm_config() failed err=%d.\n",
					__func__, ret);
			return;
		}
	}

	if (!ctrl->pwm_enabled) {
		ret = pwm_enable(ctrl->pwm_bl);
		if (ret)
			pr_err("%s: pwm_enable() failed err=%d\n", __func__,
				ret);
		ctrl->pwm_enabled = 1;
	}
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

int mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return -EINVAL;
	}

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */
	/*
	 * blocked here, until call back called
	 */

	return mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static void mdss_dsi_panel_apply_settings(struct mdss_dsi_ctrl_pdata *ctrl,
				struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if ((pinfo->dcs_cmd_by_left) && (ctrl->ndx != DSI_CTRL_LEFT))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = flags;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;
	else if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static char led_pwm1[3] = {0x51, 0x00, 0x00};
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(led_pwm1)}, led_pwm1
};

/*lrz add 2018 0911 */

static void dsi_panel_update_cabc(u32 bl_lvl, u32 cabc_lvl)
{
	struct dsi_panel_cmds *cabc_on_cmds;
	LCM_INFO("will set lcm_cabc_status to %d,bl_lvl is %d\n", cabc_lvl,bl_lvl);
	if (bl_lvl < wled_cabc_enable_level) {
		LCM_INFO("current bl_lvl is %d ,less than %d , close CABC\n", bl_lvl, wled_cabc_enable_level);
		cabc_on_cmds = &g_ctrl_pdata->lcm_cabc_off;
		if (cabc_on_cmds->cmd_cnt)
			mdss_dsi_panel_cmds_send(g_ctrl_pdata, cabc_on_cmds, CMD_REQ_COMMIT);
		lcm_cabc_state = 0;
		is_cabc_recover = 0;
	} else {
		if (cabc_lvl == 3) {
			cabc_on_cmds = &g_ctrl_pdata->lcm_cabc_level3;
			if (cabc_on_cmds->cmd_cnt)
				mdss_dsi_panel_cmds_send(g_ctrl_pdata, cabc_on_cmds, CMD_REQ_COMMIT);
		} else if (cabc_lvl == 2) {
			cabc_on_cmds = &g_ctrl_pdata->lcm_cabc_level2;
			if (cabc_on_cmds->cmd_cnt)
				mdss_dsi_panel_cmds_send(g_ctrl_pdata, cabc_on_cmds, CMD_REQ_COMMIT);
		} else if (cabc_lvl == 1) {
			cabc_on_cmds = &g_ctrl_pdata->lcm_cabc_level1;
			if (cabc_on_cmds->cmd_cnt)
				mdss_dsi_panel_cmds_send(g_ctrl_pdata, cabc_on_cmds, CMD_REQ_COMMIT);
		} else if (cabc_lvl == 0) {
			cabc_on_cmds = &g_ctrl_pdata->lcm_cabc_off;
			if (cabc_on_cmds->cmd_cnt)
				mdss_dsi_panel_cmds_send(g_ctrl_pdata, cabc_on_cmds, CMD_REQ_COMMIT);
		}
	}
}

static void vivo_adjust_cabc(u32 bl_lvl)
{
	if (bl_lvl < wled_cabc_enable_level) {
		if (is_cabc_recover) {
			dsi_panel_update_cabc(bl_lvl, 0);
			LCM_INFO("pre_lcm_cabc %d, recover %d\n", pre_lcm_cabc, is_cabc_recover);
		}
	} else {
		if (!is_cabc_recover) {
			lcm_cabc_state = pre_lcm_cabc; //restore cabc value
			is_cabc_recover = 1;//restore only once
			dsi_panel_update_cabc(bl_lvl, lcm_cabc_state);
			LCM_INFO("lcm_cabc_state %d, recover %d\n", lcm_cabc_state, is_cabc_recover);
		}
	}
}

static void mdss_dsi_sre_dimming_set(struct mdss_dsi_ctrl_pdata *ctrl, int dimming_lvl)
{
	int i;
	int step = lcm_sre_state / dimming_lvl;
	int sre_lvl = lcm_sre_state;

	if (lcm_sre_state < dimming_lvl) {
		dimming_lvl = lcm_sre_state;
		step = 1;
	}

	if (lcm_sre_state < dimming_lvl)
		goto exit;

	for (i = 0; i < dimming_lvl; i++) {
		sre_lvl -= step;
		if (sre_lvl > 0) {
			lcm_sre_state = sre_lvl;
			dsi_panel_update_sre(sre_lvl);
			usleep_range(33000, 33100);
			LCM_INFO("force close sre_lvl=%d,step=%d,dimming_lvl=%d\n", sre_lvl, step, dimming_lvl);
		}
	}

exit:
	lcm_sre_state = 0;
	dsi_panel_update_sre(0);
	LCM_INFO(">>>>>>force close sre\n");
}
atomic_t disable_sre_setting;
static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;
	struct dsi_panel_cmds *diming_on_cmds;
	struct mdss_dsi_ctrl_pdata *sctrl = NULL;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}

	if ((lcm_sre_state > 0) && (level == 0)) {
		atomic_set(&disable_sre_setting, 1);
		lcm_sre_state = 0;
		dsi_panel_update_sre(0);
		LCM_INFO("chenyu SRE=0 lcm_sre_state=%d, lcm_software_id=0x%x, level = %d\n", lcm_sre_state, lcm_software_id, level);
	} else if ((lcm_sre_state > 0) && (level < panel_max_bl_level-1)) {
		mdss_dsi_sre_dimming_set(sctrl, 16);
	}
	pr_debug("%s: level=%d\n", __func__, level);

	if ((level != 0) && (dimming_enable == 0) && (pre_bklt_level != 0) && (dimming_for_camera == 1)) {
		diming_on_cmds = &g_ctrl_pdata->lcm_cmd_backlight_dimming_enable;
		if (diming_on_cmds->cmd_cnt)
			mdss_dsi_panel_cmds_send(g_ctrl_pdata, diming_on_cmds, CMD_REQ_COMMIT);
		dimming_enable = 1;
	}

	if (level == 0)
		dimming_enable = 0;
	if ((lcm_software_id == 0x60) || (lcm_software_id == 0x63) || (lcm_software_id == 0x78) || (lcm_software_id == 0x74)) {/* for ft8006p*/
		level = level/2;
		led_pwm1[1] = (unsigned char)((level>>3)&0xFF);/*high 8bit */
		led_pwm1[2] = (unsigned char)((level&0x07) << 1);/*low  3bit */
		LCM_DEBUG("set bl level = %d, high_bit = 0x%x, low_bit=0x%x\n", level, led_pwm1[1], led_pwm1[2]);
	} else {
		led_pwm1[1] = (unsigned char)((level>>8)&0xFF);
		led_pwm1[2] = (unsigned char)(level&0xFFFF);
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	if (ctrl->bklt_dcs_op_mode == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;
	else
		cmdreq.flags |= CMD_REQ_LP_MODE;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static void mdss_dsi_panel_set_idle_mode(struct mdss_panel_data *pdata,
							bool enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);

	pr_info("%s: Idle (%d->%d)\n", __func__, ctrl->idle, enable);

	if (ctrl->idle == enable)
		return;

	MDSS_XLOG(ctrl->idle, enable);
	if (enable) {
		if (ctrl->idle_on_cmds.cmd_cnt) {
			mdss_dsi_panel_cmds_send(ctrl, &ctrl->idle_on_cmds,
					CMD_REQ_COMMIT);
			ctrl->idle = true;
			pr_debug("Idle on\n");
		}
	} else {
		if (ctrl->idle_off_cmds.cmd_cnt) {
			mdss_dsi_panel_cmds_send(ctrl, &ctrl->idle_off_cmds,
					CMD_REQ_COMMIT);
			ctrl->idle = false;
			pr_debug("Idle off\n");
		}
	}
}

static bool mdss_dsi_panel_get_idle_mode(struct mdss_panel_data *pdata)

{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return 0;
	}
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);
	return ctrl->idle;
}

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
				       rc);
			goto disp_en_gpio_err;
		}
	}
	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}
	if (gpio_is_valid(ctrl_pdata->tp_rst_gpio)) {
		rc = gpio_request(ctrl_pdata->tp_rst_gpio,
						"tp_rst_n");
		if (rc) {
			pr_err("request tp rst gpio failed, rc=%d\n",
				       rc);
			goto tp_rst_gpio_err;
		}
	}

	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		rc = gpio_request(ctrl_pdata->bklt_en_gpio,
						"bklt_enable");
		if (rc) {
			pr_err("request bklt gpio failed, rc=%d\n",
				       rc);
			goto bklt_en_gpio_err;
		}
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
		gpio_free(ctrl_pdata->bklt_en_gpio);
bklt_en_gpio_err:
	gpio_free(ctrl_pdata->tp_rst_gpio);
tp_rst_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	/* need to configure intf mux only for external interface */
	if (pinfo->is_dba_panel) {
		if (enable) {
			if (gpio_is_valid(ctrl_pdata->intf_mux_gpio)) {
				rc = gpio_request(ctrl_pdata->intf_mux_gpio,
						"intf_mux");
				if (rc) {
					pr_err("request mux gpio failed, rc=%d\n",
									rc);
					return rc;
				}
				rc = gpio_direction_output(
					ctrl_pdata->intf_mux_gpio, 0);
				if (rc) {
					pr_err("%s: unable to set dir for intf mux gpio\n",
								__func__);
					goto exit;
				}
				gpio_set_value(ctrl_pdata->intf_mux_gpio, 0);
			} else {
				pr_debug("%s:%d, intf mux gpio not specified\n",
							__func__, __LINE__);
			}
		} else {
			if (gpio_is_valid(ctrl_pdata->intf_mux_gpio))
				gpio_free(ctrl_pdata->intf_mux_gpio);
		}
	}

	if ((mdss_dsi_is_right_ctrl(ctrl_pdata) &&
		mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)) ||
			pinfo->is_dba_panel) {
		pr_debug("%s:%d, right ctrl gpio configuration not needed\n",
			__func__, __LINE__);
		return rc;
	}

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	if (pinfo->skip_panel_reset && !pinfo->cont_splash_enabled) {
		pr_debug("%s: skip_panel_reset is set\n", __func__);
		return 0;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
				rc = gpio_direction_output(
					ctrl_pdata->disp_en_gpio, 1);
				if (rc) {
					pr_err("%s: unable to set dir for en gpio\n",
						__func__);
					goto exit;
				}
			}

			if (gpio_is_valid(ctrl_pdata->tp_rst_gpio)) {
				rc = gpio_direction_output(ctrl_pdata->tp_rst_gpio, 1);
				if (rc) {
					pr_err("%s: unable to set dir for rst gpio\n",
						__func__);
					goto exit;
				}
			}
			usleep_range(1000, 1010);
			/*setting of ft8006p use tp reset and lcm reset*/
			if ((lcm_software_id == 0x60) || (lcm_software_id == 0x4f) || (lcm_software_id == 0x63) || (lcm_software_id == 0x78) || (lcm_software_id == 0x74)  || (lcm_software_id == 0x75))  {
				if (gpio_is_valid(ctrl_pdata->tp_rst_gpio)) {
					gpio_set_value((ctrl_pdata->tp_rst_gpio), 0);
				}
				usleep_range(6000, 6010);
			if (pdata->panel_info.rst_seq_len) {
				rc = gpio_direction_output(ctrl_pdata->rst_gpio,
					pdata->panel_info.rst_seq[0]);
				if (rc) {
					pr_err("%s: unable to set dir for rst gpio\n",
						__func__);
					goto exit;
				}
			}
				for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
					if (enable != INCELL_LCM_POWER_ON_WITHOUT_LCM_RST)
						gpio_set_value((ctrl_pdata->rst_gpio), pdata->panel_info.rst_seq[i]);
					if (!(pdata->panel_info.rst_seq[i])) {
						usleep_range(6000, 6010);
						if (gpio_is_valid(ctrl_pdata->tp_rst_gpio)) {
							gpio_set_value((ctrl_pdata->tp_rst_gpio), 1);
							usleep_range(1000, 1010);
						}
					}
					if (pdata->panel_info.rst_seq[++i])
						usleep_range((pinfo->rst_seq[i] * 1000),
						(pinfo->rst_seq[i] * 1000) + 10);
				}

			} else {
				if (pdata->panel_info.rst_seq_len) {
					rc = gpio_direction_output(ctrl_pdata->rst_gpio,
						pdata->panel_info.rst_seq[0]);
					if (rc) {
						pr_err("%s: unable to set dir for rst gpio\n",
							__func__);
						goto exit;
					}
				}
				for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
					gpio_set_value((ctrl_pdata->rst_gpio),
						pdata->panel_info.rst_seq[i]);
					if (pdata->panel_info.rst_seq[++i])
						usleep_range((pinfo->rst_seq[i] * 1000),
						(pinfo->rst_seq[i] * 1000) + 10);
				}
			}
			}

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {

				if (ctrl_pdata->bklt_en_gpio_invert)
					rc = gpio_direction_output(
						ctrl_pdata->bklt_en_gpio, 0);
				else
					rc = gpio_direction_output(
						ctrl_pdata->bklt_en_gpio, 1);

				if (rc) {
					pr_err("%s: unable to set dir for bklt gpio\n",
						__func__);
					goto exit;
			}
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			bool out = false;

			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				out = true;
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				out = false;

			rc = gpio_direction_output(ctrl_pdata->mode_gpio, out);
			if (rc) {
				pr_err("%s: unable to set dir for mode gpio\n",
					__func__);
				goto exit;
			}
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {

			if (ctrl_pdata->bklt_en_gpio_invert)
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
			else
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);

			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
		}

		if (gpio_is_valid(ctrl_pdata->tp_rst_gpio)) {
			gpio_set_value((ctrl_pdata->tp_rst_gpio), 0);
		}

		if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			usleep_range(3000, 3010);
		}
	}

	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		gpio_free(ctrl_pdata->bklt_en_gpio);
	}

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		gpio_free(ctrl_pdata->disp_en_gpio);
	}

	gpio_free(ctrl_pdata->rst_gpio);

	if (gpio_is_valid(ctrl_pdata->mode_gpio))
		gpio_free(ctrl_pdata->mode_gpio);

	if (gpio_is_valid(ctrl_pdata->tp_rst_gpio)) {
		gpio_free(ctrl_pdata->tp_rst_gpio);
	}

exit:
	return rc;
}
/* add for incell panel ,  for Touch panel to control reset and power */
void incell_tp_reset_output(int status)
{
	/*/int rc = 0;*/
	/*pr_err("%s: status is %d \n", __func__,status);*/
	if (gpio_is_valid(g_ctrl_pdata->tp_rst_gpio))
		gpio_set_value((g_ctrl_pdata->tp_rst_gpio), status);

	/*gpio_direction_output(fts_wq_data->pdata->reset_gpio, status);*/

}
EXPORT_SYMBOL(incell_tp_reset_output);

/**
 * mdss_dsi_roi_merge() -  merge two roi into single roi
 *
 * Function used by partial update with only one dsi intf take 2A/2B
 * (column/page) dcs commands.
 */
static int mdss_dsi_roi_merge(struct mdss_dsi_ctrl_pdata *ctrl,
					struct mdss_rect *roi)
{
	struct mdss_panel_info *l_pinfo;
	struct mdss_rect *l_roi;
	struct mdss_rect *r_roi;
	struct mdss_dsi_ctrl_pdata *other = NULL;
	int ans = 0;

	if (ctrl->ndx == DSI_CTRL_LEFT) {
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_RIGHT);
		if (!other)
			return ans;
		l_pinfo = &(ctrl->panel_data.panel_info);
		l_roi = &(ctrl->panel_data.panel_info.roi);
		r_roi = &(other->panel_data.panel_info.roi);
	} else  {
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_LEFT);
		if (!other)
			return ans;
		l_pinfo = &(other->panel_data.panel_info);
		l_roi = &(other->panel_data.panel_info.roi);
		r_roi = &(ctrl->panel_data.panel_info.roi);
	}

	if (l_roi->w == 0 && l_roi->h == 0) {
		/* right only */
		*roi = *r_roi;
		roi->x += l_pinfo->xres;/* add left full width to x-offset */
	} else {
		/* left only and left+righ */
		*roi = *l_roi;
		roi->w +=  r_roi->w; /* add right width */
		ans = 1;
	}

	return ans;
}

static char pageset[] = {0xfe, 0x00};			/* DTYPE_DCS_WRITE1 */
static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	/* DTYPE_DCS_LWRITE */
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	/* DTYPE_DCS_LWRITE */

/* pack into one frame before sent */
static struct dsi_cmd_desc set_col_page_addr_cmd[] = {
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(pageset)}, pageset},
	{{DTYPE_DCS_LWRITE, 0, 0, 0, 1, sizeof(caset)}, caset},	/* packed */
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static void mdss_dsi_send_col_page_addr(struct mdss_dsi_ctrl_pdata *ctrl,
				struct mdss_rect *roi, int unicast)
{
	struct dcs_cmd_req cmdreq;

	set_col_page_addr_cmd[0].payload = pageset;

	caset[1] = (((roi->x) & 0xFF00) >> 8);
	caset[2] = (((roi->x) & 0xFF));
	caset[3] = (((roi->x - 1 + roi->w) & 0xFF00) >> 8);
	caset[4] = (((roi->x - 1 + roi->w) & 0xFF));
	set_col_page_addr_cmd[1].payload = caset;

	paset[1] = (((roi->y) & 0xFF00) >> 8);
	paset[2] = (((roi->y) & 0xFF));
	paset[3] = (((roi->y - 1 + roi->h) & 0xFF00) >> 8);
	paset[4] = (((roi->y - 1 + roi->h) & 0xFF));
	set_col_page_addr_cmd[2].payload = paset;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds_cnt = 3;
	cmdreq.flags = CMD_REQ_COMMIT;
	if (unicast)
		cmdreq.flags |= CMD_REQ_UNICAST;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	cmdreq.cmds = set_col_page_addr_cmd;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static int mdss_dsi_set_col_page_addr(struct mdss_panel_data *pdata,
		bool force_send)
{
	struct mdss_panel_info *pinfo;
	struct mdss_rect roi = {0};
	struct mdss_rect *p_roi;
	struct mdss_rect *c_roi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;
	int left_or_both = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &pdata->panel_info;
	p_roi = &pinfo->roi;

	/*
	 * to avoid keep sending same col_page info to panel,
	 * if roi_merge enabled, the roi of left ctrl is used
	 * to compare against new merged roi and saved new
	 * merged roi to it after comparing.
	 * if roi_merge disabled, then the calling ctrl's roi
	 * and pinfo's roi are used to compare.
	 */
	if (pinfo->partial_update_roi_merge) {
		left_or_both = mdss_dsi_roi_merge(ctrl, &roi);
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_LEFT);
		c_roi = &other->roi;
	} else {
		c_roi = &ctrl->roi;
		roi = *p_roi;
	}

	/* roi had changed, do col_page update */
	if (force_send || !mdss_rect_cmp(c_roi, &roi)) {
		pr_debug("%s: ndx=%d x=%d y=%d w=%d h=%d\n",
				__func__, ctrl->ndx, p_roi->x,
				p_roi->y, p_roi->w, p_roi->h);

		*c_roi = roi; /* keep to ctrl */
		if (c_roi->w == 0 || c_roi->h == 0) {
			/* no new frame update */
			pr_debug("%s: ctrl=%d, no partial roi set\n",
						__func__, ctrl->ndx);
			return 0;
		}

		if (pinfo->partial_update_col_addr_offset)
			roi.x += pinfo->partial_update_col_addr_offset;

		if (pinfo->partial_update_row_addr_offset)
			roi.y += pinfo->partial_update_row_addr_offset;

		if (pinfo->dcs_cmd_by_left) {
			if (left_or_both && ctrl->ndx == DSI_CTRL_RIGHT) {
				/* 2A/2B sent by left already */
				return 0;
			}
		}

		if (!mdss_dsi_sync_wait_enable(ctrl)) {
			if (pinfo->dcs_cmd_by_left)
				ctrl = mdss_dsi_get_ctrl_by_index(
							DSI_CTRL_LEFT);
			mdss_dsi_send_col_page_addr(ctrl, &roi, 0);
		} else {
			/*
			 * when sync_wait_broadcast enabled,
			 * need trigger at right ctrl to
			 * start both dcs cmd transmission
			 */
			other = mdss_dsi_get_other_ctrl(ctrl);
			if (!other)
				goto end;

			if (mdss_dsi_is_left_ctrl(ctrl)) {
				if (pinfo->partial_update_roi_merge) {
					/*
					 * roi is the one after merged
					 * to dsi-1 only
					 */
					mdss_dsi_send_col_page_addr(other,
							&roi, 0);
				} else {
					mdss_dsi_send_col_page_addr(ctrl,
							&ctrl->roi, 1);
					mdss_dsi_send_col_page_addr(other,
							&other->roi, 1);
				}
			} else {
				if (pinfo->partial_update_roi_merge) {
					/*
					 * roi is the one after merged
					 * to dsi-1 only
					 */
					mdss_dsi_send_col_page_addr(ctrl,
							&roi, 0);
				} else {
					mdss_dsi_send_col_page_addr(other,
							&other->roi, 1);
					mdss_dsi_send_col_page_addr(ctrl,
							&ctrl->roi, 1);
				}
			}
		}
	}

end:
	return 0;
}

static int mdss_dsi_panel_apply_display_setting(struct mdss_panel_data *pdata,
							u32 mode)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct dsi_panel_cmds *lp_on_cmds;
	struct dsi_panel_cmds *lp_off_cmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	lp_on_cmds = &ctrl->lp_on_cmds;
	lp_off_cmds = &ctrl->lp_off_cmds;

	/* Apply display settings for low-persistence mode */
	if ((mode == MDSS_PANEL_LOW_PERSIST_MODE_ON) &&
			(lp_on_cmds->cmd_cnt))
		mdss_dsi_panel_apply_settings(ctrl, lp_on_cmds);
	else if ((mode == MDSS_PANEL_LOW_PERSIST_MODE_OFF) &&
			(lp_off_cmds->cmd_cnt))
		mdss_dsi_panel_apply_settings(ctrl, lp_off_cmds);
	else
		return -EINVAL;

	pr_debug("%s: Persistence mode %d applied\n", __func__, mode);
	return 0;
}

static void mdss_dsi_panel_switch_mode(struct mdss_panel_data *pdata,
							int mode)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mipi_panel_info *mipi;
	struct dsi_panel_cmds *pcmds;
	u32 flags = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	mipi  = &pdata->panel_info.mipi;

	if (!mipi->dms_mode)
		return;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (mipi->dms_mode != DYNAMIC_MODE_RESOLUTION_SWITCH_IMMEDIATE) {
		flags |= CMD_REQ_COMMIT;
		if (mode == SWITCH_TO_CMD_MODE)
			pcmds = &ctrl_pdata->video2cmd;
		else
			pcmds = &ctrl_pdata->cmd2video;
	} else if ((mipi->dms_mode ==
				DYNAMIC_MODE_RESOLUTION_SWITCH_IMMEDIATE)
			&& pdata->current_timing
			&& !list_empty(&pdata->timings_list)) {
		struct dsi_panel_timing *pt;

		pt = container_of(pdata->current_timing,
				struct dsi_panel_timing, timing);

		pr_debug("%s: sending switch commands\n", __func__);
		pcmds = &pt->switch_cmds;
		flags |= CMD_REQ_DMA_TPG;
		flags |= CMD_REQ_COMMIT;
	} else {
		pr_warn("%s: Invalid mode switch attempted\n", __func__);
		return;
	}

	if ((pdata->panel_info.compression_mode == COMPRESSION_DSC) &&
			(pdata->panel_info.send_pps_before_switch))
		mdss_dsi_panel_dsc_pps_send(ctrl_pdata, &pdata->panel_info);

	mdss_dsi_panel_cmds_send(ctrl_pdata, pcmds, flags);

	if ((pdata->panel_info.compression_mode == COMPRESSION_DSC) &&
			(!pdata->panel_info.send_pps_before_switch))
		mdss_dsi_panel_dsc_pps_send(ctrl_pdata, &pdata->panel_info);
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_dsi_ctrl_pdata *sctrl = NULL;
	struct dsi_panel_cmds *cabc_off_cmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	cabc_off_cmds = &g_ctrl_pdata->lcm_cabc_off;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	/*
	 * Some backlight controllers specify a minimum duty cycle
	 * for the backlight brightness. If the brightness is less
	 * than it, the controller can malfunction.
	 */

	if ((bl_level < pdata->panel_info.bl_min) && (bl_level != 0))
		bl_level = pdata->panel_info.bl_min;
	LCM_INFO("bklt_ctrl=%d, bl_level=%d , panel_max_bl_level=%d\n", ctrl_pdata->bklt_ctrl, bl_level, panel_max_bl_level);

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		if (!mdss_dsi_sync_wait_enable(ctrl_pdata)) {
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);

			/*vivo display modify for When the call is far away from infrared, the backlight turns on slowly*/
			vivo_adjust_cabc(bl_level);

			pre_bklt_level = bl_level;
			break;
		}
		/*
		 * DCS commands to update backlight are usually sent at
		 * the same time to both the controllers. However, if
		 * sync_wait is enabled, we need to ensure that the
		 * dcs commands are first sent to the non-trigger
		 * controller so that when the commands are triggered,
		 * both controllers receive it at the same time.
		 */
		sctrl = mdss_dsi_get_other_ctrl(ctrl_pdata);
		if (mdss_dsi_sync_wait_trigger(ctrl_pdata)) {
			if (sctrl)
				mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		} else {
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
			if (sctrl)
				mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
		}
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

bool is_panel_backlight_on(void)
{
	if (!(g_ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT))	{
		LCM_DEBUG("is_panel_backlight_on  Invalid panel state\n");
		return PANEL_BACKLIGHT_OFF;
	}

	if (pre_bklt_level)
		return PANEL_BACKLIGHT_ON;
	else {
		return PANEL_BACKLIGHT_OFF;
	}
}
EXPORT_SYMBOL(is_panel_backlight_on);

/*****************do not use hdmi********************

#ifdef TARGET_HW_MDSS_HDMI
static void mdss_dsi_panel_on_hdmi(struct mdss_dsi_ctrl_pdata *ctrl,
			struct mdss_panel_info *pinfo)
{
	if (ctrl->ds_registered)
		mdss_dba_utils_video_on(pinfo->dba_data, pinfo);
}
#else
static void mdss_dsi_panel_on_hdmi(struct mdss_dsi_ctrl_pdata *ctrl,
			struct mdss_panel_info *pinfo)
{
	(void)(*ctrl);
	(void)(*pinfo);
}
#endif
*************************************************/

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct dsi_panel_cmds *on_cmds;
	int ret = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ndx=%d\n", __func__, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	panel_off_state = 0;
	on_cmds = &ctrl->on_cmds;

	if ((pinfo->mipi.dms_mode == DYNAMIC_MODE_SWITCH_IMMEDIATE) &&
			(pinfo->mipi.boot_mode != pinfo->mipi.mode))
		on_cmds = &ctrl->post_dms_on_cmds;

	pr_debug("%s: ndx=%d cmd_cnt=%d\n", __func__,
				ctrl->ndx, on_cmds->cmd_cnt);

	usleep_range(2000, 2010);

	if (on_cmds->cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, on_cmds, CMD_REQ_COMMIT);

	if (pinfo->compression_mode == COMPRESSION_DSC)
		mdss_dsi_panel_dsc_pps_send(ctrl, pinfo);

	/**********do not use hdmi**********/
	/*mdss_dsi_panel_on_hdmi(ctrl, pinfo);*/

	/* Ensure low persistence mode is set as before */
	mdss_dsi_panel_apply_display_setting(pdata, pinfo->persist_mode);

	/**********do not read 0a when panel disconnect**********/
	if ((lcm_software_id != 0x00) && (lcm_software_id != 0xff)) {
		mdss_dsi_panel_cmd_read(ctrl, 0x0a, 0x00, NULL, ctrl->rx_buf.data, 1);
		LCM_INFO("lcm power status = 0x%x\n", *(ctrl->rx_buf.data));
	}

	atomic_set(&disable_sre_setting, 0);

end:
	pr_debug("%s:-\n", __func__);
	return ret;
}

#ifdef TARGET_HW_MDSS_HDMI
static void mdss_dsi_post_panel_on_hdmi(struct mdss_panel_info *pinfo)
{
	u32 vsync_period = 0;

	if (pinfo->is_dba_panel && pinfo->is_pluggable) {
		/* ensure at least 1 frame transfers to down stream device */
		vsync_period = (MSEC_PER_SEC / pinfo->mipi.frame_rate) + 1;
		msleep(vsync_period);
		mdss_dba_utils_hdcp_enable(pinfo->dba_data, true);
	}
}
#else
static void mdss_dsi_post_panel_on_hdmi(struct mdss_panel_info *pinfo)
{
	(void)(*pinfo);
}
#endif

static int mdss_dsi_post_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct dsi_panel_cmds *cmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);

	pinfo = &pdata->panel_info;
	if (pinfo->dcs_cmd_by_left && ctrl->ndx != DSI_CTRL_LEFT)
		goto end;

	cmds = &ctrl->post_panel_on_cmds;
	if (cmds->cmd_cnt) {
		msleep(VSYNC_DELAY);	/* wait for a vsync passed */
		mdss_dsi_panel_cmds_send(ctrl, cmds, CMD_REQ_COMMIT);
	}

	mdss_dsi_post_panel_on_hdmi(pinfo);

end:
	pr_debug("%s:-\n", __func__);
	return 0;
}

/*****************do not use hdmi********************
#ifdef TARGET_HW_MDSS_HDMI
static void mdss_dsi_panel_off_hdmi(struct mdss_dsi_ctrl_pdata *ctrl,
			struct mdss_panel_info *pinfo)
{
	if (ctrl->ds_registered && pinfo->is_pluggable) {
		mdss_dba_utils_video_off(pinfo->dba_data);
		mdss_dba_utils_hdcp_enable(pinfo->dba_data, false);
	}
}
#else
static void mdss_dsi_panel_off_hdmi(struct mdss_dsi_ctrl_pdata *ctrl,
			struct mdss_panel_info *pinfo)
{
	(void)(*ctrl);
	(void)(*pinfo);
}
#endif
**************************************************/

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	if (ctrl->off_cmds.cmd_cnt) {
		pr_err("%s:send off cmd\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds, CMD_REQ_COMMIT);
	}

	/*****************do not use hdmi********************/
	/*mdss_dsi_panel_off_hdmi(ctrl, pinfo);*/

	panel_off_state = 1;

end:
	/* clear idle state */
	ctrl->idle = false;
	pr_debug("%s:-\n", __func__);
	return 0;
}

static int mdss_dsi_panel_low_power_config(struct mdss_panel_data *pdata,
	int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d enable=%d\n", __func__, ctrl, ctrl->ndx,
		enable);

	/* Any panel specific low power commands/config */
	/* Control idle mode for panel */
	if (enable)
		mdss_dsi_panel_set_idle_mode(pdata, true);
	else
		mdss_dsi_panel_set_idle_mode(pdata, false);
	pr_debug("%s:-\n", __func__);
	return 0;
}

static void mdss_dsi_parse_mdp_kickoff_threshold(struct device_node *np,
	struct mdss_panel_info *pinfo)
{
	int len, rc;
	const u32 *src;
	u32 tmp;
	u32 max_delay_us;

	pinfo->mdp_koff_thshold = false;
	src = of_get_property(np, "qcom,mdss-mdp-kickoff-threshold", &len);
	if (!src || (len == 0))
		return;

	rc = of_property_read_u32(np, "qcom,mdss-mdp-kickoff-delay", &tmp);
	if (!rc)
		pinfo->mdp_koff_delay = tmp;
	else
		return;

	if (pinfo->mipi.frame_rate == 0) {
		pr_err("cannot enable guard window, unexpected panel fps\n");
		return;
	}

	pinfo->mdp_koff_thshold_low = be32_to_cpu(src[0]);
	pinfo->mdp_koff_thshold_high = be32_to_cpu(src[1]);
	max_delay_us = 1000000 / pinfo->mipi.frame_rate;

	/* enable the feature if threshold is valid */
	if ((pinfo->mdp_koff_thshold_low < pinfo->mdp_koff_thshold_high) &&
	   ((pinfo->mdp_koff_delay > 0) ||
	    (pinfo->mdp_koff_delay < max_delay_us)))
		pinfo->mdp_koff_thshold = true;

	pr_debug("panel kickoff thshold:[%d, %d] delay:%d (max:%d) enable:%d\n",
		pinfo->mdp_koff_thshold_low,
		pinfo->mdp_koff_thshold_high,
		pinfo->mdp_koff_delay,
		max_delay_us,
		pinfo->mdp_koff_thshold);
}

static void mdss_dsi_parse_trigger(struct device_node *np, char *trigger,
		char *trigger_key)
{
	const char *data;

	*trigger = DSI_CMD_TRIGGER_SW;
	data = of_get_property(np, trigger_key, NULL);
	if (data) {
		if (!strcmp(data, "none"))
			*trigger = DSI_CMD_TRIGGER_NONE;
		else if (!strcmp(data, "trigger_te"))
			*trigger = DSI_CMD_TRIGGER_TE;
		else if (!strcmp(data, "trigger_sw_seof"))
			*trigger = DSI_CMD_TRIGGER_SW_SEOF;
		else if (!strcmp(data, "trigger_sw_te"))
			*trigger = DSI_CMD_TRIGGER_SW_TE;
	}
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kcalloc(blen, sizeof(char), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kcalloc(cnt, sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		data = of_get_property(np, link_key, NULL);
		if (data && !strcmp(data, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}


int mdss_panel_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;

	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int mdss_dsi_parse_fbc_params(struct device_node *np,
			struct mdss_panel_timing *timing)
{
	int rc, fbc_enabled = 0;
	u32 tmp;
	struct fbc_panel_info *fbc = &timing->fbc;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		fbc->enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		fbc->target_bpp = (!rc ? tmp : 24);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		fbc->comp_mode = (!rc ? tmp : 0);
		fbc->qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		fbc->cd_bias = (!rc ? tmp : 0);
		fbc->pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		fbc->vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		fbc->bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		fbc->line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		fbc->block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		fbc->block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		fbc->lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		fbc->lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		fbc->lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		fbc->lossy_mode_idx = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-slice-height", &tmp);
		fbc->slice_height = (!rc ? tmp : 0);
		fbc->pred_mode = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-2d-pred-mode");
		fbc->enc_mode = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-ver2-mode");
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-max-pred-err", &tmp);
		fbc->max_pred_err = (!rc ? tmp : 0);

		timing->compression_mode = COMPRESSION_FBC;
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		fbc->enabled = 0;
		fbc->target_bpp = 24;
	}
	return 0;
}

void mdss_dsi_panel_dsc_pps_send(struct mdss_dsi_ctrl_pdata *ctrl,
				struct mdss_panel_info *pinfo)
{
	struct dsi_panel_cmds pcmds;
	struct dsi_cmd_desc cmd;

	if (!pinfo || (pinfo->compression_mode != COMPRESSION_DSC))
		return;

	memset(&pcmds, 0, sizeof(pcmds));
	memset(&cmd, 0, sizeof(cmd));

	cmd.dchdr.dlen = mdss_panel_dsc_prepare_pps_buf(&pinfo->dsc,
		ctrl->pps_buf, 0);
	cmd.dchdr.dtype = DTYPE_PPS;
	cmd.dchdr.last = 1;
	cmd.dchdr.wait = 10;
	cmd.dchdr.vc = 0;
	cmd.dchdr.ack = 0;
	cmd.payload = ctrl->pps_buf;

	pcmds.cmd_cnt = 1;
	pcmds.cmds = &cmd;
	pcmds.link_state = DSI_LP_MODE;

	mdss_dsi_panel_cmds_send(ctrl, &pcmds, CMD_REQ_COMMIT);
}

static int mdss_dsi_parse_hdr_settings(struct device_node *np,
		struct mdss_panel_info *pinfo)
{
	int rc = 0;
	struct mdss_panel_hdr_properties *hdr_prop;

	if (!np) {
		pr_err("%s: device node pointer is NULL\n", __func__);
		return -EINVAL;
	}

	if (!pinfo) {
		pr_err("%s: panel info is NULL\n", __func__);
		return -EINVAL;
	}

	hdr_prop = &pinfo->hdr_properties;
	hdr_prop->hdr_enabled = of_property_read_bool(np,
		"qcom,mdss-dsi-panel-hdr-enabled");

	if (hdr_prop->hdr_enabled) {
		rc = of_property_read_u32_array(np,
				"qcom,mdss-dsi-panel-hdr-color-primaries",
				hdr_prop->display_primaries,
				DISPLAY_PRIMARIES_COUNT);
		if (rc) {
			pr_info("%s:%d, Unable to read color primaries,rc:%u",
					__func__, __LINE__,
					hdr_prop->hdr_enabled = false);
		}

		rc = of_property_read_u32(np,
			"qcom,mdss-dsi-panel-peak-brightness",
			&(hdr_prop->peak_brightness));
		if (rc) {
			pr_info("%s:%d, Unable to read hdr brightness, rc:%u",
				__func__, __LINE__, rc);
			hdr_prop->hdr_enabled = false;
		}

		rc = of_property_read_u32(np,
			"qcom,mdss-dsi-panel-blackness-level",
			&(hdr_prop->blackness_level));
		if (rc) {
			pr_info("%s:%d, Unable to read hdr brightness, rc:%u",
				__func__, __LINE__, rc);
			hdr_prop->hdr_enabled = false;
		}
	}
	return 0;
}

static int mdss_dsi_parse_dsc_version(struct device_node *np,
		struct mdss_panel_timing *timing)
{
	u32 data;
	int rc = 0;
	struct dsc_desc *dsc = &timing->dsc;

	rc = of_property_read_u32(np, "qcom,mdss-dsc-version", &data);
	if (rc) {
		dsc->version = 0x11;
		rc = 0;
	} else {
		dsc->version = data & 0xff;
		/* only support DSC 1.1 rev */
		if (dsc->version != 0x11) {
			pr_err("%s: DSC version:%d not supported\n", __func__,
				dsc->version);
			rc = -EINVAL;
			goto end;
		}
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsc-scr-version", &data);
	if (rc) {
		dsc->scr_rev = 0x0;
		rc = 0;
	} else {
		dsc->scr_rev = data & 0xff;
		/* only one scr rev supported */
		if (dsc->scr_rev > 0x1) {
			pr_err("%s: DSC scr version:%d not supported\n",
				__func__, dsc->scr_rev);
			rc = -EINVAL;
			goto end;
		}
	}

end:
	return rc;
}

static int mdss_dsi_parse_dsc_params(struct device_node *np,
		struct mdss_panel_timing *timing, bool is_split_display)
{
	u32 data, intf_width;
	int rc = 0;
	struct dsc_desc *dsc = &timing->dsc;

	if (!np) {
		pr_err("%s: device node pointer is NULL\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsc-encoders", &data);
	if (rc) {
		if (!of_find_property(np, "qcom,mdss-dsc-encoders", NULL)) {
			/* property is not defined, default to 1 */
			data = 1;
		} else {
			pr_err("%s: Error parsing qcom,mdss-dsc-encoders\n",
				__func__);
			goto end;
		}
	}

	timing->dsc_enc_total = data;

	if (is_split_display && (timing->dsc_enc_total > 1)) {
		pr_err("%s: Error: for split displays, more than 1 dsc encoder per panel is not allowed.\n",
			__func__);
		goto end;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsc-slice-height", &data);
	if (rc)
		goto end;
	dsc->slice_height = data;

	rc = of_property_read_u32(np, "qcom,mdss-dsc-slice-width", &data);
	if (rc)
		goto end;
	dsc->slice_width = data;
	intf_width = timing->xres;

	if (intf_width % dsc->slice_width) {
		pr_err("%s: Error: multiple of slice-width:%d should match panel-width:%d\n",
			__func__, dsc->slice_width, intf_width);
		goto end;
	}

	data = intf_width / dsc->slice_width;
	if (((timing->dsc_enc_total > 1) && ((data != 2) && (data != 4))) ||
	    ((timing->dsc_enc_total == 1) && (data > 2))) {
		pr_err("%s: Error: max 2 slice per encoder. slice-width:%d should match panel-width:%d dsc_enc_total:%d\n",
			__func__, dsc->slice_width,
			intf_width, timing->dsc_enc_total);
		goto end;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsc-slice-per-pkt", &data);
	if (rc)
		goto end;
	dsc->slice_per_pkt = data;

	/*
	 * slice_per_pkt can be either 1 or all slices_per_intf
	 */
	if ((dsc->slice_per_pkt > 1) && (dsc->slice_per_pkt !=
			DIV_ROUND_UP(intf_width, dsc->slice_width))) {
		pr_err("Error: slice_per_pkt can be either 1 or all slices_per_intf\n");
		pr_err("%s: slice_per_pkt=%d, slice_width=%d intf_width=%d\n",
			__func__,
			dsc->slice_per_pkt, dsc->slice_width, intf_width);
		rc = -EINVAL;
		goto end;
	}

	pr_debug("%s: num_enc:%d :slice h=%d w=%d s_pkt=%d\n", __func__,
		timing->dsc_enc_total, dsc->slice_height,
		dsc->slice_width, dsc->slice_per_pkt);

	rc = of_property_read_u32(np, "qcom,mdss-dsc-bit-per-component", &data);
	if (rc)
		goto end;
	dsc->bpc = data;

	rc = of_property_read_u32(np, "qcom,mdss-dsc-bit-per-pixel", &data);
	if (rc)
		goto end;
	dsc->bpp = data;

	pr_debug("%s: bpc=%d bpp=%d\n", __func__,
		dsc->bpc, dsc->bpp);

	dsc->block_pred_enable = of_property_read_bool(np,
			"qcom,mdss-dsc-block-prediction-enable");

	dsc->enable_422 = 0;
	dsc->convert_rgb = 1;
	dsc->vbr_enable = 0;

	dsc->config_by_manufacture_cmd = of_property_read_bool(np,
		"qcom,mdss-dsc-config-by-manufacture-cmd");

	mdss_panel_dsc_parameters_calc(&timing->dsc);
	mdss_panel_dsc_pclk_param_calc(&timing->dsc, intf_width);

	timing->dsc.full_frame_slices =
		DIV_ROUND_UP(intf_width, timing->dsc.slice_width);

	timing->compression_mode = COMPRESSION_DSC;

end:
	return rc;
}

static struct device_node *mdss_dsi_panel_get_dsc_cfg_np(
		struct device_node *np, struct mdss_panel_data *panel_data,
		bool default_timing)
{
	struct device_node *dsc_cfg_np = NULL;


	/* Read the dsc config node specified by command line */
	if (default_timing) {
		dsc_cfg_np = of_get_child_by_name(np,
				panel_data->dsc_cfg_np_name);
		if (!dsc_cfg_np)
			pr_warn_once("%s: cannot find dsc config node:%s\n",
				__func__, panel_data->dsc_cfg_np_name);
	}

	/*
	 * Fall back to default from DT as nothing is specified
	 * in command line.
	 */
	if (!dsc_cfg_np && of_find_property(np, "qcom,config-select", NULL)) {
		dsc_cfg_np = of_parse_phandle(np, "qcom,config-select", 0);
		if (!dsc_cfg_np)
			pr_warn_once("%s:err parsing qcom,config-select\n",
					__func__);
	}

	return dsc_cfg_np;
}

static int mdss_dsi_parse_topology_config(struct device_node *np,
	struct dsi_panel_timing *pt, struct mdss_panel_data *panel_data,
	bool default_timing)
{
	int rc = 0;
	bool is_split_display = panel_data->panel_info.is_split_display;
	const char *data;
	struct mdss_panel_timing *timing = &pt->timing;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;
	struct device_node *cfg_np = NULL;

	ctrl_pdata = container_of(panel_data, struct mdss_dsi_ctrl_pdata,
							panel_data);
	pinfo = &ctrl_pdata->panel_data.panel_info;

	cfg_np = mdss_dsi_panel_get_dsc_cfg_np(np,
				&ctrl_pdata->panel_data, default_timing);

	if (cfg_np) {
		if (!of_property_read_u32_array(cfg_np, "qcom,lm-split",
		    timing->lm_widths, 2)) {
			if (mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)
			    && (timing->lm_widths[1] != 0)) {
				pr_err("%s: lm-split not allowed with split display\n",
					__func__);
				rc = -EINVAL;
				goto end;
			}
		}

		if (!of_property_read_string(cfg_np, "qcom,split-mode",
		    &data) && !strcmp(data, "pingpong-split"))
			pinfo->use_pingpong_split = true;

		if (((timing->lm_widths[0]) || (timing->lm_widths[1])) &&
		    pinfo->use_pingpong_split) {
			pr_err("%s: pingpong_split cannot be used when lm-split[%d,%d] is specified\n",
				__func__,
				timing->lm_widths[0], timing->lm_widths[1]);
			return -EINVAL;
		}

		pr_info("%s: cfg_node name %s lm_split:%dx%d pp_split:%s\n",
			__func__, cfg_np->name,
			timing->lm_widths[0], timing->lm_widths[1],
			pinfo->use_pingpong_split ? "yes" : "no");
	}

	if (!pinfo->use_pingpong_split &&
	    (timing->lm_widths[0] == 0) && (timing->lm_widths[1] == 0))
		timing->lm_widths[0] = pt->timing.xres;

	data = of_get_property(np, "qcom,compression-mode", NULL);
	if (data) {
		if (cfg_np && !strcmp(data, "dsc")) {
			rc = mdss_dsi_parse_dsc_version(np, &pt->timing);
			if (rc)
				goto end;

			pinfo->send_pps_before_switch =
				of_property_read_bool(np,
				"qcom,mdss-dsi-send-pps-before-switch");

			rc = mdss_dsi_parse_dsc_params(cfg_np, &pt->timing,
					is_split_display);
		} else if (!strcmp(data, "fbc")) {
			rc = mdss_dsi_parse_fbc_params(np, &pt->timing);
		}
	}

end:
	of_node_put(cfg_np);
	return rc;
}

static void mdss_panel_parse_te_params(struct device_node *np,
		struct mdss_panel_timing *timing)
{
	struct mdss_mdp_pp_tear_check *te = &timing->te;
	u32 tmp;
	int rc = 0;
	/*
	 * TE default: dsi byte clock calculated base on 70 fps;
	 * around 14 ms to complete a kickoff cycle if te disabled;
	 * vclk_line base on 60 fps; write is faster than read;
	 * init == start == rdptr;
	 */
	te->tear_check_en =
		!of_property_read_bool(np, "qcom,mdss-tear-check-disable");
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-cfg-height", &tmp);
	te->sync_cfg_height = (!rc ? tmp : 0xfff0);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-init-val", &tmp);
	te->vsync_init_val = (!rc ? tmp : timing->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-start", &tmp);
	te->sync_threshold_start = (!rc ? tmp : 4);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-continue", &tmp);
	te->sync_threshold_continue = (!rc ? tmp : 4);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-frame-rate", &tmp);
	te->refx100 = (!rc ? tmp : 6000);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-start-pos", &tmp);
	te->start_pos = (!rc ? tmp : timing->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-rd-ptr-trigger-intr", &tmp);
	te->rd_ptr_irq = (!rc ? tmp : timing->yres + 1);
	te->wr_ptr_irq = 0;
}


static int mdss_dsi_parse_reset_seq(struct device_node *np,
		u32 rst_seq[MDSS_DSI_RST_SEQ_LEN], u32 *rst_len,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];
	*rst_len = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_DSI_RST_SEQ_LEN || num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				rst_seq[i] = tmp[i];
			*rst_len = num;
		}
	}
	return 0;
}

static bool mdss_dsi_cmp_panel_reg_v2(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i, j = 0;
	int len = 0, *lenp;
	int group = 0;

	lenp = ctrl->status_valid_params ?: ctrl->status_cmds_rlen;

	for (i = 0; i < ctrl->status_cmds.cmd_cnt; i++)
		len += lenp[i];

	for (j = 0; j < ctrl->groups; ++j) {
		for (i = 0; i < len; ++i) {
			pr_debug("[%i] return:0x%x status:0x%x\n",
				i, ctrl->return_buf[i],
				(unsigned int)ctrl->status_value[group + i]);
			MDSS_XLOG(ctrl->ndx, ctrl->return_buf[i],
					ctrl->status_value[group + i]);
			if (ctrl->return_buf[i] !=
				ctrl->status_value[group + i])
				break;
		}

		if (i == len)
			return true;
		group += len;
	}

	return false;
}

static int mdss_dsi_gen_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (!mdss_dsi_cmp_panel_reg_v2(ctrl_pdata)) {
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		return 1;
	}
}

static int mdss_dsi_nt35596_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (!mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
		ctrl_pdata->status_value, 0)) {
		ctrl_pdata->status_error_count = 0;
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	}
	{
		if (!mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
			ctrl_pdata->status_value, 3)) {
			ctrl_pdata->status_error_count = 0;
		} else {
			if (mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
				ctrl_pdata->status_value, 4) ||
				mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
				ctrl_pdata->status_value, 5))
				ctrl_pdata->status_error_count = 0;
			else
				ctrl_pdata->status_error_count++;
			if (ctrl_pdata->status_error_count >=
					ctrl_pdata->max_status_error_count) {
				ctrl_pdata->status_error_count = 0;
				pr_err("%s: Read value bad. Error_cnt = %i\n",
					 __func__,
					ctrl_pdata->status_error_count);
				return -EINVAL;
			}
		}
		return 1;
	}
}

static void mdss_dsi_parse_roi_alignment(struct device_node *np,
		struct dsi_panel_timing *pt)
{
	int len = 0;
	u32 value[6];
	struct property *data;
	struct mdss_panel_timing *timing = &pt->timing;

	data = of_find_property(np, "qcom,panel-roi-alignment", &len);
	len /= sizeof(u32);
	if (!data || (len != 6)) {
		pr_debug("%s: Panel roi alignment not found", __func__);
	} else {
		int rc = of_property_read_u32_array(np,
				"qcom,panel-roi-alignment", value, len);
		if (rc)
			pr_debug("%s: Error reading panel roi alignment values",
					__func__);
		else {
			timing->roi_alignment.xstart_pix_align = value[0];
			timing->roi_alignment.ystart_pix_align = value[1];
			timing->roi_alignment.width_pix_align = value[2];
			timing->roi_alignment.height_pix_align = value[3];
			timing->roi_alignment.min_width = value[4];
			timing->roi_alignment.min_height = value[5];
		}

		pr_debug("%s: ROI alignment: [%d, %d, %d, %d, %d, %d]",
			__func__, timing->roi_alignment.xstart_pix_align,
			timing->roi_alignment.width_pix_align,
			timing->roi_alignment.ystart_pix_align,
			timing->roi_alignment.height_pix_align,
			timing->roi_alignment.min_width,
			timing->roi_alignment.min_height);
	}
}

static void mdss_dsi_parse_dms_config(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct mdss_panel_info *pinfo = &ctrl->panel_data.panel_info;
	const char *data;
	bool dms_enabled;

	dms_enabled = of_property_read_bool(np,
		"qcom,dynamic-mode-switch-enabled");

	if (!dms_enabled) {
		pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_DISABLED;
		goto exit;
	}

	/* default mode is suspend_resume */
	pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_SUSPEND_RESUME;
	data = of_get_property(np, "qcom,dynamic-mode-switch-type", NULL);
	if (data && !strcmp(data, "dynamic-resolution-switch-immediate")) {
		if (!list_empty(&ctrl->panel_data.timings_list))
			pinfo->mipi.dms_mode =
				DYNAMIC_MODE_RESOLUTION_SWITCH_IMMEDIATE;
		else
			pinfo->mipi.dms_mode =
				DYNAMIC_MODE_SWITCH_DISABLED;
		goto exit;
	}

	if (data && !strcmp(data, "dynamic-switch-immediate"))
		pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_IMMEDIATE;
	else
		pr_debug("%s: default dms suspend/resume\n", __func__);

	mdss_dsi_parse_dcs_cmds(np, &ctrl->video2cmd,
		"qcom,video-to-cmd-mode-switch-commands",
		"qcom,mode-switch-commands-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl->cmd2video,
		"qcom,cmd-to-video-mode-switch-commands",
		"qcom,mode-switch-commands-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl->post_dms_on_cmds,
		"qcom,mdss-dsi-post-mode-switch-on-command",
		"qcom,mdss-dsi-post-mode-switch-on-command-state");

	if (pinfo->mipi.dms_mode == DYNAMIC_MODE_SWITCH_IMMEDIATE &&
		!ctrl->post_dms_on_cmds.cmd_cnt) {
		pr_warn("%s: No post dms on cmd specified\n", __func__);
		pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_DISABLED;
	}

	if (!ctrl->video2cmd.cmd_cnt || !ctrl->cmd2video.cmd_cnt) {
		pr_warn("%s: No commands specified for dynamic switch\n",
			__func__);
		pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_DISABLED;
	}
exit:
	pr_info("%s: dynamic switch feature enabled: %d\n", __func__,
		pinfo->mipi.dms_mode);
}

/* the length of all the valid values to be checked should not be great
 * than the length of returned data from read command.
 */
static bool
mdss_dsi_parse_esd_check_valid_params(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i;

	for (i = 0; i < ctrl->status_cmds.cmd_cnt; ++i) {
		if (ctrl->status_valid_params[i] > ctrl->status_cmds_rlen[i]) {
			pr_debug("%s: ignore valid params!\n", __func__);
			return false;
		}
	}

	return true;
}

static bool mdss_dsi_parse_esd_status_len(struct device_node *np,
	char *prop_key, u32 **target, u32 cmd_cnt)
{
	int tmp;

	if (!of_find_property(np, prop_key, &tmp))
		return false;

	tmp /= sizeof(u32);
	if (tmp != cmd_cnt) {
		pr_err("%s: request property number(%d) not match command count(%d)\n",
			__func__, tmp, cmd_cnt);
		return false;
	}

	*target = kcalloc(tmp, sizeof(u32), GFP_KERNEL);
	if (IS_ERR_OR_NULL(*target)) {
		pr_err("%s: Error allocating memory for property\n",
			__func__);
		return false;
	}

	if (of_property_read_u32_array(np, prop_key, *target, tmp)) {
		pr_err("%s: cannot get values from dts\n", __func__);
		kfree(*target);
		*target = NULL;
		return false;
	}

	return true;
}

static void mdss_dsi_parse_esd_params(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 tmp;
	u32 i, status_len, *lenp;
	int rc;
	struct property *data;
	const char *string;
	struct mdss_panel_info *pinfo = &ctrl->panel_data.panel_info;

	pinfo->esd_check_enabled = of_property_read_bool(np,
		"qcom,esd-check-enabled");

	if (is_atboot)
		pinfo->esd_check_enabled = false;

	if (!pinfo->esd_check_enabled)
		return;

	ctrl->status_mode = ESD_MAX;
	rc = of_property_read_string(np,
			"qcom,mdss-dsi-panel-status-check-mode", &string);
	if (!rc) {
		if (!strcmp(string, "bta_check")) {
			ctrl->status_mode = ESD_BTA;
		} else if (!strcmp(string, "reg_read")) {
			ctrl->status_mode = ESD_REG;
			ctrl->check_read_status =
				mdss_dsi_gen_read_status;
		} else if (!strcmp(string, "reg_read_nt35596")) {
			ctrl->status_mode = ESD_REG_NT35596;
			ctrl->status_error_count = 0;
			ctrl->check_read_status =
				mdss_dsi_nt35596_read_status;
		} else if (!strcmp(string, "te_signal_check")) {
			if (pinfo->mipi.mode == DSI_CMD_MODE) {
				ctrl->status_mode = ESD_TE;
			} else {
				pr_err("TE-ESD not valid for video mode\n");
				goto error;
			}
		} else {
			pr_err("No valid panel-status-check-mode string\n");
			goto error;
		}
	}

	if ((ctrl->status_mode == ESD_BTA) || (ctrl->status_mode == ESD_TE) ||
			(ctrl->status_mode == ESD_MAX))
		return;

	mdss_dsi_parse_dcs_cmds(np, &ctrl->status_cmds,
			"qcom,mdss-dsi-panel-status-command",
				"qcom,mdss-dsi-panel-status-command-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-max-error-count",
		&tmp);
	ctrl->max_status_error_count = (!rc ? tmp : 0);

	if (!mdss_dsi_parse_esd_status_len(np,
		"qcom,mdss-dsi-panel-status-read-length",
		&ctrl->status_cmds_rlen, ctrl->status_cmds.cmd_cnt)) {
		pinfo->esd_check_enabled = false;
		return;
	}

	if (mdss_dsi_parse_esd_status_len(np,
		"qcom,mdss-dsi-panel-status-valid-params",
		&ctrl->status_valid_params, ctrl->status_cmds.cmd_cnt)) {
		if (!mdss_dsi_parse_esd_check_valid_params(ctrl))
			goto error1;
	}

	status_len = 0;
	lenp = ctrl->status_valid_params ?: ctrl->status_cmds_rlen;
	for (i = 0; i < ctrl->status_cmds.cmd_cnt; ++i)
		status_len += lenp[i];

	data = of_find_property(np, "qcom,mdss-dsi-panel-status-value", &tmp);
	tmp /= sizeof(u32);
	if (!IS_ERR_OR_NULL(data) && tmp != 0 && (tmp % status_len) == 0) {
		ctrl->groups = tmp / status_len;
	} else {
		pr_err("%s: Error parse panel-status-value\n", __func__);
		goto error1;
	}

	ctrl->status_value = kcalloc(status_len * ctrl->groups, sizeof(u32),
				GFP_KERNEL);
	if (!ctrl->status_value)
		goto error1;

	ctrl->return_buf = kcalloc(status_len * ctrl->groups,
			sizeof(unsigned char), GFP_KERNEL);
	if (!ctrl->return_buf)
		goto error2;

	rc = of_property_read_u32_array(np,
		"qcom,mdss-dsi-panel-status-value",
		ctrl->status_value, ctrl->groups * status_len);
	if (rc) {
		pr_debug("%s: Error reading panel status values\n",
				__func__);
		memset(ctrl->status_value, 0, ctrl->groups * status_len);
	}

	return;

error2:
	kfree(ctrl->status_value);
error1:
	kfree(ctrl->status_valid_params);
	kfree(ctrl->status_cmds_rlen);
error:
	pinfo->esd_check_enabled = false;
}

static int mdss_dsi_parse_panel_features(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 value[2];
	struct mdss_panel_info *pinfo;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl->panel_data.panel_info;

	pinfo->partial_update_supported = of_property_read_bool(np,
		"qcom,partial-update-enabled");
	if (pinfo->mipi.mode == DSI_CMD_MODE) {
		pinfo->partial_update_enabled = pinfo->partial_update_supported;
		pr_info("%s: partial_update_enabled=%d\n", __func__,
					pinfo->partial_update_enabled);
		ctrl->set_col_page_addr = mdss_dsi_set_col_page_addr;
		if (pinfo->partial_update_enabled) {
			int rc = of_property_read_u32_array(np,
				"qcom,partial-update-addr-offset",
				value, 2);
			pinfo->partial_update_col_addr_offset =
				(!rc ? value[0] : 0);
			pinfo->partial_update_row_addr_offset =
				(!rc ? value[1] : 0);

			pinfo->partial_update_roi_merge =
					of_property_read_bool(np,
					"qcom,partial-update-roi-merge");
		}
	}

	pinfo->dcs_cmd_by_left = of_property_read_bool(np,
		"qcom,dcs-cmd-by-left");

	pinfo->ulps_feature_enabled = of_property_read_bool(np,
		"qcom,ulps-enabled");
	pr_info("%s: ulps feature %s\n", __func__,
		(pinfo->ulps_feature_enabled ? "enabled" : "disabled"));

	pinfo->ulps_suspend_enabled = of_property_read_bool(np,
		"qcom,suspend-ulps-enabled");
	pr_info("%s: ulps during suspend feature %s", __func__,
		(pinfo->ulps_suspend_enabled ? "enabled" : "disabled"));

	mdss_dsi_parse_dms_config(np, ctrl);

	pinfo->panel_ack_disabled = pinfo->sim_panel_mode ?
		1 : of_property_read_bool(np, "qcom,panel-ack-disabled");

	pinfo->allow_phy_power_off = of_property_read_bool(np,
		"qcom,panel-allow-phy-poweroff");

	mdss_dsi_parse_esd_params(np, ctrl);

	if (pinfo->panel_ack_disabled && pinfo->esd_check_enabled) {
		pr_warn("ESD should not be enabled if panel ACK is disabled\n");
		pinfo->esd_check_enabled = false;
	}

	if (ctrl->disp_en_gpio <= 0) {
		ctrl->disp_en_gpio = of_get_named_gpio(
			np,
			"qcom,5v-boost-gpio", 0);

		if (!gpio_is_valid(ctrl->disp_en_gpio))
			pr_debug("%s:%d, Disp_en gpio not specified\n",
					__func__, __LINE__);
	}

	mdss_dsi_parse_dcs_cmds(np, &ctrl->lp_on_cmds,
			"qcom,mdss-dsi-lp-mode-on", NULL);

	mdss_dsi_parse_dcs_cmds(np, &ctrl->lp_off_cmds,
			"qcom,mdss-dsi-lp-mode-off", NULL);

	return 0;
}

static void mdss_dsi_parse_panel_horizintal_line_idle(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	const u32 *src;
	int i, len, cnt;
	struct panel_horizontal_idle *kp;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return;
	}

	src = of_get_property(np, "qcom,mdss-dsi-hor-line-idle", &len);
	if (!src || len == 0)
		return;

	cnt = len % 3; /* 3 fields per entry */
	if (cnt) {
		pr_err("%s: invalid horizontal idle len=%d\n", __func__, len);
		return;
	}

	cnt = len / sizeof(u32);

	kp = kcalloc((cnt / 3), sizeof(*kp), GFP_KERNEL);
	if (kp == NULL)
		return;

	ctrl->line_idle = kp;
	for (i = 0; i < cnt; i += 3) {
		kp->min = be32_to_cpu(src[i]);
		kp->max = be32_to_cpu(src[i+1]);
		kp->idle = be32_to_cpu(src[i+2]);
		kp++;
		ctrl->horizontal_idle_cnt++;
	}

	/*
	 * idle is enabled for this controller, this will be used to
	 * enable/disable burst mode since both features are mutually
	 * exclusive.
	 */
	ctrl->idle_enabled = true;

	pr_debug("%s: horizontal_idle_cnt=%d\n", __func__,
				ctrl->horizontal_idle_cnt);
}

static int mdss_dsi_set_refresh_rate_range(struct device_node *pan_node,
		struct mdss_panel_info *pinfo)
{
	int rc = 0;

	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-min-refresh-rate",
			&pinfo->min_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read min refresh rate\n",
				__func__, __LINE__);

		/*
		 * If min refresh rate is not specified, set it to the
		 * default panel refresh rate.
		 */
		pinfo->min_fps = pinfo->mipi.frame_rate;
		rc = 0;
	}

	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-max-refresh-rate",
			&pinfo->max_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read max refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since max refresh rate was not specified when dynamic
		 * fps is enabled, using the default panel refresh rate
		 * as max refresh rate supported.
		 */
		pinfo->max_fps = pinfo->mipi.frame_rate;
		rc = 0;
	}

	pr_info("dyn_fps: min = %d, max = %d\n",
			pinfo->min_fps, pinfo->max_fps);
	return rc;
}

static void mdss_dsi_parse_dfps_config(struct device_node *pan_node,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	const char *data;
	bool dynamic_fps;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	dynamic_fps = of_property_read_bool(pan_node,
			"qcom,mdss-dsi-pan-enable-dynamic-fps");

	if (!dynamic_fps)
		return;

	pinfo->dynamic_fps = true;
	data = of_get_property(pan_node, "qcom,mdss-dsi-pan-fps-update", NULL);
	if (data) {
		if (!strcmp(data, "dfps_suspend_resume_mode")) {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("dfps mode: suspend/resume\n");
		} else if (!strcmp(data, "dfps_immediate_clk_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_CLK_UPDATE_MODE;
			pr_debug("dfps mode: Immediate clk\n");
		} else if (!strcmp(data, "dfps_immediate_porch_mode_hfp")) {
			pinfo->dfps_update =
				DFPS_IMMEDIATE_PORCH_UPDATE_MODE_HFP;
			pr_debug("dfps mode: Immediate porch HFP\n");
		} else if (!strcmp(data, "dfps_immediate_porch_mode_vfp")) {
			pinfo->dfps_update =
				DFPS_IMMEDIATE_PORCH_UPDATE_MODE_VFP;
			pr_debug("dfps mode: Immediate porch VFP\n");
		} else {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("default dfps mode: suspend/resume\n");
		}
		mdss_dsi_set_refresh_rate_range(pan_node, pinfo);
	} else {
		pinfo->dynamic_fps = false;
		pr_debug("dfps update mode not configured: disable\n");
	}
	pinfo->new_fps = pinfo->mipi.frame_rate;
	pinfo->current_fps = pinfo->mipi.frame_rate;
}

int mdss_panel_parse_bl_settings(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	const char *data;
	int rc = 0;
	u32 tmp;

	ctrl_pdata->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strcmp(data, "bl_ctrl_wled")) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strcmp(data, "bl_ctrl_pwm")) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			ctrl_pdata->pwm_pmi = of_property_read_bool(np,
					"qcom,mdss-dsi-bl-pwm-pmi");
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			if (ctrl_pdata->pwm_pmi) {
				ctrl_pdata->pwm_bl = of_pwm_get(np, NULL);
				if (IS_ERR(ctrl_pdata->pwm_bl)) {
					pr_err("%s: Error, pwm device\n",
								__func__);
					ctrl_pdata->pwm_bl = NULL;
					return -EINVAL;
				}
			} else {
				rc = of_property_read_u32(np,
					"qcom,mdss-dsi-bl-pmic-bank-select",
								 &tmp);
				if (rc) {
					pr_err("%s:%d, Error, lpg channel\n",
							__func__, __LINE__);
					return -EINVAL;
				}
				ctrl_pdata->pwm_lpg_chan = tmp;
				tmp = of_get_named_gpio(np,
					"qcom,mdss-dsi-pwm-gpio", 0);
				ctrl_pdata->pwm_pmic_gpio = tmp;
				pr_debug("%s: Configured PWM bklt ctrl\n",
								 __func__);
			}
		} else if (!strcmp(data, "bl_ctrl_dcs")) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
			data = of_get_property(np,
				"qcom,mdss-dsi-bl-dcs-command-state", NULL);
			if (data && !strcmp(data, "dsi_hs_mode"))
				ctrl_pdata->bklt_dcs_op_mode = DSI_HS_MODE;
			else
				ctrl_pdata->bklt_dcs_op_mode = DSI_LP_MODE;

			pr_debug("%s: Configured DCS_CMD bklt ctrl\n",
								__func__);
		}
	}
	return 0;
}

int mdss_dsi_panel_timing_switch(struct mdss_dsi_ctrl_pdata *ctrl,
			struct mdss_panel_timing *timing)
{
	struct dsi_panel_timing *pt;
	struct mdss_panel_info *pinfo = &ctrl->panel_data.panel_info;
	int i;

	if (!timing)
		return -EINVAL;

	if (timing == ctrl->panel_data.current_timing) {
		pr_warn("%s: panel timing \"%s\" already set\n", __func__,
				timing->name);
		return 0; /* nothing to do */
	}

	pr_debug("%s: ndx=%d switching to panel timing \"%s\"\n", __func__,
			ctrl->ndx, timing->name);

	mdss_panel_info_from_timing(timing, pinfo);

	pt = container_of(timing, struct dsi_panel_timing, timing);
	pinfo->mipi.t_clk_pre = pt->t_clk_pre;
	pinfo->mipi.t_clk_post = pt->t_clk_post;

	for (i = 0; i < ARRAY_SIZE(pt->phy_timing); i++)
		pinfo->mipi.dsi_phy_db.timing[i] = pt->phy_timing[i];

	for (i = 0; i < ARRAY_SIZE(pt->phy_timing_8996); i++)
		pinfo->mipi.dsi_phy_db.timing_8996[i] = pt->phy_timing_8996[i];

	for (i = 0; i < ARRAY_SIZE(pt->phy_timing_12nm); i++)
		pinfo->mipi.dsi_phy_db.timing_12nm[i] = pt->phy_timing_12nm[i];

	ctrl->on_cmds = pt->on_cmds;
	ctrl->post_panel_on_cmds = pt->post_panel_on_cmds;

	ctrl->panel_data.current_timing = timing;
	if (!timing->clk_rate)
		ctrl->refresh_clk_rate = true;
	mdss_dsi_clk_refresh(&ctrl->panel_data, ctrl->update_phy_timing);

	return 0;
}

void mdss_dsi_unregister_bl_settings(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (ctrl_pdata->bklt_ctrl == BL_WLED)
		led_trigger_unregister_simple(bl_led_trigger);
}
extern char *get_bbk_board_version(void);
static int mdss_dsi_panel_timing_from_dt(struct device_node *np,
		struct dsi_panel_timing *pt,
		struct mdss_panel_data *panel_data)
{
	u32 tmp;
	u64 tmp64;
	int rc, i, len;
	const char *data;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mdss_panel_info *pinfo;
	bool phy_timings_present = false;
	char *board_version = NULL;

	pinfo = &panel_data->panel_info;

	ctrl_pdata = container_of(panel_data, struct mdss_dsi_ctrl_pdata,
				panel_data);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pt->timing.xres = tmp;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}

	board_version = get_bbk_board_version();
	LCM_INFO("board_version=%s, board_version[0]=%c, board_version[1]=%c\n", board_version, board_version[0], board_version[1]);
	pt->timing.yres = tmp;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pt->timing.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pt->timing.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pt->timing.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pt->timing.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pt->timing.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pt->timing.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pt->timing.v_pulse_width = (!rc ? tmp : 2);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pt->timing.border_left = !rc ? tmp : 0;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	pt->timing.border_right = !rc ? tmp : 0;

	/* overriding left/right borders for split display cases */
	if (mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)) {
		if (panel_data->next)
			pt->timing.border_right = 0;
		else
			pt->timing.border_left = 0;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pt->timing.border_top = !rc ? tmp : 0;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	pt->timing.border_bottom = !rc ? tmp : 0;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate", &tmp);
	pt->timing.frame_rate = !rc ? tmp : DEFAULT_FRAME_RATE;
	rc = of_property_read_u64(np, "qcom,mdss-dsi-panel-clockrate", &tmp64);
	if (rc == -EOVERFLOW) {
		tmp64 = 0;
		rc = of_property_read_u32(np,
			"qcom,mdss-dsi-panel-clockrate", (u32 *)&tmp64);
	}
	pt->timing.clk_rate = !rc ? tmp64 : 0;

	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_debug("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
	} else {
		for (i = 0; i < len; i++)
			pt->phy_timing[i] = data[i];
		phy_timings_present = true;
	}

	data = of_get_property(np, "qcom,mdss-dsi-panel-timings-phy-v2", &len);
	if ((!data) || (len != 40)) {
		pr_debug("%s:%d, Unable to read 8996 Phy lane timing settings",
		       __func__, __LINE__);
	} else {
		for (i = 0; i < len; i++)
			pt->phy_timing_8996[i] = data[i];
		phy_timings_present = true;
	}

	data = of_get_property(np,
		"qcom,mdss-dsi-panel-timings-phy-12nm", &len);
	if ((!data) || (len != 8)) {
		pr_debug("%s:%d, Unable to read 12nm Phy lane timing settings",
		       __func__, __LINE__);
	} else {
		for (i = 0; i < len; i++)
			pt->phy_timing_12nm[i] = data[i];
		phy_timings_present = true;
	}

	if ((board_version[0] == '1') && (board_version[1] == '0') && (board_version[2] == '0') && (board_version[3] == '1')) {
		LCM_INFO("russia timing config \n");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch-russia", &tmp);
		if (!rc)
			pt->timing.h_front_porch = tmp;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch-russia", &tmp);
		if (!rc)
			pt->timing.h_back_porch = tmp;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch-russia", &tmp);
		if (!rc)
			pt->timing.v_back_porch = tmp;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch-russia", &tmp);
		if (!rc)
			pt->timing.v_front_porch = tmp;

		data = of_get_property(np,
			"qcom,mdss-dsi-panel-timings-phy-12nm-russia", &len);
		if ((!data) || (len != 8)) {
			LCM_ERR("Unable to read 12nm Phy lane timing settings\n");
		} else {
			for (i = 0; i < len; i++)
				pt->phy_timing_12nm[i] = data[i];
			phy_timings_present = true;
		}

	}

	if (!phy_timings_present) {
		pr_err("%s: phy timing settings not present\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pt->t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pt->t_clk_post = (!rc ? tmp : 0x03);

	if (np->name) {
		pt->timing.name = kstrdup(np->name, GFP_KERNEL);
		pr_info("%s: found new timing \"%s\" (%pK)\n", __func__,
				np->name, &pt->timing);
	}

	return 0;
}

static int  mdss_dsi_panel_config_res_properties(struct device_node *np,
		struct dsi_panel_timing *pt,
		struct mdss_panel_data *panel_data,
		bool default_timing)
{
	int rc = 0;

	mdss_dsi_parse_roi_alignment(np, pt);

	mdss_dsi_parse_dcs_cmds(np, &pt->on_cmds,
		"qcom,mdss-dsi-on-command",
		"qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &pt->post_panel_on_cmds,
		"qcom,mdss-dsi-post-panel-on-command", NULL);

	mdss_dsi_parse_dcs_cmds(np, &pt->switch_cmds,
		"qcom,mdss-dsi-timing-switch-command",
		"qcom,mdss-dsi-timing-switch-command-state");

	rc = mdss_dsi_parse_topology_config(np, pt, panel_data, default_timing);
	if (rc) {
		pr_err("%s: parsing compression params failed. rc:%d\n",
			__func__, rc);
		return rc;
	}

	mdss_panel_parse_te_params(np, &pt->timing);
	return rc;
}

static int mdss_panel_parse_display_timings(struct device_node *np,
		struct mdss_panel_data *panel_data)
{
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct dsi_panel_timing *modedb;
	struct device_node *timings_np;
	struct device_node *entry;
	int num_timings, rc;
	int i = 0, active_ndx = 0;
	bool default_timing = false;

	ctrl = container_of(panel_data, struct mdss_dsi_ctrl_pdata, panel_data);

	INIT_LIST_HEAD(&panel_data->timings_list);

	timings_np = of_get_child_by_name(np, "qcom,mdss-dsi-display-timings");
	if (!timings_np) {
		struct dsi_panel_timing pt;

		memset(&pt, 0, sizeof(struct dsi_panel_timing));

		/*
		 * display timings node is not available, fallback to reading
		 * timings directly from root node instead
		 */
		pr_debug("reading display-timings from panel node\n");
		rc = mdss_dsi_panel_timing_from_dt(np, &pt, panel_data);
		if (!rc) {
			mdss_dsi_panel_config_res_properties(np, &pt,
					panel_data, true);
			rc = mdss_dsi_panel_timing_switch(ctrl, &pt.timing);
		}
		return rc;
	}

	num_timings = of_get_child_count(timings_np);
	if (num_timings == 0) {
		pr_err("no timings found within display-timings\n");
		rc = -EINVAL;
		goto exit;
	}

	modedb = kcalloc(num_timings, sizeof(*modedb), GFP_KERNEL);
	if (!modedb) {
		rc = -ENOMEM;
		goto exit;
	}

	for_each_child_of_node(timings_np, entry) {
		rc = mdss_dsi_panel_timing_from_dt(entry, (modedb + i),
				panel_data);
		if (rc) {
			kfree(modedb);
			goto exit;
		}

		default_timing = of_property_read_bool(entry,
				"qcom,mdss-dsi-timing-default");
		if (default_timing)
			active_ndx = i;

		mdss_dsi_panel_config_res_properties(entry, (modedb + i),
				panel_data, default_timing);

		list_add(&modedb[i].timing.list,
				&panel_data->timings_list);
		i++;
	}

	/* Configure default timing settings */
	rc = mdss_dsi_panel_timing_switch(ctrl, &modedb[active_ndx].timing);
	if (rc)
		pr_err("unable to configure default timing settings\n");

exit:
	of_node_put(timings_np);

	return rc;
}

#ifdef TARGET_HW_MDSS_HDMI
static int mdss_panel_parse_dt_hdmi(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int len = 0;
	const char *bridge_chip_name;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	pinfo->is_dba_panel = of_property_read_bool(np,
			"qcom,dba-panel");

	if (pinfo->is_dba_panel) {
		bridge_chip_name = of_get_property(np,
			"qcom,bridge-name", &len);
		if (!bridge_chip_name || len <= 0) {
			pr_err("%s:%d Unable to read qcom,bridge_name, data=%pK,len=%d\n",
				__func__, __LINE__, bridge_chip_name, len);
			return -EINVAL;
		}
		strlcpy(ctrl_pdata->bridge_name, bridge_chip_name,
			MSM_DBA_CHIP_NAME_MAX_LEN);
	}
	return 0;
}
#else
static int mdss_panel_parse_dt_hdmi(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	(void)(*np);
	(void)(*ctrl_pdata);
	return 0;
}
#endif

unsigned int report_lcm_id(void)
{
	LCM_INFO("[%s] panel_id = %d\n", __func__, panel_id);
	return panel_id;
}

static int dsi_panel_parse_lcmid(struct device_node *np)
{
	int rc = 0;
	int i;
	u32 length = 0;
	u32 size = 0;
	u32 offset = 0;
	u32 *arr_32 = NULL;
	const u32 *arr;

	bl_hw_id = 0;
	panel_id = report_lcm_id();
	arr = of_get_property(np,
			"qcom,mdss-dsi-lcmid-sequence", &length);
	if (!arr) {
		LCM_ERR("dsi_panel_parse_lcmid not found\n");
		/* add for didn't set this property use panel id */
		composite_id = panel_id;
		rc = -EINVAL;
		goto error;
	}
	if (length & 0x1) {
		LCM_ERR("[%s] syntax error for dsi_panel_parse_lcmid\n",
		       __func__);
		rc = -EINVAL;
		goto error;
	}

	length = length / sizeof(u32);

	size = length * sizeof(u32);

	arr_32 = kzalloc(size, GFP_KERNEL);
	if (!arr_32) {
		rc = -ENOMEM;
		goto error;
	}

	rc = of_property_read_u32_array(np, "qcom,mdss-dsi-lcmid-sequence",
					arr_32, length);
	if (rc) {
		LCM_ERR("[%s] cannot read qcom,mdss-dsi-lcmid-sequence\n", __func__);
		/* add for didn't set this property use panel id */
		composite_id = panel_id;
		goto error_free_arr_32;
	}
	LCM_INFO("lcm id LENGTH = %d\n", length);

	for (i = 0; i < length; i += 2) {
		if (lcm_software_id == arr_32[i + 1]) {
			panel_id = arr_32[i];
			break;
		}
	}
	if (i >= length) {
		LCM_ERR("lcm id missmatch set defalut value FF\n");
		panel_id = 0xFF;
	}

	/*add for backlight id start*/
	rc = of_property_read_u32(np, "qcom,mdss-dsi-backlight-hw-id-offset", &offset);

	if (rc) {
		LCM_ERR("[%s] cannot read qcom,mdss-dsi-backlight-id-offset\n", __func__);
		goto error_free_arr_32;
	}

	if (offset > BBK_LCM_BL_ID_LEN - 1) {
		LCM_ERR("bl offset is over the bbk_board_version arrary\n");
		goto error_free_arr_32;
	}

	if (bbk_board_version[offset] == '0')
		bl_hw_id = 1; //second
	else
		bl_hw_id = 0; //first

	LCM_ERR("dsi bl_id %s, bl_hw_id %d, bl_id_length %d\n", bbk_board_version, bl_hw_id, offset);
	/*add for backlight id end*/

error_free_arr_32:
	kfree(arr_32);
error:
	LCM_ERR("panel_id 0x%02x\n", panel_id);
	composite_id = (bl_hw_id << 24) | panel_id;
	LCM_ERR("composite_id 0x%08x\n", composite_id);
	return rc;
}

static int panel_parse_vivo_panel_info(struct vivo_panel_info *vinfo)
{
	struct device_node *np = NULL;
	int i = 0, count = 0;
	const char *materiel_id = NULL;
	const char *lcm_ids = NULL;

	vinfo->num_panel_supply = 0;

#define VIVO_PANEL_INFO_NODE	"vivo,panel_info"
#define VIVO_PANEL_MATERIEL_IDS	"vivo,panel-materiel-id-list"
	np = of_find_node_by_name(NULL, VIVO_PANEL_INFO_NODE);
	if (!np) {
		LCM_ERR("%s node does not exist\n", VIVO_PANEL_INFO_NODE);
		return -ENODEV;
	}

	count = of_property_count_strings(np, VIVO_PANEL_MATERIEL_IDS);
	if (count < 0 || !count || count % 2) {
		LCM_ERR("%s node count %d invalid\n", VIVO_PANEL_MATERIEL_IDS, count);
		return -EINVAL;
	}

	count /= 2;
	vinfo->panel_supply = kzalloc(sizeof(struct vivo_panel_supply) * count, GFP_KERNEL);
	if (!vinfo->panel_supply)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		of_property_read_string_index(np, VIVO_PANEL_MATERIEL_IDS, i * 2, &materiel_id);
		of_property_read_string_index(np, VIVO_PANEL_MATERIEL_IDS, i * 2 + 1, &lcm_ids);

		vinfo->panel_supply[i].materiel_id = materiel_id;
		vinfo->panel_supply[i].lcm_ids = lcm_ids;
	}
	vinfo->num_panel_supply = count;

	return 0;
}

static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	u8 lanes = 0;
	int rc = 0;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data))
		pinfo->is_split_display = true;

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strcmp(data, "dsi_cmd_mode"))
		pinfo->mipi.mode = DSI_CMD_MODE;
	pinfo->mipi.boot_mode = pinfo->mipi.mode;
	tmp = 0;
	data = of_get_property(np, "qcom,mdss-dsi-pixel-packing", NULL);
	if (data && !strcmp(data, "loose"))
		pinfo->mipi.pixel_packing = 1;
	else
		pinfo->mipi.pixel_packing = 0;
	rc = mdss_panel_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, pinfo->mipi.pixel_packing,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(np,
		"qcom,mdss-dsi-panel-destination", NULL);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0x00);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-orientation", NULL);
	if (data) {
		pr_debug("panel orientation is %s\n", data);
		if (!strcmp(data, "180"))
			pinfo->panel_orientation = MDP_ROT_180;
		else if (!strcmp(data, "hflip"))
			pinfo->panel_orientation = MDP_FLIP_LR;
		else if (!strcmp(data, "vflip"))
			pinfo->panel_orientation = MDP_FLIP_UD;
	}

	rc = of_property_read_u32(np, "qcom,mdss-brightness-max-level", &tmp);
	pinfo->brightness_max = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");

	if (pinfo->sim_panel_mode == SIM_SW_TE_MODE)
		pinfo->mipi.hw_vsync_mode = false;
	else
		pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
			"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.last_line_interleave_en = of_property_read_bool(np,
		"qcom,mdss-dsi-last-line-interleave");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	data = of_get_property(np, "qcom,mdss-dsi-traffic-mode", NULL);
	if (data) {
		if (!strcmp(data, "non_burst_sync_event"))
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
		else if (!strcmp(data, "burst_mode"))
			pinfo->mipi.traffic_mode = DSI_BURST_MODE;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-continue", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-start", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);
	pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	data = of_get_property(np, "qcom,mdss-dsi-color-order", NULL);
	if (data) {
		if (!strcmp(data, "rgb_swap_rbg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RBG;
		else if (!strcmp(data, "rgb_swap_bgr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		else if (!strcmp(data, "rgb_swap_brg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BRG;
		else if (!strcmp(data, "rgb_swap_grb"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GRB;
		else if (!strcmp(data, "rgb_swap_gbr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GBR;
	}
	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	if (pinfo->mipi.data_lane0)
		lanes++;
	if (pinfo->mipi.data_lane1)
		lanes++;
	if (pinfo->mipi.data_lane2)
		lanes++;
	if (pinfo->mipi.data_lane3)
		lanes++;
	/*
	 * needed to set default lanes during
	 * resolution switch for bridge chips
	 */
	pinfo->mipi.default_lanes = lanes;

	dsi_panel_parse_lcmid(np);

	/* parse vivo panel materiel id and lcm id list */
	panel_parse_vivo_panel_info(&ctrl_pdata->vivo_panel_info);

	rc = mdss_panel_parse_display_timings(np, &ctrl_pdata->panel_data);
	if (rc)
		return rc;
	rc = mdss_dsi_parse_hdr_settings(np, pinfo);
	if (rc)
		return rc;

	pinfo->mipi.rx_eot_ignore = of_property_read_bool(np,
		"qcom,mdss-dsi-rx-eot-ignore");
	pinfo->mipi.tx_eot_append = of_property_read_bool(np,
		"qcom,mdss-dsi-tx-eot-append");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", NULL);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-mdp-transfer-time-us", &tmp);
	pinfo->mdp_transfer_time_us = (!rc ? tmp : DEFAULT_MDP_TRANSFER_TIME);

	mdss_dsi_parse_mdp_kickoff_threshold(np, pinfo);

	pinfo->mipi.lp11_init = of_property_read_bool(np,
					"qcom,mdss-dsi-lp11-init");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-init-delay-us", &tmp);
	pinfo->mipi.init_delay = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-post-init-delay", &tmp);
	pinfo->mipi.post_init_delay = (!rc ? tmp : 0);

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.mdp_trigger),
		"qcom,mdss-dsi-mdp-trigger");

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.dma_trigger),
		"qcom,mdss-dsi-dma-trigger");

	mdss_dsi_parse_reset_seq(np, pinfo->rst_seq, &(pinfo->rst_seq_len),
		"qcom,mdss-dsi-reset-sequence");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pre_off_cmds,
		"qcom,mdss-dsi-pre-off-command", "qcom,mdss-dsi-off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pre_on_cmds,
				"qcom,mdss-dsi-pre-on-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds_without_gesture,
		" qcom,mdss-dsi-off-command-without-gesture-wake", "qcom,mdss-dsi-off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->idle_on_cmds,
		"qcom,mdss-dsi-idle-on-command",
		"qcom,mdss-dsi-idle-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->idle_off_cmds,
		"qcom,mdss-dsi-idle-off-command",
		"qcom,mdss-dsi-idle-off-command-state");

	if ((lcm_software_id == 0x60) || (lcm_software_id == 0x63)
			|| (lcm_software_id == 0x74) || (lcm_software_id == 0x78)) {/* for ft8006p*/
		ctrl_pdata->tp_rst_gpio = of_get_named_gpio(np, "qcom,platform-tp_reset-gpio", 0);
		pr_err("%s: ctrl_pdata->tp_rst_gpio %d \n", __func__, ctrl_pdata->tp_rst_gpio);
		if (!gpio_is_valid(ctrl_pdata->tp_rst_gpio))
			pr_err("%s:%d, tp reset gpio not specified\n",
							__func__, __LINE__);
	}

	/*lcm cabc contrl*/
	if (lcm_software_id == 0x64) {
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cabc_level1,
			"qcom,mdss-dsi-on-cabc-levle1", "qcom,mdss-dsi-on-cabc-state");
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cabc_level2,
			"qcom,mdss-dsi-on-cabc-levle2", "qcom,mdss-dsi-on-cabc-state");
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cabc_level3,
			"qcom,mdss-dsi-on-cabc-levle3", "qcom,mdss-dsi-on-cabc-state");
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cabc_off,
			"qcom,mdss-dsi-on-cabc-off", "qcom,mdss-dsi-on-cabc-state");
	} else {
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cabc_level1,
			"qcom,mdss-dsi-on-cabc-levle1", "qcom,mdss-dsi-on-command-state");
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cabc_level2,
			"qcom,mdss-dsi-on-cabc-levle2", "qcom,mdss-dsi-on-command-state");
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cabc_level3,
			"qcom,mdss-dsi-on-cabc-levle3", "qcom,mdss-dsi-on-command-state");
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cabc_off,
			"qcom,mdss-dsi-on-cabc-off", "qcom,mdss-dsi-on-command-state");
	}
	/*lcm cabc end*/
	/*lcm_sre_max_level*/
	rc = of_property_read_u32(np, "qcom,mdss-dsi-lcm-sre-max-level", &tmp);
	ctrl_pdata->lcm_sre_max_level = (!rc ? tmp : 0);

	/*vivo_sre_enable*/
	ctrl_pdata->vivo_sre_enable = of_property_read_bool(np, "qcom,mdss-panel-sre");

	/*tp-esd-delay-ms*/
	rc = of_property_read_u32(np, "qcom,mdss-tp-esd-delay-ms", &tmp);
	interval = (!rc ? tmp : 2000);

	/*lcm backlight diming*/

	if (lcm_software_id == 0x64) {
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cmd_backlight_dimming_enable,
			"qcom,mdss-dsi-on-backlight-diming", "qcom,mdss-dsi-on-backlight-diming-state");
	} else {
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_cmd_backlight_dimming_enable,
			"qcom,mdss-dsi-on-backlight-diming", "qcom,mdss-dsi-on-command-state");
	}
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lcm_backlight_dimming_off_cmd,
		"qcom,mdss-dsi-dimming-off-command", "qcom,mdss-dsi-on-command-state");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-idle-fps", &tmp);
	pinfo->mipi.frame_rate_idle = (!rc ? tmp : 60);

	rc = of_property_read_u32(np, "qcom,adjust-timer-wakeup-ms", &tmp);
	pinfo->adjust_timer_delay_ms = (!rc ? tmp : 0);

	pinfo->mipi.force_clk_lane_hs = of_property_read_bool(np,
		"qcom,mdss-dsi-force-clock-lane-hs");

	pinfo->skip_panel_reset =
		of_property_read_bool(np, "qcom,mdss-skip-panel-reset");

	rc = mdss_dsi_parse_panel_features(np, ctrl_pdata);
	if (rc) {
		pr_err("%s: failed to parse panel features\n", __func__);
		goto error;
	}

	mdss_dsi_parse_panel_horizintal_line_idle(np, ctrl_pdata);

	mdss_dsi_parse_dfps_config(np, ctrl_pdata);

	rc = mdss_panel_parse_dt_hdmi(np, ctrl_pdata);
	if (rc)
		goto error;

	return 0;

error:
	return -EINVAL;
}

/* vivo display add for get bbk_board_version start*/
static int vivo_panel_parse_bootargs(void)
{
	const char *cmdline = NULL;
	static struct device_node *of_dtb_chosen;
	char *bbkboardversion = NULL;

	of_dtb_chosen = of_find_node_opts_by_path("/chosen", NULL);
	if (!of_dtb_chosen) {
		LCM_ERR(KERN_ERR "/chosen cannot be found\n");
		return -EINVAL;
	}

	if (of_property_read_string(of_dtb_chosen, "bootargs", &cmdline)) {
		LCM_ERR(KERN_ERR "bootargs read fail\n");
		return -EINVAL;
	}

	if (!cmdline) {
		LCM_ERR(KERN_ERR "cmdline is null\n");
		return -EFAULT;
	}

	bbkboardversion = strnstr(cmdline, bbk_board_version_str, strlen(cmdline));
	if (bbkboardversion) {
		strlcpy(bbk_board_version, bbkboardversion + strlen(bbk_board_version_str), sizeof(bbk_board_version));
		LCM_INFO("dsi bbk_board_version %s\n", bbk_board_version);
	} else {
		LCM_ERR("can't find bbk_board_version");
	}

	LCM_INFO("LCM cmdline:%s\n", cmdline);
	return 0;
}
/* vivo display add for get bbk_board_version end*/

static void dsi_panel_update_sre(unsigned int level)
{
	struct dcs_cmd_req cmdreq;
	static char reg_sre[2] = {0x97, 0x00};
	static struct dsi_cmd_desc reg_sre_cmd = {
		{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(reg_sre)}, reg_sre
	};

	static char NT36525_CMD22[2] = {0xFF, 0x22};
	static char NT36525_CMD10[2] = {0xFF, 0x10};
	static char NT36525_ON[2] = {0x55, 0x00};
	static char NT36525_LVL[2] = {0x5E, 0x00};

	static struct dsi_cmd_desc NT36525_set_sre_cmd[4] = {
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(NT36525_CMD22)}, NT36525_CMD22},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(NT36525_LVL)}, NT36525_LVL},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(NT36525_CMD10)}, NT36525_CMD10},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(NT36525_ON)}, NT36525_ON},
    };

	static char ILI9881H_CMD[4] = {0xFF, 0x98, 0x91, 0x00};
	static char ILI9881H_ON[2] = {0x55, 0x00};

	static struct dsi_cmd_desc ILI9881H_set_sre_cmd[4] = {
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(ILI9881H_CMD)}, ILI9881H_CMD},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(ILI9881H_ON)}, ILI9881H_ON},
    };

	static char ILI9882N_CMD[4] = {0xFF, 0x98, 0x82, 0x00};
	static char ILI9882N_ON[2] = {0x55, 0x00};

	static struct dsi_cmd_desc ILI9882N_set_sre_cmd[4] = {
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(ILI9882N_CMD)}, ILI9882N_CMD},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(ILI9882N_ON)}, ILI9882N_ON},
    };
	static char ILI9882H_CMD[4] = {0xFF, 0x98, 0x81, 0x00};
	static char ILI9882H_ON[2] = {0x55, 0x40};

	static struct dsi_cmd_desc ILI9882H_set_sre_cmd[4] = {
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(ILI9882H_CMD)}, ILI9882H_CMD},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(ILI9882H_ON)}, ILI9882H_ON},
    };

	if ((lcm_software_id == 0x60) || (lcm_software_id == 0x63)
			|| (lcm_software_id == 0x74) || (lcm_software_id == 0x78)) {

		if (level != 0)
			reg_sre[1] = (level & 0x7F) << 1 | 1;
		else
			reg_sre[1] = level;

		memset(&cmdreq, 0, sizeof(cmdreq));
		cmdreq.cmds = &reg_sre_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		if (g_ctrl_pdata->bklt_dcs_op_mode == DSI_HS_MODE)
			cmdreq.flags |= CMD_REQ_HS_MODE;
		else
			cmdreq.flags |= CMD_REQ_LP_MODE;

		mdss_dsi_cmdlist_put(g_ctrl_pdata, &cmdreq);
	} else if ((lcm_software_id == 0x61) || (lcm_software_id == 0x64) || (lcm_software_id == 0x6D)) {
		if (level != 0) {
			NT36525_LVL[1] = (level & 0xFF) | 0x10;
			NT36525_ON[1] = 4 << 4 | lcm_cabc_state;
		} else
			NT36525_ON[1] = 0 << 4 | lcm_cabc_state;

		memset(&cmdreq, 0, sizeof(cmdreq));
		cmdreq.cmds = NT36525_set_sre_cmd;
		cmdreq.cmds_cnt = 4;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		if (g_ctrl_pdata->bklt_dcs_op_mode == DSI_HS_MODE)
			cmdreq.flags |= CMD_REQ_HS_MODE;
		else
			cmdreq.flags |= CMD_REQ_LP_MODE;

		mdss_dsi_cmdlist_put(g_ctrl_pdata, &cmdreq);

	} else if (lcm_software_id == 0x75) {
		if (level != 0) {
			ILI9881H_ON[1] = 4 << 4 | lcm_cabc_state;
		} else
			ILI9881H_ON[1] = 0 << 4 | lcm_cabc_state;

		memset(&cmdreq, 0, sizeof(cmdreq));
		cmdreq.cmds = ILI9881H_set_sre_cmd;
		cmdreq.cmds_cnt = 2;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		if (g_ctrl_pdata->bklt_dcs_op_mode == DSI_HS_MODE)
			cmdreq.flags |= CMD_REQ_HS_MODE;
		else
			cmdreq.flags |= CMD_REQ_LP_MODE;

		mdss_dsi_cmdlist_put(g_ctrl_pdata, &cmdreq);
	} else if (lcm_software_id == 0x4F) {
		/*for ili9882n,sre write the upper four bits of the 0x55 register and have 3 levels*/
		/*0100,xxxx,sre low*/
		/*0101,xxxx,sre medium*/
		/*0110,xxxx,sre high*/
		if (level != 0) {
			ILI9882N_ON[1] = 4 << 4 | lcm_cabc_state;
		} else
			ILI9882N_ON[1] = 0 << 4 | lcm_cabc_state;

		memset(&cmdreq, 0, sizeof(cmdreq));
		cmdreq.cmds = ILI9882N_set_sre_cmd;
		cmdreq.cmds_cnt = 2;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		if (g_ctrl_pdata->bklt_dcs_op_mode == DSI_HS_MODE)
			cmdreq.flags |= CMD_REQ_HS_MODE;
		else
			cmdreq.flags |= CMD_REQ_LP_MODE;

		mdss_dsi_cmdlist_put(g_ctrl_pdata, &cmdreq);
	} else if (lcm_software_id == 0x45) {
		/*for ili9882H*/
		if (level != 0) {
			ILI9882H_ON[1] = 4 << 4 | lcm_cabc_state;
		} else
			ILI9882H_ON[1] = 0 << 4 | lcm_cabc_state;

		memset(&cmdreq, 0, sizeof(cmdreq));
		cmdreq.cmds = ILI9882H_set_sre_cmd;
		cmdreq.cmds_cnt = 2;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		if (g_ctrl_pdata->bklt_dcs_op_mode == DSI_HS_MODE)
			cmdreq.flags |= CMD_REQ_HS_MODE;
		else
			cmdreq.flags |= CMD_REQ_LP_MODE;

		mdss_dsi_cmdlist_put(g_ctrl_pdata, &cmdreq);
	}else
		LCM_INFO("not support sre, panel id = 0x%x\n", lcm_software_id);
}

static struct kobject disp_kobject;

unsigned int mdss_report_lcm_id(void)
{
	return report_lcm_id();
}
EXPORT_SYMBOL(mdss_report_lcm_id);

/* show lcm id */
static ssize_t lcm_id_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, LCMID_NODE_BUFERR_COUNT, "0x%08x\n", composite_id);
}

static ssize_t lcm_vddi_high_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	unsigned int vddi_high = 0;

	/* extra standard  for  FT8006P non-flash current  at AT mode*/
	if ((lcm_software_id == 0x60) || (lcm_software_id == 0x63) || (lcm_software_id == 0x78) || (lcm_software_id == 0x74) || (lcm_software_id == 0x75))
			vddi_high = 0x01;
	return snprintf(buf, 4, "%02x\n", vddi_high);
}

/*esd check*/
static ssize_t vivo_esd_check_show(struct kobject *kobj,
								struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 3, "%02x\n", vivo_esd_check_ps_status);
}

static ssize_t vivo_esd_check_store(struct kobject *kobj,
							struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, temp;
	ret = kstrtoint(buf, 10, &temp);
	if (ret) {
		LCM_ERR("Invalid input for lcm esd check\n");
	}
	vivo_esd_check_ps_status = temp;
	LCM_INFO("lcm esd check set to %d\n", vivo_esd_check_ps_status);
	return count;
}

static ssize_t vivo_esd_check_enable_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	if ((lcm_software_id == 0x60) || (lcm_software_id == 0x63)
			|| (lcm_software_id == 0x74) || (lcm_software_id == 0x78))  /* for ft8006p*/
		vivo_esd_check_enable_status = 0;
	else
		vivo_esd_check_enable_status = 1;
	return snprintf(buf, 3, "%d", vivo_esd_check_enable_status);
}

static ssize_t vivo_esd_check_enable_store(struct kobject *kobj,
						struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, temp;
	ret = kstrtoint(buf, 10, &temp);
	if (ret) {
		LCM_ERR("Invalid input for vivo_esd_check_enable_store\n");
	}
	if (lcmpr_debug_enabled == 2)
		vivo_esd_check_enable_status = temp;
	return count;
}
/*esd check end*/

/* SRE enable check begin */
static ssize_t vivo_sre_check_enable_show(struct kobject *kobj,
					  struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%02x\n", g_ctrl_pdata->vivo_sre_enable);
}

static ssize_t vivo_sre_check_enable_store(struct kobject *kobj,
					  struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, temp;
	ret = kstrtoint(buf, 10, &temp);
	if (ret) {
		LCM_ERR("Invalid input for vivo_sre_check_enable_store\n");
	}
	g_ctrl_pdata->vivo_sre_enable = temp;
	LCM_INFO("vivo_sre_check_enable_store set to %d\n", g_ctrl_pdata->vivo_sre_enable);
	return count;
}

/* SRE enable check end */


/* show lcm sre max level */
static ssize_t lcm_sre_max_level_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%02x\n", g_ctrl_pdata->lcm_sre_max_level);
}

/* show  sunlight enhance  SRE mode state*/
static ssize_t lcm_sre_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%02x\n", lcm_sre_state);
}

static ssize_t lcm_sre_store(struct kobject *kobj,
						struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, temp;
	ret = kstrtoint(buf, 10, &temp);
	if (ret) {
		LCM_ERR("Invalid input for lcm_sre_store\n");
		return count;
	}

	if (!(g_ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT))	{
		pr_err("Invalid panel state\n");
		return count;
	}

	if ((!pre_bklt_level) || (atomic_read(&disable_sre_setting)) == 1) {
			LCM_ERR("Invalid panel backlight level 0\n");
			return count;
	}

	if (pre_bklt_level < panel_max_bl_level -1) {
		lcm_sre_state = 0;
		LCM_INFO("Invalid panel backlight level(%d) < sre_min_brightness(%d)\n",
			pre_bklt_level, panel_max_bl_level -1);
		return count;
	}

	lcm_sre_state = temp;
	LCM_INFO("will set sre from 0x%x to 0x%x\n", pre_lcm_sre, lcm_sre_state);

	if (pre_lcm_sre != lcm_sre_state) {
		if (lcm_sre_state > g_ctrl_pdata->lcm_sre_max_level)
			lcm_sre_state = g_ctrl_pdata->lcm_sre_max_level;

		dsi_panel_update_sre(lcm_sre_state);
	}

	pre_lcm_sre = lcm_sre_state;
	return count;
}
/* show oled acl mode state*/
static ssize_t lcm_cabc_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%x\n", lcm_cabc_state);
}
static ssize_t lcm_cabc_store(struct kobject *kobj,
						struct kobj_attribute *attr, const char *buf, size_t count)
{
    int ret, temp;
    ret = kstrtoint(buf, 10, &temp);
    if (ret) {
		pr_err("Invalid input for lcm_cabc_store\n");
		return count;
	}
	if (!(g_ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) || !pre_bklt_level) {
		pr_err("Invalid panel state,  not to set cabc, backlight is %d\n", pre_bklt_level);
		return count;
	}

	lcm_cabc_state = temp;

	LCM_INFO("will set lcm_cabc_status to %d\n", lcm_cabc_state);

	pre_lcm_cabc = lcm_cabc_state;
	dsi_panel_update_cabc(pre_bklt_level, lcm_cabc_state);

	return count;
}

/* show BL diming state*/
static ssize_t lcm_diming_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%x\n", dimming_enable);
}
static ssize_t lcm_diming_store(struct kobject *kobj,
						struct kobj_attribute *attr, const char *buf, size_t count)
{
    int ret, temp;
    ret = kstrtoint(buf, 10, &temp);
    if (ret) {
		pr_err("Invalid input for lcm_diming_store\n");
		return count;
	}

	pr_err("%s: vincent+dimming_enable=%d, temp = %d, lcm_software_id=0x%x\n", __func__, dimming_enable, temp, lcm_software_id);
	return count;
}


/*For LCM max brightness standard:
 1:low brightness status--320,360; 0:more than 400 brightness*/
static ssize_t lcm_brightness_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{

	unsigned int brighness = 0;

	return snprintf(buf, 4, "%2x\n", brighness);
}

static ssize_t materiel_id_show(struct kobject *kobj,
					  struct kobj_attribute *attr, char *buf)
{
	struct vivo_panel_info *vinfo = NULL;
	ssize_t count = 0;
	int i = 0;

	vinfo = &g_ctrl_pdata->vivo_panel_info;

	if (!vinfo->panel_supply || !vinfo->num_panel_supply)
		return LCMID_NODE_BUFERR_COUNT;

	for (i = 0; i < vinfo->num_panel_supply; i++)
		count += snprintf(buf + count, PAGE_SIZE - count, "%s\t%s\n",
				vinfo->panel_supply[i].materiel_id, vinfo->panel_supply[i].lcm_ids);

	return count;
}

static struct kobj_attribute lcm_id_attribute = __ATTR(lcm_id, 0444, lcm_id_show, NULL);
static struct kobj_attribute lcm_cabc_attribute = __ATTR(lcm_cabc, 0644, lcm_cabc_show, lcm_cabc_store);
static struct kobj_attribute lcm_brightness_attribute = __ATTR(lcm_brightness, 0444, lcm_brightness_show, NULL);
static struct kobj_attribute lcm_diming_attribute = __ATTR(lcm_diming, 0644, lcm_diming_show, lcm_diming_store);
static struct kobj_attribute lcm_vddi_high_attribute = __ATTR(lcm_vddi_high, 0644, lcm_vddi_high_show, NULL);
static struct kobj_attribute vivo_esd_check_ps_attribute = __ATTR(vivo_esd_check_ps, 0644, vivo_esd_check_show, vivo_esd_check_store);
static struct kobj_attribute lcm_sre_attribute = __ATTR(lcm_sre, 0644, lcm_sre_show, lcm_sre_store);
static struct kobj_attribute lcm_sre_max_attribute = __ATTR(lcm_sre_max_level, 0444, lcm_sre_max_level_show, NULL);
static struct kobj_attribute vivo_sre_check_enable_attribute = __ATTR(sre_enable, 0644, vivo_sre_check_enable_show, vivo_sre_check_enable_store);
static struct kobj_attribute materiel_id_attribute = __ATTR(materiel_id, 0444, materiel_id_show, NULL);
static struct kobj_attribute vivo_esd_check_enable_attribute = __ATTR(vivo_esd_check_enable, 0644, vivo_esd_check_enable_show, vivo_esd_check_enable_store);

static struct attribute *disp_lcm_sys_attrs[] = {
	&lcm_id_attribute.attr,
	&lcm_vddi_high_attribute.attr,
	&lcm_cabc_attribute.attr,
	&lcm_brightness_attribute.attr,
	&lcm_diming_attribute.attr,
	&vivo_esd_check_ps_attribute.attr,
	&lcm_sre_attribute.attr,
	&lcm_sre_max_attribute.attr,
	&vivo_sre_check_enable_attribute.attr,
	&materiel_id_attribute.attr,
	&vivo_esd_check_enable_attribute.attr,
	NULL
};

static ssize_t disp_lcm_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t disp_lcm_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void disp_lcm_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops disp_lcm_object_sysfs_ops = {
	.show = disp_lcm_object_show,
	.store = disp_lcm_object_store,
};

static struct kobj_type disp_lcm_object_type = {
	.sysfs_ops	= &disp_lcm_object_sysfs_ops,
	.release	= disp_lcm_object_release,
	.default_attrs = disp_lcm_sys_attrs,
};

static int disp_creat_sys_file(void)
{
	memset(&disp_kobject, 0x00, sizeof(disp_kobject));
	if (kobject_init_and_add(&disp_kobject,
		&disp_lcm_object_type, NULL, "lcm")) {
		kobject_put(&disp_kobject);
		return -ENOMEM;
		}
	kobject_uevent(&disp_kobject, KOBJ_ADD);
	return 0;
}
 /* add lcm sysfs interface end*/
int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	int ndx)
{
	int rc = 0;
	static const char *panel_name,  *project_name_tmp;
	struct mdss_panel_info *pinfo;

	if (!node || !ctrl_pdata) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	vivo_panel_parse_bootargs();

	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_debug("%s:%d\n", __func__, __LINE__);
	pinfo->panel_name[0] = '\0';
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name) {
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	} else {
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);
		strlcpy(&pinfo->panel_name[0], panel_name, MDSS_MAX_PANEL_LEN);
	}

	project_name_tmp = of_get_property(node, "qcom,mdss-dsi-project-name", NULL);/* add for project Identify*/
	if (!project_name_tmp) {
		pr_info("%s:%d, Project name not specified\n",
						__func__, __LINE__);
	} else {
		strlcpy(&project_name[0], project_name_tmp, MDSS_MAX_PANEL_LEN);
		pr_info("%s: Project Name = %s\n", __func__, project_name);
	}
	rc = of_property_read_u32(node, "qcom,mdss-dsi-panel-id", &panel_id); /*for #*225# LCM ID*/
      if (rc) {
		pr_err("%s:%d panel id dt parse failed\n", __func__, __LINE__);
	} else
		pr_info("%s: Panel ID = 0x%x\n", __func__, panel_id);
	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	pinfo->dynamic_switch_pending = false;
	pinfo->is_lpm_mode = false;
	pinfo->esd_rdy = false;
	pinfo->persist_mode = false;

	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->post_panel_on = mdss_dsi_post_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->low_power_config = mdss_dsi_panel_low_power_config;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;
	ctrl_pdata->panel_data.apply_display_setting =
			mdss_dsi_panel_apply_display_setting;
	ctrl_pdata->switch_mode = mdss_dsi_panel_switch_mode;
	ctrl_pdata->panel_data.get_idle = mdss_dsi_panel_get_idle_mode;
	disp_creat_sys_file(); /* add lcm sysfs id interface*/
	if (!g_ctrl_pdata)
		g_ctrl_pdata = ctrl_pdata;
	return 0;
}
