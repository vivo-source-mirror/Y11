/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2018, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_esdcheck.c
*
*    Author: Focaltech Driver Team
*
*   Created: 2016-08-03
*
*  Abstract: ESD check function
*
*   Version: v1.0
*
* Revision History:
*        v1.0:
*            First release. By luougojin 2016-08-03
*        v1.1: By luougojin 2017-02-15
*            1. Add LCD_ESD_PATCH to control idc_esdcheck_lcderror
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

#if FTS_ESDCHECK_EN
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define ESDCHECK_WAIT_TIME              1000    /* ms */
#define LCD_ESD_PATCH                   1

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct fts_esdcheck_st {
    u8      mode                : 1;    /* 1- need check esd 0- no esd check */
    u8      suspend             : 1;
    u8      proc_debug          : 1;    /* apk or adb use */
    u8      intr                : 1;    /* 1- Interrupt trigger */
    u8      unused              : 4;
    u8      flow_work_hold_cnt;         /* Flow Work Cnt(reg0x91) keep a same value for x times. >=5 times is ESD, need reset */
    u8      flow_work_cnt_last;         /* Save Flow Work Cnt(reg0x91) value */
    u32     hardware_reset_cnt;
    u32     nack_cnt;
    u32     dataerror_cnt;
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_esdcheck_st fts_esdcheck_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/*****************************************************************************
* functions body
*****************************************************************************/
#if LCD_ESD_PATCH
/*****************************************************************************
*  Name: lcd_esdcheck
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
extern int mdss_dsi_panel_reset_and_powerctl(int enable);

int lcd_need_reset;
int lcd_need_recovery;
static int tp_need_recovery; /* LCD reset cause Tp reset */
int idc_esdcheck_lcderror(struct fts_ts_data *ts_data)
{
    int ret = 0;
    u8 val = 0;

    VTD("[ESD]Check LCD ESD");
    if ( (tp_need_recovery == 1) && (lcd_need_reset == 0) ) {
        tp_need_recovery = 0;
        /* LCD reset, need recover TP state */
        fts_release_all_finger();
        fts_tp_state_recovery();
    }

    ret = fts_read_reg_byte(FTS_REG_ESD_SATURATE, &val);
    if ( ret < 0) {
        VTE("[ESD]: Read ESD_SATURATE(0xED) failed ret=%d!", ret);
        return -EIO;
    }

    if (val == 0xAA) {
        /*
        * 1. Set flag lcd_need_reset = 1;
        * 2. LCD driver need reset(recovery) LCD and set lcd_need_reset to 0
        * 3. recover TP state
        */
        VTI("LCD ESD, Execute Panel Recovery!");
		lcd_need_reset = 1;
        tp_need_recovery = 1;
    }

    return 0;
}
#endif

/*****************************************************************************
*  Name: fts_esdcheck_tp_reset
*  Brief: esd check algorithm
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_esdcheck_tp_reset(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();

    fts_esdcheck_data.flow_work_hold_cnt = 0;
    fts_esdcheck_data.hardware_reset_cnt++;

    fts_reset_proc(200);
    fts_release_all_finger();
    fts_tp_state_recovery();

    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: get_chip_id
*  Brief: Read Chip Id 3 times
*  Input:
*  Output:
*  Return:  1(ture) - Read Chip Id 3 times failed
*           0(false) - Read Chip Id pass
*****************************************************************************/
static bool get_chip_id(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int i = 0;
    u8 reg_value = 0;
    u8 reg_addr = 0;
    u8 chip_id = ts_data->ic_info.ids.chip_idh;

    for (i = 0; i < 3; i++) {
        reg_addr = FTS_REG_CHIP_ID;
        ret = fts_read_reg_byte(reg_addr, &reg_value);
        if (ret < 0) {
            VTE("[ESD]: Read Reg 0xA3 failed ret = %d!!", ret);
            fts_esdcheck_data.nack_cnt++;
        } else {
            if (reg_value == chip_id) {
                break;
            } else {
                fts_esdcheck_data.dataerror_cnt++;
            }
        }
        msleep(10);
    }

    /* if can't get correct data in 3 times, then need hardware reset */
    if (i >= 3) {
        VTE("[ESD]: Read Chip id 3 times failed, need execute TP reset!!");
        return true;
    }

    return false;
}

/*****************************************************************************
*  Name: get_flow_cnt
*  Brief: Read flow cnt(0x91)
*  Input:
*  Output:
*  Return:  1(true) - Reg 0x91(flow cnt) abnormal: hold a value for 5 times
*           0(false) - Reg 0x91(flow cnt) normal
*****************************************************************************/
static bool get_flow_cnt(struct fts_ts_data *ts_data)
{
    int     ret = 0;
    u8      reg_value = 0;
    u8      reg_addr = 0;

    reg_addr = FTS_REG_FLOW_WORK_CNT;
    ret = fts_read_reg_byte(reg_addr, &reg_value);
    if (ret < 0) {
        VTE("[ESD]: Read Reg 0x91 failed ret = %d!!", ret);
        fts_esdcheck_data.nack_cnt++;
    } else {
        if ( reg_value == fts_esdcheck_data.flow_work_cnt_last ) {
            fts_esdcheck_data.flow_work_hold_cnt++;
        } else {
            fts_esdcheck_data.flow_work_hold_cnt = 0;
        }

        fts_esdcheck_data.flow_work_cnt_last = reg_value;
    }

    /* if read flow work cnt 5 times and the value are all the same, then need hardware_reset */
    if (fts_esdcheck_data.flow_work_hold_cnt >= 5) {
        VTD("[ESD]: Flow Work Cnt(reg0x91) keep a value for 5 times, need execute TP reset!!");
        return true;
    }

    return false;
}

/*****************************************************************************
*  Name: esdcheck_algorithm
*  Brief: esd check algorithm
*  Input:
*  Output:
*  Return:
*****************************************************************************/
extern int incell_avdd_avee_output(int status);
static int esdcheck_algorithm(struct fts_ts_data *ts_data)
{
	int     err = 0;
	int     err1 = 0;
	u8      reg_value = 0;
	u8      reg_addr = 0;
	u8      temp_data = 0;
	bool    hardware_reset = 0;	
	
	/*check register 0xc8*/
	//VTI("enter in register 0xc8!!!");
	reg_addr = 0xc8;
	err = fts_read_reg_byte(reg_addr, &reg_value);
	if (err < 0) {
		VTI("read register 0xc8 fail!!!!");
		fts_esdcheck_data.nack_cnt++;
	} else if (reg_value ==  1) {
		VTI("enter in function diadian!!!!!");
		mdss_dsi_panel_reset_and_powerctl(6);
		//msleep(2000);
		mdss_dsi_panel_reset_and_powerctl(7);
		err1 = fts_write_reg_byte(0xc9, 1);
		if (err1 < 0) {
			VTI("fail to wirte 0xc9 register 1 !!!");
		}
		mdelay(5);
		err1 = fts_read_reg_byte(0xc9, &temp_data);
		if (err1 < 0)
			VTI("fail to read 0xc9 register %d",temp_data);
		if (temp_data != 1) {
			VTI("Fail to write 0xc9 = 1");
		} else {
			VTI("FTS : Success to write 0xc9= 1");
		}
		
	} else if (reg_value ==  2) {
		VTI("enter in function display recovery!!!!!");
		lcd_need_recovery = 1;
	}

	/* 1. esdcheck is interrupt, then return */
    if (fts_esdcheck_data.intr == 1) { 
		 VTD("[ESD]: In interrupt state, not check esd, return immediately!!");
         return 0;
    }

    /* 2. check power state, if suspend, no need check esd */
    if (fts_esdcheck_data.suspend == 1) {
        VTD("[ESD]: In suspend, not check esd, return immediately!!");
        /* because in suspend state, adb can be used, when upgrade FW, will active ESD check(active = 1)
        *  But in suspend, then will don't queue_delayed_work, when resume, don't check ESD again
        */
        return 0;
    }

    /* 3. check fts_esdcheck_data.proc_debug state, if 1-proc busy, no need check esd*/
    if (fts_esdcheck_data.proc_debug == 1) {
        VTI("[ESD]: In apk or adb command mode, not check esd, return immediately!!");
        return 0;
    }

    /* 4. In factory mode, can't check esd */
    reg_addr = FTS_REG_WORKMODE;
    err = fts_read_reg_byte(reg_addr, &reg_value);
    if ( err < 0 ) {
        fts_esdcheck_data.nack_cnt++;
    } else if ( (reg_value & 0x70) !=  FTS_REG_WORKMODE_WORK_VALUE) {
        VTD("[ESD]: not in work mode, no check esd, return immediately!!");
        return 0;
    }

    /* 5. IDC esd check lcd  default:close */
#if LCD_ESD_PATCH
    idc_esdcheck_lcderror(ts_data);
#endif

    /* 6. Get Chip ID */
    hardware_reset = get_chip_id(ts_data);

    /* 7. get Flow work cnt: 0x91 If no change for 5 times, then ESD and reset */
    if (!hardware_reset) {
        hardware_reset = get_flow_cnt(ts_data);
    }

    /* 8. If need hardware reset, then handle it here */
    if ( hardware_reset == 1) {
        fts_esdcheck_tp_reset(ts_data);
    }

    VTD("[ESD]: NoACK=%d, Error Data=%d, Hardware Reset=%d", fts_esdcheck_data.nack_cnt, fts_esdcheck_data.dataerror_cnt, fts_esdcheck_data.hardware_reset_cnt);
    return 0;
}

/*****************************************************************************
*  Name: fts_esdcheck_func
*  Brief: fts_esdcheck_func
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void esdcheck_func(struct work_struct *work)
{
    struct fts_ts_data *ts_data = container_of(work,
                                  struct fts_ts_data, esdcheck_work.work);

    //FTS_FUNC_ENTER();
    VTD("Enter esdcheck_func !");
    if (ENABLE == fts_esdcheck_data.mode) {
        esdcheck_algorithm(ts_data);
        queue_delayed_work(ts_data->ts_workqueue, &ts_data->esdcheck_work,
                           msecs_to_jiffies(ESDCHECK_WAIT_TIME));
    }
    //FTS_FUNC_EXIT();
    VTD("Exit esdcheck_func !");
}

/*****************************************************************************
*  Name: fts_esdcheck_set_intr
*  Brief: interrupt flag (main used in interrupt tp report)
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_esdcheck_set_intr(bool intr)
{
    /* interrupt don't add debug message */
    fts_esdcheck_data.intr = intr;
    return 0;
}

/*****************************************************************************
*  Name: fts_esdcheck_get_status(void)
*  Brief: get current status
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_esdcheck_get_status(void)
{
    /* interrupt don't add debug message */
    return fts_esdcheck_data.mode;
}

/*****************************************************************************
*  Name: fts_esdcheck_proc_busy
*  Brief: When APK or ADB command access TP via driver, then need set proc_debug,
*         then will not check ESD.
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_esdcheck_proc_busy(bool proc_debug)
{
    fts_esdcheck_data.proc_debug = proc_debug;
    return 0;
}

/*****************************************************************************
*  Name: fts_esdcheck_switch
*  Brief: FTS esd check function switch.
*  Input:   enable:  1 - Enable esd check
*                    0 - Disable esd check
*  Output:
*  Return:
*****************************************************************************/
int fts_esdcheck_switch(bool enable)
{
    struct fts_ts_data *ts_data = fts_data;
    FTS_FUNC_ENTER();
    if (fts_esdcheck_data.mode == ENABLE) {
        if (enable) {
            VTD("[ESD]: ESD check start!!");
            fts_esdcheck_data.flow_work_hold_cnt = 0;
            fts_esdcheck_data.flow_work_cnt_last = 0;
            queue_delayed_work(ts_data->ts_workqueue, &ts_data->esdcheck_work,
                               msecs_to_jiffies(ESDCHECK_WAIT_TIME));
        } else {
            VTD("[ESD]: ESD check stop!!");
            cancel_delayed_work(&ts_data->esdcheck_work);
        }
    } else {
        VTD("[ESD]: ESD should disable!!");
        cancel_delayed_work(&ts_data->esdcheck_work);
    }

    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: fts_esdcheck_suspend
*  Brief: Run when tp enter into suspend
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_esdcheck_suspend(void)
{
    FTS_FUNC_ENTER();
    fts_esdcheck_switch(DISABLE);
    fts_esdcheck_data.suspend = 1;
    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: fts_esdcheck_resume
*  Brief: Run when tp resume
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_esdcheck_resume( void )
{
    FTS_FUNC_ENTER();
    fts_esdcheck_switch(ENABLE);
    fts_esdcheck_data.suspend = 0;
    FTS_FUNC_EXIT();
    return 0;
}

/************************************************************************
* Name: fts_esdcheck_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_esdcheck_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        VTD("enable esdcheck");
        fts_esdcheck_data.mode = ENABLE;
        fts_esdcheck_switch(ENABLE);
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        VTD("disable esdcheck");
        fts_esdcheck_data.mode = DISABLE;
        fts_esdcheck_switch(DISABLE);
    }
    mutex_unlock(&input_dev->mutex);

    return count;
}

/************************************************************************
* Name: fts_esdcheck_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_esdcheck_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    count = snprintf(buf, PAGE_SIZE, "Esd check: %s\n", \
                     fts_esdcheck_get_status() ? "On" : "Off");
    mutex_unlock(&input_dev->mutex);

    return count;
}

/* sysfs esd node
 *   read example: cat  fts_esd_mode        ---read esd mode
 *   write example:echo 01 > fts_esd_mode   ---make esdcheck enable
 *
 */
static DEVICE_ATTR (fts_esd_mode, S_IRUGO | S_IWUSR, fts_esdcheck_show, fts_esdcheck_store);

static struct attribute *fts_esd_mode_attrs[] = {

    &dev_attr_fts_esd_mode.attr,
    NULL,
};

static struct attribute_group fts_esd_group = {
    .attrs = fts_esd_mode_attrs,
};
/*****************************************************************************
*   Name: fts_create_gesture_sysfs
*  Brief:
*  Input:
* Output:
* Return: 0-success or others-error
*****************************************************************************/
int fts_create_esd_sysfs(struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &fts_esd_group);
    if (ret) {
        VTE( "esd sys node create fail");
        sysfs_remove_group(&dev->kobj, &fts_esd_group);
        return ret;
    }
    return 0;
}

/*****************************************************************************
*  Name: fts_esdcheck_init
*  Brief: Init and create a queue work to check esd
*  Input:
*  Output:
*  Return: < 0: Fail to create esd check queue
*****************************************************************************/
int fts_esdcheck_init(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    if (ts_data->ts_workqueue) {
        INIT_DELAYED_WORK(&ts_data->esdcheck_work, esdcheck_func);
    } else {
        VTE("fts workqueue is NULL, can't run esd check function");
        return -EINVAL;
    }

    memset((u8 *)&fts_esdcheck_data, 0, sizeof(struct fts_esdcheck_st));
    fts_esdcheck_data.mode = ENABLE;
    fts_esdcheck_switch(ENABLE);
    fts_create_esd_sysfs(&ts_data->spi->dev);
    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: fts_esdcheck_exit
*  Brief: When FTS TP driver is removed, then call this function to destory work queue
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_esdcheck_exit(struct fts_ts_data *ts_data)
{
    sysfs_remove_group(&ts_data->spi->dev.kobj, &fts_esd_group);
    return 0;
}
#endif /* FTS_ESDCHECK_EN */

