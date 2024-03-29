/*
 *  linux/drivers/mmc/core/mmc_vsm.c
 *
 *  Copyright (C) 2017 Ruyi Liu, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC card vendor support mode
 */

#include "Samsung_VSM_PWD.h"
#include "mmc_vsm.h"
#include "core.h"
#include "mmc_ops.h"

/*
 * Checks that a normal transfer didn't have any errors
 */
static int mmc_test_check_result(struct mmc_card *card,
				 struct mmc_request *mrq)
{
	int ret;

	BUG_ON(!mrq || !mrq->cmd || !mrq->data);

	ret = 0;

	if (!ret && mrq->cmd->error)
		ret = mrq->cmd->error;
	if (!ret && mrq->data->error)
		ret = mrq->data->error;
	if (!ret && mrq->stop && mrq->stop->error)
		ret = mrq->stop->error;
	if (!ret && mrq->data->bytes_xfered !=
		mrq->data->blocks * mrq->data->blksz)
		ret = RESULT_FAIL;

	if (ret == -EINVAL)
		ret = RESULT_UNSUP_HOST;

	return ret;
}

static int mmc_test_busy(struct mmc_command *cmd)
{
	return !(cmd->resp[0] & R1_READY_FOR_DATA) ||
		(R1_CURRENT_STATE(cmd->resp[0]) == R1_STATE_PRG);
}

/*
 * Wait for the card to finish the busy state
 */
static int mmc_test_wait_busy(struct mmc_card *card)
{
	int ret, busy;
	struct mmc_command cmd = {0};

	busy = 0;
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));

		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (ret)
			break;

		if (!busy && mmc_test_busy(&cmd)) {
			busy = 1;
			if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY)
				pr_info("%s: Warning: Host did not "
					"wait for busy state to end.\n",
					mmc_hostname(card->host));
		}
	} while (mmc_test_busy(&cmd));

	return ret;
}


/*
 * Fill in the mmc_request structure given a set of transfer parameters.
 */
static void mmc_test_prepare_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
	unsigned dev_addr, unsigned blocks, unsigned blksz, int write)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1) {
		mrq->cmd->opcode = write ?
			MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	} else {
		mrq->cmd->opcode = write ?
			MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
	}

	mrq->cmd->arg = dev_addr;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;

	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1)
		mrq->stop = NULL;
	else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

/*
 * Tests a basic transfer with certain parameters
 */
int mmc_test_simple_transfer(struct mmc_card *card,
	struct scatterlist *sg, unsigned sg_len, unsigned dev_addr,
	unsigned blocks, unsigned blksz, int write)
{
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_command stop = {0};
	struct mmc_data data = {0};

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;

	mmc_test_prepare_mrq(card, &mrq, sg, sg_len, dev_addr,
		blocks, blksz, write);

	mmc_wait_for_req(card->host, &mrq);

	mmc_test_wait_busy(card);

	return mmc_test_check_result(card, &mrq);
}

/*
 * Fill in the mmc_request structure given a set of transfer parameters.
 */
static void mmc_test_prepare_general_command_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
	unsigned dev_addr, unsigned blocks, unsigned blksz, int write)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	mrq->cmd->opcode = MMC_GEN_CMD;

	mrq->cmd->arg = dev_addr;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;

	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1)
		mrq->stop = NULL;
	else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

/*
 * Tests a special transfer with certain parameters
 */
static int mmc_test_general_command_transfer(struct mmc_card *card,
	struct scatterlist *sg, unsigned sg_len, unsigned dev_addr,
	unsigned blocks, unsigned blksz, int write)
{
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_command stop = {0};
	struct mmc_data data = {0};

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;

	mmc_test_prepare_general_command_mrq(card, &mrq, sg, sg_len, dev_addr,
		blocks, blksz, write);

	mmc_wait_for_req(card->host, &mrq);

	mmc_test_wait_busy(card);

	return mmc_test_check_result(card, &mrq);
}

static int samsung_mmc_nand_info_get(struct mmc_card *card, char *buf, u8 *nand_info)
{
	struct scatterlist sg_write, sg_read;
	int n = 0;
	int err = 0;
	bool set_cmdq_flag = false;
	u8 *pwd = NULL;

	if (!mmc_vsm_support(card)) {
		pr_err("[VSM]%s: Device not support VSM mode\n", mmc_hostname(card->host));
		goto out_halt;
	}
	pwd = kzalloc(512, GFP_KERNEL);
	if (!pwd)
		return ENOMEM;
	memcpy(pwd, Samsung_VSM_PWD, 512);

	/* disable C.Q */
	if (mmc_card_cmdq(card)) {
		err = mmc_cmdq_halt_on_empty_queue(card->host, 0);
		if (err) {
			pr_err("[VSM]%s: halt failed while doing %s err (%d)\n",
					mmc_hostname(card->host), __func__,
					err);
			goto out_halt;
		}

		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_CMDQ, 0,
			 card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("[VSM]%s: cmdq mode disable failed %d\n",
					mmc_hostname(card->host), err);
			goto set_cq_flag;
		}
		mmc_card_clr_cmdq(card);
		set_cmdq_flag = true;
	}

	sg_init_one(&sg_write, pwd, 512);
	sg_init_one(&sg_read, nand_info, 512);

	/* set device to VSM mode */
	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_MODE_CONFIG,
			MMC_VSM_MODE_SET, card->ext_csd.generic_cmd6_time);
	if (err) {
		pr_err("[VSM]%s: switch to VSM mode failed (%d)\n", mmc_hostname(card->host), err);
		goto enable_cq;
	}

	/* write PWD */
	err = mmc_test_simple_transfer(card, &sg_write, 1, VSM_SAMSUNG_ARG, 1, 512, MMC_VSM_WRITE);
	if (err) {
		pr_err("[VSM]%s: samsung_mmc_nand_info_get Writer err(%d)\n", mmc_hostname(card->host), err);
		goto enable_cq;
	}

	/* read NAND INFO */
	err = mmc_test_simple_transfer(card, &sg_read, 1, VSM_SAMSUNG_ARG, 1, 512, MMC_VSM_READ);
	if (err) {
		pr_err("[VSM]%s: samsung_mmc_nand_info_get Read err(%d)\n", mmc_hostname(card->host), err);
		goto enable_cq;
	}

	n = snprintf(buf, 1026,
		"cid:%08x%08x%08x%08x\n"
		"firmware version:%08x\n"
		"dev life time a,b:%02x,%02x\n"
		"pre EOL info:%02x\n"
		"Maximum block erase(total):%08x\n"
		"Minimum block erase(total):%08x\n"
		"Average block erase(total):%08x\n"
		"Maximum block erase(SLC):%08x\n"
		"Minimum block erase(SLC):%08x\n"
		"Average block erase(SLC):%08x\n"
		"Maximum block erase(MLC):%08x\n"
		"Minimum block erase(MLC):%08x\n"
		"Average block erase(MLC):%08x\n"
		"Read Reclaim count:%08x\n"
		"Initial Bad block count:%08x\n"
		"Runtime Bad block count:%08x\n"
		"Remain Bad block count:%08x\n"
		"Firmware patch trial count:%08x\n"
		"Firmware patch release Date:%08x\n"
		"Firmware patch success count:%08x\n"
		"Cumulative initialization count:%08x\n"
		"Cumulative written data size:%08x\n"
		"Cumulative read data size:%08x\n"
		"VCC Drop count:%08x\n"
		"VPC count:%08x\n"
		,
		card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
		card->ext_csd.firmware_version,
		card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
		card->ext_csd.dev_left_time,
		nand_info[3]<<24|nand_info[2]<<16|nand_info[1]<<8|nand_info[0],
		nand_info[7]<<24|nand_info[6]<<16|nand_info[5]<<8|nand_info[4],
		nand_info[11]<<24|nand_info[10]<<16|nand_info[9]<<8|nand_info[8],
		nand_info[47]<<24|nand_info[46]<<16|nand_info[45]<<8|nand_info[44],
		nand_info[51]<<24|nand_info[50]<<16|nand_info[49]<<8|nand_info[48],
		nand_info[55]<<24|nand_info[54]<<16|nand_info[53]<<8|nand_info[52],
		nand_info[59]<<24|nand_info[58]<<16|nand_info[57]<<8|nand_info[56],
		nand_info[63]<<24|nand_info[62]<<16|nand_info[61]<<8|nand_info[60],
		nand_info[67]<<24|nand_info[66]<<16|nand_info[65]<<8|nand_info[64],
		nand_info[15]<<24|nand_info[14]<<16|nand_info[13]<<8|nand_info[12],
		nand_info[19]<<24|nand_info[18]<<16|nand_info[17]<<8|nand_info[16],
		nand_info[23]<<24|nand_info[22]<<16|nand_info[21]<<8|nand_info[20],
		nand_info[27]<<24|nand_info[26]<<16|nand_info[25]<<8|nand_info[24],
		nand_info[31]<<24|nand_info[30]<<16|nand_info[29]<<8|nand_info[28],
		nand_info[35]<<24|nand_info[34]<<16|nand_info[33]<<8|nand_info[32],
		nand_info[71]<<24|nand_info[70]<<16|nand_info[69]<<8|nand_info[68],
		nand_info[43]<<24|nand_info[42]<<16|nand_info[41]<<8|nand_info[40],
		nand_info[39]<<24|nand_info[38]<<16|nand_info[37]<<8|nand_info[36],
		nand_info[75]<<24|nand_info[74]<<16|nand_info[73]<<8|nand_info[72],
		nand_info[79]<<24|nand_info[78]<<16|nand_info[77]<<8|nand_info[76],
		nand_info[83]<<24|nand_info[82]<<16|nand_info[81]<<8|nand_info[80]
	);

	pr_err("[VSM]%s: samsung_mmc_nand_info_get n (%d)\n", mmc_hostname(card->host), n);

enable_cq:
	/* set device to NORMAL mode */
	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_MODE_CONFIG,
			MMC_NORMAL_MODE_SET, card->ext_csd.generic_cmd6_time);
	if (err) {
		pr_err("VSM: %s: error %d VSM NORMAL is not supported\n",
			mmc_hostname(card->host), err);
	}

	if (set_cmdq_flag) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_CMDQ, 1,
				 card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("%s: cmdq mode enable failed %d\n",
			       mmc_hostname(card->host), err);
		} else {
			mmc_card_set_cmdq(card);
			set_cmdq_flag = false;
		}
	}

set_cq_flag:
	if (mmc_card_cmdq(card)) {
		if (mmc_cmdq_halt(card->host, false))
			pr_err("%s: %s: cmdq unhalt failed\n",
			       mmc_hostname(card->host), __func__);
	}

out_halt:
	if (strlen(buf) == 0)
		n = snprintf(buf, 1026,
			"cid:%08x%08x%08x%08x\n"
			"firmware version:%08x\n"
			"dev life time a,b:%02x,%02x\n"
			"pre EOL info:%02x\n"
			,
			card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
			card->ext_csd.firmware_version,
			card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
			card->ext_csd.dev_left_time
		);
	kfree(pwd);

	return err;
}

static int
mmc_send_cxd_data_hynix(struct mmc_card *card, struct mmc_host *host,
		u32 opcode, void *buf, unsigned len)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = opcode;
	cmd.arg = 0x53454852;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, buf, len);

	if (opcode == MMC_SEND_CSD || opcode == MMC_SEND_CID) {
		/*
		 * The spec states that CSR and CID accesses have a timeout
		 * of 64 clock cycles.
		 */
		data.timeout_ns = 0;
		data.timeout_clks = 64;
	} else
		mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

static int mmc_get_ext_csd_hynix(struct mmc_card *card, u8 **new_ext_csd)
{
	int err;
	u8 *ext_csd;

	if (!card || !new_ext_csd)
		return -EINVAL;

	if (!mmc_can_ext_csd(card))
		return -EOPNOTSUPP;

	/*
	 * As the ext_csd is so large and mostly unused, we don't store the
	 * raw block in mmc_card.
	 */
	ext_csd = kzalloc(512, GFP_KERNEL);
	if (!ext_csd)
		return -ENOMEM;

	err = mmc_send_cxd_data_hynix(card, card->host, MMC_SEND_EXT_CSD, ext_csd,
				512);
	if (err)
		kfree(ext_csd);
	else
		*new_ext_csd = ext_csd;

	return err;
}

static int hynix_mmc_nand_info_get_v1(struct mmc_card *card, char *buf, u8 *nand_info)
{
	int n = 0;
	int err = 0;
	u8 *ext_csd;
	bool set_cmdq_flag = false;

	/* disable C.Q */
	if (mmc_card_cmdq(card)) {
		err = mmc_cmdq_halt_on_empty_queue(card->host, 0);
		if (err) {
			pr_err("[VSM]%s: halt failed while doing %s err (%d)\n",
					mmc_hostname(card->host), __func__,
					err);
			goto out_halt;
		}

		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_CMDQ, 0,
			 card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("[VSM]%s: cmdq mode disable failed %d\n",
					mmc_hostname(card->host), err);
			goto set_cq_flag;
		}
		mmc_card_clr_cmdq(card);
		set_cmdq_flag = true;
	}
	/* read NAND INFO */
	err = mmc_get_ext_csd_hynix(card, &ext_csd);
	if (err) {
		pr_err("[VSM]%s: hynix_mmc_nand_info_get Read err(%d)\n", mmc_hostname(card->host), err);
		goto enable_cq;
	}

	memcpy(nand_info, ext_csd, 512);
	kfree(ext_csd);

	n = snprintf(buf, 1026,
	"cid:%08x%08x%08x%08x\n"
	"firmware version:%08x\n"
	"dev life time a,b:%02x,%02x\n"
	"pre EOL info:%02x\n"
	"Initial bad block count:%04x\n"
	"Run Time bad block count:%04x\n"
	"Maximum NAND block erase count:%04x\n"
	"Minimum NAND block erase count:%04x\n"
	"Average NAND block erase count:%04x\n"
	"Read Reclaim Count:%04x\n"
	"SPO Count:%08x\n"
	"Cumulative written data size:%08x\n"
	"LVD Count:%08x\n"
	"FFU Count:%04x\n"
	"Reserved Blocks:%04x\n"
	"Cumulative read data size:%08x\n"

	,
	card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
	card->ext_csd.firmware_version,
	card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
	card->ext_csd.dev_left_time,
	nand_info[271]<<8|nand_info[270],
	nand_info[273]<<8|nand_info[272],
	nand_info[275]<<8|nand_info[274],
	nand_info[277]<<8|nand_info[276],
	nand_info[279]<<8|nand_info[278],
	nand_info[281]<<8|nand_info[280],
	nand_info[285]<<24|nand_info[284]<<16|nand_info[283]<<8|nand_info[282],
	nand_info[289]<<24|nand_info[288]<<16|nand_info[287]<<8|nand_info[286],
	nand_info[293]<<24|nand_info[292]<<16|nand_info[291]<<8|nand_info[290],
	nand_info[295]<<8|nand_info[294],
	nand_info[297]<<8|nand_info[296],
	nand_info[301]<<24|nand_info[300]<<16|nand_info[299]<<8|nand_info[298]
	);

	pr_err("[VSM]%s: hynix_mmc_nand_info_get n (%d)\n", mmc_hostname(card->host), n);

enable_cq:
	if (set_cmdq_flag) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_CMDQ, 1,
				 card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("%s: cmdq mode enable failed %d\n",
				   mmc_hostname(card->host), err);
		} else {
			mmc_card_set_cmdq(card);
			set_cmdq_flag = false;
		}
	}

set_cq_flag:
	if (mmc_card_cmdq(card)) {
		if (mmc_cmdq_halt(card->host, false))
			pr_err("%s: %s: cmdq unhalt failed\n",
				   mmc_hostname(card->host), __func__);
	}

out_halt:
	if (strlen(buf) == 0)
		n = snprintf(buf, 1026,
			"cid:%08x%08x%08x%08x\n"
			"firmware version:%08x\n"
			"dev life time a,b:%02x,%02x\n"
			"pre EOL info:%02x\n"
			,
			card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
			card->ext_csd.firmware_version,
			card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
			card->ext_csd.dev_left_time
		);
	return err;
}

static int hynix_mmc_nand_info_get_v0(struct mmc_card *card, char *buf, u8 *nand_info)
{
#if 0
	struct mmc_command cmd = {0};
	int n = 0;
	int err = 0;
	u8 *ext_csd;
	bool set_cmdq_flag = false;

	/* disable C.Q */
	if (mmc_card_cmdq(card)) {
		err = mmc_cmdq_halt_on_empty_queue(card->host, 0);
		if (err) {
			pr_err("[VSM]%s: halt failed while doing %s err (%d)\n",
					mmc_hostname(card->host), __func__,
					err);
			goto out_halt;
		}

		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_CMDQ, 0,
			 card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("[VSM]%s: cmdq mode disable failed %d\n",
					mmc_hostname(card->host), err);
			goto set_cq_flag;
		}
		mmc_card_clr_cmdq(card);
		set_cmdq_flag = true;
	}

	cmd.opcode = CUSTOM_CMD_60;
	cmd.arg = VSM_HYNIX_ARG_1;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err) {
		pr_err("[VSM]%s: VSM_HYNIX_ARG_1 CMD err(%d)\n", mmc_hostname(card->host), err);
		goto enable_cq;
	}

	usleep_range(1000, 1500);

	cmd.opcode = CUSTOM_CMD_60;
	cmd.arg = VSM_HYNIX_ARG_2;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err) {
		pr_err("[VSM]%s: VSM_HYNIX_ARG_2 CMD err(%d)\n", mmc_hostname(card->host), err);
		goto enable_cq;
	}

	/* read NAND INFO */
	err = mmc_get_ext_csd(card, &ext_csd);
	if (err) {
		pr_err("[VSM]%s: hynix_mmc_nand_info_get Read err(%d)\n", mmc_hostname(card->host), err);
		goto enable_cq;
	}
	memcpy(nand_info, ext_csd, 512);
	kfree(ext_csd);

	n = snprintf(buf,
	4096,
	"cid:%08x%08x%08x%08x\n"
	"firmware version:%08x\n"
	"dev life time a,b:%02x,%02x\n"
	"pre EOL info:%02x\n"
	"Reserved Blocks(SLC):%08x\n"
	"Maximum block erase(SLC):%08x\n"
	"Minimum block erase(SLC):%08x\n"
	"Average block erase(SLC):%08x\n"
	"Reserved Blocks(MLC):%08x\n"
	"Maximum block erase(MLC):%08x\n"
	"Minimum block erase(MLC):%08x\n"
	"Average block erase(MLC):%08x\n"
	"Reserved Blocks(SLC+MLC):%08x\n"
	"Cumulative initialization count:%08x\n"
	"Cumulative written data size:%08x\n"
	"Cumulative read data size:%08x\n"
	"Runtime Bad block count:%08x\n"
	"Read Reclaim count:%08x\n"
	"VCC Drop count:%08x\n"
	,
	card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
	card->ext_csd.firmware_version,
	card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
	card->ext_csd.dev_left_time,
	nand_info[35]<<24|nand_info[34]<<16|nand_info[33]<<8|nand_info[32],
	nand_info[39]<<24|nand_info[38]<<16|nand_info[37]<<8|nand_info[36],
	nand_info[43]<<24|nand_info[42]<<16|nand_info[41]<<8|nand_info[40],
	nand_info[47]<<24|nand_info[46]<<16|nand_info[45]<<8|nand_info[44],
	nand_info[51]<<24|nand_info[50]<<16|nand_info[49]<<8|nand_info[48],
	nand_info[55]<<24|nand_info[54]<<16|nand_info[53]<<8|nand_info[52],
	nand_info[59]<<24|nand_info[58]<<16|nand_info[57]<<8|nand_info[56],
	nand_info[63]<<24|nand_info[62]<<16|nand_info[61]<<8|nand_info[60],
	nand_info[67]<<24|nand_info[66]<<16|nand_info[65]<<8|nand_info[64],
	nand_info[95]<<24|nand_info[94]<<16|nand_info[93]<<8|nand_info[92],
	nand_info[99]<<24|nand_info[98]<<16|nand_info[97]<<8|nand_info[96],
	nand_info[103]<<24|nand_info[102]<<16|nand_info[101]<<8|nand_info[100],
	nand_info[107]<<24|nand_info[106]<<16|nand_info[105]<<8|nand_info[104],
	nand_info[139]<<24|nand_info[138]<<16|nand_info[137]<<8|nand_info[136],
	nand_info[143]<<24|nand_info[142]<<16|nand_info[141]<<8|nand_info[140]

	);

	pr_err("[VSM]%s: hynix_mmc_nand_info_get n (%d)\n", mmc_hostname(card->host), n);

enable_cq:
	if (set_cmdq_flag) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_CMDQ, 1,
				 card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("%s: cmdq mode enable failed %d\n",
				   mmc_hostname(card->host), err);
		} else {
			mmc_card_set_cmdq(card);
			set_cmdq_flag = false;
		}
	}

set_cq_flag:
	if (mmc_card_cmdq(card)) {
		if (mmc_cmdq_halt(card->host, false))
			pr_err("%s: %s: cmdq unhalt failed\n",
				   mmc_hostname(card->host), __func__);
	}

out_halt:
	if (strlen(buf) == 0)
		n = snprintf(buf, 1026,
			"cid:%08x%08x%08x%08x\n"
			"firmware version:%08x\n"
			"dev life time a,b:%02x,%02x\n"
			"pre EOL info:%02x\n"
			,
			card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
			card->ext_csd.firmware_version,
			card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
			card->ext_csd.dev_left_time
		);
	return err;
#endif
	int n = 0;
	int err = 0;

	n = snprintf(buf, 1026,
		"cid:%08x%08x%08x%08x\n"
		"firmware version:%08x\n"
		"dev life time a,b:%02x,%02x\n"
		"pre EOL info:%02x\n"
		,
		card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
		card->ext_csd.firmware_version,
		card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
		card->ext_csd.dev_left_time
	);
	pr_err("[VSM]%s: %s n (%d)\n", mmc_hostname(card->host), __func__, n);

	return err;
}

static int micron_mmc_nand_info_get(struct mmc_card *card, char *buf, u8 *nand_info)
{
	struct scatterlist sg_read;
	int n = 0;
	int err = 0;
	bool set_cmdq_flag = false;

	/* disable C.Q */
	if (mmc_card_cmdq(card)) {
		err = mmc_cmdq_halt_on_empty_queue(card->host, 0);
		if (err) {
			pr_err("[VSM]%s: halt failed while doing %s err (%d)\n",
					mmc_hostname(card->host), __func__,
					err);
			goto out_halt;
		}

		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_CMDQ, 0,
			 card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("[VSM]%s: cmdq mode disable failed %d\n",
					mmc_hostname(card->host), err);
			goto set_cq_flag;
		}
		mmc_card_clr_cmdq(card);
		set_cmdq_flag = true;
	}
	sg_init_one(&sg_read, nand_info, 512);

	/* read NAND INFO */
	err = mmc_test_general_command_transfer(card, &sg_read, 1, 0x0000000D, 1, 512, 0);
	if (err) {
		pr_err("[VSM]%s: mmc_test_general_command_transfer Read err(%d)\n", mmc_hostname(card->host), err);
		goto enable_cq;
	}

	n = snprintf(buf, 1026,
	"cid:%08x%08x%08x%08x\n"
	"firmware version:%08x\n"
	"dev life time a,b:%02x,%02x\n"
	"pre EOL info:%02x\n"
	"Initial Bad block count:%04x\n"
	"Runtime Bad block count:%04x\n"
	"Remaining spare block count:%04x\n"
	"Minimum block erase(MLC):%08x\n"
	"Maximum block erase(MLC):%08x\n"
	"Average block erase(MLC):%08x\n"
	"Total block erase(MLC):%08x\n"
	"Minimum block erase(SLC):%08x\n"
	"Maximum block erase(SLC):%08x\n"
	"Average block erase(SLC):%08x\n"
	"Total block erase(SLC):%08x\n"
	"Cumulative initialization count:%08x\n"
	"Read Reclaim count:%08x\n"
	"Cumulative written data size:%08x\n"
	"Firmware patch trial count:%04x\n"
	"Firmware patch success count:%04x\n"
	,
	card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
	card->ext_csd.firmware_version,
	card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
	card->ext_csd.dev_left_time,
	nand_info[0]<<8|nand_info[1],
	nand_info[2]<<8|nand_info[3],
	nand_info[4]<<8|nand_info[5],
	nand_info[16]<<24|nand_info[17]<<16|nand_info[18]<<8|nand_info[19],
	nand_info[20]<<24|nand_info[21]<<16|nand_info[22]<<8|nand_info[23],
	nand_info[24]<<24|nand_info[25]<<16|nand_info[26]<<8|nand_info[27],
	nand_info[28]<<24|nand_info[29]<<16|nand_info[30]<<8|nand_info[31],
	nand_info[32]<<24|nand_info[33]<<16|nand_info[34]<<8|nand_info[35],
	nand_info[36]<<24|nand_info[37]<<16|nand_info[38]<<8|nand_info[39],
	nand_info[40]<<24|nand_info[41]<<16|nand_info[42]<<8|nand_info[43],
	nand_info[44]<<24|nand_info[45]<<16|nand_info[46]<<8|nand_info[47],
	nand_info[51]<<24|nand_info[50]<<16|nand_info[49]<<8|nand_info[48],
	nand_info[55]<<24|nand_info[54]<<16|nand_info[53]<<8|nand_info[52],
	nand_info[71]<<24|nand_info[70]<<16|nand_info[69]<<8|nand_info[68],
	nand_info[117]<<8|nand_info[116],
	nand_info[119]<<8|nand_info[118]
	);

	pr_err("[VSM]%s: micron_mmc_nand_info_get n (%d)\n", mmc_hostname(card->host), n);

enable_cq:
	if (set_cmdq_flag) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_CMDQ, 1,
				 card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("%s: cmdq mode enable failed %d\n",
				   mmc_hostname(card->host), err);
		} else {
			mmc_card_set_cmdq(card);
			set_cmdq_flag = false;
		}
	}

set_cq_flag:
	if (mmc_card_cmdq(card)) {
		if (mmc_cmdq_halt(card->host, false))
			pr_err("%s: %s: cmdq unhalt failed\n",
				   mmc_hostname(card->host), __func__);
	}

out_halt:
	if (strlen(buf) == 0)
		n = snprintf(buf, 1026,
			"cid:%08x%08x%08x%08x\n"
			"firmware version:%08x\n"
			"dev life time a,b:%02x,%02x\n"
			"pre EOL info:%02x\n"
			,
			card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
			card->ext_csd.firmware_version,
			card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
			card->ext_csd.dev_left_time
		);
	return err;

}

static int toshiba_mmc_nand_info_get(struct mmc_card *card, char *buf, u8 *nand_info)
{
	int n = 0;
	int err = 0;

	n = snprintf(buf, 1026,
	"cid:%08x%08x%08x%08x\n"
	"firmware version:%08x\n"
	"dev life time a,b:%02x,%02x\n"
	"pre EOL info:%02x\n"
	,
	card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3],
	card->ext_csd.firmware_version,
	card->ext_csd.dev_left_time_a, card->ext_csd.dev_left_time_b,
	card->ext_csd.dev_left_time
	);

	pr_err("[VSM]%s: toshiba_mmc_nand_info_get n (%d)\n", mmc_hostname(card->host), n);

	return err;
}

int mmc_nand_info_get(struct mmc_card *card, char *buf, u8 *nand_info)
{
	int err = 0;

	pr_err("[VSM]%s: card->cid.manfid = 0x%x \n", mmc_hostname(card->host), card->cid.manfid);
	/*  Can not read nand information when eMMC in non-user data partitions */
	if (card->part_curr) {
		pr_warn("[VSM]%s: the current partition is %d \n",
			mmc_hostname(card->host), card->part_curr);
		err = -1;
		return err;
	}
	switch (card->cid.manfid) {
	case CID_MANFID_SAMSUNG:
		err = samsung_mmc_nand_info_get(card, buf, nand_info);
		if (err)
			pr_err("[VSM]%s: samsung_mmc_nand_info_get err (%d)\n", mmc_hostname(card->host), err);
		break;
	case CID_MANFID_HYNIX:
		if (!strncmp("hDEaP3", (char *)card->cid.prod_name, 6)
			|| !strncmp("hC8aP>", (char *)card->cid.prod_name, 6)
			|| !strncmp("hB8aP>", (char *)card->cid.prod_name, 6))
			err = hynix_mmc_nand_info_get_v1(card, buf, nand_info);
		else
			err = hynix_mmc_nand_info_get_v0(card, buf, nand_info);
		if (err)
			pr_err("[VSM]%s: hynix_mmc_nand_info_get err (%d)\n", mmc_hostname(card->host), err);
		break;
	case CID_MANFID_MICRON:
		err = micron_mmc_nand_info_get(card, buf, nand_info);
		if (err)
			pr_err("[VSM]%s: micron_mmc_nand_info_get err (%d)\n", mmc_hostname(card->host), err);
		break;
	case CID_MANFID_TOSHIBA:
		err = toshiba_mmc_nand_info_get(card, buf, nand_info);
		if (err)
			pr_err("[VSM]%s: toshiba_mmc_nand_info_get err (%d)\n", mmc_hostname(card->host), err);
		break;
	default:
		pr_err("[VSM]%s:%s card->cid.manfid = 0x%x not support\n", mmc_hostname(card->host), __func__, card->cid.manfid);
		break;
	}

	return err;
}

