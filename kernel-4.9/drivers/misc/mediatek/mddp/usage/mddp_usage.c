/*
 * mddp_usage.c - Data usage API.
 *
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/skbuff.h>

#include "mddp_export.h"
#include "mddp_filter.h"
#include "mddp_ipc.h"
#include "mddp_usage.h"
#include "mtk_ccci_common.h"

//------------------------------------------------------------------------------
// Struct definition.
// -----------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private helper macro.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Function prototype.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Private variables.
//------------------------------------------------------------------------------
static uint32_t mddp_u_iq_trans_id_s;

//------------------------------------------------------------------------------
// Private helper macro.
//------------------------------------------------------------------------------
#define MDDP_U_GET_IQ_TRANS_ID()        (mddp_u_iq_trans_id_s++)

//------------------------------------------------------------------------------
// Private functions.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Public functions - WiFi Hotspot.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Public functions.
//------------------------------------------------------------------------------
int32_t mddp_usage_init(void)
{
	return 0;
}

void mddp_usage_uninit(void)
{

}

void mddp_u_get_data_stats(void *buf, uint32_t *buf_len)
{
	struct mddp_u_md_data_stats_t      *md_stats;
	uint32_t                            sm_len = 0;

	md_stats = get_smem_start_addr(MD_SYS1, SMEM_USER_RAW_NETD, &sm_len);

	if (sm_len >= sizeof(struct mddp_u_data_stats_t)) {
		*buf_len = sizeof(sizeof(struct mddp_u_data_stats_t));
		memcpy(buf, md_stats, *buf_len);
	} else {
		pr_notice("%s: Failed to copy data stats, sm_len(%d), buf_len(%d)!\n",
				__func__, sm_len, *buf_len);
		*buf_len = 0;
	}
}

int32_t mddp_u_set_data_limit(uint8_t *buf, uint32_t buf_len)
{
	uint32_t                                md_status;
	struct mddp_md_msg_t                   *md_msg;
	struct mddp_dev_req_set_data_limit_t   *in_req;
	struct mddp_u_data_limit_t              limit;
	int8_t                                  id;

	if (buf_len != sizeof(struct mddp_dev_req_set_data_limit_t)) {
		pr_notice("%s: Invalid parameter, buf_len(%d)!\n",
				__func__, buf_len);
		WARN_ON(1);
		return -EINVAL;
	}

	md_status = exec_ccci_kern_func_by_md_id(0, ID_GET_MD_STATE, NULL, 0);

	if (md_status != MD_STATE_READY) {
		pr_notice("%s: Invalid state, md_status(%d)!\n",
				__func__, md_status);
		return -ENODEV;
	}

	md_msg = kzalloc(sizeof(struct mddp_md_msg_t) + sizeof(limit),
			GFP_ATOMIC);
	if (unlikely(!md_msg)) {
		pr_notice("%s: failed to alloc md_msg bug!\n", __func__);
		WARN_ON(1);
		return -EAGAIN;
	}

	in_req = (struct mddp_dev_req_set_data_limit_t *)buf;
	id = mddp_f_data_usage_wan_dev_name_to_id(in_req->ul_dev_name);
	if (unlikely(id < 0)) {
		pr_notice("%s: Invalid dev_name, dev_name(%s)!\n",
				__func__, in_req->ul_dev_name);
		WARN_ON(1);
		return -EINVAL;
	}

	memset(&limit, 0, sizeof(limit));
	limit.cmd = MSG_ID_MDT_SET_IQUOTA_REQ;
	limit.trans_id = MDDP_U_GET_IQ_TRANS_ID();
	limit.limit_buffer_size = in_req->limit_size;
	limit.id = id;
	pr_notice("%s: Send cmd(%d)/id(%d)/name(%s) limit(%llx) to MD.\n",
		__func__, limit.cmd, limit.id, in_req->ul_dev_name,
		limit.limit_buffer_size);

	// <TJ_TODO> this msg_id is enum define not sync. to MDDPv1
	md_msg->msg_id = IPC_MSG_ID_MDT_DATA_USAGE_CMD;
	md_msg->data_len = sizeof(limit);
	memcpy(md_msg->data, &limit, sizeof(limit));
	mddp_ipc_send_md(NULL, md_msg, MD_MOD_MDT);

	return 0;
}


int32_t mddp_u_msg_hdlr(void)
{
	// <TJ_TODO> mddp usage msg hdlr

	return 0;
}
