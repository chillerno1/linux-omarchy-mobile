// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Secure Execution Environment Communicator (QSEECOM) userspace
 * interface.
 *
 * Provides /dev/qseecom with the ioctl interface spoken by the vendor QSEECOM
 * client library and listener daemon: trusted applications (TAs) in the QSEE
 * can be loaded and sent commands, and userspace listener services serve the
 * requests a TA makes while one of its commands is in progress.
 *
 * Ported from the vendor QSEECOM driver:
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Only the interface needed by a userspace client and the listener daemon is
 * provided. Key management, crypto engine and ICE, partition hash, enterprise
 * security, MDTP, external ELF loading and the GlobalPlatform (QTEEC) ioctls
 * are refused with -EOPNOTSUPP. Of the secure service commands only the
 * read-only RPMB provisioning status query is allowed; RPMB key provisioning
 * and RPMB erase are irreversible and always refused with -EPERM.
 *
 * Every buffer handed to the TrustZone must lie inside a registered shared
 * memory bridge. Internal buffers come from a TZ memory pool. Userspace
 * buffers are dma-bufs, which must be physically contiguous; the driver
 * keeps them attached and bridged for as long as userspace holds them, so
 * that the same buffer can be passed again (and kept by a TA) without the
 * bridge being created twice.
 */

#define pr_fmt(fmt) "qseecom: " fmt

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/fs.h>
#include <linux/iosys-map.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/qseecom.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define QSEECOM_DEV_NAME		"qseecom"

#define QSEOS_VERSION_14		0x14
#define QSEE_VERSION_03			0x403000
#define QSEE_VERSION_40			0x1000000	/* TZ.BF.4.0 */

#define PHY_ADDR_4G			BIT_ULL(32)

#define QSEECOM_SCM_EBUSY_WAIT_MS	30
#define QSEECOM_SCM_EBUSY_MAX_RETRY	67
#define QSEE_RESULT_FAIL_APP_BUSY	315

/* Result of an SHM bridge creation for memory already covered by a bridge */
#define QSEECOM_SHMBRIDGE_E_OVERLAP	15

#define FEATURE_ID_WHITELIST		15
#define MAKE_WHITELIST_VERSION(major, minor, patch) \
	((((major) & 0x3ff) << 22) | (((minor) & 0x3ff) << 12) | ((patch) & 0xfff))

/* SMC owners and services */
#define TZ_OWNER_SIP			2
#define TZ_OWNER_TZ_APPS		48
#define TZ_OWNER_QSEE_OS		50

#define TZ_SVC_APP_ID_PLACEHOLDER	0
#define TZ_SVC_APP_MGR			1
#define TZ_SVC_LISTENER			2
#define TZ_SVC_RPMB			4
#define TZ_SVC_INFO			6

#define QSEECOM_SMC_ID(o, s, f) \
	((u32)((((o) & 0x3f) << 24) | (((s) & 0xff) << 8) | ((f) & 0xff)))
#define QSEECOM_SMC_OWNER(id)		(((id) >> 24) & 0x3f)
#define QSEECOM_SMC_SVC(id)		(((id) >> 8) & 0xff)
#define QSEECOM_SMC_FN(id)		((id) & 0xff)

#define TZ_INFO_GET_FEATURE_VERSION_ID \
	QSEECOM_SMC_ID(TZ_OWNER_SIP, TZ_SVC_INFO, 0x03)
#define TZ_OS_APP_START_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_APP_MGR, 0x01)
#define TZ_OS_APP_SHUTDOWN_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_APP_MGR, 0x02)
#define TZ_OS_APP_LOOKUP_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_APP_MGR, 0x03)
#define TZ_OS_REGISTER_LISTENER_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_LISTENER, 0x01)
#define TZ_OS_DEREGISTER_LISTENER_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_LISTENER, 0x02)
#define TZ_OS_LISTENER_RESPONSE_HANDLER_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_LISTENER, 0x03)
#define TZ_OS_CONTINUE_BLOCKED_REQUEST_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_LISTENER, 0x04)
#define TZ_OS_LISTENER_RESPONSE_HANDLER_WITH_WHITELIST_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_LISTENER, 0x05)
#define TZ_OS_REGISTER_LISTENER_SMCINVOKE_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_LISTENER, 0x06)
#define TZ_OS_CONTINUE_BLOCKED_REQUEST_SMCINVOKE_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_LISTENER, 0x07)
#define TZ_OS_RPMB_CHECK_PROV_STATUS_ID \
	QSEECOM_SMC_ID(TZ_OWNER_QSEE_OS, TZ_SVC_RPMB, 0x03)
#define TZ_APP_QSAPP_SEND_DATA_ID \
	QSEECOM_SMC_ID(TZ_OWNER_TZ_APPS, TZ_SVC_APP_ID_PLACEHOLDER, 0x01)
#define TZ_APP_QSAPP_SEND_DATA_WITH_WHITELIST_ID \
	QSEECOM_SMC_ID(TZ_OWNER_TZ_APPS, TZ_SVC_APP_ID_PLACEHOLDER, 0x06)

/* SMC argument descriptors */
#define QSEECOM_VAL			0
#define QSEECOM_RO			1
#define QSEECOM_RW			2

#define QSEECOM_ARGS_IMPL(n, a, b, c, d, e, f, g, ...) \
	(((n) & 0xf) | ((a) & 0x3) << 4 | ((b) & 0x3) << 6 | \
	 ((c) & 0x3) << 8 | ((d) & 0x3) << 10 | ((e) & 0x3) << 12 | \
	 ((f) & 0x3) << 14 | ((g) & 0x3) << 16)
#define QSEECOM_ARGS(...) \
	QSEECOM_ARGS_IMPL(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0)

enum qseecom_qceos_cmd_status {
	QSEOS_RESULT_SUCCESS = 0,
	QSEOS_RESULT_INCOMPLETE,
	QSEOS_RESULT_BLOCKED_ON_LISTENER,
	QSEOS_RESULT_CBACK_REQUEST,
	QSEOS_RESULT_FAILURE = 0xFFFFFFFF
};

enum qseecom_command_scm_resp_type {
	QSEOS_APP_ID = 0xEE01,
	QSEOS_LISTENER_ID
};

#define SMCINVOKE_RESULT_INBOUND_REQ_NEEDED	3

/* Secure service command ids (QSEECOM_IOCTL_SEND_CMD_SERVICE_REQ) */
enum qseecom_svc_cmd_id {
	QSEOS_RPMB_PROVISION_KEY_COMMAND = 0x0F,
	QSEOS_RPMB_ERASE_COMMAND = 0x10,
	QSEOS_RPMB_CHECK_PROV_STATUS_COMMAND = 0x1B,
	QSEOS_FSM_LTEOTA_REQ_CMD = 0x109,
	QSEOS_FSM_LTEOTA_REQ_RSP_CMD = 0x110,
	QSEOS_FSM_IKE_REQ_CMD = 0x203,
	QSEOS_FSM_IKE_REQ_RSP_CMD = 0x204,
	QSEOS_FSM_OEM_FUSE_WRITE_ROW = 0x301,
	QSEOS_FSM_OEM_FUSE_READ_ROW = 0x302,
	QSEOS_FSM_ENCFS_REQ_CMD = 0x403,
	QSEOS_FSM_ENCFS_REQ_RSP_CMD = 0x404,
	QSEOS_DIAG_FUSE_REQ_CMD = 0x501,
	QSEOS_DIAG_FUSE_REQ_RSP_CMD = 0x502,
};

struct qseecom_rpmb_provision_key {
	u32 key_type;
};

/* Result words of a QSEECOM call, truncated to 32 bits like QSEE reports them */
struct qseecom_command_scm_resp {
	u32 result;
	u32 resp_type;
	u32 data;
};

/*
 * Whitelist table passed alongside a request or a listener response, telling
 * QSEE which locations of the request buffer hold physical addresses of
 * memory shared with the TA.
 *
 * Bit 31 of @index_and_flags is set when the location holds one address and
 * @size_or_count is the size of the memory at that address; otherwise the
 * location holds a list of {address, length} entries and @size_or_count is
 * their number. Bit 30 is set for 64-bit addresses. Bits 0-29 hold the offset
 * of the location in the request buffer.
 */
struct sglist_info {
	u32 index_and_flags;
	u32 size_or_count;
};

#define SGLISTINFO_SET_INDEX_FLAG(c, s, i) \
	((u32)((((c) & 1) << 31) | (((s) & 1) << 30) | ((i) & 0x3fffffff)))
#define SGLISTINFO_TABLE_SIZE	(sizeof(struct sglist_info) * MAX_ION_FD)

enum qseecom_client_handle_type {
	QSEECOM_CLIENT_APP = 1,
	QSEECOM_LISTENER_SERVICE,
	QSEECOM_SECURE_SERVICE,
	QSEECOM_GENERIC,
};

/**
 * struct qseecom_shm - userspace dma-buf shared with the TrustZone
 * @node:    entry in qseecom.shm_list
 * @dmabuf:  the dma-buf, referenced for as long as the entry exists
 * @attach:  attachment of the dma-buf to the qseecom device
 * @sgt:     mapping of @attach
 * @paddr:   physical start address of the (contiguous) buffer
 * @size:    size of the buffer
 * @bridge:  handle of the SHM bridge covering the buffer, if @bridged
 * @bridged: the driver created (and must delete) the SHM bridge
 * @users:   number of active users in the driver
 *
 * Entries without users stay cached, bridged, while userspace still holds the
 * dma-buf: a TA may keep using a buffer it was given in an earlier command, and
 * the client library reuses its buffers. They are freed once the driver holds
 * the last reference to the dma-buf.
 */
struct qseecom_shm {
	struct list_head node;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	phys_addr_t paddr;
	size_t size;
	u64 bridge;
	bool bridged;
	unsigned int users;
};

/* A shared buffer the driver also accesses through a kernel mapping */
struct qseecom_sbuf {
	struct qseecom_shm *shm;
	struct iosys_map map;
	u8 *virt;
	phys_addr_t phys;
	size_t length;
};

struct qseecom_registered_listener_list {
	struct list_head list;
	struct qseecom_register_listener_req svc;
	uintptr_t user_virt_sb_base;
	struct qseecom_sbuf sb;
	wait_queue_head_t rcv_req_wq;
	/* rcv_req_flag: 0: ready and empty; 1: received req */
	int rcv_req_flag;
	int send_resp_flag;
	bool listener_in_use;
	/* wq for thread blocked on this listener */
	wait_queue_head_t listener_block_app_wq;
	struct sglist_info *sglistinfo_ptr;
	u32 sglist_cnt;
	/* buffers passed with the pending modfd response */
	struct qseecom_shm *resp_shm[MAX_ION_FD];
	int abort;
	bool unregister_pending;
};

struct qseecom_unregister_pending_list {
	struct list_head list;
	struct qseecom_dev_handle *data;
};

struct qseecom_registered_app_list {
	struct list_head list;
	u32 app_id;
	u32 ref_cnt;
	char app_name[MAX_APP_NAME_SIZE];
	u32 app_arch;
	bool app_blocked;
	u32 check_block;
	u32 blocked_on_listener_id;
	/* Found loaded (by the bootloader), not loaded through this driver */
	bool preloaded;
};

struct qseecom_unload_app_pending_list {
	struct list_head list;
	struct qseecom_dev_handle *data;
};

struct qseecom_client_handle {
	u32 app_id;
	struct qseecom_sbuf sb;
	size_t sb_length;
	uintptr_t user_virt_sb_base;
	char app_name[MAX_APP_NAME_SIZE];
	u32 app_arch;
	bool unload_pending;
	bool from_loadapp;
};

struct qseecom_listener_handle {
	u32 id;
	bool register_pending;
	bool release_called;
};

struct qseecom_dev_handle {
	enum qseecom_client_handle_type type;
	struct qseecom_client_handle client;
	struct qseecom_listener_handle listener;
	bool released;
	int abort;
	wait_queue_head_t abort_wq;
	atomic_t ioctl_count;
	struct sglist_info *sglistinfo_ptr;
	u32 sglist_cnt;
};

struct qseecom_control {
	struct device *dev;
	struct miscdevice misc;
	struct qcom_tzmem_pool *pool;

	struct list_head registered_listener_list_head;

	struct list_head registered_app_list_head;
	spinlock_t registered_app_list_lock;	/* protects the app list */

	wait_queue_head_t send_resp_wq;
	int send_resp_flag;

	u32 qseos_version;
	u32 qsee_version;
	bool whitelist_support;
	bool smcinvoke_support;

	u32 app_block_ref_cnt;
	wait_queue_head_t app_block_wq;

	struct list_head unregister_lsnr_pending_list_head;
	wait_queue_head_t register_lsnr_pending_wq;
	struct work_struct unregister_lsnr_work;

	struct list_head unload_app_pending_list_head;
	struct work_struct unload_app_work;

	struct mutex shm_lock;			/* protects shm_list */
	struct list_head shm_list;
};

static DEFINE_MUTEX(app_access_lock);
static DEFINE_MUTEX(listener_access_lock);
static DEFINE_MUTEX(unload_app_pending_list_lock);

static struct qseecom_control qseecom;

static int __qseecom_unload_app(struct qseecom_dev_handle *data, u32 app_id);

/*
 * Shared memory
 */

static void qseecom_shm_free(struct qseecom_shm *shm)
{
	if (shm->bridged)
		qcom_tzmem_shm_bridge_delete(shm->bridge);
	dma_buf_unmap_attachment_unlocked(shm->attach, shm->sgt,
					  DMA_BIDIRECTIONAL);
	dma_buf_detach(shm->dmabuf, shm->attach);
	dma_buf_put(shm->dmabuf);
	kfree(shm);
}

/* Free cached buffers that userspace no longer holds. */
static void __qseecom_shm_reclaim(void)
{
	struct qseecom_shm *shm, *tmp;

	lockdep_assert_held(&qseecom.shm_lock);
	list_for_each_entry_safe(shm, tmp, &qseecom.shm_list, node) {
		if (shm->users || file_count(shm->dmabuf->file) > 1)
			continue;
		list_del(&shm->node);
		qseecom_shm_free(shm);
	}
}

static void qseecom_shm_reclaim(void)
{
	guard(mutex)(&qseecom.shm_lock);
	__qseecom_shm_reclaim();
}

/*
 * The TrustZone is given physical addresses, so the buffer must be one
 * physically contiguous range, mapped without translation or bouncing.
 */
static int qseecom_shm_check_contiguous(struct qseecom_shm *shm)
{
	struct sg_table *sgt = shm->sgt;
	struct scatterlist *sg;
	phys_addr_t next;
	size_t len = 0;
	int i;

	if (!sgt->orig_nents || sgt->nents != sgt->orig_nents)
		return -EINVAL;

	next = sg_phys(sgt->sgl);
	for_each_sgtable_sg(sgt, sg, i) {
		if (sg_phys(sg) != next || sg_dma_address(sg) != sg_phys(sg))
			return -EINVAL;
		next += sg->length;
		len += sg->length;
	}

	shm->paddr = sg_phys(sgt->sgl);
	shm->size = min(len, shm->dmabuf->size);

	if (!PAGE_ALIGNED(shm->paddr) || !shm->size)
		return -EINVAL;

	return 0;
}

static struct qseecom_shm *qseecom_shm_get(int fd)
{
	struct qseecom_shm *shm;
	struct dma_buf *dmabuf;
	int ret;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		pr_err("dma_buf_get() for fd %d failed\n", fd);
		return ERR_CAST(dmabuf);
	}

	/* Held across creation so that a buffer is bridged only once. */
	guard(mutex)(&qseecom.shm_lock);

	__qseecom_shm_reclaim();

	list_for_each_entry(shm, &qseecom.shm_list, node) {
		if (shm->dmabuf == dmabuf) {
			shm->users++;
			dma_buf_put(dmabuf);
			return shm;
		}
	}

	shm = kzalloc_obj(*shm);
	if (!shm) {
		ret = -ENOMEM;
		goto err_put;
	}
	shm->dmabuf = dmabuf;

	shm->attach = dma_buf_attach(dmabuf, qseecom.dev);
	if (IS_ERR(shm->attach)) {
		ret = PTR_ERR(shm->attach);
		pr_err("dma_buf_attach() for fd %d failed: %d\n", fd, ret);
		goto err_free;
	}

	shm->sgt = dma_buf_map_attachment_unlocked(shm->attach,
						   DMA_BIDIRECTIONAL);
	if (IS_ERR(shm->sgt)) {
		ret = PTR_ERR(shm->sgt);
		pr_err("dma_buf_map_attachment() for fd %d failed: %d\n",
		       fd, ret);
		goto err_detach;
	}

	ret = qseecom_shm_check_contiguous(shm);
	if (ret) {
		pr_err("buffer of fd %d is not physically contiguous\n", fd);
		goto err_unmap;
	}

	/*
	 * The bridge covers whole pages. The memory may already be part of a
	 * bridge someone else created, in which case the firmware refuses a
	 * second one and the existing bridge is used.
	 */
	ret = qcom_tzmem_shm_bridge_create(shm->paddr, PAGE_ALIGN(shm->size),
					   &shm->bridge);
	if (ret == QSEECOM_SHMBRIDGE_E_OVERLAP) {
		pr_debug("buffer of fd %d is already bridged\n", fd);
	} else if (ret) {
		pr_err("failed to create SHM bridge for fd %d: %d\n", fd, ret);
		ret = ret < 0 ? ret : -EINVAL;
		goto err_unmap;
	} else {
		shm->bridged = true;
	}

	shm->users = 1;
	list_add_tail(&shm->node, &qseecom.shm_list);

	return shm;

err_unmap:
	dma_buf_unmap_attachment_unlocked(shm->attach, shm->sgt,
					  DMA_BIDIRECTIONAL);
err_detach:
	dma_buf_detach(dmabuf, shm->attach);
err_free:
	kfree(shm);
err_put:
	dma_buf_put(dmabuf);
	return ERR_PTR(ret);
}

static void qseecom_shm_put(struct qseecom_shm *shm)
{
	if (!shm)
		return;

	mutex_lock(&qseecom.shm_lock);
	shm->users--;
	mutex_unlock(&qseecom.shm_lock);
}

/* Make CPU writes visible to the TrustZone. */
static int qseecom_shm_clean(struct qseecom_shm *shm)
{
	if (!shm)
		return -EINVAL;

	return dma_buf_end_cpu_access(shm->dmabuf, DMA_BIDIRECTIONAL);
}

/* Make TrustZone writes visible to the CPU. */
static int qseecom_shm_invalidate(struct qseecom_shm *shm)
{
	if (!shm)
		return -EINVAL;

	return dma_buf_begin_cpu_access(shm->dmabuf, DMA_BIDIRECTIONAL);
}

static int qseecom_sbuf_map(int fd, struct qseecom_sbuf *sbuf)
{
	struct qseecom_shm *shm;
	int ret;

	shm = qseecom_shm_get(fd);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	ret = dma_buf_vmap_unlocked(shm->dmabuf, &sbuf->map);
	if (ret) {
		pr_err("dma_buf_vmap() for fd %d failed: %d\n", fd, ret);
		qseecom_shm_put(shm);
		return ret;
	}
	if (sbuf->map.is_iomem) {
		dma_buf_vunmap_unlocked(shm->dmabuf, &sbuf->map);
		qseecom_shm_put(shm);
		return -EINVAL;
	}

	sbuf->shm = shm;
	sbuf->virt = sbuf->map.vaddr;
	sbuf->phys = shm->paddr;
	sbuf->length = shm->size;
	return 0;
}

static void qseecom_sbuf_unmap(struct qseecom_sbuf *sbuf)
{
	if (!sbuf->shm)
		return;

	dma_buf_vunmap_unlocked(sbuf->shm->dmabuf, &sbuf->map);
	qseecom_shm_put(sbuf->shm);
	memset(sbuf, 0, sizeof(*sbuf));
}

static void *qseecom_alloc_tzbuf(size_t size)
{
	void *buf = qcom_tzmem_alloc(qseecom.pool, size, GFP_KERNEL);

	if (buf)
		memset(buf, 0, size);
	return buf;
}

/*
 * SCM calls
 */

static int qseecom_scm_call(u32 smc_id, u32 arginfo, const u64 *args,
			    struct qseecom_command_scm_resp *resp)
{
	struct qcom_scm_qseecom_resp res = {};
	int retry_count = 0;
	int ret;

	lockdep_assert_held(&app_access_lock);

	for (;;) {
		ret = qcom_scm_qseecom_call_raw(QSEECOM_SMC_OWNER(smc_id),
						QSEECOM_SMC_SVC(smc_id),
						QSEECOM_SMC_FN(smc_id),
						arginfo, args, &res);
		if (ret != -EBUSY &&
		    (u32)res.result != (u32)-QSEE_RESULT_FAIL_APP_BUSY)
			break;
		if (retry_count++ >= QSEECOM_SCM_EBUSY_MAX_RETRY)
			break;
		if (retry_count == 33)
			pr_warn("secure world has been busy for 1 second!\n");

		mutex_unlock(&app_access_lock);
		msleep(QSEECOM_SCM_EBUSY_WAIT_MS);
		mutex_lock(&app_access_lock);
	}

	resp->result = res.result;
	resp->resp_type = res.resp_type;
	resp->data = res.data;

	pr_debug("smc_id 0x%x arginfo 0x%x: ret %d result 0x%x type 0x%x data 0x%x\n",
		 smc_id, arginfo, ret, resp->result, resp->resp_type,
		 resp->data);

	return ret;
}

static struct qseecom_registered_listener_list *__qseecom_find_svc(u32 listener_id)
{
	struct qseecom_registered_listener_list *entry;

	lockdep_assert_held(&listener_access_lock);

	list_for_each_entry(entry, &qseecom.registered_listener_list_head, list) {
		if (entry->svc.listener_id == listener_id)
			return entry;
	}

	pr_debug("Service id: %u is not found\n", listener_id);
	return NULL;
}

/*
 * QSEE doesn't support OS level cmds reentrancy until RE phase-3, and QSEE OS
 * level scm_call cmds will fail if there is any blocked TZ app. This platform
 * is at re-entrancy phase 2, so OS level calls first wait until all apps are
 * unblocked.
 */
static void qseecom_wait_no_app_blocked(u32 smc_id)
{
	u32 owner = QSEECOM_SMC_OWNER(smc_id);

	/* Owners 50 to 63 are trusted OS calls */
	if (owner < TZ_OWNER_QSEE_OS || owner > 63)
		return;

	while (qseecom.app_block_ref_cnt > 0) {
		int ret;

		mutex_unlock(&app_access_lock);
		ret = wait_event_killable(qseecom.app_block_wq,
					  !qseecom.app_block_ref_cnt);
		mutex_lock(&app_access_lock);
		if (ret)
			break;
	}
}

/*
 * scm_call of send data will fail if this TA is blocked or there are more
 * than one TA requesting listener services; So, first check to see if need
 * to wait.
 */
static void qseecom_wait_app_unblocked(struct qseecom_registered_app_list *ptr_app)
{
	ptr_app->check_block++;
	while (ptr_app->app_blocked || qseecom.app_block_ref_cnt > 1) {
		int ret;

		mutex_unlock(&app_access_lock);
		ret = wait_event_killable(qseecom.app_block_wq,
					  !ptr_app->app_blocked &&
					  qseecom.app_block_ref_cnt <= 1);
		mutex_lock(&app_access_lock);
		if (ret)
			break;
	}
	ptr_app->check_block--;
}

/*
 * Listeners
 */

static void qseecom_release_listener_resp_shm(struct qseecom_registered_listener_list *ptr_svc)
{
	int i;

	for (i = 0; i < MAX_ION_FD; i++) {
		if (!ptr_svc->resp_shm[i])
			continue;
		qseecom_shm_invalidate(ptr_svc->resp_shm[i]);
		qseecom_shm_put(ptr_svc->resp_shm[i]);
		ptr_svc->resp_shm[i] = NULL;
	}
}

static void __qseecom_clean_listener_sglistinfo(struct qseecom_registered_listener_list *ptr_svc)
{
	if (ptr_svc->sglist_cnt) {
		memset(ptr_svc->sglistinfo_ptr, 0, SGLISTINFO_TABLE_SIZE);
		ptr_svc->sglist_cnt = 0;
	}
	qseecom_release_listener_resp_shm(ptr_svc);
}

static int __qseecom_set_sb_memory(struct qseecom_registered_listener_list *svc,
				   struct qseecom_register_listener_req *listener)
{
	struct qseecom_command_scm_resp resp;
	u64 args[3];
	int ret;

	ret = qseecom_sbuf_map(listener->ifd_data_fd, &svc->sb);
	if (ret) {
		pr_err("failed to convert fd %d for lsnr %d with err: %d\n",
		       listener->ifd_data_fd, svc->svc.listener_id, ret);
		return ret;
	}

	args[0] = svc->svc.listener_id;
	args[1] = svc->sb.phys;
	args[2] = svc->sb.length;

	resp.result = QSEOS_RESULT_INCOMPLETE;

	mutex_unlock(&listener_access_lock);
	mutex_lock(&app_access_lock);
	qseecom_wait_no_app_blocked(TZ_OS_REGISTER_LISTENER_SMCINVOKE_ID);
	qseecom.smcinvoke_support = true;
	ret = qseecom_scm_call(TZ_OS_REGISTER_LISTENER_SMCINVOKE_ID,
			       QSEECOM_ARGS(3, QSEECOM_VAL, QSEECOM_RW, QSEECOM_VAL),
			       args, &resp);
	if (ret) {
		/* smcinvoke is not supported */
		qseecom.smcinvoke_support = false;
		ret = qseecom_scm_call(TZ_OS_REGISTER_LISTENER_ID,
				       QSEECOM_ARGS(3, QSEECOM_VAL, QSEECOM_RW,
						    QSEECOM_VAL),
				       args, &resp);
	}
	mutex_unlock(&app_access_lock);
	mutex_lock(&listener_access_lock);
	if (ret) {
		pr_err("qseecom_scm_call failed with err: %d\n", ret);
		ret = -EINVAL;
		goto err;
	}

	if (resp.result != QSEOS_RESULT_SUCCESS) {
		pr_err("Error SB registration req: resp.result = %d\n",
		       resp.result);
		ret = -EPERM;
		goto err;
	}
	return 0;
err:
	qseecom_sbuf_unmap(&svc->sb);
	return ret;
}

static int qseecom_register_listener(struct qseecom_dev_handle *data,
				     void __user *argp)
{
	struct qseecom_registered_listener_list *new_entry;
	struct qseecom_registered_listener_list *ptr_svc;
	struct list_head *pending = &qseecom.unregister_lsnr_pending_list_head;
	struct qseecom_register_listener_req rcvd_lstnr;
	int ret;

	if (data->listener.register_pending) {
		pr_err("Already a listener registration is in process on this FD\n");
		return -EINVAL;
	}

	if (copy_from_user(&rcvd_lstnr, argp, sizeof(rcvd_lstnr)))
		return -EFAULT;

	if (!access_ok((void __user *)rcvd_lstnr.virt_sb_base,
		       rcvd_lstnr.sb_size))
		return -EFAULT;

	ptr_svc = __qseecom_find_svc(data->listener.id);
	if (ptr_svc) {
		pr_err("Already a listener registered on this data: lid=%d\n",
		       data->listener.id);
		return -EINVAL;
	}

	ptr_svc = __qseecom_find_svc(rcvd_lstnr.listener_id);
	if (ptr_svc) {
		if (!ptr_svc->unregister_pending) {
			pr_err("Service %d is not unique\n",
			       rcvd_lstnr.listener_id);
			data->released = true;
			return -EBUSY;
		}

		/* wait until listener is unregistered */
		pr_debug("register %d has to wait\n", rcvd_lstnr.listener_id);
		mutex_unlock(&listener_access_lock);
		ret = wait_event_interruptible(qseecom.register_lsnr_pending_wq,
					       list_empty(pending));
		mutex_lock(&listener_access_lock);
		if (ret) {
			pr_err("interrupted register_pending_wq %d\n",
			       rcvd_lstnr.listener_id);
			return -ERESTARTSYS;
		}
	}

	new_entry = kzalloc_obj(*new_entry);
	if (!new_entry)
		return -ENOMEM;
	new_entry->svc = rcvd_lstnr;
	new_entry->rcv_req_flag = 0;

	new_entry->sglistinfo_ptr = qseecom_alloc_tzbuf(SGLISTINFO_TABLE_SIZE);
	if (!new_entry->sglistinfo_ptr) {
		kfree(new_entry);
		return -ENOMEM;
	}
	new_entry->user_virt_sb_base = (uintptr_t)rcvd_lstnr.virt_sb_base;
	data->listener.register_pending = true;
	ret = __qseecom_set_sb_memory(new_entry, &rcvd_lstnr);
	if (ret) {
		pr_err("qseecom_set_sb_memory failed for listener %d, size %d\n",
		       rcvd_lstnr.listener_id, rcvd_lstnr.sb_size);
		qcom_tzmem_free(new_entry->sglistinfo_ptr);
		kfree_sensitive(new_entry);
		data->listener.register_pending = false;
		return ret;
	}
	data->listener.register_pending = false;

	init_waitqueue_head(&new_entry->rcv_req_wq);
	init_waitqueue_head(&new_entry->listener_block_app_wq);
	new_entry->send_resp_flag = 0;
	new_entry->listener_in_use = false;
	list_add_tail(&new_entry->list, &qseecom.registered_listener_list_head);

	data->listener.id = rcvd_lstnr.listener_id;
	pr_debug("Service %d is registered\n", rcvd_lstnr.listener_id);
	return 0;
}

static int __qseecom_unregister_listener(struct qseecom_dev_handle *data,
					 struct qseecom_registered_listener_list *ptr_svc)
{
	struct qseecom_command_scm_resp resp;
	u64 args[1] = { data->listener.id };
	int ret;

	resp.result = QSEOS_RESULT_INCOMPLETE;

	mutex_unlock(&listener_access_lock);
	mutex_lock(&app_access_lock);
	qseecom_wait_no_app_blocked(TZ_OS_DEREGISTER_LISTENER_ID);
	ret = qseecom_scm_call(TZ_OS_DEREGISTER_LISTENER_ID,
			       QSEECOM_ARGS(1, QSEECOM_VAL), args, &resp);
	mutex_unlock(&app_access_lock);
	mutex_lock(&listener_access_lock);
	if (ret) {
		pr_err("scm_call() failed with err: %d (lstnr id=%d)\n",
		       ret, data->listener.id);
		return ret;
	}

	if (resp.result != QSEOS_RESULT_SUCCESS) {
		pr_err("Failed resp.result=%d,(lstnr id=%d)\n",
		       resp.result, data->listener.id);
		ret = -EPERM;
		goto exit;
	}

	while (atomic_read(&data->ioctl_count) > 1) {
		if (wait_event_interruptible(data->abort_wq,
					     atomic_read(&data->ioctl_count) <= 1)) {
			pr_err("Interrupted from abort\n");
			ret = -ERESTARTSYS;
		}
	}

exit:
	qseecom_release_listener_resp_shm(ptr_svc);
	qseecom_sbuf_unmap(&ptr_svc->sb);
	qcom_tzmem_free(ptr_svc->sglistinfo_ptr);
	list_del(&ptr_svc->list);
	kfree_sensitive(ptr_svc);

	data->released = true;
	pr_debug("Service %d is unregistered\n", data->listener.id);
	return ret;
}

static int qseecom_unregister_listener(struct qseecom_dev_handle *data)
{
	struct qseecom_registered_listener_list *ptr_svc;
	struct qseecom_unregister_pending_list *entry;

	if (data->released) {
		pr_err("Don't unregister lsnr %d\n", data->listener.id);
		return -EINVAL;
	}

	ptr_svc = __qseecom_find_svc(data->listener.id);
	if (!ptr_svc) {
		pr_err("Unregister invalid listener ID %d\n", data->listener.id);
		return -ENODATA;
	}
	/* stop CA thread waiting for listener response */
	ptr_svc->abort = 1;
	wake_up_all(&qseecom.send_resp_wq);

	/* stop listener thread waiting for listener request */
	data->abort = 1;
	wake_up_all(&ptr_svc->rcv_req_wq);

	/* return directly if pending */
	if (ptr_svc->unregister_pending)
		return 0;

	/* add unregistration into pending list */
	entry = kzalloc_obj(*entry);
	if (!entry)
		return -ENOMEM;
	entry->data = data;
	list_add_tail(&entry->list, &qseecom.unregister_lsnr_pending_list_head);
	ptr_svc->unregister_pending = true;
	pr_debug("unregister %d pending\n", data->listener.id);
	return 0;
}

static void qseecom_free_handle(struct qseecom_dev_handle *data)
{
	qseecom_sbuf_unmap(&data->client.sb);
	qcom_tzmem_free(data->sglistinfo_ptr);
	kfree_sensitive(data);
}

static void __qseecom_processing_pending_lsnr_unregister(void)
{
	struct qseecom_registered_listener_list *ptr_svc;
	struct qseecom_unregister_pending_list *entry;
	int ret;

	mutex_lock(&listener_access_lock);
	while (!list_empty(&qseecom.unregister_lsnr_pending_list_head)) {
		entry = list_first_entry(&qseecom.unregister_lsnr_pending_list_head,
					 struct qseecom_unregister_pending_list,
					 list);
		if (entry->data) {
			pr_debug("process pending unregister %d\n",
				 entry->data->listener.id);
			/* don't process the entry if qseecom_release is not called */
			if (!entry->data->listener.release_called) {
				list_move_tail(&entry->list,
					       &qseecom.unregister_lsnr_pending_list_head);
				break;
			}
			ptr_svc = __qseecom_find_svc(entry->data->listener.id);
			if (ptr_svc) {
				ret = __qseecom_unregister_listener(entry->data,
								    ptr_svc);
				if (ret) {
					pr_debug("unregister %d pending again\n",
						 entry->data->listener.id);
					mutex_unlock(&listener_access_lock);
					return;
				}
			} else {
				pr_err("invalid listener %d\n",
				       entry->data->listener.id);
			}
			qseecom_free_handle(entry->data);
		}
		list_del(&entry->list);
		kfree_sensitive(entry);
	}
	mutex_unlock(&listener_access_lock);
	wake_up_all(&qseecom.register_lsnr_pending_wq);
}

static void qseecom_unregister_lsnr_work(struct work_struct *work)
{
	__qseecom_processing_pending_lsnr_unregister();
}

static void __wakeup_unregister_listener_kthread(void)
{
	queue_work(system_unbound_wq, &qseecom.unregister_lsnr_work);
}

/*
 * Client shared buffer
 */

static int qseecom_set_client_mem_param(struct qseecom_dev_handle *data,
					void __user *argp)
{
	struct qseecom_set_sb_mem_param_req req;
	int ret;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	if (req.ifd_data_fd <= 0 || !req.virt_sb_base || req.sb_len == 0) {
		pr_err("Invalid input(s) fd(%d), sb_len(%d)\n",
		       req.ifd_data_fd, req.sb_len);
		return -EFAULT;
	}
	if (!access_ok((void __user *)req.virt_sb_base, req.sb_len))
		return -EFAULT;

	/* A new buffer replaces the previous one */
	qseecom_sbuf_unmap(&data->client.sb);
	data->client.sb_length = 0;
	data->client.user_virt_sb_base = 0;

	ret = qseecom_sbuf_map(req.ifd_data_fd, &data->client.sb);
	if (ret) {
		pr_err("failed to convert fd %d for client %d with err: %d\n",
		       req.ifd_data_fd, data->client.app_id, ret);
		return ret;
	}

	if (data->client.sb.length < req.sb_len) {
		pr_err("Requested length (0x%x) is > allocated (%zu)\n",
		       req.sb_len, data->client.sb.length);
		qseecom_sbuf_unmap(&data->client.sb);
		return -EINVAL;
	}
	data->client.sb_length = req.sb_len;
	data->client.user_virt_sb_base = (uintptr_t)req.virt_sb_base;

	return 0;
}

/*
 * Listener requests while a command is in progress
 */

static int qseecom_listener_has_sent_rsp(struct qseecom_dev_handle *data,
					 struct qseecom_registered_listener_list *ptr_svc)
{
	return ptr_svc->send_resp_flag || data->abort || ptr_svc->abort;
}

static int qseecom_send_listener_resp(u32 lstnr, u32 status,
				      struct qseecom_registered_listener_list *ptr_svc,
				      struct qseecom_command_scm_resp *resp)
{
	u64 args[4] = { lstnr, status };

	if (qseecom.whitelist_support && ptr_svc) {
		args[2] = qcom_tzmem_to_phys(ptr_svc->sglistinfo_ptr);
		args[3] = SGLISTINFO_TABLE_SIZE;
		return qseecom_scm_call(TZ_OS_LISTENER_RESPONSE_HANDLER_WITH_WHITELIST_ID,
					QSEECOM_ARGS(4, QSEECOM_VAL, QSEECOM_VAL,
						     QSEECOM_RW, QSEECOM_VAL),
					args, resp);
	}

	return qseecom_scm_call(TZ_OS_LISTENER_RESPONSE_HANDLER_ID,
				QSEECOM_ARGS(2, QSEECOM_VAL, QSEECOM_VAL),
				args, resp);
}

/*
 * Serve listener requests for OS level commands (app load/unload, services).
 * Called with app_access_lock held, which stays held while waiting.
 */
static int __qseecom_process_incomplete_cmd(struct qseecom_dev_handle *data,
					    struct qseecom_command_scm_resp *resp)
{
	struct qseecom_registered_listener_list *ptr_svc;
	u32 lstnr, status;
	int ret = 0;
	int rc = 0;

	qseecom.app_block_ref_cnt++;
	while (!ret && resp->result == QSEOS_RESULT_INCOMPLETE) {
		lstnr = resp->data;
		/* Wake up blocking listener service with the lstnr id */
		mutex_lock(&listener_access_lock);
		ptr_svc = __qseecom_find_svc(lstnr);
		if (!ptr_svc) {
			pr_err("Listener Svc %d does not exist\n", lstnr);
			rc = -EINVAL;
			status = QSEOS_RESULT_FAILURE;
			goto err_resp;
		}

		ptr_svc->listener_in_use = true;
		ptr_svc->rcv_req_flag = 1;
		ret = qseecom_shm_invalidate(ptr_svc->sb.shm);
		if (ret) {
			rc = -EINVAL;
			status = QSEOS_RESULT_FAILURE;
			goto err_resp;
		}
		wake_up_all(&ptr_svc->rcv_req_wq);

		if (ptr_svc->abort == 1) {
			pr_debug("Service %d abort %d\n", lstnr, ptr_svc->abort);
			rc = -ENODEV;
			status = QSEOS_RESULT_FAILURE;
			goto err_resp;
		}

		pr_debug("waking up rcv_req_wq and waiting for send_resp_wq\n");

		/* Not interruptible: QSEE waits for the listener's answer. */
		mutex_unlock(&listener_access_lock);
		wait_event_idle(qseecom.send_resp_wq,
				qseecom_listener_has_sent_rsp(data, ptr_svc));
		mutex_lock(&listener_access_lock);
		if (data->abort || ptr_svc->abort) {
			pr_err("Abort clnt %d waiting on lstnr svc %d, ret %d\n",
			       data->client.app_id, lstnr, ret);
			rc = -ENODEV;
			status = QSEOS_RESULT_FAILURE;
		} else {
			status = QSEOS_RESULT_SUCCESS;
		}
err_resp:
		qseecom.send_resp_flag = 0;
		if (ptr_svc) {
			ptr_svc->send_resp_flag = 0;
			ret = qseecom_shm_clean(ptr_svc->sb.shm);
			if (ret)
				goto exit;

			ret = qseecom_send_listener_resp(lstnr, status, ptr_svc,
							 resp);
			ptr_svc->listener_in_use = false;
			__qseecom_clean_listener_sglistinfo(ptr_svc);
			wake_up_all(&ptr_svc->listener_block_app_wq);
		} else {
			ret = qseecom_send_listener_resp(lstnr, status, NULL,
							 resp);
		}
		if (ret) {
			pr_err("scm_call() failed with err: %d (app_id = %d)\n",
			       ret, data->client.app_id);
			goto exit;
		}

		pr_debug("resp status %d, res= %d, app_id = %d, lstr = %d\n",
			 status, resp->result, data->client.app_id, lstnr);
		if (resp->result != QSEOS_RESULT_SUCCESS &&
		    resp->result != QSEOS_RESULT_INCOMPLETE) {
			pr_err("fail:resp res= %d,app_id = %d,lstr = %d\n",
			       resp->result, data->client.app_id, lstnr);
			ret = -EINVAL;
		}
exit:
		mutex_unlock(&listener_access_lock);
	}
	qseecom.app_block_ref_cnt--;
	wake_up_all(&qseecom.app_block_wq);
	if (rc)
		return rc;

	return ret;
}

static int qseecom_process_blocked_on_listener(struct qseecom_command_scm_resp *resp,
					       struct qseecom_registered_app_list *ptr_app,
					       struct qseecom_dev_handle *data)
{
	struct qseecom_registered_app_list dummy_app_entry = {};
	struct qseecom_registered_listener_list *list_ptr;
	struct qseecom_command_scm_resp continue_resp;
	unsigned int session_id;
	unsigned long flags;
	bool found_app = false;
	u64 args[1];
	int ret = 0;

	/* find app_id & img_name from list */
	if (!ptr_app) {
		if (data->client.from_loadapp) {
			pr_debug("This request is from load_app\n");
			ptr_app = &dummy_app_entry;
			ptr_app->app_id = data->client.app_id;
		} else {
			spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
			list_for_each_entry(ptr_app, &qseecom.registered_app_list_head, list) {
				if (ptr_app->app_id == data->client.app_id &&
				    !strcmp(ptr_app->app_name, data->client.app_name)) {
					found_app = true;
					break;
				}
			}
			spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);
			if (!found_app) {
				pr_err("app_id %d (%s) is not found\n",
				       data->client.app_id, data->client.app_name);
				return -ENOENT;
			}
		}
	}

	do {
		session_id = resp->resp_type;
		mutex_lock(&listener_access_lock);
		list_ptr = __qseecom_find_svc(resp->data);
		if (!list_ptr) {
			pr_err("Invalid listener ID %d\n", resp->data);
			mutex_unlock(&listener_access_lock);
			return -ENODATA;
		}
		ptr_app->blocked_on_listener_id = resp->data;

		pr_warn("Lsntr %d in_use %d, block session(%d) app(%d)\n",
			resp->data, list_ptr->listener_in_use,
			session_id, data->client.app_id);

		/* sleep until listener is available */
		do {
			qseecom.app_block_ref_cnt++;
			ptr_app->app_blocked = true;
			mutex_unlock(&listener_access_lock);
			mutex_unlock(&app_access_lock);
			wait_event_idle(list_ptr->listener_block_app_wq,
					!list_ptr->listener_in_use);
			mutex_lock(&app_access_lock);
			mutex_lock(&listener_access_lock);
			ptr_app->app_blocked = false;
			qseecom.app_block_ref_cnt--;
		} while (list_ptr->listener_in_use);

		ptr_app->blocked_on_listener_id = 0;
		pr_warn("Lsntr %d is available, unblock session(%d) app(%d)\n",
			resp->data, session_id, data->client.app_id);

		/* notify TZ that listener is available */
		if (qseecom.smcinvoke_support) {
			args[0] = session_id;
			ret = qseecom_scm_call(TZ_OS_CONTINUE_BLOCKED_REQUEST_SMCINVOKE_ID,
					       QSEECOM_ARGS(1, QSEECOM_VAL),
					       args, &continue_resp);
			if (ret) {
				/* retry with legacy cmd */
				pr_warn("falling back to legacy method\n");
				args[0] = data->client.app_id;
				ret = qseecom_scm_call(TZ_OS_CONTINUE_BLOCKED_REQUEST_ID,
						       QSEECOM_ARGS(1, QSEECOM_VAL),
						       args, &continue_resp);
			}
		} else {
			args[0] = data->client.app_id;
			ret = qseecom_scm_call(TZ_OS_CONTINUE_BLOCKED_REQUEST_ID,
					       QSEECOM_ARGS(1, QSEECOM_VAL),
					       args, &continue_resp);
		}
		mutex_unlock(&listener_access_lock);
		if (ret) {
			pr_err("unblock app %d or session %d fail\n",
			       data->client.app_id, session_id);
			return ret;
		}
		*resp = continue_resp;
		pr_debug("unblock resp = %d\n", resp->result);
	} while (resp->result == QSEOS_RESULT_BLOCKED_ON_LISTENER);

	if (resp->result != QSEOS_RESULT_INCOMPLETE) {
		pr_err("Unexpected unblock resp %d\n", resp->result);
		ret = -EINVAL;
	}

	return ret;
}

/*
 * Serve listener requests for a TA command. Called with app_access_lock held;
 * both locks are dropped while waiting for the listener.
 */
static int __qseecom_reentrancy_process_incomplete_cmd(struct qseecom_dev_handle *data,
						       struct qseecom_command_scm_resp *resp)
{
	struct qseecom_registered_listener_list *ptr_svc;
	u32 lstnr, status;
	int ret = 0;
	int rc = 0;

	while (ret == 0 && resp->result == QSEOS_RESULT_INCOMPLETE) {
		lstnr = resp->data;
		/* Wake up blocking listener service with the lstnr id */
		mutex_lock(&listener_access_lock);
		ptr_svc = __qseecom_find_svc(lstnr);
		if (!ptr_svc) {
			pr_err("Listener Svc %d does not exist\n", lstnr);
			rc = -EINVAL;
			status = QSEOS_RESULT_FAILURE;
			goto err_resp;
		}

		ptr_svc->listener_in_use = true;
		ptr_svc->rcv_req_flag = 1;
		ret = qseecom_shm_invalidate(ptr_svc->sb.shm);
		if (ret) {
			rc = -EINVAL;
			status = QSEOS_RESULT_FAILURE;
			goto err_resp;
		}
		wake_up_all(&ptr_svc->rcv_req_wq);

		if (ptr_svc->abort == 1) {
			pr_debug("Service %d abort %d\n", lstnr, ptr_svc->abort);
			rc = -ENODEV;
			status = QSEOS_RESULT_FAILURE;
			goto err_resp;
		}

		pr_debug("waking up rcv_req_wq and waiting for send_resp_wq\n");

		/* unlock mutex btw waking listener and sleep-wait */
		mutex_unlock(&listener_access_lock);
		mutex_unlock(&app_access_lock);
		/* Not interruptible: QSEE waits for the listener's answer. */
		wait_event_idle(qseecom.send_resp_wq,
				qseecom_listener_has_sent_rsp(data, ptr_svc));
		/* lock mutex again after resp sent */
		mutex_lock(&app_access_lock);
		mutex_lock(&listener_access_lock);
		ptr_svc->send_resp_flag = 0;
		qseecom.send_resp_flag = 0;

		if (data->abort || ptr_svc->abort) {
			pr_err("Abort clnt %d waiting on lstnr svc %d, ret %d\n",
			       data->client.app_id, lstnr, ret);
			rc = -ENODEV;
			status = QSEOS_RESULT_FAILURE;
		} else {
			status = QSEOS_RESULT_SUCCESS;
		}
err_resp:
		if (ptr_svc) {
			ret = qseecom_shm_clean(ptr_svc->sb.shm);
			if (ret)
				goto exit;

			ret = qseecom_send_listener_resp(lstnr, status, ptr_svc,
							 resp);
			ptr_svc->listener_in_use = false;
			__qseecom_clean_listener_sglistinfo(ptr_svc);
			wake_up_all(&ptr_svc->listener_block_app_wq);
		} else {
			ret = qseecom_send_listener_resp(lstnr, status, NULL,
							 resp);
		}
		if (ret) {
			pr_err("scm_call() failed with err: %d (app_id = %d)\n",
			       ret, data->client.app_id);
			goto exit;
		}

		switch (resp->result) {
		case QSEOS_RESULT_BLOCKED_ON_LISTENER:
			pr_warn("send lsr %d rsp, but app %d block on lsr %d\n",
				lstnr, data->client.app_id, resp->data);
			if (lstnr == resp->data) {
				pr_err("lstnr %d should not be blocked!\n", lstnr);
				ret = -EINVAL;
				goto exit;
			}
			mutex_unlock(&listener_access_lock);
			ret = qseecom_process_blocked_on_listener(resp, NULL, data);
			mutex_lock(&listener_access_lock);
			if (ret) {
				pr_err("failed to process App(%d) %s blocked on listener %d\n",
				       data->client.app_id,
				       data->client.app_name, resp->data);
				goto exit;
			}
			fallthrough;
		case QSEOS_RESULT_SUCCESS:
		case QSEOS_RESULT_INCOMPLETE:
			break;
		case QSEOS_RESULT_CBACK_REQUEST:
			pr_warn("get cback req app_id = %d, resp->data = %d\n",
				data->client.app_id, resp->data);
			resp->resp_type = SMCINVOKE_RESULT_INBOUND_REQ_NEEDED;
			break;
		default:
			pr_err("fail:resp res= %d,app_id = %d,lstr = %d\n",
			       resp->result, data->client.app_id, lstnr);
			ret = -EINVAL;
			goto exit;
		}
exit:
		mutex_unlock(&listener_access_lock);
	}
	if (rc)
		return rc;

	return ret;
}

static int __qseecom_process_reentrancy(struct qseecom_command_scm_resp *resp,
					struct qseecom_registered_app_list *ptr_app,
					struct qseecom_dev_handle *data)
{
	int ret;

	switch (resp->result) {
	case QSEOS_RESULT_BLOCKED_ON_LISTENER:
		pr_warn("App(%d) %s is blocked on listener %d\n",
			data->client.app_id, data->client.app_name, resp->data);
		ret = qseecom_process_blocked_on_listener(resp, ptr_app, data);
		if (ret) {
			pr_err("failed to process App(%d) %s is blocked on listener %d\n",
			       data->client.app_id, data->client.app_name,
			       resp->data);
			return ret;
		}
		/* process the incomplete request that follows */
		fallthrough;
	case QSEOS_RESULT_INCOMPLETE:
		qseecom.app_block_ref_cnt++;
		ptr_app->app_blocked = true;
		ret = __qseecom_reentrancy_process_incomplete_cmd(data, resp);
		ptr_app->app_blocked = false;
		qseecom.app_block_ref_cnt--;
		wake_up_all(&qseecom.app_block_wq);
		if (ret)
			pr_err("process_incomplete_cmd failed err: %d\n", ret);
		return ret;
	case QSEOS_RESULT_SUCCESS:
		return 0;
	default:
		pr_err("Response result %d not supported\n", resp->result);
		return -EINVAL;
	}
}

/*
 * Apps
 */

static int __qseecom_check_app_exists(const char *app_name, u32 *app_id)
{
	struct qseecom_registered_app_list *entry;
	struct qseecom_command_scm_resp resp = {};
	unsigned long flags;
	bool found_app = false;
	size_t name_len;
	u64 args[2];
	char *name;
	int ret;

	*app_id = 0;

	/* check if app exists and has been registered locally */
	spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
	list_for_each_entry(entry, &qseecom.registered_app_list_head, list) {
		if (!strcmp(entry->app_name, app_name)) {
			found_app = true;
			*app_id = entry->app_id;
			break;
		}
	}
	spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);
	if (found_app) {
		pr_debug("Found app with id %d\n", *app_id);
		return 0;
	}

	name_len = strnlen(app_name, MAX_APP_NAME_SIZE - 1);
	name = qseecom_alloc_tzbuf(MAX_APP_NAME_SIZE);
	if (!name)
		return -ENOMEM;
	memcpy(name, app_name, name_len);

	args[0] = qcom_tzmem_to_phys(name);
	args[1] = name_len;

	/* SCM_CALL to check if app_id for the mentioned app exists */
	qseecom_wait_no_app_blocked(TZ_OS_APP_LOOKUP_ID);
	ret = qseecom_scm_call(TZ_OS_APP_LOOKUP_ID,
			       QSEECOM_ARGS(2, QSEECOM_RW, QSEECOM_VAL),
			       args, &resp);
	qcom_tzmem_free(name);
	if (ret) {
		pr_err("scm_call to check if app is already loaded failed\n");
		return -EINVAL;
	}

	if (resp.result == QSEOS_RESULT_FAILURE)
		return 0;

	switch (resp.resp_type) {
	/* qsee returned listener type response */
	case QSEOS_LISTENER_ID:
		pr_err("resp type is of listener type instead of app\n");
		return -EINVAL;
	case QSEOS_APP_ID:
		*app_id = resp.data;
		return 0;
	default:
		pr_err("invalid resp type (%d) from qsee\n", resp.resp_type);
		return -ENODEV;
	}
}

static int qseecom_load_app(struct qseecom_dev_handle *data, void __user *argp)
{
	struct qseecom_registered_app_list *entry = NULL;
	struct qseecom_command_scm_resp resp;
	struct qseecom_load_img_req load_img_req;
	struct qseecom_shm *shm = NULL;
	bool first_time = false;
	unsigned long flags;
	u32 app_id = 0;
	u64 args[3];
	int ret;

	/* Copy the relevant information needed for loading the image */
	if (copy_from_user(&load_img_req, argp, sizeof(load_img_req)))
		return -EFAULT;

	/*
	 * The common libraries are loaded by the bootloader on this platform
	 * (64-bit one by UEFI), so they are never loaded from here.
	 */

	load_img_req.img_name[MAX_APP_NAME_SIZE - 1] = '\0';

	ret = __qseecom_check_app_exists(load_img_req.img_name, &app_id);
	if (ret < 0)
		return ret;

	if (app_id) {
		pr_debug("App id %d (%s) already exists\n", app_id,
			 load_img_req.img_name);
		spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
		list_for_each_entry(entry, &qseecom.registered_app_list_head, list) {
			if (entry->app_id == app_id) {
				if (entry->ref_cnt == U32_MAX) {
					spin_unlock_irqrestore(&qseecom.registered_app_list_lock,
							       flags);
					pr_err("App %d (%s) ref_cnt overflow\n",
					       app_id, load_img_req.img_name);
					return -EINVAL;
				}
				entry->ref_cnt++;
				break;
			}
		}
		spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);
		ret = 0;
	} else {
		first_time = true;
		pr_warn("App (%s) doesn't exist, loading apps for first time\n",
			load_img_req.img_name);

		shm = qseecom_shm_get(load_img_req.ifd_data_fd);
		if (IS_ERR(shm)) {
			pr_err("could not map the image buffer\n");
			return PTR_ERR(shm);
		}

		if (load_img_req.mdt_len > shm->size ||
		    load_img_req.img_len > shm->size) {
			pr_err("buffer len %zu is smaller than mdt_len %u or img_len %u\n",
			       shm->size, load_img_req.mdt_len,
			       load_img_req.img_len);
			ret = -EINVAL;
			goto loadapp_err;
		}

		args[0] = load_img_req.mdt_len;
		args[1] = load_img_req.img_len;
		args[2] = shm->paddr;

		ret = qseecom_shm_clean(shm);
		if (ret) {
			pr_err("cache operation failed %d\n", ret);
			goto loadapp_err;
		}

		/* SCM_CALL to load the app and get the app_id back */
		qseecom_wait_no_app_blocked(TZ_OS_APP_START_ID);
		ret = qseecom_scm_call(TZ_OS_APP_START_ID,
				       QSEECOM_ARGS(3, QSEECOM_VAL, QSEECOM_VAL,
						    QSEECOM_VAL),
				       args, &resp);
		if (ret) {
			pr_err("scm_call to load app failed\n");
			ret = -EINVAL;
			goto loadapp_err;
		}
		ret = qseecom_shm_invalidate(shm);
		if (ret) {
			pr_err("cache operation failed %d\n", ret);
			goto loadapp_err;
		}

		do {
			if (resp.result == QSEOS_RESULT_FAILURE) {
				pr_err("scm_call rsp.result is QSEOS_RESULT_FAILURE\n");
				ret = -EFAULT;
				goto loadapp_err;
			}

			if (resp.result == QSEOS_RESULT_INCOMPLETE) {
				ret = __qseecom_process_incomplete_cmd(data, &resp);
				if (ret) {
					/* TZ has created app_id, need to unload it */
					pr_err("incomp_cmd err %d, %d, unload %d %s\n",
					       ret, resp.result, resp.data,
					       load_img_req.img_name);
					__qseecom_unload_app(data, resp.data);
					ret = -EFAULT;
					goto loadapp_err;
				}
			}

			if (resp.result == QSEOS_RESULT_BLOCKED_ON_LISTENER) {
				pr_err("load app blocked on listener\n");
				data->client.app_id = resp.result;
				data->client.from_loadapp = true;
				ret = qseecom_process_blocked_on_listener(&resp, NULL,
									  data);
				data->client.from_loadapp = false;
				data->client.app_id = 0;
				if (ret) {
					pr_err("load app fail proc block on listener,ret :%d\n",
					       ret);
					ret = -EFAULT;
					goto loadapp_err;
				}
			}
		} while (resp.result == QSEOS_RESULT_BLOCKED_ON_LISTENER ||
			 resp.result == QSEOS_RESULT_INCOMPLETE);

		if (resp.result != QSEOS_RESULT_SUCCESS) {
			pr_err("scm_call failed resp.result unknown, %d\n",
			       resp.result);
			ret = -EFAULT;
			goto loadapp_err;
		}

		app_id = resp.data;

		entry = kzalloc_obj(*entry);
		if (!entry) {
			ret = -ENOMEM;
			goto loadapp_err;
		}
		entry->app_id = app_id;
		entry->ref_cnt = 1;
		entry->app_arch = load_img_req.app_arch;
		/*
		 * keymaster app may be first loaded as "keymaste" by qseecomd,
		 * and then used as "keymaster" on some targets. To avoid app
		 * name checking error, register "keymaster" into app_list and
		 * thread private data.
		 */
		if (!strcmp(load_img_req.img_name, "keymaste"))
			strscpy(entry->app_name, "keymaster", MAX_APP_NAME_SIZE);
		else
			strscpy(entry->app_name, load_img_req.img_name,
				MAX_APP_NAME_SIZE);
		entry->app_blocked = false;
		entry->blocked_on_listener_id = 0;
		entry->check_block = 0;

		spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
		list_add_tail(&entry->list, &qseecom.registered_app_list_head);
		spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);

		pr_warn("App with id %u (%s) now loaded\n", app_id,
			load_img_req.img_name);
	}
	data->client.app_id = app_id;
	data->client.app_arch = load_img_req.app_arch;
	if (!strcmp(load_img_req.img_name, "keymaste"))
		strscpy(data->client.app_name, "keymaster", MAX_APP_NAME_SIZE);
	else
		strscpy(data->client.app_name, load_img_req.img_name,
			MAX_APP_NAME_SIZE);
	load_img_req.app_id = app_id;
	if (copy_to_user(argp, &load_img_req, sizeof(load_img_req))) {
		pr_err("copy_to_user failed\n");
		ret = -EFAULT;
		if (first_time) {
			spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
			list_del(&entry->list);
			spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);
			kfree_sensitive(entry);
		}
	}

loadapp_err:
	qseecom_shm_put(shm);
	return ret;
}

static int __qseecom_cleanup_app(struct qseecom_dev_handle *data)
{
	int ret = 0;

	wake_up_all(&qseecom.send_resp_wq);
	mutex_unlock(&app_access_lock);
	while (atomic_read(&data->ioctl_count) > 1) {
		if (wait_event_interruptible(data->abort_wq,
					     atomic_read(&data->ioctl_count) <= 1)) {
			pr_err("Interrupted from abort\n");
			ret = -ERESTARTSYS;
			break;
		}
	}
	mutex_lock(&app_access_lock);
	return ret;
}

static int __qseecom_unload_app(struct qseecom_dev_handle *data, u32 app_id)
{
	struct qseecom_command_scm_resp resp;
	u64 args[1] = { app_id };
	int ret;

	/* SCM_CALL to unload the app */
	ret = qseecom_scm_call(TZ_OS_APP_SHUTDOWN_ID,
			       QSEECOM_ARGS(1, QSEECOM_VAL), args, &resp);
	if (ret) {
		pr_err("scm_call to unload app (id = %d) failed ret: %d\n",
		       app_id, ret);
		return ret;
	}

	do {
		switch (resp.result) {
		case QSEOS_RESULT_SUCCESS:
			pr_warn("App (%d) is unloaded\n", app_id);
			break;
		case QSEOS_RESULT_INCOMPLETE:
			ret = __qseecom_process_incomplete_cmd(data, &resp);
			if (ret)
				pr_err("unload app %d fail proc incom cmd: %d,%d,%d\n",
				       app_id, ret, resp.result, resp.data);
			else
				pr_warn("App (%d) is unloaded\n", app_id);
			break;
		case QSEOS_RESULT_FAILURE:
			pr_err("app (%d) unload_failed!!\n", app_id);
			ret = -EFAULT;
			break;
		case QSEOS_RESULT_BLOCKED_ON_LISTENER:
			pr_err("unload app (%d) blocked on listener\n", app_id);
			ret = qseecom_process_blocked_on_listener(&resp, NULL, data);
			if (ret) {
				pr_err("unload app fail proc block on listener cmd,ret :%d\n",
				       ret);
				ret = -EFAULT;
			}
			break;
		default:
			pr_err("unload app %d get unknown resp.result %d\n",
			       app_id, resp.result);
			ret = -EFAULT;
			break;
		}
	} while (!ret && (resp.result == QSEOS_RESULT_INCOMPLETE ||
			  resp.result == QSEOS_RESULT_BLOCKED_ON_LISTENER));
	return ret;
}

static int qseecom_unload_app(struct qseecom_dev_handle *data, bool app_crash)
{
	struct qseecom_registered_app_list *ptr_app;
	bool found_app = false;
	unsigned long flags;
	int ret = 0;

	pr_debug("unload app %d(%s), app_crash flag %d\n", data->client.app_id,
		 data->client.app_name, app_crash);

	if (!memcmp(data->client.app_name, "keymaste", strlen("keymaste"))) {
		pr_debug("Do not unload keymaster app from tz\n");
		goto unload_exit;
	}

	ret = __qseecom_cleanup_app(data);
	if (ret && !app_crash) {
		pr_err("cleanup app failed, pending ioctl:%d\n",
		       atomic_read(&data->ioctl_count));
		return ret;
	}

	qseecom_wait_no_app_blocked(TZ_OS_APP_SHUTDOWN_ID);

	/* ignore app_id 0, it happens when close qseecom_fd if load app fail */
	if (!data->client.app_id)
		goto unload_exit;

	spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
	list_for_each_entry(ptr_app, &qseecom.registered_app_list_head, list) {
		if (ptr_app->app_id == data->client.app_id &&
		    !strcmp(ptr_app->app_name, data->client.app_name)) {
			pr_debug("unload app %d (%s), ref_cnt %d\n",
				 ptr_app->app_id, ptr_app->app_name,
				 ptr_app->ref_cnt);
			ptr_app->ref_cnt--;
			found_app = true;
			break;
		}
	}
	spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);
	if (!found_app) {
		pr_err("Cannot find app with id = %d (%s)\n",
		       data->client.app_id, data->client.app_name);
		ret = -EINVAL;
		goto unload_exit;
	}

	if (!ptr_app->ref_cnt) {
		/*
		 * An app that was already loaded when its first client asked
		 * for it belongs to the bootloader. The stock driver shuts
		 * such an app down with its last client; here it stays loaded,
		 * so a mere query can never take a boot-time app away.
		 */
		if (ptr_app->preloaded)
			ret = 0;
		else
			ret = __qseecom_unload_app(data, data->client.app_id);
		if (ret == -EBUSY) {
			/*
			 * If unload failed due to EBUSY, don't free mem
			 * just restore app ref_cnt and return -EBUSY
			 */
			pr_warn("unload ta %d(%s) EBUSY\n",
				data->client.app_id, data->client.app_name);
			ptr_app->ref_cnt++;
			return ret;
		}
		spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
		list_del(&ptr_app->list);
		spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);
		kfree_sensitive(ptr_app);
	}

unload_exit:
	qseecom_sbuf_unmap(&data->client.sb);
	data->released = true;
	return ret;
}

static int qseecom_prepare_unload_app(struct qseecom_dev_handle *data)
{
	struct qseecom_unload_app_pending_list *entry;

	pr_debug("prepare to unload app(%d)(%s), pending %d\n",
		 data->client.app_id, data->client.app_name,
		 data->client.unload_pending);

	/*
	 * The keymaster TA is never unloaded, so it is not added to the
	 * pending list: only the client's shared buffer is released.
	 */
	if (!memcmp(data->client.app_name, "keymaste", strlen("keymaste"))) {
		qseecom_free_handle(data);
		return 0;
	}

	if (data->client.unload_pending)
		return 0;
	entry = kzalloc_obj(*entry);
	if (!entry)
		return -ENOMEM;
	entry->data = data;
	list_add_tail(&entry->list, &qseecom.unload_app_pending_list_head);
	data->client.unload_pending = true;
	pr_debug("unload ta %d pending\n", data->client.app_id);
	return 0;
}

static void __wakeup_unload_app_kthread(void)
{
	queue_work(system_unbound_wq, &qseecom.unload_app_work);
}

static bool __qseecom_find_pending_unload_app(u32 app_id, char *app_name)
{
	struct qseecom_unload_app_pending_list *entry;
	bool found = false;

	mutex_lock(&unload_app_pending_list_lock);
	list_for_each_entry(entry, &qseecom.unload_app_pending_list_head, list) {
		if (entry->data->client.app_id == app_id &&
		    !strcmp(entry->data->client.app_name, app_name)) {
			found = true;
			break;
		}
	}
	mutex_unlock(&unload_app_pending_list_lock);
	return found;
}

static void __qseecom_processing_pending_unload_app(void)
{
	struct qseecom_unload_app_pending_list *entry;
	int ret;

	mutex_lock(&unload_app_pending_list_lock);
	while (!list_empty(&qseecom.unload_app_pending_list_head)) {
		entry = list_first_entry(&qseecom.unload_app_pending_list_head,
					 struct qseecom_unload_app_pending_list,
					 list);
		if (entry->data) {
			pr_debug("process pending unload app %d (%s)\n",
				 entry->data->client.app_id,
				 entry->data->client.app_name);
			mutex_unlock(&unload_app_pending_list_lock);
			mutex_lock(&app_access_lock);
			ret = qseecom_unload_app(entry->data, true);
			if (ret)
				pr_err("unload app %d pending failed %d\n",
				       entry->data->client.app_id, ret);
			mutex_unlock(&app_access_lock);
			mutex_lock(&unload_app_pending_list_lock);
			qseecom_free_handle(entry->data);
		}
		list_del(&entry->list);
		kfree_sensitive(entry);
	}
	mutex_unlock(&unload_app_pending_list_lock);
}

static void qseecom_unload_app_work(struct work_struct *work)
{
	__qseecom_processing_pending_unload_app();
}

/*
 * Commands
 */

static phys_addr_t __qseecom_uvirt_to_kphys(struct qseecom_dev_handle *data,
					    uintptr_t virt)
{
	return data->client.sb.phys + (virt - data->client.user_virt_sb_base);
}

static void *__qseecom_uvirt_to_kvirt(struct qseecom_dev_handle *data,
				      uintptr_t virt)
{
	return data->client.sb.virt + (virt - data->client.user_virt_sb_base);
}

static int __validate_send_service_cmd_inputs(struct qseecom_dev_handle *data,
					      struct qseecom_send_svc_cmd_req *req)
{
	uintptr_t base = data->client.user_virt_sb_base;
	size_t sb_length = data->client.sb_length;
	uintptr_t cmd = (uintptr_t)req->cmd_req_buf;
	uintptr_t rsp = (uintptr_t)req->resp_buf;

	if (!req->resp_buf || !req->cmd_req_buf) {
		pr_err("req or cmd buffer or response buffer is null\n");
		return -EINVAL;
	}

	if (!data->client.sb.virt) {
		pr_err("Client or client buf is not initialized\n");
		return -EINVAL;
	}

	if (base == 0) {
		pr_err("user_virt_sb_base is null\n");
		return -EINVAL;
	}

	if (sb_length == 0) {
		pr_err("sb_length is 0\n");
		return -EINVAL;
	}

	if (cmd < base || cmd >= base + sb_length) {
		pr_err("cmd buffer address not within shared buffer\n");
		return -EINVAL;
	}
	if (rsp < base || rsp >= base + sb_length) {
		pr_err("response buffer address not within shared buffer\n");
		return -EINVAL;
	}
	if (req->cmd_req_len == 0 || req->resp_len == 0 ||
	    req->cmd_req_len > sb_length || req->resp_len > sb_length) {
		pr_err("cmd buf length or response buf length not valid\n");
		return -EINVAL;
	}
	if (req->cmd_req_len > UINT_MAX - req->resp_len) {
		pr_err("Integer overflow detected in req_len & rsp_len\n");
		return -EINVAL;
	}

	if ((req->cmd_req_len + req->resp_len) > sb_length) {
		pr_debug("Not enough memory to fit cmd_buf.\n");
		pr_debug("resp_buf. Required: %u, Available: %zu\n",
			 (req->cmd_req_len + req->resp_len), sb_length);
		return -ENOMEM;
	}
	if (cmd > (ULONG_MAX - req->cmd_req_len)) {
		pr_err("Integer overflow in req_len & cmd_req_buf\n");
		return -EINVAL;
	}
	if (rsp > (ULONG_MAX - req->resp_len)) {
		pr_err("Integer overflow in resp_len & resp_buf\n");
		return -EINVAL;
	}
	if (base > (ULONG_MAX - sb_length)) {
		pr_err("Integer overflow in user_virt_sb_base & sb_length\n");
		return -EINVAL;
	}
	if ((cmd + req->cmd_req_len) > (base + sb_length) ||
	    (rsp + req->resp_len) > (base + sb_length)) {
		pr_err("cmd buf or resp buf is out of shared buffer region\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * Only the read-only RPMB provisioning status query is supported. RPMB key
 * provisioning and RPMB erase cannot be undone and are always refused; the
 * FSM, key and fuse service commands are not supported.
 */
static int qseecom_send_service_cmd(struct qseecom_dev_handle *data,
				    struct qseecom_send_svc_cmd_req *req)
{
	struct qseecom_command_scm_resp resp;
	int ret;

	if (__validate_send_service_cmd_inputs(data, req))
		return -EINVAL;

	data->type = QSEECOM_SECURE_SERVICE;

	if (req->cmd_id != QSEOS_RPMB_CHECK_PROV_STATUS_COMMAND) {
		pr_err("Unsupported cmd_id %d\n", req->cmd_id);
		return -EINVAL;
	}

	/* Clients need to ensure req_buf is at base offset of shared buffer */
	if ((uintptr_t)req->cmd_req_buf != data->client.user_virt_sb_base) {
		pr_err("cmd buf not pointing to base offset of shared buffer\n");
		return -EINVAL;
	}

	if (data->client.sb_length < sizeof(struct qseecom_rpmb_provision_key)) {
		pr_err("shared buffer is too small to hold key type\n");
		return -EINVAL;
	}

	ret = qseecom_shm_clean(data->client.sb.shm);
	if (ret) {
		pr_err("cache operation failed %d\n", ret);
		return ret;
	}

	qseecom_wait_no_app_blocked(TZ_OS_RPMB_CHECK_PROV_STATUS_ID);
	ret = qseecom_scm_call(TZ_OS_RPMB_CHECK_PROV_STATUS_ID, QSEECOM_ARGS(0),
			       NULL, &resp);
	if (ret) {
		pr_err("qseecom_scm_call failed with err: %d\n", ret);
		return ret;
	}

	ret = qseecom_shm_invalidate(data->client.sb.shm);
	if (ret) {
		pr_err("cache operation failed %d\n", ret);
		return ret;
	}

	switch (resp.result) {
	case QSEOS_RESULT_SUCCESS:
		break;
	case QSEOS_RESULT_INCOMPLETE:
		pr_debug("qseos_result_incomplete\n");
		ret = __qseecom_process_incomplete_cmd(data, &resp);
		if (ret)
			pr_err("process_incomplete_cmd fail with result: %d\n",
			       resp.result);
		pr_warn("RPMB key status is 0x%x\n", resp.result);
		if (put_user(resp.result, (u32 __user *)req->resp_buf))
			return -EINVAL;
		ret = 0;
		break;
	case QSEOS_RESULT_FAILURE:
		pr_err("scm call failed with resp.result: %d\n", resp.result);
		ret = -EINVAL;
		break;
	default:
		pr_err("Response result %d not supported\n", resp.result);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int __validate_send_cmd_inputs(struct qseecom_dev_handle *data,
				      struct qseecom_send_cmd_req *req)
{
	uintptr_t base = data->client.user_virt_sb_base;
	size_t sb_length = data->client.sb_length;
	uintptr_t cmd = (uintptr_t)req->cmd_req_buf;
	uintptr_t rsp = (uintptr_t)req->resp_buf;

	if (!data->client.sb.virt) {
		pr_err("Client or client buf is not initialized\n");
		return -EINVAL;
	}
	if ((!req->resp_buf && req->resp_len != 0) || !req->cmd_req_buf) {
		pr_err("cmd buffer or response buffer is null\n");
		return -EINVAL;
	}
	if (cmd < base || cmd >= base + sb_length) {
		pr_err("cmd buffer address not within shared buffer\n");
		return -EINVAL;
	}
	if (rsp < base || rsp >= base + sb_length) {
		pr_err("response buffer address not within shared buffer\n");
		return -EINVAL;
	}
	if (req->cmd_req_len == 0 || req->cmd_req_len > sb_length ||
	    req->resp_len > sb_length) {
		pr_err("cmd buf length or response buf length not valid\n");
		return -EINVAL;
	}
	if (req->cmd_req_len > UINT_MAX - req->resp_len) {
		pr_err("Integer overflow detected in req_len & rsp_len\n");
		return -EINVAL;
	}

	if ((req->cmd_req_len + req->resp_len) > sb_length) {
		pr_debug("Not enough memory to fit cmd_buf.\n");
		pr_debug("resp_buf. Required: %u, Available: %zu\n",
			 (req->cmd_req_len + req->resp_len), sb_length);
		return -ENOMEM;
	}
	if (cmd > (ULONG_MAX - req->cmd_req_len)) {
		pr_err("Integer overflow in req_len & cmd_req_buf\n");
		return -EINVAL;
	}
	if (rsp > (ULONG_MAX - req->resp_len)) {
		pr_err("Integer overflow in resp_len & resp_buf\n");
		return -EINVAL;
	}
	if (base > (ULONG_MAX - sb_length)) {
		pr_err("Integer overflow in user_virt_sb_base & sb_length\n");
		return -EINVAL;
	}
	if ((cmd + req->cmd_req_len) > (base + sb_length) ||
	    (rsp + req->resp_len) > (base + sb_length)) {
		pr_err("cmd buf or resp buf is out of shared buffer region\n");
		return -EINVAL;
	}
	return 0;
}

static int __qseecom_send_cmd(struct qseecom_dev_handle *data,
			      u64 req_ptr, u32 req_len, u64 rsp_ptr, u32 rsp_len)
{
	struct qseecom_registered_app_list *ptr_app;
	struct qseecom_command_scm_resp resp;
	bool found_app = false;
	unsigned long flags;
	u64 args[7];
	int ret;

	/* find app_id & img_name from list */
	spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
	list_for_each_entry(ptr_app, &qseecom.registered_app_list_head, list) {
		if (ptr_app->app_id == data->client.app_id &&
		    !strcmp(ptr_app->app_name, data->client.app_name)) {
			found_app = true;
			break;
		}
	}
	spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);

	if (!found_app) {
		pr_err("app_id %d (%s) is not found\n", data->client.app_id,
		       data->client.app_name);
		return -ENOENT;
	}

	if (__qseecom_find_pending_unload_app(data->client.app_id,
					      data->client.app_name)) {
		pr_err("app %d (%s) unload is pending\n",
		       data->client.app_id, data->client.app_name);
		return -ENOENT;
	}

	/* check if 32bit app's phys_addr region is under 4GB. */
	if (data->client.app_arch == ELFCLASS32 &&
	    (req_ptr >= PHY_ADDR_4G - req_len ||
	     rsp_ptr >= PHY_ADDR_4G - rsp_len)) {
		pr_err("32bit app %s PA exceeds 4G: req_ptr=%llx, req_len=%x, rsp_ptr=%llx, rsp_len=%x\n",
		       data->client.app_name, req_ptr, req_len, rsp_ptr,
		       rsp_len);
		return -EFAULT;
	}

	args[0] = data->client.app_id;
	args[1] = req_ptr;
	args[2] = req_len;
	args[3] = rsp_ptr;
	args[4] = rsp_len;
	args[5] = qcom_tzmem_to_phys(data->sglistinfo_ptr);
	args[6] = SGLISTINFO_TABLE_SIZE;

	ret = qseecom_shm_clean(data->client.sb.shm);
	if (ret) {
		pr_err("cache operation failed %d\n", ret);
		return ret;
	}

	qseecom_wait_app_unblocked(ptr_app);

	if (qseecom.whitelist_support)
		ret = qseecom_scm_call(TZ_APP_QSAPP_SEND_DATA_WITH_WHITELIST_ID,
				       QSEECOM_ARGS(7, QSEECOM_VAL, QSEECOM_RW,
						    QSEECOM_VAL, QSEECOM_RW,
						    QSEECOM_VAL, QSEECOM_RW,
						    QSEECOM_VAL),
				       args, &resp);
	else
		ret = qseecom_scm_call(TZ_APP_QSAPP_SEND_DATA_ID,
				       QSEECOM_ARGS(5, QSEECOM_VAL, QSEECOM_RW,
						    QSEECOM_VAL, QSEECOM_RW,
						    QSEECOM_VAL),
				       args, &resp);
	if (ret) {
		pr_err("scm_call() failed with err: %d (app_id = %d)\n",
		       ret, data->client.app_id);
		return ret;
	}

	ret = __qseecom_process_reentrancy(&resp, ptr_app, data);
	if (ret)
		return ret;

	ret = qseecom_shm_invalidate(data->client.sb.shm);
	if (ret)
		pr_err("cache operation failed %d\n", ret);

	return ret;
}

static int qseecom_send_cmd(struct qseecom_dev_handle *data, void __user *argp)
{
	struct qseecom_send_cmd_req req;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	if (__validate_send_cmd_inputs(data, &req))
		return -EINVAL;

	return __qseecom_send_cmd(data,
				  __qseecom_uvirt_to_kphys(data, (uintptr_t)req.cmd_req_buf),
				  req.cmd_req_len,
				  __qseecom_uvirt_to_kphys(data, (uintptr_t)req.resp_buf),
				  req.resp_len);
}

static int __boundary_checks_offset(struct qseecom_send_modfd_cmd_req *req,
				    struct qseecom_send_modfd_listener_resp *lstnr_resp,
				    struct qseecom_dev_handle *data, int i,
				    size_t size)
{
	char *curr_field;
	char *temp_field;
	int j;

	if (data->type != QSEECOM_LISTENER_SERVICE && req->ifd_data[i].fd > 0) {
		if (req->cmd_req_len < size ||
		    req->ifd_data[i].cmd_buf_offset > req->cmd_req_len - size) {
			pr_err("Invalid offset (req len) 0x%x\n",
			       req->ifd_data[i].cmd_buf_offset);
			return -EINVAL;
		}

		curr_field = (char *)req->cmd_req_buf +
			     req->ifd_data[i].cmd_buf_offset;
		for (j = 0; j < MAX_ION_FD; j++) {
			if (req->ifd_data[j].fd > 0 && i != j) {
				temp_field = (char *)req->cmd_req_buf +
					     req->ifd_data[j].cmd_buf_offset;
				if (temp_field >= curr_field &&
				    temp_field < curr_field + size) {
					pr_err("Invalid field offset 0x%x\n",
					       req->ifd_data[i].cmd_buf_offset);
					return -EINVAL;
				}
			}
		}
	} else if (data->type == QSEECOM_LISTENER_SERVICE &&
		   lstnr_resp->ifd_data[i].fd > 0) {
		if (lstnr_resp->resp_len < size ||
		    lstnr_resp->ifd_data[i].cmd_buf_offset >
		    lstnr_resp->resp_len - size) {
			pr_err("Invalid offset (lstnr resp len) 0x%x\n",
			       lstnr_resp->ifd_data[i].cmd_buf_offset);
			return -EINVAL;
		}

		curr_field = (char *)lstnr_resp->resp_buf_ptr +
			     lstnr_resp->ifd_data[i].cmd_buf_offset;
		for (j = 0; j < MAX_ION_FD; j++) {
			if (lstnr_resp->ifd_data[j].fd > 0 && i != j) {
				temp_field = (char *)lstnr_resp->resp_buf_ptr +
					     lstnr_resp->ifd_data[j].cmd_buf_offset;
				if (temp_field >= curr_field &&
				    temp_field < curr_field + size) {
					pr_err("Invalid lstnr field offset 0x%x\n",
					       lstnr_resp->ifd_data[i].cmd_buf_offset);
					return -EINVAL;
				}
			}
		}
	}
	return 0;
}

/*
 * Patch the physical addresses of the dma-bufs passed alongside a modfd
 * command (@req, kernel copy) or listener response (@lstnr_resp, in the
 * listener's buffer) into the message, and record them in the whitelist
 * table. The buffers are kept in @shms until the TrustZone is done with them;
 * with @cleanup set, the addresses are cleared again and the buffers released.
 *
 * Every buffer must be physically contiguous, so each one is described by a
 * single address (32 or 64 bit, per @is_64bit) rather than a list.
 */
static int __qseecom_update_cmd_buf(struct qseecom_send_modfd_cmd_req *req,
				    struct qseecom_send_modfd_listener_resp *lstnr_resp,
				    struct qseecom_dev_handle *data,
				    struct qseecom_shm **shms, bool cleanup,
				    bool is_64bit)
{
	struct qseecom_registered_listener_list *this_lstnr = NULL;
	size_t field_size = is_64bit ? sizeof(u64) : sizeof(u32);
	struct sglist_info *table;
	struct qseecom_shm *shm;
	u32 offset;
	char *field;
	int fd;
	int ret;
	int i;

	if (data->type != QSEECOM_LISTENER_SERVICE &&
	    data->type != QSEECOM_CLIENT_APP)
		return -EFAULT;

	if (data->type == QSEECOM_LISTENER_SERVICE) {
		this_lstnr = __qseecom_find_svc(data->listener.id);
		if (!this_lstnr) {
			pr_err("Invalid listener ID\n");
			return -ENOMEM;
		}
	}

	for (i = 0; i < MAX_ION_FD; i++) {
		if (data->type != QSEECOM_LISTENER_SERVICE &&
		    req->ifd_data[i].fd > 0) {
			fd = req->ifd_data[i].fd;
			field = (char *)req->cmd_req_buf +
				req->ifd_data[i].cmd_buf_offset;
		} else if (data->type == QSEECOM_LISTENER_SERVICE &&
			   lstnr_resp->ifd_data[i].fd > 0) {
			fd = lstnr_resp->ifd_data[i].fd;
			field = (char *)lstnr_resp->resp_buf_ptr +
				lstnr_resp->ifd_data[i].cmd_buf_offset;
		} else {
			continue;
		}

		if (cleanup) {
			shm = shms[i];
			if (!shm)
				continue;
			if (is_64bit)
				put_unaligned(0ULL, (u64 *)field);
			else
				put_unaligned(0U, (u32 *)field);
			ret = qseecom_shm_invalidate(shm);
			qseecom_shm_put(shm);
			shms[i] = NULL;
			if (ret) {
				pr_err("cache operation failed %d\n", ret);
				return -ENOMEM;
			}
			continue;
		}

		if (__boundary_checks_offset(req, lstnr_resp, data, i,
					     field_size))
			return -ENOMEM;

		if (!is_64bit && data->type == QSEECOM_CLIENT_APP &&
		    data->client.app_arch != ELFCLASS32 &&
		    data->client.app_arch != ELFCLASS64) {
			pr_err("QSEE app arch %u is not supported\n",
			       data->client.app_arch);
			return -ENOMEM;
		}

		/* Populate the cmd data structure with the phys_addr */
		shm = qseecom_shm_get(fd);
		if (IS_ERR(shm)) {
			pr_err("could not map buffer of fd %d\n", fd);
			return PTR_ERR(shm);
		}

		/* Check if the buffer lies below 4GB for 32-bit addresses */
		if (!is_64bit && shm->paddr >= PHY_ADDR_4G - shm->size) {
			pr_err("App %s buffer PA exceeds 4G: phy_addr=%pa, len=%zx\n",
			       data->client.app_name, &shm->paddr, shm->size);
			qseecom_shm_put(shm);
			return -ENOMEM;
		}

		ret = qseecom_shm_clean(shm);
		if (ret) {
			pr_err("cache operation failed %d\n", ret);
			qseecom_shm_put(shm);
			return -ENOMEM;
		}

		qseecom_shm_put(shms[i]);
		shms[i] = shm;

		if (is_64bit)
			put_unaligned((u64)shm->paddr, (u64 *)field);
		else
			put_unaligned((u32)shm->paddr, (u32 *)field);

		if (data->type == QSEECOM_CLIENT_APP) {
			offset = req->ifd_data[i].cmd_buf_offset;
			table = data->sglistinfo_ptr;
			data->sglist_cnt = i + 1;
		} else {
			offset = lstnr_resp->ifd_data[i].cmd_buf_offset +
				 ((u8 *)lstnr_resp->resp_buf_ptr -
				  this_lstnr->sb.virt);
			table = this_lstnr->sglistinfo_ptr;
			this_lstnr->sglist_cnt = i + 1;
		}
		table[i].index_and_flags =
			SGLISTINFO_SET_INDEX_FLAG(1, is_64bit, offset);
		table[i].size_or_count = shm->size;
	}

	return 0;
}

static void qseecom_put_shms(struct qseecom_shm **shms)
{
	int i;

	for (i = 0; i < MAX_ION_FD; i++) {
		qseecom_shm_put(shms[i]);
		shms[i] = NULL;
	}
}

static int __qseecom_send_modfd_cmd(struct qseecom_dev_handle *data,
				    void __user *argp, bool is_64bit_addr)
{
	struct qseecom_shm *shms[MAX_ION_FD] = {};
	struct qseecom_send_modfd_cmd_req req;
	struct qseecom_send_cmd_req send_cmd_req;
	void *origin_req_buf_kvirt, *origin_rsp_buf_kvirt;
	phys_addr_t pa;
	u8 *va;
	int ret;
	int i;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	send_cmd_req.cmd_req_buf = req.cmd_req_buf;
	send_cmd_req.cmd_req_len = req.cmd_req_len;
	send_cmd_req.resp_buf = req.resp_buf;
	send_cmd_req.resp_len = req.resp_len;

	if (__validate_send_cmd_inputs(data, &send_cmd_req))
		return -EINVAL;

	/* validate offsets */
	for (i = 0; i < MAX_ION_FD; i++) {
		if (req.ifd_data[i].cmd_buf_offset >= req.cmd_req_len) {
			pr_err("Invalid offset %d = 0x%x\n",
			       i, req.ifd_data[i].cmd_buf_offset);
			return -EINVAL;
		}
	}

	/* Back up original address */
	origin_req_buf_kvirt = __qseecom_uvirt_to_kvirt(data,
							(uintptr_t)req.cmd_req_buf);
	origin_rsp_buf_kvirt = __qseecom_uvirt_to_kvirt(data,
							(uintptr_t)req.resp_buf);

	/* Allocate kernel buffer for request and response */
	va = qseecom_alloc_tzbuf(req.cmd_req_len + req.resp_len);
	if (!va)
		return -ENOMEM;
	pa = qcom_tzmem_to_phys(va);

	req.cmd_req_buf = va;
	req.resp_buf = va + req.cmd_req_len;

	/* Copy the data to kernel request and response buffers */
	memcpy(req.cmd_req_buf, origin_req_buf_kvirt, req.cmd_req_len);
	memcpy(req.resp_buf, origin_rsp_buf_kvirt, req.resp_len);

	ret = __qseecom_update_cmd_buf(&req, NULL, data, shms, false,
				       is_64bit_addr);
	if (ret)
		goto out;
	ret = __qseecom_send_cmd(data, pa, req.cmd_req_len,
				 pa + req.cmd_req_len, req.resp_len);
	if (ret)
		goto out;
	ret = __qseecom_update_cmd_buf(&req, NULL, data, shms, true,
				       is_64bit_addr);
	if (ret)
		goto out;

	/* Copy the response back to the userspace buffer */
	memcpy(origin_rsp_buf_kvirt, req.resp_buf, req.resp_len);
	memcpy(origin_req_buf_kvirt, req.cmd_req_buf, req.cmd_req_len);

out:
	qseecom_put_shms(shms);
	qcom_tzmem_free(va);
	return ret;
}

static int __qseecom_listener_has_rcvd_req(struct qseecom_dev_handle *data,
					   struct qseecom_registered_listener_list *svc)
{
	return svc->rcv_req_flag == 1 || data->abort;
}

static int qseecom_receive_req(struct qseecom_dev_handle *data)
{
	struct qseecom_registered_listener_list *this_lstnr;

	mutex_lock(&listener_access_lock);
	this_lstnr = __qseecom_find_svc(data->listener.id);
	if (!this_lstnr) {
		pr_err("Invalid listener ID\n");
		mutex_unlock(&listener_access_lock);
		return -ENODATA;
	}
	mutex_unlock(&listener_access_lock);

	if (wait_event_interruptible(this_lstnr->rcv_req_wq,
				     __qseecom_listener_has_rcvd_req(data, this_lstnr))) {
		pr_debug("Interrupted: exiting Listener Service = %d\n",
			 data->listener.id);
		/* woken up for different reason */
		return -ERESTARTSYS;
	}

	if (data->abort) {
		pr_err("Aborting Listener Service = %d\n", data->listener.id);
		return -ENODEV;
	}
	mutex_lock(&listener_access_lock);
	this_lstnr->rcv_req_flag = 0;
	mutex_unlock(&listener_access_lock);

	return 0;
}

static int qseecom_reentrancy_send_resp(struct qseecom_dev_handle *data)
{
	struct qseecom_registered_listener_list *this_lstnr;

	pr_debug("lstnr %d send resp, wakeup\n", data->listener.id);
	this_lstnr = __qseecom_find_svc(data->listener.id);
	if (!this_lstnr)
		return -EINVAL;
	qseecom.send_resp_flag = 1;
	this_lstnr->send_resp_flag = 1;
	wake_up_all(&qseecom.send_resp_wq);
	return 0;
}

static int __validate_send_modfd_resp_inputs(struct qseecom_dev_handle *data,
					     struct qseecom_send_modfd_listener_resp *resp,
					     struct qseecom_registered_listener_list *this_lstnr)
{
	uintptr_t base = this_lstnr->user_virt_sb_base;
	size_t sb_length = this_lstnr->sb.length;
	uintptr_t rsp = (uintptr_t)resp->resp_buf_ptr;
	int i;

	if (!resp->resp_buf_ptr) {
		pr_err("resp buffer is null\n");
		return -EINVAL;
	}
	/* validate resp buf length */
	if (resp->resp_len == 0 || resp->resp_len > sb_length) {
		pr_err("resp buf length %d not valid\n", resp->resp_len);
		return -EINVAL;
	}

	if (rsp > (ULONG_MAX - resp->resp_len)) {
		pr_err("Integer overflow in resp_len & resp_buf\n");
		return -EINVAL;
	}
	if (base > (ULONG_MAX - sb_length)) {
		pr_err("Integer overflow in user_virt_sb_base & sb_length\n");
		return -EINVAL;
	}
	/* validate resp buf */
	if (rsp < base || rsp >= base + sb_length ||
	    rsp + resp->resp_len > base + sb_length) {
		pr_err("resp buf is out of shared buffer region\n");
		return -EINVAL;
	}

	/* validate offsets */
	for (i = 0; i < MAX_ION_FD; i++) {
		if (resp->ifd_data[i].cmd_buf_offset >= resp->resp_len) {
			pr_err("Invalid offset %d = 0x%x\n",
			       i, resp->ifd_data[i].cmd_buf_offset);
			return -EINVAL;
		}
	}

	return 0;
}

static int __qseecom_send_modfd_resp(struct qseecom_dev_handle *data,
				     void __user *argp, bool is_64bit_addr)
{
	struct qseecom_registered_listener_list *this_lstnr;
	struct qseecom_send_modfd_listener_resp resp;

	if (copy_from_user(&resp, argp, sizeof(resp)))
		return -EINVAL;

	this_lstnr = __qseecom_find_svc(data->listener.id);
	if (!this_lstnr)
		return -EINVAL;

	if (__validate_send_modfd_resp_inputs(data, &resp, this_lstnr))
		return -EINVAL;

	resp.resp_buf_ptr = this_lstnr->sb.virt +
		((uintptr_t)resp.resp_buf_ptr - this_lstnr->user_virt_sb_base);

	/*
	 * The buffers stay mapped and bridged until the response has been
	 * delivered to QSEE. As before, a failure here still releases the
	 * waiting client, which then fails in the TA.
	 */
	if (__qseecom_update_cmd_buf(NULL, &resp, data, this_lstnr->resp_shm,
				     false, is_64bit_addr))
		pr_err("failed to update listener %d response buffers\n",
		       data->listener.id);

	qseecom.send_resp_flag = 1;
	this_lstnr->send_resp_flag = 1;
	wake_up_all(&qseecom.send_resp_wq);
	return 0;
}

static int qseecom_get_qseos_version(struct qseecom_dev_handle *data,
				     void __user *argp)
{
	struct qseecom_qseos_version_req req;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EINVAL;
	req.qseos_version = qseecom.qseos_version;
	if (copy_to_user(argp, &req, sizeof(req)))
		return -EINVAL;
	return 0;
}

static int qseecom_query_app_loaded(struct qseecom_dev_handle *data,
				    void __user *argp)
{
	struct qseecom_qseos_app_load_query query_req = {};
	struct qseecom_registered_app_list *entry;
	u32 app_arch = 0, app_id = 0;
	bool found_app = false;
	unsigned long flags;
	int ret;

	/* Copy the relevant information needed for loading the image */
	if (copy_from_user(&query_req, argp, sizeof(query_req)))
		return -EFAULT;

	query_req.app_name[MAX_APP_NAME_SIZE - 1] = '\0';

	ret = __qseecom_check_app_exists(query_req.app_name, &app_id);
	if (ret) {
		pr_err(" scm call to check if app is loaded failed\n");
		return ret;
	}
	if (!app_id)
		return 0;	/* app not loaded */

	pr_debug("App id %d (%s) already exists\n", app_id, query_req.app_name);
	spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
	list_for_each_entry(entry, &qseecom.registered_app_list_head, list) {
		if (entry->app_id == app_id) {
			app_arch = entry->app_arch;
			if (entry->ref_cnt == U32_MAX) {
				pr_err("App %d (%s) ref_cnt overflow\n",
				       app_id, query_req.app_name);
				spin_unlock_irqrestore(&qseecom.registered_app_list_lock,
						       flags);
				return -EINVAL;
			}
			entry->ref_cnt++;
			found_app = true;
			break;
		}
	}
	spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);
	data->client.app_id = app_id;
	query_req.app_id = app_id;
	data->client.app_arch = app_arch;
	query_req.app_arch = app_arch;
	strscpy(data->client.app_name, query_req.app_name, MAX_APP_NAME_SIZE);
	/*
	 * If app was loaded by appsbl before and was not registered,
	 * register this app now.
	 */
	if (!found_app) {
		pr_debug("Register app %d [%s] which was loaded before\n",
			 app_id, query_req.app_name);
		entry = kzalloc_obj(*entry);
		if (!entry)
			return -ENOMEM;
		entry->app_id = app_id;
		entry->ref_cnt = 1;
		entry->app_arch = data->client.app_arch;
		strscpy(entry->app_name, data->client.app_name,
			MAX_APP_NAME_SIZE);
		entry->app_blocked = false;
		entry->blocked_on_listener_id = 0;
		entry->check_block = 0;
		entry->preloaded = true;
		spin_lock_irqsave(&qseecom.registered_app_list_lock, flags);
		list_add_tail(&entry->list, &qseecom.registered_app_list_head);
		spin_unlock_irqrestore(&qseecom.registered_app_list_lock, flags);
	}
	if (copy_to_user(argp, &query_req, sizeof(query_req))) {
		pr_err("copy_to_user failed\n");
		return -EFAULT;
	}

	return -EEXIST;	/* app already loaded */
}

static void __qseecom_clean_data_sglistinfo(struct qseecom_dev_handle *data)
{
	if (data->sglist_cnt) {
		memset(data->sglistinfo_ptr, 0, SGLISTINFO_TABLE_SIZE);
		data->sglist_cnt = 0;
	}
}

/*
 * ioctl
 */

static long qseecom_ioctl_send_cmd(struct qseecom_dev_handle *data,
				   unsigned int cmd, void __user *argp)
{
	long ret;

	/* Only one client allowed here at a time */
	mutex_lock(&app_access_lock);
	if (data->client.app_id == 0 || data->type != QSEECOM_CLIENT_APP) {
		pr_err("send cmd req: invalid handle (%d) app_id(%d)\n",
		       data->type, data->client.app_id);
		mutex_unlock(&app_access_lock);
		return -EINVAL;
	}
	atomic_inc(&data->ioctl_count);
	switch (cmd) {
	case QSEECOM_IOCTL_SEND_CMD_REQ:
		ret = qseecom_send_cmd(data, argp);
		break;
	case QSEECOM_IOCTL_SEND_MODFD_CMD_REQ:
		ret = __qseecom_send_modfd_cmd(data, argp, false);
		break;
	default:
		ret = __qseecom_send_modfd_cmd(data, argp, true);
		break;
	}
	atomic_dec(&data->ioctl_count);
	wake_up_all(&data->abort_wq);
	mutex_unlock(&app_access_lock);
	if (ret)
		pr_err("failed qseecom_send_cmd: %ld\n", ret);
	__qseecom_clean_data_sglistinfo(data);
	return ret;
}

static long qseecom_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct qseecom_dev_handle *data = file->private_data;
	void __user *argp = (void __user *)arg;
	long ret = 0;

	if (data->abort) {
		pr_err("Aborting qseecom driver\n");
		return -ENODEV;
	}
	if (cmd != QSEECOM_IOCTL_RECEIVE_REQ &&
	    cmd != QSEECOM_IOCTL_SEND_RESP_REQ &&
	    cmd != QSEECOM_IOCTL_SEND_MODFD_RESP &&
	    cmd != QSEECOM_IOCTL_SEND_MODFD_RESP_64)
		__wakeup_unregister_listener_kthread();
	__wakeup_unload_app_kthread();

	switch (cmd) {
	case QSEECOM_IOCTL_REGISTER_LISTENER_REQ:
		mutex_lock(&listener_access_lock);
		if (data->type != QSEECOM_GENERIC) {
			pr_err("reg lstnr req: invalid handle (%d)\n",
			       data->type);
			mutex_unlock(&listener_access_lock);
			ret = -EINVAL;
			break;
		}
		atomic_inc(&data->ioctl_count);
		data->type = QSEECOM_LISTENER_SERVICE;
		ret = qseecom_register_listener(data, argp);
		atomic_dec(&data->ioctl_count);
		wake_up_all(&data->abort_wq);
		mutex_unlock(&listener_access_lock);
		if (ret)
			pr_err("failed qseecom_register_listener: %ld\n", ret);
		break;
	case QSEECOM_IOCTL_UNREGISTER_LISTENER_REQ:
		mutex_lock(&listener_access_lock);
		if (data->listener.id == 0 ||
		    data->type != QSEECOM_LISTENER_SERVICE) {
			pr_err("unreg lstnr req: invalid handle (%d) lid(%d)\n",
			       data->type, data->listener.id);
			mutex_unlock(&listener_access_lock);
			ret = -EINVAL;
			break;
		}
		atomic_inc(&data->ioctl_count);
		ret = qseecom_unregister_listener(data);
		atomic_dec(&data->ioctl_count);
		wake_up_all(&data->abort_wq);
		mutex_unlock(&listener_access_lock);
		if (ret)
			pr_err("failed qseecom_unregister_listener: %ld\n", ret);
		break;
	case QSEECOM_IOCTL_SEND_CMD_REQ:
	case QSEECOM_IOCTL_SEND_MODFD_CMD_REQ:
	case QSEECOM_IOCTL_SEND_MODFD_CMD_64_REQ:
		ret = qseecom_ioctl_send_cmd(data, cmd, argp);
		break;
	case QSEECOM_IOCTL_RECEIVE_REQ:
		mutex_lock(&listener_access_lock);
		if (data->listener.id == 0 ||
		    data->type != QSEECOM_LISTENER_SERVICE) {
			pr_err("receive req: invalid handle (%d), lid(%d)\n",
			       data->type, data->listener.id);
			mutex_unlock(&listener_access_lock);
			ret = -EINVAL;
			break;
		}
		mutex_unlock(&listener_access_lock);
		atomic_inc(&data->ioctl_count);
		ret = qseecom_receive_req(data);
		atomic_dec(&data->ioctl_count);
		wake_up_all(&data->abort_wq);
		if (ret && ret != -ERESTARTSYS)
			pr_err("failed qseecom_receive_req: %ld\n", ret);
		break;
	case QSEECOM_IOCTL_SEND_RESP_REQ:
		mutex_lock(&listener_access_lock);
		if (data->listener.id == 0 ||
		    data->type != QSEECOM_LISTENER_SERVICE) {
			pr_err("send resp req: invalid handle (%d), lid(%d)\n",
			       data->type, data->listener.id);
			mutex_unlock(&listener_access_lock);
			ret = -EINVAL;
			break;
		}
		atomic_inc(&data->ioctl_count);
		ret = qseecom_reentrancy_send_resp(data);
		atomic_dec(&data->ioctl_count);
		wake_up_all(&data->abort_wq);
		mutex_unlock(&listener_access_lock);
		if (ret)
			pr_err("failed qseecom_send_resp: %ld\n", ret);
		break;
	case QSEECOM_IOCTL_SEND_MODFD_RESP:
	case QSEECOM_IOCTL_SEND_MODFD_RESP_64:
		mutex_lock(&listener_access_lock);
		if (data->listener.id == 0 ||
		    data->type != QSEECOM_LISTENER_SERVICE) {
			pr_err("send modfd resp: invalid handle (%d), lid(%d)\n",
			       data->type, data->listener.id);
			mutex_unlock(&listener_access_lock);
			ret = -EINVAL;
			break;
		}
		atomic_inc(&data->ioctl_count);
		ret = __qseecom_send_modfd_resp(data, argp,
						cmd == QSEECOM_IOCTL_SEND_MODFD_RESP_64);
		atomic_dec(&data->ioctl_count);
		wake_up_all(&data->abort_wq);
		mutex_unlock(&listener_access_lock);
		if (ret)
			pr_err("failed qseecom_send_mod_resp: %ld\n", ret);
		__qseecom_clean_data_sglistinfo(data);
		break;
	case QSEECOM_IOCTL_SET_MEM_PARAM_REQ:
		mutex_lock(&app_access_lock);
		if (data->type != QSEECOM_CLIENT_APP &&
		    data->type != QSEECOM_GENERIC &&
		    data->type != QSEECOM_SECURE_SERVICE) {
			pr_err("set mem param req: invalid handle (%d)\n",
			       data->type);
			mutex_unlock(&app_access_lock);
			ret = -EINVAL;
			break;
		}
		atomic_inc(&data->ioctl_count);
		ret = qseecom_set_client_mem_param(data, argp);
		atomic_dec(&data->ioctl_count);
		mutex_unlock(&app_access_lock);
		if (ret)
			pr_err("failed qseecom_set_mem_param request: %ld\n",
			       ret);
		break;
	case QSEECOM_IOCTL_LOAD_APP_REQ:
		mutex_lock(&app_access_lock);
		if (data->type != QSEECOM_GENERIC &&
		    data->type != QSEECOM_CLIENT_APP) {
			pr_err("load app req: invalid handle (%d)\n",
			       data->type);
			mutex_unlock(&app_access_lock);
			ret = -EINVAL;
			break;
		}
		data->type = QSEECOM_CLIENT_APP;
		atomic_inc(&data->ioctl_count);
		ret = qseecom_load_app(data, argp);
		atomic_dec(&data->ioctl_count);
		mutex_unlock(&app_access_lock);
		if (ret)
			pr_err("failed load_app request: %ld\n", ret);
		__wakeup_unload_app_kthread();
		break;
	case QSEECOM_IOCTL_UNLOAD_APP_REQ:
		mutex_lock(&app_access_lock);
		if (data->client.app_id == 0 ||
		    data->type != QSEECOM_CLIENT_APP) {
			pr_err("unload app req:invalid handle(%d) app_id(%d)\n",
			       data->type, data->client.app_id);
			mutex_unlock(&app_access_lock);
			ret = -EINVAL;
			break;
		}
		atomic_inc(&data->ioctl_count);
		ret = qseecom_unload_app(data, false);
		atomic_dec(&data->ioctl_count);
		mutex_unlock(&app_access_lock);
		if (ret)
			pr_err("failed unload_app request: %ld\n", ret);
		__wakeup_unload_app_kthread();
		break;
	case QSEECOM_IOCTL_GET_QSEOS_VERSION_REQ:
		atomic_inc(&data->ioctl_count);
		ret = qseecom_get_qseos_version(data, argp);
		if (ret)
			pr_err("qseecom_get_qseos_version: %ld\n", ret);
		atomic_dec(&data->ioctl_count);
		break;
	case QSEECOM_IOCTL_PERF_ENABLE_REQ:
	case QSEECOM_IOCTL_PERF_DISABLE_REQ:
		/*
		 * The crypto engine clocks are not handled by Linux on this
		 * platform, so there is nothing to vote for.
		 */
		mutex_lock(&app_access_lock);
		if ((cmd == QSEECOM_IOCTL_PERF_ENABLE_REQ &&
		     data->type != QSEECOM_GENERIC &&
		     data->type != QSEECOM_CLIENT_APP) ||
		    (cmd == QSEECOM_IOCTL_PERF_DISABLE_REQ &&
		     data->type != QSEECOM_SECURE_SERVICE &&
		     data->type != QSEECOM_CLIENT_APP) ||
		    (data->type == QSEECOM_CLIENT_APP &&
		     data->client.app_id == 0)) {
			pr_err("perf req: invalid handle (%d) appid(%d)\n",
			       data->type, data->client.app_id);
			ret = -EINVAL;
		}
		mutex_unlock(&app_access_lock);
		break;
	case QSEECOM_IOCTL_SET_BUS_SCALING_REQ:
		/* No bus scaling is handled by Linux on this platform. */
		break;
	case QSEECOM_IOCTL_APP_LOADED_QUERY_REQ:
		mutex_lock(&app_access_lock);
		if (data->type != QSEECOM_GENERIC &&
		    data->type != QSEECOM_CLIENT_APP) {
			pr_err("app loaded query req: invalid handle (%d)\n",
			       data->type);
			mutex_unlock(&app_access_lock);
			ret = -EINVAL;
			break;
		}
		data->type = QSEECOM_CLIENT_APP;
		atomic_inc(&data->ioctl_count);
		ret = qseecom_query_app_loaded(data, argp);
		atomic_dec(&data->ioctl_count);
		mutex_unlock(&app_access_lock);
		break;
	case QSEECOM_IOCTL_SEND_CMD_SERVICE_REQ: {
		struct qseecom_send_svc_cmd_req req;

		if (copy_from_user(&req, argp, sizeof(req)))
			return -EFAULT;

		switch (req.cmd_id) {
		case QSEOS_RPMB_PROVISION_KEY_COMMAND:
		case QSEOS_RPMB_ERASE_COMMAND:
			/* Irreversible: never allowed. */
			pr_err("RPMB command 0x%x refused\n", req.cmd_id);
			return -EPERM;
		case QSEOS_FSM_LTEOTA_REQ_CMD:
		case QSEOS_FSM_LTEOTA_REQ_RSP_CMD:
		case QSEOS_FSM_IKE_REQ_CMD:
		case QSEOS_FSM_IKE_REQ_RSP_CMD:
		case QSEOS_FSM_OEM_FUSE_WRITE_ROW:
		case QSEOS_FSM_OEM_FUSE_READ_ROW:
		case QSEOS_FSM_ENCFS_REQ_CMD:
		case QSEOS_FSM_ENCFS_REQ_RSP_CMD:
		case QSEOS_DIAG_FUSE_REQ_CMD:
		case QSEOS_DIAG_FUSE_REQ_RSP_CMD:
			return -EOPNOTSUPP;
		}

		mutex_lock(&app_access_lock);
		if (data->type != QSEECOM_GENERIC) {
			pr_err("send cmd svc req: invalid handle (%d)\n",
			       data->type);
			mutex_unlock(&app_access_lock);
			ret = -EINVAL;
			break;
		}
		if (qseecom.qsee_version < QSEE_VERSION_03) {
			pr_err("SEND_CMD_SERVICE_REQ: Invalid qsee ver %u\n",
			       qseecom.qsee_version);
			mutex_unlock(&app_access_lock);
			return -EINVAL;
		}
		atomic_inc(&data->ioctl_count);
		ret = qseecom_send_service_cmd(data, &req);
		atomic_dec(&data->ioctl_count);
		mutex_unlock(&app_access_lock);
		break;
	}
	case QSEECOM_IOCTL_CREATE_KEY_REQ:
	case QSEECOM_IOCTL_WIPE_KEY_REQ:
	case QSEECOM_IOCTL_UPDATE_KEY_USER_INFO_REQ:
	case QSEECOM_IOCTL_SAVE_PARTITION_HASH_REQ:
	case QSEECOM_IOCTL_IS_ES_ACTIVATED_REQ:
	case QSEECOM_IOCTL_MDTP_CIPHER_DIP_REQ:
	case QSEECOM_IOCTL_GET_CE_PIPE_INFO:
	case QSEECOM_IOCTL_FREE_CE_PIPE_INFO:
	case QSEECOM_IOCTL_QUERY_CE_PIPE_INFO:
	case QSEECOM_IOCTL_SET_ICE_INFO:
	case QSEECOM_IOCTL_FBE_CLEAR_KEY:
	case QSEECOM_IOCTL_LOAD_EXTERNAL_ELF_REQ:
	case QSEECOM_IOCTL_UNLOAD_EXTERNAL_ELF_REQ:
	case QSEECOM_QTEEC_IOCTL_OPEN_SESSION_REQ:
	case QSEECOM_QTEEC_IOCTL_CLOSE_SESSION_REQ:
	case QSEECOM_QTEEC_IOCTL_INVOKE_MODFD_CMD_REQ:
	case QSEECOM_QTEEC_IOCTL_REQUEST_CANCELLATION_REQ:
		/* Key management, crypto engine, ICE, ES, MDTP, GP TEE */
		return -EOPNOTSUPP;
	default:
		return -ENOIOCTLCMD;
	}
	return ret;
}

static int qseecom_open(struct inode *inode, struct file *file)
{
	struct qseecom_dev_handle *data;

	qseecom_shm_reclaim();

	data = kzalloc_obj(*data);
	if (!data)
		return -ENOMEM;

	data->sglistinfo_ptr = qseecom_alloc_tzbuf(SGLISTINFO_TABLE_SIZE);
	if (!data->sglistinfo_ptr) {
		kfree(data);
		return -ENOMEM;
	}

	data->abort = 0;
	data->type = QSEECOM_GENERIC;
	data->released = false;
	init_waitqueue_head(&data->abort_wq);
	atomic_set(&data->ioctl_count, 0);
	file->private_data = data;

	return 0;
}

static int qseecom_release(struct inode *inode, struct file *file)
{
	struct qseecom_dev_handle *data = file->private_data;
	bool free_private_data = true;
	int ret = 0;

	if (!data->released) {
		pr_debug("data: released=false, type=%d, data=0x%p\n",
			 data->type, data);
		switch (data->type) {
		case QSEECOM_LISTENER_SERVICE:
			mutex_lock(&listener_access_lock);
			ret = qseecom_unregister_listener(data);
			if (!ret)
				free_private_data = false;
			data->listener.release_called = true;
			mutex_unlock(&listener_access_lock);
			__wakeup_unregister_listener_kthread();
			break;
		case QSEECOM_CLIENT_APP:
			pr_debug("release app %d (%s)\n",
				 data->client.app_id, data->client.app_name);
			if (data->client.app_id) {
				free_private_data = false;
				mutex_lock(&unload_app_pending_list_lock);
				ret = qseecom_prepare_unload_app(data);
				mutex_unlock(&unload_app_pending_list_lock);
				__wakeup_unload_app_kthread();
			}
			break;
		case QSEECOM_SECURE_SERVICE:
		case QSEECOM_GENERIC:
			break;
		default:
			pr_err("Unsupported clnt_handle_type %d\n", data->type);
			break;
		}
	}

	if (free_private_data)
		qseecom_free_handle(data);

	qseecom_shm_reclaim();

	return ret;
}

static const struct file_operations qseecom_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = qseecom_ioctl,
	.open = qseecom_open,
	.release = qseecom_release,
};

/*
 * Platform
 */

static bool qseecom_check_whitelist_feature(void)
{
	struct qseecom_command_scm_resp resp;
	u64 args[1] = { FEATURE_ID_WHITELIST };
	int ret;

	ret = qseecom_scm_call(TZ_INFO_GET_FEATURE_VERSION_ID,
			       QSEECOM_ARGS(1, QSEECOM_VAL), args, &resp);
	if (ret)
		return false;

	return resp.result >= MAKE_WHITELIST_VERSION(1, 0, 0);
}

static int qseecom_init_control(void)
{
	struct qseecom_command_scm_resp resp;
	u64 args[1] = { 10 };	/* QSEE version feature */
	int ret;

	mutex_lock(&app_access_lock);
	ret = qseecom_scm_call(TZ_INFO_GET_FEATURE_VERSION_ID,
			       QSEECOM_ARGS(1, QSEECOM_VAL), args, &resp);
	if (!ret) {
		qseecom.qsee_version = resp.result;
		qseecom.whitelist_support = qseecom_check_whitelist_feature();
	}
	mutex_unlock(&app_access_lock);

	return ret;
}

static void qseecom_release_all(void)
{
	struct qseecom_registered_listener_list *svc, *svc_tmp;
	struct qseecom_registered_app_list *app, *app_tmp;
	struct qseecom_shm *shm, *shm_tmp;

	list_for_each_entry_safe(svc, svc_tmp,
				 &qseecom.registered_listener_list_head, list) {
		qseecom_release_listener_resp_shm(svc);
		qseecom_sbuf_unmap(&svc->sb);
		qcom_tzmem_free(svc->sglistinfo_ptr);
		list_del(&svc->list);
		kfree_sensitive(svc);
	}

	list_for_each_entry_safe(app, app_tmp,
				 &qseecom.registered_app_list_head, list) {
		list_del(&app->list);
		kfree_sensitive(app);
	}

	list_for_each_entry_safe(shm, shm_tmp, &qseecom.shm_list, node) {
		if (shm->users)
			pr_warn("buffer still in use at removal\n");
		list_del(&shm->node);
		qseecom_shm_free(shm);
	}
}

static int qseecom_probe(struct platform_device *pdev)
{
	struct qcom_tzmem_pool_config pool_config = {
		.policy = QCOM_TZMEM_POLICY_ON_DEMAND,
		.max_size = SZ_16M,
	};
	struct device *dev = &pdev->dev;
	int ret;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	qseecom.dev = dev;
	INIT_LIST_HEAD(&qseecom.registered_listener_list_head);
	INIT_LIST_HEAD(&qseecom.registered_app_list_head);
	spin_lock_init(&qseecom.registered_app_list_lock);
	init_waitqueue_head(&qseecom.send_resp_wq);
	init_waitqueue_head(&qseecom.app_block_wq);
	INIT_LIST_HEAD(&qseecom.unregister_lsnr_pending_list_head);
	init_waitqueue_head(&qseecom.register_lsnr_pending_wq);
	INIT_WORK(&qseecom.unregister_lsnr_work, qseecom_unregister_lsnr_work);
	INIT_LIST_HEAD(&qseecom.unload_app_pending_list_head);
	INIT_WORK(&qseecom.unload_app_work, qseecom_unload_app_work);
	mutex_init(&qseecom.shm_lock);
	INIT_LIST_HEAD(&qseecom.shm_list);
	qseecom.send_resp_flag = 0;
	qseecom.app_block_ref_cnt = 0;
	qseecom.qseos_version = QSEOS_VERSION_14;

	qseecom.pool = devm_qcom_tzmem_pool_new(dev, &pool_config);
	if (IS_ERR(qseecom.pool))
		return dev_err_probe(dev, PTR_ERR(qseecom.pool),
				     "failed to create TZ memory pool\n");

	ret = qseecom_init_control();
	if (ret)
		return dev_err_probe(dev, ret, "failed to get QSEE version\n");

	/* Only the 64-bit address interface of QSEE 4.0 and later is used. */
	if (qseecom.qsee_version < QSEE_VERSION_40)
		return dev_err_probe(dev, -ENODEV,
				     "unsupported QSEE version 0x%x\n",
				     qseecom.qsee_version);

	qseecom.misc.minor = MISC_DYNAMIC_MINOR;
	qseecom.misc.name = QSEECOM_DEV_NAME;
	qseecom.misc.fops = &qseecom_fops;
	qseecom.misc.parent = dev;
	ret = misc_register(&qseecom.misc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register misc device\n");

	dev_info(dev, "QSEE version 0x%x, whitelist %ssupported\n",
		 qseecom.qsee_version, qseecom.whitelist_support ? "" : "not ");

	return 0;
}

static void qseecom_remove(struct platform_device *pdev)
{
	misc_deregister(&qseecom.misc);

	/* All files are closed: run what their release deferred. */
	flush_work(&qseecom.unregister_lsnr_work);
	__qseecom_processing_pending_lsnr_unregister();
	flush_work(&qseecom.unload_app_work);
	__qseecom_processing_pending_unload_app();

	qseecom_release_all();
}

static struct platform_driver qseecom_driver = {
	.probe = qseecom_probe,
	.remove = qseecom_remove,
	.driver = {
		.name = "qcom_qseecom_user",
		.suppress_bind_attrs = true,
	},
};

static struct platform_device *qseecom_pdev;

/*
 * There is no devicetree node for this interface; it is only offered on
 * machines whose firmware is known to implement it as the driver expects:
 * no crypto engine clocks handled by Linux, apps loaded by the bootloader
 * stay loaded, the common libraries (64-bit one included) are loaded by the
 * bootloader, and QSEE re-entrancy phase 2.
 */
static const char * const qseecom_machines[] = {
	"osom,ingot",
};

static int __init qseecom_init(void)
{
	const struct platform_device_info info = {
		.name = "qcom_qseecom_user",
		.id = PLATFORM_DEVID_NONE,
		.dma_mask = DMA_BIT_MASK(64),
	};
	bool found = false;
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(qseecom_machines); i++)
		found |= of_machine_is_compatible(qseecom_machines[i]);
	if (!found)
		return -ENODEV;

	ret = platform_driver_register(&qseecom_driver);
	if (ret)
		return ret;

	qseecom_pdev = platform_device_register_full(&info);
	if (IS_ERR(qseecom_pdev)) {
		platform_driver_unregister(&qseecom_driver);
		return PTR_ERR(qseecom_pdev);
	}

	return 0;
}
module_init(qseecom_init);

static void __exit qseecom_exit(void)
{
	platform_device_unregister(qseecom_pdev);
	platform_driver_unregister(&qseecom_driver);
}
module_exit(qseecom_exit);

MODULE_DESCRIPTION("Qualcomm QSEECOM userspace interface");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
