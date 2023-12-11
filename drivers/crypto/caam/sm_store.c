// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * CAAM Secure Memory Storage Interface
 *
 * Copyright 2008-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * Loosely based on the SHW Keystore API for SCC/SCC2
 * Experimental implementation and NOT intended for upstream use. Expect
 * this interface to be amended significantly in the future once it becomes
 * integrated into live applications.
 *
 * Known issues:
 *
 * - Executes one instance of an secure memory "driver". This is tied to the
 *   fact that job rings can't run as standalone instances in the present
 *   configuration.
 *
 * - It does not expose a userspace interface. The value of a userspace
 *   interface for access to secrets is a point for further architectural
 *   discussion.
 *
 * - Partition/permission management is not part of this interface. It
 *   depends on some level of "knowledge" agreed upon between bootloader,
 *   provisioning applications, and OS-hosted software (which uses this
 *   driver).
 *
 * - No means of identifying the location or purpose of secrets managed by
 *   this interface exists; "slot location" and format of a given secret
 *   needs to be agreed upon between bootloader, provisioner, and OS-hosted
 *   application.
 */

#include "compat.h"
#include "regs.h"
#include "jr.h"
#include "desc.h"
#include "intern.h"
#include "error.h"
#include "sm.h"
#include <linux/of_address.h>

#define SECMEM_KEYMOD_LEN 8
#define GENMEM_KEYMOD_LEN 16

#ifdef SM_DEBUG_CONT
void sm_show_page(struct device *dev, struct sm_page_descriptor *pgdesc)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	u32 i, *smdata;

	dev_info(dev, "physical page %d content at 0x%08x\n",
		 pgdesc->phys_pagenum, pgdesc->pg_base);
	smdata = pgdesc->pg_base;
	for (i = 0; i < (smpriv->page_size / sizeof(u32)); i += 4)
		dev_info(dev, "[0x%08x] 0x%08x 0x%08x 0x%08x 0x%08x\n",
			 (u32)&smdata[i], smdata[i], smdata[i+1], smdata[i+2],
			 smdata[i+3]);
}
#endif

#define INITIAL_DESCSZ 16	/* size of tmp buffer for descriptor const. */

static __always_inline u32 sm_send_cmd(struct caam_drv_private_sm *smpriv,
					     struct caam_drv_private_jr *jrpriv,
					     u32 cmd, u32 *status)
{
	void __iomem *write_address;
	void __iomem *read_address;

	if (smpriv->sm_reg_offset == SM_V1_OFFSET) {
		struct caam_secure_mem_v1 *sm_regs_v1;

		sm_regs_v1 = (struct caam_secure_mem_v1 *)
			((void *)jrpriv->rregs + SM_V1_OFFSET);
		write_address = &sm_regs_v1->sm_cmd;
		read_address = &sm_regs_v1->sm_status;

	} else if (smpriv->sm_reg_offset == SM_V2_OFFSET) {
		struct caam_secure_mem_v2 *sm_regs_v2;

		sm_regs_v2 = (struct caam_secure_mem_v2 *)
			((void *)jrpriv->rregs + SM_V2_OFFSET);
		write_address = &sm_regs_v2->sm_cmd;
		read_address = &sm_regs_v2->sm_status;

	} else {
		return -EINVAL;
	}

	wr_reg32(write_address, cmd);

	udelay(10);

	/* Read until the command has terminated and the status is correct */
	do {
		*status = rd_reg32(read_address);
	} while (((*status & SMCS_CMDERR_MASK) >>  SMCS_CMDERR_SHIFT)
				   == SMCS_CMDERR_INCOMP);

	return 0;
}

/*
 * Construct a black key conversion job descriptor
 *
 * This function constructs a job descriptor capable of performing
 * a key blackening operation on a plaintext secure memory resident object.
 *
 * - desc	pointer to a pointer to the descriptor generated by this
 *		function. Caller will be responsible to kfree() this
 *		descriptor after execution.
 * - key	physical pointer to the plaintext, which will also hold
 *		the result. Since encryption occurs in place, caller must
 *              ensure that the space is large enough to accommodate the
 *              blackened key
 * - keysz	size of the plaintext
 * - auth	if a CCM-covered key is required, use KEY_COVER_CCM, else
 *		use KEY_COVER_ECB.
 *
 * KEY to key1 from @key_addr LENGTH 16 BYTES;
 * FIFO STORE from key1[ecb] TO @key_addr LENGTH 16 BYTES;
 *
 * Note that this variant uses the JDKEK only; it does not accommodate the
 * trusted key encryption key at this time.
 *
 */
static int blacken_key_jobdesc(u32 **desc, void *key, u16 keysz, bool auth)
{
	u32 *tdesc, tmpdesc[INITIAL_DESCSZ];
	u16 dsize, idx;

	memset(tmpdesc, 0, INITIAL_DESCSZ * sizeof(u32));
	idx = 1;

	/* Load key to class 1 key register */
	tmpdesc[idx++] = CMD_KEY | CLASS_1 | (keysz & KEY_LENGTH_MASK);
	tmpdesc[idx++] = (uintptr_t)key;

	/* ...and write back out via FIFO store*/
	tmpdesc[idx] = CMD_FIFO_STORE | CLASS_1 | (keysz & KEY_LENGTH_MASK);

	/* plus account for ECB/CCM option in FIFO_STORE */
	if (auth == KEY_COVER_ECB)
		tmpdesc[idx] |= FIFOST_TYPE_KEY_KEK;
	else
		tmpdesc[idx] |= FIFOST_TYPE_KEY_CCM_JKEK;

	idx++;
	tmpdesc[idx++] = (uintptr_t)key;

	/* finish off the job header */
	tmpdesc[0] = CMD_DESC_HDR | HDR_ONE | (idx & HDR_DESCLEN_MASK);
	dsize = idx * sizeof(u32);

	/* now allocate execution buffer and coat it with executable */
	tdesc = kmalloc(dsize, GFP_KERNEL | GFP_DMA);
	if (tdesc == NULL)
		return 0;

	memcpy(tdesc, tmpdesc, dsize);
	*desc = tdesc;

	return dsize;
}

/*
 * Construct a blob encapsulation job descriptor
 *
 * This function dynamically constructs a blob encapsulation job descriptor
 * from the following arguments:
 *
 * - desc	pointer to a pointer to the descriptor generated by this
 *		function. Caller will be responsible to kfree() this
 *		descriptor after execution.
 * - keymod	Physical pointer to a key modifier, which must reside in a
 *		contiguous piece of memory. Modifier will be assumed to be
 *		8 bytes long for a blob of type SM_SECMEM, or 16 bytes long
 *		for a blob of type SM_GENMEM (see blobtype argument).
 * - secretbuf	Physical pointer to a secret, normally a black or red key,
 *		possibly residing within an accessible secure memory page,
 *		of the secret to be encapsulated to an output blob.
 * - outbuf	Physical pointer to the destination buffer to receive the
 *		encapsulated output. This buffer will need to be 48 bytes
 *		larger than the input because of the added encapsulation data.
 *		The generated descriptor will account for the increase in size,
 *		but the caller must also account for this increase in the
 *		buffer allocator.
 * - secretsz	Size of input secret, in bytes. This is limited to 65536
 *		less the size of blob overhead, since the length embeds into
 *		DECO pointer in/out instructions.
 * - keycolor   Determines if the source data is covered (black key) or
 *		plaintext (red key). RED_KEY or BLACK_KEY are defined in
 *		for this purpose.
 * - blobtype	Determine if encapsulated blob should be a secure memory
 *		blob (SM_SECMEM), with partition data embedded with key
 *		material, or a general memory blob (SM_GENMEM).
 * - auth	If BLACK_KEY source is covered via AES-CCM, specify
 *		KEY_COVER_CCM, else uses AES-ECB (KEY_COVER_ECB).
 *
 * Upon completion, desc points to a buffer containing a CAAM job
 * descriptor which encapsulates data into an externally-storable blob
 * suitable for use across power cycles.
 *
 * This is an example of a black key encapsulation job into a general memory
 * blob. Notice the 16-byte key modifier in the LOAD instruction. Also note
 * the output 48 bytes longer than the input:
 *
 * [00] B0800008       jobhdr: stidx=0 len=8
 * [01] 14400010           ld: ccb2-key len=16 offs=0
 * [02] 08144891               ptr->@0x08144891
 * [03] F800003A    seqoutptr: len=58
 * [04] 01000000               out_ptr->@0x01000000
 * [05] F000000A     seqinptr: len=10
 * [06] 09745090               in_ptr->@0x09745090
 * [07] 870D0004    operation: encap blob  reg=memory, black, format=normal
 *
 * This is an example of a red key encapsulation job for storing a red key
 * into a secure memory blob. Note the 8 byte modifier on the 12 byte offset
 * in the LOAD instruction; this accounts for blob permission storage:
 *
 * [00] B0800008       jobhdr: stidx=0 len=8
 * [01] 14400C08           ld: ccb2-key len=8 offs=12
 * [02] 087D0784               ptr->@0x087d0784
 * [03] F8000050    seqoutptr: len=80
 * [04] 09251BB2               out_ptr->@0x09251bb2
 * [05] F0000020     seqinptr: len=32
 * [06] 40000F31               in_ptr->@0x40000f31
 * [07] 870D0008    operation: encap blob  reg=memory, red, sec_mem,
 *                             format=normal
 *
 * Note: this function only generates 32-bit pointers at present, and should
 * be refactored using a scheme that allows both 32 and 64 bit addressing
 */

static int blob_encap_jobdesc(u32 **desc, dma_addr_t keymod,
			      void *secretbuf, dma_addr_t outbuf,
			      u16 secretsz, u8 keycolor, u8 blobtype, u8 auth)
{
	u32 *tdesc, tmpdesc[INITIAL_DESCSZ];
	u16 dsize, idx;

	memset(tmpdesc, 0, INITIAL_DESCSZ * sizeof(u32));
	idx = 1;

	/*
	 * Key modifier works differently for secure/general memory blobs
	 * This accounts for the permission/protection data encapsulated
	 * within the blob if a secure memory blob is requested
	 */
	if (blobtype == SM_SECMEM)
		tmpdesc[idx++] = CMD_LOAD | LDST_CLASS_2_CCB |
				 LDST_SRCDST_BYTE_KEY |
				 ((12 << LDST_OFFSET_SHIFT) & LDST_OFFSET_MASK)
				 | (8 & LDST_LEN_MASK);
	else /* is general memory blob */
		tmpdesc[idx++] = CMD_LOAD | LDST_CLASS_2_CCB |
				 LDST_SRCDST_BYTE_KEY | (16 & LDST_LEN_MASK);

	tmpdesc[idx++] = (u32)keymod;

	/*
	 * Encapsulation output must include space for blob key encryption
	 * key and MAC tag
	 */
	tmpdesc[idx++] = CMD_SEQ_OUT_PTR | (secretsz + BLOB_OVERHEAD);
	tmpdesc[idx++] = (u32)outbuf;

	/* Input data, should be somewhere in secure memory */
	tmpdesc[idx++] = CMD_SEQ_IN_PTR | secretsz;
	tmpdesc[idx++] = (uintptr_t)secretbuf;

	/* Set blob encap, then color */
	tmpdesc[idx] = CMD_OPERATION | OP_TYPE_ENCAP_PROTOCOL | OP_PCLID_BLOB;

	if (blobtype == SM_SECMEM)
		tmpdesc[idx] |= OP_PCL_BLOB_PTXT_SECMEM;

	if (auth == KEY_COVER_CCM)
		tmpdesc[idx] |= OP_PCL_BLOB_EKT;

	if (keycolor == BLACK_KEY)
		tmpdesc[idx] |= OP_PCL_BLOB_BLACK;

	idx++;
	tmpdesc[0] = CMD_DESC_HDR | HDR_ONE | (idx & HDR_DESCLEN_MASK);
	dsize = idx * sizeof(u32);

	tdesc = kmalloc(dsize, GFP_KERNEL | GFP_DMA);
	if (tdesc == NULL)
		return 0;

	memcpy(tdesc, tmpdesc, dsize);
	*desc = tdesc;
	return dsize;
}

/*
 * Construct a blob decapsulation job descriptor
 *
 * This function dynamically constructs a blob decapsulation job descriptor
 * from the following arguments:
 *
 * - desc	pointer to a pointer to the descriptor generated by this
 *		function. Caller will be responsible to kfree() this
 *		descriptor after execution.
 * - keymod	Physical pointer to a key modifier, which must reside in a
 *		contiguous piece of memory. Modifier will be assumed to be
 *		8 bytes long for a blob of type SM_SECMEM, or 16 bytes long
 *		for a blob of type SM_GENMEM (see blobtype argument).
 * - blobbuf	Physical pointer (into external memory) of the blob to
 *		be decapsulated. Blob must reside in a contiguous memory
 *		segment.
 * - outbuf	Physical pointer of the decapsulated output, possibly into
 *		a location within a secure memory page. Must be contiguous.
 * - secretsz	Size of encapsulated secret in bytes (not the size of the
 *		input blob).
 * - keycolor   Determines if decapsulated content is encrypted (BLACK_KEY)
 *		or left as plaintext (RED_KEY).
 * - blobtype	Determine if encapsulated blob should be a secure memory
 *		blob (SM_SECMEM), with partition data embedded with key
 *		material, or a general memory blob (SM_GENMEM).
 * - auth	If decapsulation path is specified by BLACK_KEY, then if
 *		AES-CCM is requested for key covering use KEY_COVER_CCM, else
 *		use AES-ECB (KEY_COVER_ECB).
 *
 * Upon completion, desc points to a buffer containing a CAAM job descriptor
 * that decapsulates a key blob from external memory into a black (encrypted)
 * key or red (plaintext) content.
 *
 * This is an example of a black key decapsulation job from a general memory
 * blob. Notice the 16-byte key modifier in the LOAD instruction.
 *
 * [00] B0800008       jobhdr: stidx=0 len=8
 * [01] 14400010           ld: ccb2-key len=16 offs=0
 * [02] 08A63B7F               ptr->@0x08a63b7f
 * [03] F8000010    seqoutptr: len=16
 * [04] 01000000               out_ptr->@0x01000000
 * [05] F000003A     seqinptr: len=58
 * [06] 01000010               in_ptr->@0x01000010
 * [07] 860D0004    operation: decap blob  reg=memory, black, format=normal
 *
 * This is an example of a red key decapsulation job for restoring a red key
 * from a secure memory blob. Note the 8 byte modifier on the 12 byte offset
 * in the LOAD instruction:
 *
 * [00] B0800008       jobhdr: stidx=0 len=8
 * [01] 14400C08           ld: ccb2-key len=8 offs=12
 * [02] 01000000               ptr->@0x01000000
 * [03] F8000020    seqoutptr: len=32
 * [04] 400000E6               out_ptr->@0x400000e6
 * [05] F0000050     seqinptr: len=80
 * [06] 08F0C0EA               in_ptr->@0x08f0c0ea
 * [07] 860D0008    operation: decap blob  reg=memory, red, sec_mem,
 *			       format=normal
 *
 * Note: this function only generates 32-bit pointers at present, and should
 * be refactored using a scheme that allows both 32 and 64 bit addressing
 */

static int blob_decap_jobdesc(u32 **desc, dma_addr_t keymod, dma_addr_t blobbuf,
			      u8 *outbuf, u16 secretsz, u8 keycolor,
			      u8 blobtype, u8 auth)
{
	u32 *tdesc, tmpdesc[INITIAL_DESCSZ];
	u16 dsize, idx;

	memset(tmpdesc, 0, INITIAL_DESCSZ * sizeof(u32));
	idx = 1;

	/* Load key modifier */
	if (blobtype == SM_SECMEM)
		tmpdesc[idx++] = CMD_LOAD | LDST_CLASS_2_CCB |
				 LDST_SRCDST_BYTE_KEY |
				 ((12 << LDST_OFFSET_SHIFT) & LDST_OFFSET_MASK)
				 | (8 & LDST_LEN_MASK);
	else /* is general memory blob */
		tmpdesc[idx++] = CMD_LOAD | LDST_CLASS_2_CCB |
				 LDST_SRCDST_BYTE_KEY | (16 & LDST_LEN_MASK);

	tmpdesc[idx++] = (u32)keymod;

	/* Compensate BKEK + MAC tag over size of encapsulated secret */
	tmpdesc[idx++] = CMD_SEQ_IN_PTR | (secretsz + BLOB_OVERHEAD);
	tmpdesc[idx++] = (u32)blobbuf;
	tmpdesc[idx++] = CMD_SEQ_OUT_PTR | secretsz;
	tmpdesc[idx++] = (uintptr_t)outbuf;

	/* Decapsulate from secure memory partition to black blob */
	tmpdesc[idx] = CMD_OPERATION | OP_TYPE_DECAP_PROTOCOL | OP_PCLID_BLOB;

	if (blobtype == SM_SECMEM)
		tmpdesc[idx] |= OP_PCL_BLOB_PTXT_SECMEM;

	if (auth == KEY_COVER_CCM)
		tmpdesc[idx] |= OP_PCL_BLOB_EKT;

	if (keycolor == BLACK_KEY)
		tmpdesc[idx] |= OP_PCL_BLOB_BLACK;

	idx++;
	tmpdesc[0] = CMD_DESC_HDR | HDR_ONE | (idx & HDR_DESCLEN_MASK);
	dsize = idx * sizeof(u32);

	tdesc = kmalloc(dsize, GFP_KERNEL | GFP_DMA);
	if (tdesc == NULL)
		return 0;

	memcpy(tdesc, tmpdesc, dsize);
	*desc = tdesc;
	return dsize;
}

/*
 * Pseudo-synchronous ring access functions for carrying out key
 * encapsulation and decapsulation
 */

struct sm_key_job_result {
	int error;
	struct completion completion;
};

void sm_key_job_done(struct device *dev, u32 *desc, u32 err, void *context)
{
	struct sm_key_job_result *res = context;

	if (err)
		caam_jr_strstatus(dev, err);

	res->error = err;	/* save off the error for postprocessing */

	complete(&res->completion);	/* mark us complete */
}

static int sm_key_job(struct device *ksdev, u32 *jobdesc)
{
	struct sm_key_job_result testres = {0};
	struct caam_drv_private_sm *kspriv;
	int rtn = 0;

	kspriv = dev_get_drvdata(ksdev);

	init_completion(&testres.completion);

	rtn = caam_jr_enqueue(kspriv->smringdev, jobdesc, sm_key_job_done,
			      &testres);
	if (rtn)
		goto exit;

	wait_for_completion_interruptible(&testres.completion);
	rtn = testres.error;

exit:
	return rtn;
}

/*
 * Following section establishes the default methods for keystore access
 * They are NOT intended for use external to this module
 *
 * In the present version, these are the only means for the higher-level
 * interface to deal with the mechanics of accessing the phyiscal keystore
 */


int slot_alloc(struct device *dev, u32 unit, u32 size, u32 *slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;
	u32 i;
#ifdef SM_DEBUG
	dev_info(dev, "slot_alloc(): requesting slot for %d bytes\n", size);
#endif

	if (size > smpriv->slot_size)
		return -EKEYREJECTED;

	for (i = 0; i < ksdata->slot_count; i++) {
		if (ksdata->slot[i].allocated == 0) {
			ksdata->slot[i].allocated = 1;
			(*slot) = i;
#ifdef SM_DEBUG
			dev_info(dev, "slot_alloc(): new slot %d allocated\n",
				 *slot);
#endif
			return 0;
		}
	}

	return -ENOSPC;
}
EXPORT_SYMBOL(slot_alloc);

int slot_dealloc(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;
	u8 __iomem *slotdata;

#ifdef SM_DEBUG
	dev_info(dev, "slot_dealloc(): releasing slot %d\n", slot);
#endif
	if (slot >= ksdata->slot_count)
		return -EINVAL;
	slotdata = ksdata->base_address + slot * smpriv->slot_size;

	if (ksdata->slot[slot].allocated == 1) {
		/* Forcibly overwrite the data from the keystore */
		memset_io(ksdata->base_address + slot * smpriv->slot_size, 0,
		       smpriv->slot_size);

		ksdata->slot[slot].allocated = 0;
#ifdef SM_DEBUG
		dev_info(dev, "slot_dealloc(): slot %d released\n", slot);
#endif
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(slot_dealloc);

void *slot_get_address(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;

	if (slot >= ksdata->slot_count)
		return NULL;

#ifdef SM_DEBUG
	dev_info(dev, "slot_get_address(): slot %d is 0x%08x\n", slot,
		 (u32)ksdata->base_address + slot * smpriv->slot_size);
#endif

	return ksdata->base_address + slot * smpriv->slot_size;
}

void *slot_get_physical(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;

	if (slot >= ksdata->slot_count)
		return NULL;

#ifdef SM_DEBUG
	dev_info(dev, "%s: slot %d is 0x%08x\n", __func__, slot,
		 (u32)ksdata->phys_address + slot * smpriv->slot_size);
#endif

	return ksdata->phys_address + slot * smpriv->slot_size;
}

u32 slot_get_base(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;

	/*
	 * There could potentially be more than one secure partition object
	 * associated with this keystore.  For now, there is just one.
	 */

	(void)slot;

#ifdef SM_DEBUG
	dev_info(dev, "slot_get_base(): slot %d = 0x%08x\n",
		slot, (u32)ksdata->base_address);
#endif

	return (uintptr_t)(ksdata->base_address);
}

u32 slot_get_offset(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;

	if (slot >= ksdata->slot_count)
		return -EINVAL;

#ifdef SM_DEBUG
	dev_info(dev, "slot_get_offset(): slot %d = %d\n", slot,
		slot * smpriv->slot_size);
#endif

	return slot * smpriv->slot_size;
}

u32 slot_get_slot_size(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);


#ifdef SM_DEBUG
	dev_info(dev, "slot_get_slot_size(): slot %d = %d\n", slot,
		 smpriv->slot_size);
#endif
	/* All slots are the same size in the default implementation */
	return smpriv->slot_size;
}



int kso_init_data(struct device *dev, u32 unit)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *keystore_data = NULL;
	u32 slot_count;
	u32 keystore_data_size;

	/*
	 * Calculate the required size of the keystore data structure, based
	 * on the number of keys that can fit in the partition.
	 */
	slot_count = smpriv->page_size / smpriv->slot_size;
#ifdef SM_DEBUG
	dev_info(dev, "kso_init_data: %d slots initializing\n", slot_count);
#endif

	keystore_data_size = sizeof(struct keystore_data) +
				slot_count *
				sizeof(struct keystore_data_slot_info);

	keystore_data = kzalloc(keystore_data_size, GFP_KERNEL);

	if (!keystore_data)
		return -ENOMEM;

#ifdef SM_DEBUG
	dev_info(dev, "kso_init_data: keystore data size = %d\n",
		 keystore_data_size);
#endif

	/*
	 * Place the slot information structure directly after the keystore data
	 * structure.
	 */
	keystore_data->slot = (struct keystore_data_slot_info *)
			      (keystore_data + 1);
	keystore_data->slot_count = slot_count;

	smpriv->pagedesc[unit].ksdata = keystore_data;
	smpriv->pagedesc[unit].ksdata->base_address =
		smpriv->pagedesc[unit].pg_base;
	smpriv->pagedesc[unit].ksdata->phys_address =
		smpriv->pagedesc[unit].pg_phys;

	return 0;
}

void kso_cleanup_data(struct device *dev, u32 unit)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *keystore_data = NULL;

	if (smpriv->pagedesc[unit].ksdata != NULL)
		keystore_data = smpriv->pagedesc[unit].ksdata;

	/* Release the allocated keystore management data */
	kfree(smpriv->pagedesc[unit].ksdata);

	return;
}



/*
 * Keystore management section
 */

void sm_init_keystore(struct device *dev)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

	smpriv->data_init = kso_init_data;
	smpriv->data_cleanup = kso_cleanup_data;
	smpriv->slot_alloc = slot_alloc;
	smpriv->slot_dealloc = slot_dealloc;
	smpriv->slot_get_address = slot_get_address;
	smpriv->slot_get_physical = slot_get_physical;
	smpriv->slot_get_base = slot_get_base;
	smpriv->slot_get_offset = slot_get_offset;
	smpriv->slot_get_slot_size = slot_get_slot_size;
#ifdef SM_DEBUG
	dev_info(dev, "sm_init_keystore(): handlers installed\n");
#endif
}
EXPORT_SYMBOL(sm_init_keystore);

/* Return available pages/units */
u32 sm_detect_keystore_units(struct device *dev)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

	return smpriv->localpages;
}
EXPORT_SYMBOL(sm_detect_keystore_units);

/*
 * Do any keystore specific initializations
 */
int sm_establish_keystore(struct device *dev, u32 unit)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

#ifdef SM_DEBUG
	dev_info(dev, "sm_establish_keystore(): unit %d initializing\n", unit);
#endif

	if (smpriv->data_init == NULL)
		return -EINVAL;

	/* Call the data_init function for any user setup */
	return smpriv->data_init(dev, unit);
}
EXPORT_SYMBOL(sm_establish_keystore);

void sm_release_keystore(struct device *dev, u32 unit)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

#ifdef SM_DEBUG
	dev_info(dev, "sm_establish_keystore(): unit %d releasing\n", unit);
#endif
	if ((smpriv != NULL) && (smpriv->data_cleanup != NULL))
		smpriv->data_cleanup(dev, unit);

	return;
}
EXPORT_SYMBOL(sm_release_keystore);

/*
 * Subsequent interfacce (sm_keystore_*) forms the accessor interfacce to
 * the keystore
 */
int sm_keystore_slot_alloc(struct device *dev, u32 unit, u32 size, u32 *slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = -EINVAL;

	spin_lock(&smpriv->kslock);

	if ((smpriv->slot_alloc == NULL) ||
	    (smpriv->pagedesc[unit].ksdata == NULL))
		goto out;

	retval =  smpriv->slot_alloc(dev, unit, size, slot);

out:
	spin_unlock(&smpriv->kslock);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_alloc);

int sm_keystore_slot_dealloc(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = -EINVAL;

	spin_lock(&smpriv->kslock);

	if ((smpriv->slot_alloc == NULL) ||
	    (smpriv->pagedesc[unit].ksdata == NULL))
		goto out;

	retval = smpriv->slot_dealloc(dev, unit, slot);
out:
	spin_unlock(&smpriv->kslock);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_dealloc);

int sm_keystore_slot_load(struct device *dev, u32 unit, u32 slot,
			  const u8 *key_data, u32 key_length)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = -EINVAL;
	u32 slot_size;
	u8 __iomem *slot_location;

	spin_lock(&smpriv->kslock);

	slot_size = smpriv->slot_get_slot_size(dev, unit, slot);

	if (key_length > slot_size) {
		retval = -EFBIG;
		goto out;
	}

	slot_location = smpriv->slot_get_address(dev, unit, slot);

	memcpy_toio(slot_location, key_data, key_length);

	retval = 0;

out:
	spin_unlock(&smpriv->kslock);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_load);

int sm_keystore_slot_read(struct device *dev, u32 unit, u32 slot,
			  u32 key_length, u8 *key_data)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = -EINVAL;
	u8 __iomem *slot_addr;
	u32 slot_size;

	spin_lock(&smpriv->kslock);

	slot_addr = smpriv->slot_get_address(dev, unit, slot);
	slot_size = smpriv->slot_get_slot_size(dev, unit, slot);

	if (key_length > slot_size) {
		retval = -EKEYREJECTED;
		goto out;
	}

	memcpy_fromio(key_data, slot_addr, key_length);
	retval = 0;

out:
	spin_unlock(&smpriv->kslock);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_read);

/*
 * Blacken a clear key in a slot. Operates "in place".
 * Limited to class 1 keys at the present time
 */
int sm_keystore_cover_key(struct device *dev, u32 unit, u32 slot,
			  u16 key_length, u8 keyauth)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = 0;
	u8 __iomem *slotaddr;
	void *slotphys;
	u32 dsize, jstat;
	u32 __iomem *coverdesc = NULL;

	/* Get the address of the object in the slot */
	slotaddr = (u8 *)smpriv->slot_get_address(dev, unit, slot);
	slotphys = (u8 *)smpriv->slot_get_physical(dev, unit, slot);

	dsize = blacken_key_jobdesc(&coverdesc, slotphys, key_length, keyauth);
	if (!dsize)
		return -ENOMEM;
	jstat = sm_key_job(dev, coverdesc);
	if (jstat)
		retval = -EIO;

	kfree(coverdesc);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_cover_key);

/* Export a black/red key to a blob in external memory */
int sm_keystore_slot_export(struct device *dev, u32 unit, u32 slot, u8 keycolor,
			    u8 keyauth, u8 *outbuf, u16 keylen, u8 *keymod)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = 0;
	u8 __iomem *slotaddr, *lkeymod;
	u8 __iomem *slotphys;
	dma_addr_t keymod_dma, outbuf_dma;
	u32 dsize, jstat;
	u32 __iomem *encapdesc = NULL;
	struct device *dev_for_dma_op;

	/* Use the ring as device for DMA operations */
	dev_for_dma_op = smpriv->smringdev;

	/* Get the base address(es) of the specified slot */
	slotaddr = (u8 *)smpriv->slot_get_address(dev, unit, slot);
	slotphys = smpriv->slot_get_physical(dev, unit, slot);

	/* Allocate memory for key modifier compatible with DMA */
	lkeymod = kmalloc(SECMEM_KEYMOD_LEN, GFP_KERNEL | GFP_DMA);
	if (!lkeymod) {
		retval = (-ENOMEM);
		goto exit;
	}

	/* Get DMA address for the key modifier */
	keymod_dma = dma_map_single(dev_for_dma_op, lkeymod,
					SECMEM_KEYMOD_LEN, DMA_TO_DEVICE);
	if (dma_mapping_error(dev_for_dma_op, keymod_dma)) {
		dev_err(dev, "unable to map keymod: %p\n", lkeymod);
		retval = (-ENOMEM);
		goto free_keymod;
	}

	/* Copy the keymod and synchronize the DMA */
	memcpy(lkeymod, keymod, SECMEM_KEYMOD_LEN);
	dma_sync_single_for_device(dev_for_dma_op, keymod_dma,
					SECMEM_KEYMOD_LEN, DMA_TO_DEVICE);

	/* Get DMA address for the destination */
	outbuf_dma = dma_map_single(dev_for_dma_op, outbuf,
				keylen + BLOB_OVERHEAD, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev_for_dma_op, outbuf_dma)) {
		dev_err(dev, "unable to map outbuf: %p\n", outbuf);
		retval = (-ENOMEM);
		goto unmap_keymod;
	}

	/* Build the encapsulation job descriptor */
	dsize = blob_encap_jobdesc(&encapdesc, keymod_dma, slotphys, outbuf_dma,
				   keylen, keycolor, SM_SECMEM, keyauth);
	if (!dsize) {
		dev_err(dev, "can't alloc an encapsulation descriptor\n");
		retval = -ENOMEM;
		goto unmap_outbuf;
	}

	/* Run the job */
	jstat = sm_key_job(dev, encapdesc);
	if (jstat) {
		retval = (-EIO);
		goto free_desc;
	}

	/* Synchronize the data received */
	dma_sync_single_for_cpu(dev_for_dma_op, outbuf_dma,
			keylen + BLOB_OVERHEAD, DMA_FROM_DEVICE);

free_desc:
	kfree(encapdesc);

unmap_outbuf:
	dma_unmap_single(dev_for_dma_op, outbuf_dma, keylen + BLOB_OVERHEAD,
			DMA_FROM_DEVICE);

unmap_keymod:
	dma_unmap_single(dev_for_dma_op, keymod_dma, SECMEM_KEYMOD_LEN,
			DMA_TO_DEVICE);

free_keymod:
	kfree(lkeymod);

exit:
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_export);

/* Import a black/red key from a blob residing in external memory */
int sm_keystore_slot_import(struct device *dev, u32 unit, u32 slot, u8 keycolor,
			    u8 keyauth, u8 *inbuf, u16 keylen, u8 *keymod)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = 0;
	u8 __iomem *slotaddr, *lkeymod;
	u8 __iomem *slotphys;
	dma_addr_t keymod_dma, inbuf_dma;
	u32 dsize, jstat;
	u32 __iomem *decapdesc = NULL;
	struct device *dev_for_dma_op;

	/* Use the ring as device for DMA operations */
	dev_for_dma_op = smpriv->smringdev;

	/* Get the base address(es) of the specified slot */
	slotaddr = (u8 *)smpriv->slot_get_address(dev, unit, slot);
	slotphys = smpriv->slot_get_physical(dev, unit, slot);

	/* Allocate memory for key modifier compatible with DMA */
	lkeymod = kmalloc(SECMEM_KEYMOD_LEN, GFP_KERNEL | GFP_DMA);
	if (!lkeymod) {
		retval = (-ENOMEM);
		goto exit;
	}

	/* Get DMA address for the key modifier */
	keymod_dma = dma_map_single(dev_for_dma_op, lkeymod,
					SECMEM_KEYMOD_LEN, DMA_TO_DEVICE);
	if (dma_mapping_error(dev_for_dma_op, keymod_dma)) {
		dev_err(dev, "unable to map keymod: %p\n", lkeymod);
		retval = (-ENOMEM);
		goto free_keymod;
	}

	/* Copy the keymod and synchronize the DMA */
	memcpy(lkeymod, keymod, SECMEM_KEYMOD_LEN);
	dma_sync_single_for_device(dev_for_dma_op, keymod_dma,
					SECMEM_KEYMOD_LEN, DMA_TO_DEVICE);

	/* Get DMA address for the input */
	inbuf_dma = dma_map_single(dev_for_dma_op, inbuf,
					keylen + BLOB_OVERHEAD, DMA_TO_DEVICE);
	if (dma_mapping_error(dev_for_dma_op, inbuf_dma)) {
		dev_err(dev, "unable to map inbuf: %p\n", (void *)inbuf_dma);
		retval = (-ENOMEM);
		goto unmap_keymod;
	}

	/* synchronize the DMA */
	dma_sync_single_for_device(dev_for_dma_op, inbuf_dma,
					keylen + BLOB_OVERHEAD, DMA_TO_DEVICE);

	/* Build the encapsulation job descriptor */
	dsize = blob_decap_jobdesc(&decapdesc, keymod_dma, inbuf_dma, slotphys,
				   keylen, keycolor, SM_SECMEM, keyauth);
	if (!dsize) {
		dev_err(dev, "can't alloc a decapsulation descriptor\n");
		retval = -ENOMEM;
		goto unmap_inbuf;
	}

	/* Run the job */
	jstat = sm_key_job(dev, decapdesc);

	/*
	 * May want to expand upon error meanings a bit. Any CAAM status
	 * is reported as EIO, but we might want to look for something more
	 * meaningful for something like an ICV error on restore, otherwise
	 * the caller is left guessing.
	 */
	if (jstat) {
		retval = (-EIO);
		goto free_desc;
	}

free_desc:
	kfree(decapdesc);

unmap_inbuf:
	dma_unmap_single(dev_for_dma_op, inbuf_dma, keylen + BLOB_OVERHEAD,
			DMA_TO_DEVICE);

unmap_keymod:
	dma_unmap_single(dev_for_dma_op, keymod_dma, SECMEM_KEYMOD_LEN,
			DMA_TO_DEVICE);

free_keymod:
	kfree(lkeymod);

exit:
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_import);

/*
 * Initialization/shutdown subsystem
 * Assumes statically-invoked startup/shutdown from the controller driver
 * for the present time, to be reworked when a device tree becomes
 * available. This code will not modularize in present form.
 *
 * Also, simply uses ring 0 for execution at the present
 */

int caam_sm_startup(struct device *ctrldev)
{
	struct device *smdev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_sm *smpriv;
	struct caam_drv_private_jr *jrpriv;	/* need this for reg page */
	struct platform_device *sm_pdev;
	struct sm_page_descriptor *lpagedesc;
	u32 page, pgstat, lpagect, detectedpage, smvid, smpart;
	int ret = 0;

	struct device_node *np;
	ctrlpriv = dev_get_drvdata(ctrldev);

	if (!ctrlpriv->sm_present)
		return 0;

	/*
	 * Set up the private block for secure memory
	 * Only one instance is possible
	 */
	smpriv = kzalloc(sizeof(struct caam_drv_private_sm), GFP_KERNEL);
	if (smpriv == NULL) {
		dev_err(ctrldev, "can't alloc private mem for secure memory\n");
		ret = -ENOMEM;
		goto exit;
	}
	smpriv->parentdev = ctrldev; /* copy of parent dev is handy */
	spin_lock_init(&smpriv->kslock);

	/* Create the dev */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-caam-sm");
	if (np)
		of_node_clear_flag(np, OF_POPULATED);
	sm_pdev = of_platform_device_create(np, "caam_sm", ctrldev);

	if (sm_pdev == NULL) {
		ret = -EINVAL;
		goto free_smpriv;
	}

	/* Save a pointer to the platform device for Secure Memory */
	smpriv->sm_pdev = sm_pdev;
	smdev = &sm_pdev->dev;
	dev_set_drvdata(smdev, smpriv);
	ctrlpriv->smdev = smdev;

	/* Set the Secure Memory Register Map Version */
	smvid = rd_reg32(&ctrlpriv->jr[0]->perfmon.smvid);
	smpart = rd_reg32(&ctrlpriv->jr[0]->perfmon.smpart);

	if (smvid < SMVID_V2)
		smpriv->sm_reg_offset = SM_V1_OFFSET;
	else
		smpriv->sm_reg_offset = SM_V2_OFFSET;

	/*
	 * Collect configuration limit data for reference
	 * This batch comes from the partition data/vid registers in perfmon
	 */
	smpriv->max_pages = ((smpart & SMPART_MAX_NUMPG_MASK) >>
			    SMPART_MAX_NUMPG_SHIFT) + 1;
	smpriv->top_partition = ((smpart & SMPART_MAX_PNUM_MASK) >>
				SMPART_MAX_PNUM_SHIFT) + 1;
	smpriv->top_page =  ((smpart & SMPART_MAX_PG_MASK) >>
				SMPART_MAX_PG_SHIFT) + 1;
	smpriv->page_size = 1024 << ((smvid & SMVID_PG_SIZE_MASK) >>
				SMVID_PG_SIZE_SHIFT);
	smpriv->slot_size = 1 << CONFIG_CRYPTO_DEV_FSL_CAAM_SM_SLOTSIZE;

#ifdef SM_DEBUG
	dev_info(smdev, "max pages = %d, top partition = %d\n",
			smpriv->max_pages, smpriv->top_partition);
	dev_info(smdev, "top page = %d, page size = %d (total = %d)\n",
			smpriv->top_page, smpriv->page_size,
			smpriv->top_page * smpriv->page_size);
	dev_info(smdev, "selected slot size = %d\n", smpriv->slot_size);
#endif

	/*
	 * Now probe for partitions/pages to which we have access. Note that
	 * these have likely been set up by a bootloader or platform
	 * provisioning application, so we have to assume that we "inherit"
	 * a configuration and work within the constraints of what it might be.
	 *
	 * Assume use of the zeroth ring in the present iteration (until
	 * we can divorce the controller and ring drivers, and then assign
	 * an SM instance to any ring instance).
	 */
	smpriv->smringdev = caam_jr_alloc();
	if (!smpriv->smringdev) {
		dev_err(smdev, "Device for job ring not created\n");
		ret = -ENODEV;
		goto unregister_smpdev;
	}

	jrpriv = dev_get_drvdata(smpriv->smringdev);
	lpagect = 0;
	pgstat = 0;
	lpagedesc = kzalloc(sizeof(struct sm_page_descriptor)
			    * smpriv->max_pages, GFP_KERNEL);
	if (lpagedesc == NULL) {
		ret = -ENOMEM;
		goto free_smringdev;
	}

	for (page = 0; page < smpriv->max_pages; page++) {
		u32 page_ownership;

		if (sm_send_cmd(smpriv, jrpriv,
				((page << SMC_PAGE_SHIFT) & SMC_PAGE_MASK) |
				(SMC_CMD_PAGE_INQUIRY & SMC_CMD_MASK),
				&pgstat)) {
			ret = -EINVAL;
			goto free_lpagedesc;
		}

		page_ownership = (pgstat & SMCS_PGWON_MASK) >> SMCS_PGOWN_SHIFT;
		if ((page_ownership == SMCS_PGOWN_OWNED)
			|| (page_ownership == SMCS_PGOWN_NOOWN)) {
			/* page allocated */
			lpagedesc[page].phys_pagenum =
				(pgstat & SMCS_PAGE_MASK) >> SMCS_PAGE_SHIFT;
			lpagedesc[page].own_part =
				(pgstat & SMCS_PART_SHIFT) >> SMCS_PART_MASK;
			lpagedesc[page].pg_base = (u8 *)ctrlpriv->sm_base +
				(smpriv->page_size * page);
			if (ctrlpriv->scu_en) {
/* FIXME: get different addresses viewed by CPU and CAAM from
 * platform property
 */
				lpagedesc[page].pg_phys = (u8 *)0x20800000 +
					(smpriv->page_size * page);
			} else {
				lpagedesc[page].pg_phys =
					(u8 *) ctrlpriv->sm_phy +
					(smpriv->page_size * page);
			}
			lpagect++;
#ifdef SM_DEBUG
			dev_info(smdev,
				"physical page %d, owning partition = %d\n",
				lpagedesc[page].phys_pagenum,
				lpagedesc[page].own_part);
#endif
		}
	}

	smpriv->pagedesc = kzalloc(sizeof(struct sm_page_descriptor) * lpagect,
				   GFP_KERNEL);
	if (smpriv->pagedesc == NULL) {
		ret = -ENOMEM;
		goto free_lpagedesc;
	}
	smpriv->localpages = lpagect;

	detectedpage = 0;
	for (page = 0; page < smpriv->max_pages; page++) {
		if (lpagedesc[page].pg_base != NULL) {	/* e.g. live entry */
			memcpy(&smpriv->pagedesc[detectedpage],
			       &lpagedesc[page],
			       sizeof(struct sm_page_descriptor));
#ifdef SM_DEBUG_CONT
			sm_show_page(smdev, &smpriv->pagedesc[detectedpage]);
#endif
			detectedpage++;
		}
	}

	kfree(lpagedesc);

	sm_init_keystore(smdev);

	goto exit;

free_lpagedesc:
	kfree(lpagedesc);
free_smringdev:
	caam_jr_free(smpriv->smringdev);
unregister_smpdev:
	of_device_unregister(smpriv->sm_pdev);
free_smpriv:
	kfree(smpriv);

exit:
	return ret;
}

void caam_sm_shutdown(struct device *ctrldev)
{
	struct device *smdev;
	struct caam_drv_private *priv;
	struct caam_drv_private_sm *smpriv;

	priv = dev_get_drvdata(ctrldev);
	if (!priv->sm_present)
		return;

	smdev = priv->smdev;

	/* Return if resource not initialized by startup */
	if (smdev == NULL)
		return;

	smpriv = dev_get_drvdata(smdev);

	caam_jr_free(smpriv->smringdev);

	/* Remove Secure Memory Platform Device */
	of_device_unregister(smpriv->sm_pdev);

	kfree(smpriv->pagedesc);
	kfree(smpriv);
}
EXPORT_SYMBOL(caam_sm_shutdown);
