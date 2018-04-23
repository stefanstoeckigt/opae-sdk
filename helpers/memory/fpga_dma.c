// Copyright(c) 2018, Intel Corporation
//
// Redistribution  and  use  in source  and  binary  forms,  with  or  without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of  source code  must retain the  above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name  of Intel Corporation  nor the names of its contributors
//   may be used to  endorse or promote  products derived  from this  software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,  BUT NOT LIMITED TO,  THE
// IMPLIED WARRANTIES OF  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMEdesc.  IN NO EVENT  SHALL THE COPYRIGHT OWNER  OR CONTRIBUTORS BE
// LIABLE  FOR  ANY  DIRECT,  INDIRECT,  INCIDENTAL,  SPECIAL,  EXEMPLARY,  OR
// CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT LIMITED  TO,  PROCUREMENT  OF
// SUBSTITUTE GOODS OR SERVICES;  LOSS OF USE,  DATA, OR PROFITS;  OR BUSINESS
// INTERRUPTION)  HOWEVER CAUSED  AND ON ANY THEORY  OF LIABILITY,  WHETHER IN
// CONTRACT,  STRICT LIABILITY,  OR TORT  (INCLUDING NEGLIGENCE  OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,  EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* \fpga_dma.c
* \brief FPGA DMA User-mode driver
*/

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <opae/fpga.h>
#include <stddef.h>
#ifndef _WIN32
	#include <poll.h>
	#include <unistd.h>
	#include <sys/mman.h>
#endif
#include <errno.h>
#include <assert.h>
#include <safe_string/safe_string.h>
#include "memcpy_s_fast.h"
#include "ase_memory.h"
#include "fpga_dma_internal.h"
#include "fpga_dma.h"

#ifdef SIM
	#define USE_ASE
#else
	// TODO:  Need this until we can adequately sync MMIO R/W with pointer accesses.
	// Causes module to use fpgaMMIORead32() instead of foo = *ptr;
	#define USE_ASE
#endif

#ifdef FPGA_DMA_DEBUG
	static int err_cnt = 0;
#endif

#ifdef CHECK_DELAYS
	double poll_wait_count = 0;
	double buf_full_count = 0;
#endif

/*
* macro for checking return codes
*/
#define ON_ERR_GOTO(res, label, desc)\
	do {\
		if ((res) != FPGA_OK) {\
			error_print("%s (%d): Error %s: %s\n", __FUNCTION__, __LINE__, (desc), fpgaErrStr(res));\
			goto label;\
		}\
	} while (0)

#define ON_ERR_RETURN(res, desc)\
	do {\
		if ((res) != FPGA_OK) {\
			error_print("%s (%d): Error %s: %s\n", __FUNCTION__, __LINE__, (desc), fpgaErrStr(res));\
			return(res);\
		}\
	} while (0)

// Internal Functions

// End of feature list
static inline bool _fpga_dma_feature_eol(uint64_t dfh)
{
	return ((dfh >> AFU_DFH_EOL_OFFSET) & 1) == 1;
}

// Feature type is BBB
static inline bool _fpga_dma_feature_is_bbb(uint64_t dfh)
{
	// BBB is type 2
	return ((dfh >> AFU_DFH_TYPE_OFFSET) & 0xf) == FPGA_DMA_BBB;
}

// Offset to the next feature header
static inline uint64_t _fpga_dma_feature_next(uint64_t dfh)
{
	return (dfh >> AFU_DFH_NEXT_OFFSET) & 0xffffff;
}

/**
* _send_descriptor
*
* @brief                Queues a DMA descriptor to the FPGA
* @param[in] dma_h      Handle to the FPGA DMA object
* @param[in] desc       Pointer to a descriptor structure to send
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
static fpga_result _send_descriptor(fpga_dma_handle dma_h,
                                    msgdma_ext_desc_t *desc)
{
	fpga_result res = FPGA_OK;
	msgdma_status_t status = { 0 };

	debug_print("desc.rd_address = %x\n", desc->rd_address);
	debug_print("desc.wr_address = %x\n", desc->wr_address);
	debug_print("desc.len = %x\n", desc->len);
	debug_print("desc.wr_burst_count = %x\n", desc->wr_burst_count);
	debug_print("desc.rd_burst_count = %x\n", desc->rd_burst_count);
	debug_print("desc.wr_stride %x\n", desc->wr_stride);
	debug_print("desc.rd_stride %x\n", desc->rd_stride);
	debug_print("desc.rd_address_ext %x\n", desc->rd_address_ext);
	debug_print("desc.wr_address_ext %x\n", desc->wr_address_ext);

	debug_print("SGDMA_CSR_BASE = %lx SGDMA_DESC_BASE=%lx\n",
	            dma_h->dma_csr_base, dma_h->dma_desc_base);

#ifdef CHECK_DELAYS
	bool first = true;
#endif

	do {
		res = fpgaMMIORead32Blk(dma_h->fpga_h, dma_h->mmio_num, dma_h->mmio_va, CSR_STATUS(dma_h),
		                        (uint64_t)& status.reg, sizeof(status.reg));
		ON_ERR_GOTO(res, out, "fpgaMMIORead32Blk");
#ifdef CHECK_DELAYS

		if (first && status.st.desc_buf_full) {
			buf_full_count++;
			first = false;
		}

#endif
	} while (status.st.desc_buf_full);

	res = fpgaMMIOWrite64Blk(dma_h->fpga_h, dma_h->mmio_num, dma_h->mmio_va, dma_h->dma_desc_base,
	                         (uint64_t)desc, sizeof(*desc));
	ON_ERR_GOTO(res, out, "fpgaMMIOWrite64Blk");

out:
	return res;
}

/**
* _do_dma
*
* @brief                    Performs a DMA transaction with the FPGA
* @param[in] dma_h          Handle to the FPGA DMA object
* @param[in] dst            Pointer to a host or FPGA buffer to send or retrieve
* @param[in] src            Pointer to a host or FPGA buffer to send or retrieve
* @param[in] dst_ptr        Host-relative address to dst.  NULL if device address
* @param[in] src_ptr        Host-relative address to src.  NULL if host address
* @param[in] count          Number of bytes
* @param[in] is_last_desc   True if this is the last buffer of a batch
* @param[in] type           Direction of transfer
* @param[in] intr_en        True means to ask for an interrupt from the FPGA
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
static fpga_result _do_dma(fpga_dma_handle dma_h, uint64_t dst, uint64_t src,
                           char *dst_ptr, char *src_ptr, int count, int is_last_desc,
                           fpga_dma_transfer_t type, bool intr_en)
{
	msgdma_ext_desc_t desc = { 0 };
	fpga_result res = FPGA_OK;
	uint64_t alignment_offset = 0;

        (void)dst_ptr;
        (void)src_ptr;

	// src, dst and count must be 64-byte aligned

	if (!IS_DMA_ALIGNED(dst) || !IS_DMA_ALIGNED(src) || !IS_DMA_ALIGNED(count))  {
		return FPGA_INVALID_PARAM;
	}

	// these fields are fixed for all DMA transfers
	desc.seq_num = 0;
	desc.wr_stride = 1;
	desc.rd_stride = 1;

	desc.control.go = 1;

	if (intr_en) {
		desc.control.transfer_irq_en = 1;
	} else {
		desc.control.transfer_irq_en = 0;
	}

	// Enable "earlyreaddone" in the control field of the descriptor except the last.
	// Setting early done causes the read logic to move to the next descriptor
	// before the previous descriptor completes.
	// This elminates a few hundred clock cycles of waiting between transfers.
	if (is_last_desc || intr_en) {
		desc.control.early_done_en = 0;
	} else {
		desc.control.early_done_en = 1;
	}

	if (type == FPGA_TO_FPGA_MM) {
		// TODO:  Any alignment restrictions on this?
		desc.rd_address = src & FPGA_DMA_MASK_32_BIT;
		desc.wr_address = dst & FPGA_DMA_MASK_32_BIT;
		desc.len = count;
		desc.wr_burst_count = 4;
		desc.rd_burst_count = 4;
		desc.rd_address_ext = (src >> 32) & FPGA_DMA_MASK_32_BIT;
		desc.wr_address_ext = (dst >> 32) & FPGA_DMA_MASK_32_BIT;

		res = _send_descriptor(dma_h, &desc);
		ON_ERR_GOTO(res, out, "_send_descriptor");
	}
	// either FPGA to Host or Host to FPGA transfer so we need to make sure the DMA transaction
	// is aligned to the burst size (CCIP restriction)
	else {
		while (count > 0)
		{
			uint64_t to_copy = count;
			// need to determine if the CCIP (host) address is aligned to 4CL (256B).  When 0 the CCIP address is aligned.
			alignment_offset = (type == HOST_TO_FPGA_MM) ?
				(src % (4 * FPGA_DMA_ALIGN_BYTES)) :
				(dst % (4 * FPGA_DMA_ALIGN_BYTES));

			if (alignment_offset != 0) {
				// Short burst to get 4CL aligned
				uint64_t left_to_align = (4 * FPGA_DMA_ALIGN_BYTES) - alignment_offset;
				to_copy = min((uint64_t)count, left_to_align);
				assert(to_copy > 0);
			}

			uint8_t burst;
			if (to_copy >= (4 * FPGA_DMA_ALIGN_BYTES)) {
				burst = 4;
				to_copy = count & ~((uint64_t)(4 * FPGA_DMA_ALIGN_BYTES) - 1ULL);
			}
			else {
				burst = 1;
			}

			desc.rd_address = src & FPGA_DMA_MASK_32_BIT;
			desc.wr_address = dst & FPGA_DMA_MASK_32_BIT;
			desc.wr_burst_count = burst;
			desc.rd_burst_count = burst;
			desc.rd_address_ext = (src >> 32) & FPGA_DMA_MASK_32_BIT;
			desc.wr_address_ext = (dst >> 32) & FPGA_DMA_MASK_32_BIT;

			desc.len = to_copy;

			res = _send_descriptor(dma_h, &desc);
			ON_ERR_GOTO(res, out, "_send_descriptor");

			src += to_copy;
			dst += to_copy;
			assert(to_copy <= (uint64_t)count);
			count -= to_copy;
		}
	}                       // end of FPGA --> Host or Host --> FPGA transfer

out:
	return res;
}

static fpga_result clear_interrupt(fpga_dma_handle dma_h)
{
	//clear interrupt by writing 1 to IRQ bit in status register
	msgdma_status_t status = { 0 };
	status.st.irq = 1;

	return fpgaMMIOWrite32Blk(dma_h->fpga_h, dma_h->mmio_num, dma_h->mmio_va, CSR_STATUS(dma_h),
	                          (uint64_t)& status.reg, sizeof(status.reg));
}

static fpga_result poll_interrupt(fpga_dma_handle dma_h)
{
	struct pollfd pfd = { 0 };
	fpga_result res = FPGA_OK;
	int poll_res;

	res = fpgaGetOSObjectFromEventHandle(dma_h->eh, &pfd.fd);
	ON_ERR_GOTO(res, out, "fpgaGetOSObjectFromEventHandle failed\n");

	pfd.events = POLLIN;

#ifdef CHECK_DELAYS

	if (0 == poll(&pfd, 1, 0)) {
		poll_wait_count++;
	}

#endif
	poll_res = poll(&pfd, 1, FPGA_DMA_TIMEOUT_MSEC);

	if (poll_res < 0) {
		fprintf(stderr, "Poll error errno = %s\n", strerror(errno));
		res = FPGA_EXCEPTION;
		goto out;
	} else if (poll_res == 0) {
		fprintf(stderr, "Poll(interrupt) timeout \n");
		res = FPGA_EXCEPTION;
	} else {
		uint64_t count = 0;
		ssize_t bytes_read = read(pfd.fd, &count, sizeof(count));

		if (bytes_read > 0) {
			debug_print("Poll success. Return = %d, count = %d\n", poll_res, (int)count);
			res = FPGA_OK;
		} else {
			fprintf(stderr, "Error: poll failed read: %s\n",
			        bytes_read > 0 ? strerror(errno) : "zero bytes read");
			res = FPGA_EXCEPTION;
		}
	}

out:
	clear_interrupt(dma_h);
	return res;
}

static fpga_result _issue_magic(fpga_dma_handle dma_h)
{
	fpga_result res = FPGA_OK;
	*(dma_h->magic_buf) = 0x0ULL;

	res = _do_dma(dma_h, dma_h->magic_iova | FPGA_DMA_WF_HOST_MASK, FPGA_DMA_WF_ROM_MAGIC_NO_MASK,
	              (char *)dma_h->magic_buf, NULL, 64, 1, FPGA_TO_HOST_MM, true /*intr_en */);
	return res;
}

static void _wait_magic(fpga_dma_handle dma_h)
{
	poll_interrupt(dma_h);

	while (*(dma_h->magic_buf) != FPGA_DMA_WF_MAGIC_NO);

	*(dma_h->magic_buf) = 0x0ULL;
}

static fpga_result transferHostToFpga(fpga_dma_handle dma_h, uint64_t dst,
                                      uint64_t src, size_t count,
                                      fpga_dma_transfer_t type)
{
	fpga_result res = FPGA_OK;
	uint64_t i = 0;
	uint64_t count_left = count;
	uint64_t align_bytes = 0;
	bool issued_intr = false;

	debug_print("Host To Fpga ----------- src = %08lx, dst = %08lx \n", src, dst);

	// Align to DMA alignment requirements
	if (count <= FPGA_DMA_THRESHOLD) {      // Copy all using ASE
		align_bytes = count_left;
	} else {
		align_bytes = FPGA_DMA_ALIGN_BYTES - (dst % FPGA_DMA_ALIGN_BYTES);
		align_bytes %= (FPGA_DMA_ALIGN_BYTES);
		align_bytes = min(align_bytes, count_left);

	}

	res = fpgaASEHostToFPGA(dma_h->mem_h, src, dst, align_bytes);
	ON_ERR_GOTO(res, out, "HOST_TO_FPGA_MM (fpgaASEHostToFPGA) Transfer failed\n");

	dst += align_bytes;
	src += align_bytes;
	count_left -= align_bytes;

	if (0 == count_left) {
		return res;
	}

	assert(IS_DMA_ALIGNED(dst));

	uint32_t dma_chunks = (count_left + FPGA_DMA_BUF_SIZE - 1) / FPGA_DMA_BUF_SIZE;

	debug_print("DMA TX : dma chunks = %d, count_left = %08lx, dst = %08lx, src = %08lx \n",
	            dma_chunks, count_left, dst, src);

	uint64_t to_copy = min(count_left, FPGA_DMA_BUF_SIZE) & ~(FPGA_DMA_ALIGN_BYTES - 1);

	for (i = 0; to_copy >= FPGA_DMA_THRESHOLD; i++) {
		memcpy_s_fast(dma_h->dma_buf_ptr[i % FPGA_DMA_MAX_BUF], to_copy, (void *)src, to_copy);

		bool send_last = (i == dma_chunks - 1);  // last descriptor
		bool send_int = (send_last || (i % (FPGA_DMA_MAX_BUF / 2) == (FPGA_DMA_MAX_BUF / 2) - 1));

		if (send_int && issued_intr) {  // Going to issue interrupt, but one outstanding
			poll_interrupt(dma_h);
			issued_intr = false;
		}

		res = _do_dma(dma_h, dst, dma_h->dma_buf_iova[i % FPGA_DMA_MAX_BUF] | FPGA_DMA_HOST_MASK, NULL,
		              (char *)dma_h->dma_buf_ptr[i % FPGA_DMA_MAX_BUF], to_copy, send_last, type, send_int);

		issued_intr = send_int || issued_intr;

		assert(to_copy <= count_left);  // Avoid overflow
		count_left -= to_copy;
		src += to_copy;
		dst += to_copy;

		// Ensure to_copy is a multiple of FPGA_DMA_ALIGN_BYTES
		to_copy = (uint64_t)min(count_left, FPGA_DMA_BUF_SIZE) & ~(FPGA_DMA_ALIGN_BYTES - 1);
	}

	// Copy the remaining using ASE
	res = fpgaASEHostToFPGA(dma_h->mem_h, src, dst, count_left);
	ON_ERR_GOTO(res, out, "HOST_TO_FPGA_MM (fpgaASEHostToFPGA) Transfer failed\n");

out:

	if (issued_intr) {
		poll_interrupt(dma_h);
	}

	return res;
}

static fpga_result transferFpgaToHost(fpga_dma_handle dma_h, uint64_t host,
                                      uint64_t device, size_t count,
                                      fpga_dma_transfer_t type)
{
	fpga_result res = FPGA_OK;
	uint64_t i = 0;
	uint64_t j = 0;
	uint64_t count_left = count;
	uint64_t align_bytes = 0;
	int wf_issued = 0;

	debug_print("FPGA To Host ----------- device = %08lx, host = %08lx \n", device, host);

	// Align to DMA alignment requirements
	if (count <= FPGA_DMA_THRESHOLD) {      // Copy all using ASE
		align_bytes = count_left;
	} else {
		align_bytes = FPGA_DMA_ALIGN_BYTES - (device % FPGA_DMA_ALIGN_BYTES);
		align_bytes %= (FPGA_DMA_ALIGN_BYTES);
		align_bytes = min(align_bytes, count_left);

	}

	res = fpgaASEFPGAToHost(dma_h->mem_h, host, device, align_bytes);
	ON_ERR_GOTO(res, out, "HOST_TO_FPGA_MM (fpgaASEHostToFPGA) Transfer failed\n");

	host += align_bytes;
	device += align_bytes;
	count_left -= align_bytes;

	if (0 == count_left) {
		return res;
	}

	assert(IS_DMA_ALIGNED(device));

	uint32_t dma_chunks = (count_left + FPGA_DMA_BUF_SIZE - 1) / FPGA_DMA_BUF_SIZE;

	debug_print("DMA TX : dma chunks = %d, count_left = %08lx, host = %08lx, device = %08lx \n",
	            dma_chunks, count_left, host, device);

	assert(FPGA_DMA_MAX_BUF >= 8);

	uint64_t to_copy = min(count_left, FPGA_DMA_BUF_SIZE) & ~(FPGA_DMA_ALIGN_BYTES - 1);
	uint64_t num_requests_sent = 0;
	uint64_t num_requests_copied = 0;
	uint64_t valid_bytes[FPGA_DMA_MAX_BUF];
	uint64_t buf_index = 0;         // Next buffer from which to copy

	for (i = 0; to_copy >= FPGA_DMA_THRESHOLD; i++) {
		bool isLast = (i == (dma_chunks - 1));
		res = _do_dma(dma_h, dma_h->dma_buf_iova[i % (FPGA_DMA_MAX_BUF)] | FPGA_DMA_HOST_MASK, device,
		              (char *)dma_h->dma_buf_ptr[i % (FPGA_DMA_MAX_BUF)], NULL, to_copy, isLast, type,
		              false /*intr_en */);
		ON_ERR_GOTO(res, out, "FPGA_TO_HOST_MM Transfer failed");

		valid_bytes[i % (FPGA_DMA_MAX_BUF)] = to_copy;  // Size for each buffer

		num_requests_sent++;

		if (isLast || (num_requests_sent % (FPGA_DMA_MAX_BUF / 2) == 0)) {
			if (num_requests_sent != (FPGA_DMA_MAX_BUF / 2)) {
				assert(wf_issued);
				_wait_magic(dma_h);
			}

			res = _issue_magic(dma_h);
			ON_ERR_GOTO(res, out, "Magic number issue failed");
			wf_issued = 1;

			if (num_requests_sent != (FPGA_DMA_MAX_BUF / 2)) {
				for (j = 0; j < (FPGA_DMA_MAX_BUF / 2); j++) {
					// Length in valid_bytes
					uint64_t size = valid_bytes[buf_index];
					memcpy_s_fast((void *)host, size, dma_h->dma_buf_ptr[buf_index], size);
					host += size;
					num_requests_copied++;
					buf_index = (buf_index + 1) % (FPGA_DMA_MAX_BUF);
				}
			}
		}

		device += to_copy;
		count_left -= to_copy;

		// Ensure to_copy is a multiple of FPGA_DMA_ALIGN_BYTES
		to_copy = (uint64_t)min(count_left, FPGA_DMA_BUF_SIZE) & ~(FPGA_DMA_ALIGN_BYTES - 1);
	}

	// At this point, all DMA descriptors have been sent, including the final magic
	// number descriptor.  host is the address to copy to from the DMA buffer.
	// We will first copy any leftover bytes (past the 64-byte chunks) at the end
	// in order to let the DMA work in parallel.
	uint64_t tmp_host = host;

	for (i = 0, j = num_requests_copied; j < num_requests_sent; j++, i++) {
		tmp_host += valid_bytes[buf_index + i];
	}

	// Copy partial remaining bytes (if count not multiple of FPGA_DMA_ALIGN_BYTES)
	res = fpgaASEFPGAToHost(dma_h->mem_h, tmp_host, device, count_left);
	ON_ERR_GOTO(res, out, "HOST_TO_FPGA_MM (fpgaASEHostToFPGA) Transfer failed\n");

	if (wf_issued) {
		_wait_magic(dma_h);
		wf_issued = 0;
	}

	//clear out final dma memcpy operations

	while (num_requests_copied < num_requests_sent) {
		uint64_t size = valid_bytes[buf_index];
		memcpy_s_fast((void *)host, size, dma_h->dma_buf_ptr[buf_index], size);
		host += size;
		num_requests_copied++;
		buf_index = (buf_index + 1) % (FPGA_DMA_MAX_BUF);
	}

out:
	return res;
}

static fpga_result transferFpgaToFpga(fpga_dma_handle dma_h, uint64_t dst,
                                      uint64_t src, size_t count,
                                      fpga_dma_transfer_t type)
{
	fpga_result res = FPGA_OK;
	uint64_t i = 0;
	uint64_t count_left = count;
	uint64_t *tmp_buf = NULL;

	if (IS_DMA_ALIGNED(dst) && IS_DMA_ALIGNED(src) && IS_DMA_ALIGNED(count_left)) {
		uint32_t dma_chunks = count_left / FPGA_DMA_BUF_SIZE;
		count_left -= (dma_chunks * FPGA_DMA_BUF_SIZE);
		debug_print ("!!!FPGA to FPGA!!! TX :dma chunks = %d, count = %08lx, dst = %08lx, src = %08lx \n",
		             dma_chunks, count_left, dst, src);

		for (i = 0; i < dma_chunks; i++) {
			res = _do_dma(dma_h, (dst + i * FPGA_DMA_BUF_SIZE), (src + i * FPGA_DMA_BUF_SIZE), NULL, NULL,
			              FPGA_DMA_BUF_SIZE, 0, type, false /*intr_en */);
			ON_ERR_GOTO(res, out,
			            "FPGA_TO_FPGA_MM Transfer failed");

			if ((i + 1) % FPGA_DMA_MAX_BUF == 0 || i == (dma_chunks - 1) /*last descriptor */) {
				res = _issue_magic(dma_h);
				ON_ERR_GOTO(res, out, "Magic number issue failed");
				_wait_magic(dma_h);
			}
		}

		if (count_left > 0) {
			debug_print("Count_left = %08lx  was transfered using DMA\n", count_left);
			res = _do_dma(dma_h, (dst + dma_chunks * FPGA_DMA_BUF_SIZE), (src + dma_chunks * FPGA_DMA_BUF_SIZE),
			              NULL, NULL, count_left, 1, type, false /*intr_en */);
			ON_ERR_GOTO(res, out, "FPGA_TO_FPGA_MM Transfer failed");
			res = _issue_magic(dma_h);
			ON_ERR_GOTO(res, out, "Magic number issue failed");
			_wait_magic(dma_h);
		}
	} else {
		if ((src < dst) && (src + count_left >= dst)) {
			debug_print("Overlapping addresses, Provide correct dst address\n");
			return FPGA_NOT_SUPPORTED;
		}

		uint32_t tx_chunks = count_left / FPGA_DMA_BUF_ALIGN_SIZE;
		count_left -= (tx_chunks * FPGA_DMA_BUF_ALIGN_SIZE);
		debug_print("!!!FPGA to FPGA TX!!! : tx chunks = %d, count = %08lx, dst = %08lx, src = %08lx \n",
		            tx_chunks, count_left, dst, src);
		tmp_buf = (uint64_t *)malloc(FPGA_DMA_BUF_ALIGN_SIZE);

		for (i = 0; i < tx_chunks; i++) {
			res =
			        transferFpgaToHost(dma_h,
			                           (uint64_t)tmp_buf,
			                           (src + i * FPGA_DMA_BUF_ALIGN_SIZE),
			                           FPGA_DMA_BUF_ALIGN_SIZE,
			                           FPGA_TO_HOST_MM);
			ON_ERR_GOTO(res, out_spl, "FPGA_TO_FPGA_MM Transfer failed");
			res =
			        transferHostToFpga(dma_h,
			                           (dst + i * FPGA_DMA_BUF_ALIGN_SIZE),
			                           (uint64_t)tmp_buf,
			                           FPGA_DMA_BUF_ALIGN_SIZE,
			                           HOST_TO_FPGA_MM);
			ON_ERR_GOTO(res, out_spl, "FPGA_TO_FPGA_MM Transfer failed");
		}

		if (count_left > 0) {
			res =
			        transferFpgaToHost(dma_h,
			                           (uint64_t)tmp_buf,
			                           (src + tx_chunks * FPGA_DMA_BUF_ALIGN_SIZE),
			                           count_left,
			                           FPGA_TO_HOST_MM);
			ON_ERR_GOTO(res, out_spl, "FPGA_TO_FPGA_MM Transfer failed");
			res =
			        transferHostToFpga(dma_h,
			                           (dst + tx_chunks * FPGA_DMA_BUF_ALIGN_SIZE),
			                           (uint64_t)tmp_buf,
			                           count_left,
			                           HOST_TO_FPGA_MM);
			ON_ERR_GOTO(res, out_spl, "FPGA_TO_FPGA_MM Transfer failed");
		}

		free(tmp_buf);
	}

out:
	return res;
out_spl:
	free(tmp_buf);
	return res;
}

// Public APIs
fpga_result fpgaDmaTransferSync(fpga_dma_handle dma_h, uint64_t dst,
                                uint64_t src, size_t count,
                                fpga_dma_transfer_t type)
{

	fpga_result res = FPGA_OK;

	if (!dma_h) {
		return FPGA_INVALID_PARAM;
	}

	if (type >= FPGA_MAX_TRANSFER_TYPE) {
		return FPGA_INVALID_PARAM;
	}

	if (!(type == HOST_TO_FPGA_MM || type == FPGA_TO_HOST_MM  || type == FPGA_TO_FPGA_MM)) {
		return FPGA_NOT_SUPPORTED;
	}

	if (!dma_h->fpga_h) {
		return FPGA_INVALID_PARAM;
	}

	if (type == HOST_TO_FPGA_MM) {
		res =
		        transferHostToFpga(dma_h, dst, src, count, HOST_TO_FPGA_MM);
	} else if (type == FPGA_TO_HOST_MM) {
		res =
		        transferFpgaToHost(dma_h, dst, src, count, FPGA_TO_HOST_MM);
	} else if (type == FPGA_TO_FPGA_MM) {
		res =
		        transferFpgaToFpga(dma_h, dst, src, count, FPGA_TO_FPGA_MM);
	} else {
		return FPGA_NOT_SUPPORTED;
	}

	return res;
}

fpga_result fpgaDmaTransferAsync(fpga_dma_handle dma, uint64_t dst,
                                 uint64_t src, size_t count,
                                 fpga_dma_transfer_t type,
                                 fpga_dma_transfer_cb cb, void *context)
{
        (void)dma;
        (void)dst;
        (void)src;
        (void)count;
        (void)type;
        (void)cb;
        (void)context;
	// TODO
	return FPGA_NOT_SUPPORTED;
}

fpga_result fpgaDmaOpen(fpga_handle fpga, fpga_dma_handle *dma_p)
{
	fpga_result res = FPGA_OK;
	fpga_dma_handle dma_h = NULL;
	int i = 0;

	if (!fpga) {
		return FPGA_INVALID_PARAM;
	}

	if (!dma_p) {
		return FPGA_INVALID_PARAM;
	}

	// init the dma handle
	dma_h = (fpga_dma_handle)malloc(sizeof(struct _dma_handle_t));

	if (!dma_h) {
		return FPGA_NO_MEMORY;
	}

	dma_h->fpga_h = fpga;

	for (i = 0; i < FPGA_DMA_MAX_BUF; i++) {
		dma_h->dma_buf_ptr[i] = NULL;
	}

	dma_h->mmio_num = 0;
	dma_h->mmio_offset = 0;
	dma_h->cur_ase_page = 0xffffffffffffffffUll;
	dma_h->mmio_va = 0ULL;

	// Discover DMA BBB by traversing the device feature list
	bool end_of_list = false;
	bool dma_found = false;

#ifndef USE_ASE
	res = fpgaMapMMIO(dma_h->fpga_h, 0, (uint64_t **)& dma_h->mmio_va);
	ON_ERR_GOTO(res, out, "fpgaMapMMIO");
#endif

	uint64_t offset = dma_h->mmio_offset;
	dfh_feature_t dfh;

	do {
		// Read the next feature header
		res = fpgaMMIORead64Blk(dma_h->fpga_h, dma_h->mmio_num, dma_h->mmio_va, offset, (uint64_t)& dfh,
		                        sizeof(dfh));
		ON_ERR_GOTO(res, out, "fpgaMMIORead64Blk");

		if (_fpga_dma_feature_is_bbb(dfh.dfh) &&
		                (dfh.feature_uuid_lo == FPGA_DMA_UUID_L) &&
		                (dfh.feature_uuid_hi == FPGA_DMA_UUID_H)
		   ) {
			// Found one. Record it.
			dma_h->dma_base = offset;
			dma_h->dma_csr_base = dma_h->dma_base + FPGA_DMA_CSR;
			dma_h->dma_desc_base = dma_h->dma_base + FPGA_DMA_DESC;
			dma_h->dma_ase_cntl_base =
			        dma_h->dma_base + FPGA_DMA_ADDR_SPAN_EXT_CNTL;
			dma_h->dma_ase_data_base =
			        dma_h->dma_base + FPGA_DMA_ADDR_SPAN_EXT_DATA;
			dma_found = true;
			break;
		}

		// End of the list?
		end_of_list = _fpga_dma_feature_eol(dfh.dfh);

		// Move to the next feature header
		offset = offset + _fpga_dma_feature_next(dfh.dfh);
	} while (!end_of_list);

	if (dma_found) {
		*dma_p = dma_h;
		res = FPGA_OK;
	} else {
		*dma_p = NULL;
		res = FPGA_NOT_FOUND;
		goto out;
	}

	res = fpgaASEOpen(dma_h->fpga_h, dma_h->mmio_num, dma_h->dma_base, &dma_h->mem_h);
	ON_ERR_GOTO(res, out, "fpgaASEOpen");

	// Buffer size must be page aligned for prepareBuffer
	for (i = 0; i < FPGA_DMA_MAX_BUF; i++) {
		res =
		        fpgaPrepareBuffer(dma_h->fpga_h, FPGA_DMA_BUF_SIZE,
		                          (void **) & (dma_h->dma_buf_ptr[i]),
		                          &dma_h->dma_buf_wsid[i], 0);
		ON_ERR_GOTO(res, out, "fpgaPrepareBuffer");

		// Make sure it's actually allocated (paged in, not zero-page)
		dma_h->dma_buf_ptr[i][0] = 0xff;                // Trigger page-in
		madvise((void *)dma_h->dma_buf_ptr[i], FPGA_DMA_BUF_SIZE, MADV_SEQUENTIAL);


		res =
		        fpgaGetIOAddress(dma_h->fpga_h, dma_h->dma_buf_wsid[i],
		                         &dma_h->dma_buf_iova[i]);
		ON_ERR_GOTO(res, rel_buf, "fpgaGetIOAddress");
	}

	// Allocate magic number buffer
	res =
	        fpgaPrepareBuffer(dma_h->fpga_h, FPGA_DMA_ALIGN_BYTES,
	                          (void **) & (dma_h->magic_buf), &dma_h->magic_wsid,
	                          0);
	ON_ERR_GOTO(res, out, "fpgaPrepareBuffer");

	dma_h->magic_buf[0] = 0xff;             // Trigger page-in

	res =
	        fpgaGetIOAddress(dma_h->fpga_h, dma_h->magic_wsid,
	                         &dma_h->magic_iova);
	ON_ERR_GOTO(res, rel_buf, "fpgaGetIOAddress");
	memset((void *)dma_h->magic_buf, 0, FPGA_DMA_ALIGN_BYTES);

	// turn on global interrupts
	msgdma_ctrl_t ctrl = { 0 };
	ctrl.ct.global_intr_en_mask = 1;
	res = fpgaMMIOWrite32Blk(dma_h->fpga_h, dma_h->mmio_num, dma_h->mmio_va, CSR_CONTROL(dma_h),
	                         (uint64_t)& ctrl.reg, sizeof(ctrl.reg));
	ON_ERR_GOTO(res, rel_buf, "fpgaMMIOWrite32Blk");

	// register interrupt event handle
	res = fpgaCreateEventHandle(&dma_h->eh);
	ON_ERR_GOTO(res, rel_buf, "fpgaCreateEventHandle");

	res =
	        fpgaRegisterEvent(dma_h->fpga_h, FPGA_EVENT_INTERRUPT, dma_h->eh,
	                          0 /*vector id */);
	ON_ERR_GOTO(res, destroy_eh, "fpgaRegisterEvent");

	return FPGA_OK;

destroy_eh:
	res = fpgaDestroyEventHandle(&dma_h->eh);
	ON_ERR_GOTO(res, rel_buf, "fpgaDestroyEventHandle");

rel_buf:

	for (i = 0; i < FPGA_DMA_MAX_BUF; i++) {
		res = fpgaReleaseBuffer(dma_h->fpga_h, dma_h->dma_buf_wsid[i]);
		ON_ERR_GOTO(res, out, "fpgaReleaseBuffer");
	}

out:

	if (!dma_found) {
		free(dma_h);
	}

	return res;
}

fpga_result fpgaDmaClose(fpga_dma_handle dma_h)
{
	fpga_result res = FPGA_OK;
	int i = 0;

	if (!dma_h) {
		res = FPGA_INVALID_PARAM;
		goto out;
	}

	if (!dma_h->fpga_h) {
		res = FPGA_INVALID_PARAM;
		goto out;
	}

	for (i = 0; i < FPGA_DMA_MAX_BUF; i++) {
		res = fpgaReleaseBuffer(dma_h->fpga_h, dma_h->dma_buf_wsid[i]);
		ON_ERR_GOTO(res, out, "fpgaReleaseBuffer failed");
	}

	res = fpgaReleaseBuffer(dma_h->fpga_h, dma_h->magic_wsid);
	ON_ERR_GOTO(res, out, "fpgaReleaseBuffer");

	fpgaUnregisterEvent(dma_h->fpga_h, FPGA_EVENT_INTERRUPT, dma_h->eh);
	fpgaDestroyEventHandle(&dma_h->eh);

	// turn off global interrupts
	msgdma_ctrl_t ctrl = { 0 };
	ctrl.ct.global_intr_en_mask = 0;
	res = fpgaMMIOWrite32Blk(dma_h->fpga_h, dma_h->mmio_num, dma_h->mmio_va, CSR_CONTROL(dma_h),
	                         (uint64_t)& ctrl.reg, sizeof(ctrl.reg));
	ON_ERR_GOTO(res, out, "fpgaMMIOWrite32Blk");

	res = fpgaASEClose(&dma_h->mem_h);
	ON_ERR_GOTO(res, out, "fpgaASEClose");

out:
	free((void *)dma_h);
	return res;
}
