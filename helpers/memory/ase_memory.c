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
#include <errno.h>
#ifndef _WIN32
	#include <poll.h>
	#include <unistd.h>
	#include <sys/mman.h>
#endif
#include <assert.h>
#include "memcpy_s_fast.h"
#include "ase_memory.h"
#include "ase_memory_internal.h"

#ifdef SIM
	#define USE_ASE
#else
	// TODO:  Need this until we can adequately sync MMIO R/W with pointer accesses.
	// Causes module to use fpgaMMIORead32() instead of foo = *ptr;
	#define USE_ASE
#endif

#ifdef ASE_MEMORY_DEBUG
	static int err_cnt = 0;
#endif

/*
* macro for checking return codes
*/
#define ON_ERR_RETURN(res, desc)\
	do {\
		if ((res) != FPGA_OK) {\
			error_print("Error %s: %s\n", (desc), fpgaErrStr(res));\
			return(res);\
		}\
	} while (0)

// Function pointers

static pXferBlk BlockTransfer = NULL;
static pPartial PartialTransfer = NULL;
static pPartialFixup PartialFixupTransfer = NULL;

// Internal Functions

/**
* _switch_to_ase_page
*
* @brief                Updates the current page of ASE to the address given
* @param[in] mem_h      Pointer to the memory handle structure
* @param[in] addr       Address to which the ASE page should be switched
* @return Nothing.  Side-effect is to update the current page in the memory handle.
*
*/
static inline void _switch_to_ase_page(_memory_handle mem_h, uint64_t addr)
{
	uint64_t requested_page = ASE_PAGE(addr);

	if (requested_page != mem_h->cur_ase_page) {
		fpgaMMIOWrite64Blk(mem_h->fpga_h, mem_h->mmio_num, mem_h->mmio_va, ASE_CNTL_BASE(mem_h),
		                   (uint64_t)& requested_page,
		                   sizeof(requested_page));
		mem_h->cur_ase_page = requested_page;
	}
}

/**
* Read64Partial
*
* @brief                Read a 64-bit from device
* @param[in] mem_h      Pointer to the memory handle structure
* @param[in] device     Address to which the ASE page should be switched
* @param[out] val       Address to which the 64-bit value is to be stored
* @param[out] offset    Address to which the byte offset is to be stored
* @return fpga_result FPGA_OK on success, return code otherwise. Side-effect
* is to update the current page in the memory handle.
*
*/
static inline fpga_result Read64Partial(_memory_handle mem_h, uint64_t device,
                                        uint64_t *val, uint64_t *offset)
{
	fpga_result res = FPGA_OK;

	uint64_t device_addr = device & ~((uint64_t)QWORD_BYTES - 1);  // Round down to 64-bit boundary
	_switch_to_ase_page(mem_h, device_addr);
	*offset = ASE_OFFSET(device_addr);

	res = fpgaMMIORead64Blk(mem_h->fpga_h, mem_h->mmio_num, mem_h->mmio_va,
	                        ASE_DATA_BASE(mem_h) + (*offset & ~(QWORD_BYTES - 1)), (uint64_t)val,
	                        QWORD_BYTES);
	ON_ERR_RETURN(res, "fpgaMMIORead64Blk");

	return res;
}

/**
* ReadFixup
*
* @brief                   Copy partial bytes from 64-bit aligned  data to host
* @param[in] mem_h         Pointer to the memory handle structure
* @param[in] host          Address to which the data is to be copied (updated after copy)
* @param[in/out] device    Address from which the 64-bit value is to be read (updated)
* @param[in/out] count     Address to which the number of bytes copied is to be stored
* @return fpga_result FPGA_OK on success, return code otherwise. Side-effect is to
* update the current page in the memory handle.
*
*/
static inline fpga_result ReadFixup(_memory_handle mem_h, uint64_t *host, uint64_t *device,
                                    uint64_t *count)
{
	fpga_result res = FPGA_OK;
	uint64_t tmp;
	uint64_t offset;
	uint64_t copy_size = *count;

	assert(copy_size < QWORD_BYTES);

	res = Read64Partial(mem_h, *device, &tmp, &offset);
	ON_ERR_RETURN(res, "Read64Partial");

	memcpy_s_fast((void *)*host, copy_size, ((char *)(&tmp)) + (*device % QWORD_BYTES), copy_size);

	*count = copy_size;
	*device += copy_size;
	*host += copy_size;

	return res;
}

/**
* WriteFixup
*
* @brief                   Copy partial bytes from 64-bit aligned  data to FPGA.  R/M/W.
* @param[in] mem_h         Pointer to the memory handle structure
* @param[in] host          Address from which the data is to be copied (updated after copy)
* @param[in/out] device    Address to which the 64-bit value is to be written (updated)
* @param[in/out] count     Address to which the number of bytes copied is to be stored
* @return fpga_result FPGA_OK on success, return code otherwise. Side-effect is to
* update the current page in the memory handle.
*
*/
static inline fpga_result WriteFixup(_memory_handle mem_h, uint64_t *host, uint64_t *device,
                                     uint64_t *count)
{
	fpga_result res = FPGA_OK;
	uint64_t tmp;
	uint64_t offset;
	uint64_t copy_size = *count;

	assert(copy_size < QWORD_BYTES);

	res = Read64Partial(mem_h, *device, &tmp, &offset);
	ON_ERR_RETURN(res, "Read64Partial");

	//overlay our data
	memcpy_s_fast(((char *)(&tmp)) + (*device % QWORD_BYTES), copy_size, (void *)*host, copy_size);

	//write back to device
	res = fpgaMMIOWrite64Blk(mem_h->fpga_h, mem_h->mmio_num, mem_h->mmio_va,
	                         ASE_DATA_BASE(mem_h) + (offset & ~(QWORD_BYTES - 1)),
	                         (uint64_t)&tmp, QWORD_BYTES);
	ON_ERR_RETURN(res, "fpgaMMIOWrite64Blk");

	*count = copy_size;
	*device += copy_size;
	*host += copy_size;

	return res;
}

/**
* fpgaASETransferMemoryPartial
*
* @brief                      Reads memory from FPGA via MMIO utilizing Address Span Extender (ASE)
*                             Either pointer will be QWORD aligned and size will be < QWORD_BYTES,
*                             OR pointer will be aligned, with no restriction on count.
* @param[in] mem_h            Pointer to the memory handle structure
* @param[in/out] host_ptr     Pointer to Host Buffer Address - updated on return
* @param[in/out] device_ptr   Pointer to FPGA address - updated on return
* @param[in/out] count_read   Pointer to Size in bytes actually read
* @return fpga_result         FPGA_OK on success, return code otherwise.
*
*/
static fpga_result fpgaASETransferMemoryPartial(_memory_handle mem_h, uint64_t *host_ptr,
                                                uint64_t *device_ptr, uint64_t *count_read)
{
	fpga_result res = FPGA_OK;
	uint64_t device = *device_ptr;
	uint64_t count = *count_read;
	uint64_t unaligned_size = min(count, QWORD_BYTES - (device % QWORD_BYTES));

	if (0 == count) {
		return res;
	}

	// Read count bytes, storing into dst.
	if (IS_ALIGNED_QWORD(device)) {
		if (count >= QWORD_BYTES) {     // Nothing to do
			*count_read = 0;
			return res;
		}

		unaligned_size = count;
	}

	*count_read = unaligned_size;

	uint64_t copy_size = 0;

	while (unaligned_size > 0) {
		copy_size = unaligned_size;

		// If data spans ASE pages, read the first part here
		if (unaligned_size > ASE_LEFT_IN_PAGE(device)) {
			copy_size -= ASE_LEFT_IN_PAGE(device);
			assert(copy_size < QWORD_BYTES);
		}

		PartialFixupTransfer(mem_h, host_ptr, device_ptr, &copy_size);

		unaligned_size -= copy_size;
	}

	return res;
}

/**
* fpgaASEMemoryCopy
*
* @brief                   Writes/reads memory to/from FPGA via MMIO utilizing
*                          Address Span Extender (ASE)
* @param[in] mem_h         Pointer to the memory handle structure
* @param[in] host_addr     Host Buffer Address
* @param[in] device_addr   FPGA address
* @param[in] count         Size in bytes
* @return fpga_result      FPGA_OK on success, return code otherwise.
*
*/
static fpga_result fpgaASEMemoryCopy(_memory_handle mem_h, uint64_t host_addr,
                                     uint64_t device_addr, uint64_t count)
{
	fpga_result res = FPGA_OK;
	uint64_t host = host_addr;
	uint64_t device = device_addr;
	uint64_t align_bytes = count;
	uint64_t offset = 0;
	uint64_t unaligned_size = count;

	if (0 == count) {
		return res;
	}

	// Align dst to a 64-bit boundary, copy 64-bit chunks, then copy remainder
	res = PartialTransfer(mem_h, &host, &device, &unaligned_size);
	ON_ERR_RETURN(res, "PartialTransfer");

	assert(unaligned_size < QWORD_BYTES);

	align_bytes -= unaligned_size;

	if (0 == align_bytes) {
		return res;
	}

	assert(IS_ALIGNED_QWORD(device));

	// Write blocks of 64-bit values
	while (align_bytes >= QWORD_BYTES) {
		uint64_t left_in_page = ASE_LEFT_IN_PAGE(device);
		uint64_t size_to_copy =
		        min(left_in_page, (align_bytes & ~(QWORD_BYTES - 1)));

		if (size_to_copy < QWORD_BYTES) {
			break;
		}

		_switch_to_ase_page(mem_h, device);
		offset = ASE_OFFSET(device);
		res = BlockTransfer(mem_h->fpga_h, mem_h->mmio_num, mem_h->mmio_va, ASE_DATA_BASE(mem_h) + offset,
		                    (uint64_t)host, size_to_copy);
		ON_ERR_RETURN(res, "BlockTransfer");
		host += size_to_copy;
		device += size_to_copy;
		align_bytes -= size_to_copy;
	}

	unaligned_size = align_bytes;
	res = PartialTransfer(mem_h, &host, &device, &unaligned_size);
	ON_ERR_RETURN(res, "PartialTransfer");

	align_bytes -= unaligned_size;
	assert(align_bytes == 0);

	return res;
}

/**
* validateHandle
*
* @brief                   Validates an ASE memory handle.  TODO: Add locking if appropriate
* @param[in] mem_h         Pointer to the memory handle structure
* @return fpga_result      FPGA_OK on success, FPGA_INVALID_PARAM otherwise.
*
*/
static fpga_result validateHandle(ase_mem_handle mem_h)
{
	if (!mem_h) {
		return FPGA_INVALID_PARAM;
	}

	_memory_handle mem_p = (_memory_handle)mem_h;

	if (mem_p->fpga_h == NULL) {
		return FPGA_INVALID_PARAM;
	}

	if (mem_p->mem_magic != FPGA_MEM_MAGIC) {
		return FPGA_INVALID_PARAM;
	}

	return FPGA_OK;
}



// Public APIs

fpga_result fpgaASEHostToFPGA(ase_mem_handle mem_h, uint64_t host,
                              uint64_t device, size_t count)
{
	fpga_result res = validateHandle(mem_h);

	if (FPGA_OK != res) {
		return res;
	}

	if (0 == count) {
		return res;
	}

	BlockTransfer = fpgaMMIOWrite64Blk;
	PartialTransfer = fpgaASETransferMemoryPartial;
	PartialFixupTransfer = WriteFixup;

	return fpgaASEMemoryCopy((_memory_handle)mem_h, host, device, count);
}


fpga_result fpgaASEFPGAToHost(ase_mem_handle mem_h, uint64_t host,
                              uint64_t device, size_t count)
{
	fpga_result res = validateHandle(mem_h);

	if (FPGA_OK != res) {
		return res;
	}

	if (0 == count) {
		return res;
	}

	BlockTransfer = fpgaMMIORead64Blk;
	PartialTransfer = fpgaASETransferMemoryPartial;
	PartialFixupTransfer = ReadFixup;

	return fpgaASEMemoryCopy((_memory_handle)mem_h, host, device, count);
}

fpga_result fpgaASEOpen(fpga_handle fpga_h_, uint32_t mmio_num_, uint64_t dma_base_,
                        ase_mem_handle *mem_h_)
{
	fpga_result res = FPGA_OK;

	if ((!fpga_h_) || (!mem_h_)) {
		return FPGA_INVALID_PARAM;
	}

	_memory_handle handle = (_memory_handle)calloc(1, sizeof(struct _ase_mem_handle_t));

	if (!handle) {
		return FPGA_NO_MEMORY;
	}

	handle->fpga_h = fpga_h_;
	handle->mmio_num = mmio_num_;
	handle->cur_ase_page = 0xffffffffffffffffULL;
	handle->ase_cntl_base = dma_base_ + FPGA_ASE_ADDR_SPAN_EXT_CNTL;
	handle->ase_data_base = dma_base_ + FPGA_ASE_ADDR_SPAN_EXT_DATA;

#ifndef USE_ASE
	// Simulator, *not* Address Span Extender
	res = fpgaMapMMIO(fpga_h_, mmio_num_, (uint64_t **)& handle->mmio_va);

	if (FPGA_OK != res) {
		free(handle);
		*mem_h_ = NULL;
		return res;
	}

#else
	handle->mmio_va = 0ULL;         // Trigger for block writes to use OPAE calls
#endif

	handle->mem_magic = FPGA_MEM_MAGIC;
	*mem_h_ = (ase_mem_handle)handle;

	return res;
}

fpga_result fpgaASEClose(ase_mem_handle *mem_h)
{
	fpga_result res = validateHandle(mem_h);

	if (FPGA_OK != res) {
		return res;
	}

	free((void *)*mem_h);
	*mem_h = NULL;

	return res;
}

