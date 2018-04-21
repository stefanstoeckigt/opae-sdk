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
// ARE DISCLAIMED.  IN NO EVENT  SHALL THE COPYRIGHT OWNER  OR CONTRIBUTORS BE
// LIABLE  FOR  ANY  DIRECT,  INDIRECT,  INCIDENTAL,  SPECIAL,  EXEMPLARY,  OR
// CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT LIMITED  TO,  PROCUREMENT  OF
// SUBSTITUTE GOODS OR SERVICES;  LOSS OF USE,  DATA, OR PROFITS;  OR BUSINESS
// INTERRUPTION)  HOWEVER CAUSED  AND ON ANY THEORY  OF LIABILITY,  WHETHER IN
// CONTRACT,  STRICT LIABILITY,  OR TORT  (INCLUDING NEGLIGENCE  OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,  EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * \fpga_dma_internal.h
 * \brief FPGA DMA BBB Internal Header
 */

#ifndef __ASE_MEMORY_INT_H__
#define __ASE_MEMORY_INT_H__

#include <opae/fpga.h>
#include "mmio_block.h"
#include "ase_memory.h"

#ifdef _WIN32
	#define __attribute__(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ASE_MEMORY_DEBUG
#pragma message "Compiled with -DASE_MEMORY_DEBUG.  Not to be used in production"
#endif

// Convenience macros
#ifdef ASE_MEMORY_DEBUG
#define debug_print(fmt, ...) \
	do { \
		if (ASE_MEMORY_DEBUG) {\
			fprintf(stderr, "%s (%d) : ", __FUNCTION__, __LINE__); \
			fprintf(stderr, fmt, ##__VA_ARGS__); \
		} \
	} while (0)
#define error_print(fmt, ...) \
	do { \
		fprintf(stderr, "%s (%d) : ", __FUNCTION__, __LINE__); \
		fprintf(stderr, fmt, ##__VA_ARGS__); \
		err_cnt++; \
	} while (0)
#else
#define debug_print(...)
#define error_print(...)
#endif

#ifndef max
#define max(a,b) \
	({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a > _b ? _a : _b; })
#endif

#ifndef min
#define min(a,b) \
	({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a < _b ? _a : _b; })
#endif

#define QWORD_BYTES ((uint64_t)8)
#define DWORD_BYTES ((uint64_t)4)
#define IS_ALIGNED_DWORD(addr) ((uint64_t)(addr) % DWORD_BYTES == 0)
#define IS_ALIGNED_QWORD(addr) ((uint64_t)(addr) % QWORD_BYTES == 0)

#define FPGA_MEM_MAGIC 0x53996500

// ASE Register offsets from base
#define FPGA_ASE_ADDR_SPAN_EXT_CNTL (0x200)
#define FPGA_ASE_ADDR_SPAN_EXT_DATA (0x1000)

#define DMA_ADDR_SPAN_EXT_WINDOW ((uint64_t)4*1024)
#define DMA_ADDR_SPAN_EXT_WINDOW_MASK ((uint64_t)(DMA_ADDR_SPAN_EXT_WINDOW - 1))
#define ASE_PAGE(x) ((x) & ~((uint64_t)DMA_ADDR_SPAN_EXT_WINDOW_MASK))
#define ASE_OFFSET(x) ((x) & ((uint64_t)DMA_ADDR_SPAN_EXT_WINDOW_MASK))
#define ASE_LEFT_IN_PAGE(x) ((uint64_t)DMA_ADDR_SPAN_EXT_WINDOW - ASE_OFFSET(x))

#define ASE_DATA_BASE(mem_handle) ((uint64_t)mem_handle->ase_data_base)
#define ASE_CNTL_BASE(mem_handle) ((uint64_t)mem_handle->ase_cntl_base)
#define HOST_MMIO_32_ADDR(mem_handle,offset) ((volatile uint32_t *)((uint64_t)(mem_handle)->mmio_va + (uint64_t)(offset)))
#define HOST_MMIO_64_ADDR(mem_handle,offset) ((volatile uint64_t *)((uint64_t)(mem_handle)->mmio_va + (uint64_t)(offset)))
#define HOST_MMIO_32(mem_handle,offset) (*HOST_MMIO_32_ADDR(mem_handle,offset))
#define HOST_MMIO_64(mem_handle,offset) (*HOST_MMIO_64_ADDR(mem_handle,offset))

typedef struct _ase_mem_handle_t {
	fpga_handle fpga_h;
	uint32_t mem_magic;
	uint32_t mmio_num;
	void *mmio_va;
	uint64_t cur_ase_page;
	uint64_t ase_cntl_base;
	uint64_t ase_data_base;
} *_memory_handle;

// Pointer to fpgaMMIO{Read,Write}{32,64}Blk
typedef fpga_result(*pXferBlk)(fpga_handle fpga_h, uint32_t mmio_num, void *mmio_va,
                               uint64_t device, uint64_t host, uint64_t bytes);

// Pointer to fpgaMMIO{Read,Write}MemoryPartial
typedef fpga_result(*pPartial)( _memory_handle mem_h, uint64_t *host_ptr,
                                uint64_t *device_ptr, uint64_t *count);

// Pointer to Read or Write fixup routine for non-64-bit sizes
typedef fpga_result(*pPartialFixup)(_memory_handle mem_h, uint64_t *host_ptr,
                                    uint64_t *device_ptr, uint64_t *count);

// Function prototypes
static inline void _switch_to_ase_page(_memory_handle mem_h, uint64_t addr);
static inline fpga_result Read64Partial(_memory_handle mem_h, uint64_t device,
                                        uint64_t *val, uint64_t *offset);
static inline fpga_result ReadFixup(_memory_handle mem_h, uint64_t *device, uint64_t *host,
                                    uint64_t *count);
static inline fpga_result WriteFixup(_memory_handle mem_h, uint64_t *device, uint64_t *host,
                                     uint64_t *count);
static fpga_result fpgaASETransferMemoryPartial(_memory_handle mem_h, uint64_t *host_ptr,
                                                uint64_t *device_ptr, uint64_t *count_read);
static fpga_result fpgaASEMemoryCopy(_memory_handle mem_h, uint64_t host_addr,
                                     uint64_t device_addr, uint64_t count);
static fpga_result validateHandle(ase_mem_handle mem_h);


#ifdef __cplusplus
}
#endif

#endif  // __ASE_MEMORY_INT_H__
