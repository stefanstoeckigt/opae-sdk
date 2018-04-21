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

#ifndef __MMIO_BLOCK_INTERNAL_H__
#define __MMIO_BLOCK_INTERNAL_H__

#include <opae/fpga.h>
#include "ase_memory.h"

#ifdef _WIN32
	#define __attribute__(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef MMIO_BLOCK_DEBUG
#pragma message "Compiled with -DMMIO_BLOCK_DEBUG.  Not to be used in production"
#endif

// Convenience macros
#ifdef MMIO_BLOCK_DEBUG
#define debug_print(fmt, ...) \
	do { \
		if (MMIO_BLOCK_DEBUG) {\
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

#define QWORD_BYTES ((uint64_t)8)
#define DWORD_BYTES ((uint64_t)4)
#define IS_ALIGNED_DWORD(addr) ((uint64_t)(addr) % DWORD_BYTES == 0)
#define IS_ALIGNED_QWORD(addr) ((uint64_t)(addr) % QWORD_BYTES == 0)

#define HOST_MMIO_32_ADDR(offset) ((volatile uint32_t *)((uint64_t)mmio_va + (uint64_t)(offset)))
#define HOST_MMIO_64_ADDR(offset) ((volatile uint64_t *)((uint64_t)mmio_va + (uint64_t)(offset)))
#define HOST_MMIO_32(offset) (*HOST_MMIO_32_ADDR(offset))
#define HOST_MMIO_64(offset) (*HOST_MMIO_64_ADDR(offset))

#ifdef __cplusplus
}
#endif

#endif  // __MMIO_BLOCK_INTERNAL_H__
