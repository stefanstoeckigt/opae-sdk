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
#include <assert.h>
#include "mmio_block_internal.h"

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

/**
* fpgaMMIOWrite64Blk
*
* @brief                Writes a block of 64-bit values to FPGA MMIO space.
*                       If mmio_va is NULL, use OPAE call to access MMIO space.
* @param[in] fpga_h     FPGA handle
* @param[in] mmio_num   FPGA MMIO space index
* @param[in] mmio_va    Host-relative virtual address of MMIO space
* @param[in] device     FPGA address
* @param[in] host       Host buffer address
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
fpga_result fpgaMMIOWrite64Blk(fpga_handle fpga_h, uint32_t mmio_num, void *mmio_va,
                               uint64_t device, uint64_t host, uint64_t bytes)
{
	assert(IS_ALIGNED_QWORD(device));
	assert(IS_ALIGNED_QWORD(bytes));

	uint64_t *haddr = (uint64_t *)host;
	uint64_t i;
	fpga_result res = FPGA_OK;
	volatile uint64_t *dev_addr = HOST_MMIO_64_ADDR(device);

	debug_print("copying %lld bytes from 0x%p to 0x%p\n",
	            (long long int)bytes, haddr, (void *)device);

	for (i = 0; i < bytes / sizeof(uint64_t); i++) {
		if (NULL == mmio_va) {
			res = fpgaWriteMMIO64(fpga_h, mmio_num, device, *haddr);
			ON_ERR_RETURN(res, "fpgaWriteMMIO64");
			haddr++;
			device += sizeof(uint64_t);
		} else {
			*dev_addr++ = *haddr++;
		}
	}

	return res;
}

/**
* fpgaMMIOWrite32Blk
*
* @brief                Writes a block of 32-bit values to FPGA MMIO space
*                       If mmio_va is NULL, use OPAE call to access MMIO space.
* @param[in] fpga_h     FPGA handle
* @param[in] mmio_num   FPGA MMIO space index
* @param[in] mmio_va    Host-relative virtual address of MMIO space
* @param[in] device     FPGA address
* @param[in] host       Host buffer address
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
fpga_result fpgaMMIOWrite32Blk(fpga_handle fpga_h, uint32_t mmio_num, void *mmio_va,
                               uint64_t device, uint64_t host, uint64_t bytes)
{
	assert(IS_ALIGNED_DWORD(device));
	assert(IS_ALIGNED_DWORD(bytes));

	uint32_t *haddr = (uint32_t *)host;
	uint64_t i;
	fpga_result res = FPGA_OK;
	volatile uint32_t *dev_addr = HOST_MMIO_32_ADDR(device);

	debug_print("copying %lld bytes from 0x%p to 0x%p\n",
	            (long long int)bytes, haddr, (void *)device);

	for (i = 0; i < bytes / sizeof(uint32_t); i++) {
		if (NULL == mmio_va) {
			res = fpgaWriteMMIO32(fpga_h, mmio_num, device, *haddr);
			ON_ERR_RETURN(res, "fpgaWriteMMIO32");
			haddr++;
			device += sizeof(uint32_t);
		} else {
			*dev_addr++ = *haddr++;
		}
	}

	return res;
}

/**
* fpgaMMIORead64Blk
*
* @brief                Reads a block of 64-bit values from FPGA MMIO space
*                       If mmio_va is NULL, use OPAE call to access MMIO space.
* @param[in] fpga_h     FPGA handle
* @param[in] mmio_num   FPGA MMIO space index
* @param[in] mmio_va    Host-relative virtual address of MMIO space
* @param[in] device     FPGA address
* @param[in] host       Host buffer address
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
fpga_result fpgaMMIORead64Blk(fpga_handle fpga_h, uint32_t mmio_num, void *mmio_va,
                              uint64_t device, uint64_t host, uint64_t bytes)
{
	assert(IS_ALIGNED_QWORD(device));
	assert(IS_ALIGNED_QWORD(bytes));

	uint64_t *haddr = (uint64_t *)host;
	uint64_t i;
	fpga_result res = FPGA_OK;
	volatile uint64_t *dev_addr = HOST_MMIO_64_ADDR(device);

	debug_print("copying %lld bytes from 0x%p to 0x%p\n",
	            (long long int)bytes, (void *)device, haddr);

	for (i = 0; i < bytes / sizeof(uint64_t); i++) {
		if (NULL == mmio_va) {
			res = fpgaReadMMIO64(fpga_h, mmio_num, device, haddr);
			ON_ERR_RETURN(res, "fpgaReadMMIO64");
			haddr++;
			device += sizeof(uint64_t);
		} else {
			*haddr++ = *dev_addr++;
		}
	}

	return res;
}

/**
* fpgaMMIORead32Blk
*
* @brief                Reads a block of 32-bit values from FPGA MMIO space
*                       If mmio_va is NULL, use OPAE call to access MMIO space.
* @param[in] fpga_h     FPGA handle
* @param[in] mmio_num   FPGA MMIO space index
* @param[in] mmio_va    Host-relative virtual address of MMIO space
* @param[in] device     FPGA address
* @param[in] host       Host buffer address
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
fpga_result fpgaMMIORead32Blk(fpga_handle fpga_h, uint32_t mmio_num, void *mmio_va,
                              uint64_t device, uint64_t host, uint64_t bytes)
{
	assert(IS_ALIGNED_DWORD(device));
	assert(IS_ALIGNED_DWORD(bytes));

	uint32_t *haddr = (uint32_t *)host;
	uint64_t i;
	fpga_result res = FPGA_OK;
	volatile uint32_t *dev_addr = HOST_MMIO_32_ADDR(device);

	debug_print("copying %lld bytes from 0x%p to 0x%p\n",
	            (long long int)bytes, (void *)device, haddr);

	for (i = 0; i < bytes / sizeof(uint32_t); i++) {
		if (NULL == mmio_va) {
			res = fpgaReadMMIO32(fpga_h, mmio_num, device, haddr);
			ON_ERR_RETURN(res, "fpgaReadMMIO32");
			haddr++;
			device += sizeof(uint32_t);
		} else {
			*haddr++ = *dev_addr++;
		}
	}

	return res;
}
