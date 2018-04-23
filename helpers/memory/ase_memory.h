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

#ifndef __ASE_MEMORY_H__
#define __ASE_MEMORY_H__

#include <opae/fpga.h>
#include "mmio_block.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef void *ase_mem_handle;

/**
* fpgaASEOpen
*
* @brief                 Open a handle to ASE for DMA BBB.
*
* @param[in]  fpga       Handle to the FPGA AFU object obtained via fpgaOpen()
* @param[in]  mmio_num   The MMIO number associated with the DMA BBB
* @param[in]  dma_base   The base offset of the DMA BBB
* @param[out] mem_h      Memory object handle
* @returns               FPGA_OK on success, return code otherwise
*/
fpga_result fpgaASEOpen(fpga_handle fpga_h, uint32_t mmio_num, uint64_t dma_base,
                        ase_mem_handle *mem_h);

/**
* fpgaASEClose
*
* @brief             Close the memory object handle.
*
* @param[out] mem_h  Memory object handle
* @returns           FPGA_OK on success, return code otherwise
*/
fpga_result fpgaASEClose(ase_mem_handle *mem_h);

/**
* fpgaASEHostToFPGA
*
* @brief                Perform a blocking copy of 'count' bytes from memory area pointed
*                       to by host to memory area pointed to by device using ASE.
* @param[in] mem_h      Memory object handle
* @param[in] host       Address of the source buffer
* @param[in] device     Address of the destination buffer
* @param[in] count      Size in bytes
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
fpga_result fpgaASEHostToFPGA(ase_mem_handle mem_h, uint64_t host,
                              uint64_t device, size_t count);

/**
* fpgaASEHostToFPGA
*
* @brief                Perform a blocking copy of 'count' bytes from memory area pointed
*                       to by device to memory area pointed to by host using ASE.
* @param[in] mem_h      Memory object handle
* @param[in] host       Address of the source buffer
* @param[in] device     Address of the destination buffer
* @param[in] count      Size in bytes
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
fpga_result fpgaASEFPGAToHost(ase_mem_handle mem_h, uint64_t host,
                              uint64_t device, size_t count);


#ifdef __cplusplus
}
#endif

#endif  // __ASE_MEMORY_H__
