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

#ifndef __MMIO_BLOCK_H__
#define __MMIO_BLOCK_H__

#include <opae/fpga.h>

#ifdef __cplusplus
extern "C" {
#endif


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
                               uint64_t device, uint64_t host, uint64_t bytes);

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
                               uint64_t device, uint64_t host, uint64_t bytes);

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
                              uint64_t device, uint64_t host, uint64_t bytes);

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
                              uint64_t device, uint64_t host, uint64_t bytes);


#ifdef __cplusplus
}
#endif

#endif  // __MMIO_BLOCK_H__
