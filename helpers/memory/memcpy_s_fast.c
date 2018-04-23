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

#pragma push_macro("_GNU_SOURCE")
#undef _GNU_SOURCE
#define _GNU_SOURCE

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <safe_string/safe_string.h>
#include "memcpy_s_fast.h"
#include "x86-sse2.h"

#pragma pop_macro("_GNU_SOURCE")

static void *memcpy_setup(void *dst, size_t max, const void *src, size_t n);

memcpy_fn_t memcpy_s_fast = memcpy_setup;	// Initial value points to setup routine


/**
* SSE2_memcpy
*
* @brief                memcpy using SSE2 or REP MOVSB
* @param[in] dst        Pointer to the destination memory
* @param[in] max        Size in bytes of destination
* @param[in] src        Pointer to the source memory
* @param[in] n          Size in bytes to copy
* @return dst
*
*/
static void *SSE2_memcpy(void *dst, size_t max, const void *src, size_t n)
{
	assert(n <= max);

	void *ldst = dst;
	void *lsrc = (void *)src;
	if (IS_CL_ALIGNED(src) && IS_CL_ALIGNED(dst))	// 64-byte aligned
	{
		if (n >= MIN_SSE2_SIZE)	// Arbitrary crossover performance point
		{
			debug_print("copying 0x%lx bytes with SSE2\n",
				(uint64_t)ALIGN_TO_CL(n));
			aligned_block_copy_sse2((int64_t * __restrict) dst,
				(int64_t * __restrict) src,
				ALIGN_TO_CL(n));
			ldst = (void *)((uint64_t)dst + ALIGN_TO_CL(n));
			lsrc = (void *)((uint64_t)src + ALIGN_TO_CL(n));
			n -= ALIGN_TO_CL(n);
		}
	}
	else {
		if (n >= MIN_SSE2_SIZE)	// Arbitrary crossover performance point
		{
			debug_print
			("copying 0x%lx bytes (unaligned) with SSE2\n",
				(uint64_t)ALIGN_TO_CL(n));
			unaligned_block_copy_sse2((int64_t * __restrict) dst,
				(int64_t * __restrict) src,
				ALIGN_TO_CL(n));
			ldst = (void *)((uint64_t)dst + ALIGN_TO_CL(n));
			lsrc = (void *)((uint64_t)src + ALIGN_TO_CL(n));
			n -= ALIGN_TO_CL(n);
		}
	}

	if (n) {
		register unsigned long int dummy;
		debug_print("copying 0x%lx bytes with REP MOVSB\n", n);
		__asm__ __volatile__("rep movsb\n":"=&D"(ldst), "=&S"(lsrc),
			"=&c"(dummy)
			: "0"(ldst), "1"(lsrc), "2"(n)
			: "memory");
	}

	return dst;
}

/**
* memcpy_wrap
*
* @brief                Trampoline for memcpy
* @param[in] dst        Pointer to the destination memory
* @param[in] max        Size in bytes of destination
* @param[in] src        Pointer to the source memory
* @param[in] n          Size in bytes to copy
* @return dst
*
*/

static void *memcpy_wrap(void *dst, size_t max, const void *src, size_t n)
{
        (void)max;
	return memcpy(dst, src, n);
}

/**
* memcpy_setup
* Will be called on the first memcpy_s_fast invocation only.
*
* @brief                Set up which memcpy routine will be used at runtime
* @param[in] dst        Pointer to the destination memory
* @param[in] max        Size in bytes of destination
* @param[in] src        Pointer to the source memory
* @param[in] n          Size in bytes to copy
* @return dst
*
*/

static void *memcpy_setup(void *dst, size_t max, const void *src, size_t n)
{
	// Default to SSE2_memcpy
	memcpy_s_fast = SSE2_memcpy;

	char *pmemcpy = secure_getenv(USE_MEMCPY_ENV);
	if (pmemcpy)
	{
		if (!strcasecmp(pmemcpy, "libc"))
		{
			memcpy_s_fast = memcpy_wrap;
		}
		else if (!strcasecmp(pmemcpy, "sse2"))
		{
			memcpy_s_fast = SSE2_memcpy;
		}
		else if (!strcasecmp(pmemcpy, "memcpy_s"))
		{
			memcpy_s_fast = (memcpy_fn_t)memcpy_s;
		}
	}

	return memcpy_s_fast(dst, max, src, n);
}
