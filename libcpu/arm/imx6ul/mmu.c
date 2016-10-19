/*
 * Copyright (c) 2008-2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file  mmu.c
 * @brief System memory arangement.
 */
#include <string.h>
#include "cortex_a.h"
#include "mmu.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Size in bytes of the first-level page table.
#define MMU_L1_PAGE_TABLE_SIZE (16 * 1024)

//! @brief First-level 1MB section descriptor entry.
typedef union mmu_l1_section {
    uint32_t u;
    struct {
        uint32_t id:2;  //!< ID
        uint32_t b:1;   //!< Bufferable
        uint32_t c:1;   //!< Cacheable
        uint32_t xn:1;  //!< Execute-not
        uint32_t domain:4;  //!< Domain
        uint32_t _impl_defined:1;   //!< Implementation defined, should be zero.
        uint32_t ap1_0:2;  //!< Access permissions AP[1:0]
        uint32_t tex:3; //!< TEX remap
        uint32_t ap2:1; //!< Access permissions AP[2]
        uint32_t s:1;   //!< Shareable
        uint32_t ng:1;  //!< Not-global
        uint32_t _zero:1;   //!< Should be zero.
        uint32_t ns:1;  //!< Non-secure
        uint32_t address:12;   //!< Physical base address
    };
} mmu_l1_section_t;

enum {
    kMMU_L1_Section_ID = 2,  //!< ID value for a 1MB section first-level entry.
    kMMU_L1_Section_Address_Shift = 20  //!< Bit offset of the physical base address field.
};

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

extern char __l1_page_table_start;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void mmu_enable()
{
    // invalidate all tlb
    arm_unified_tlb_invalidate();

    // read SCTLR
    uint32_t sctlr;
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);

    // set MMU enable bit
    sctlr |= BM_SCTLR_M;

    // write modified SCTLR
    _ARM_MCR(15, 0, sctlr, 1, 0, 0);
}

void mmu_disable()
{
    // read current SCTLR
    uint32_t sctlr;
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);

    // clear MMU enable bit
    sctlr &=~ BM_SCTLR_M;

    // write modified SCTLR
    _ARM_MCR(15, 0, sctlr, 1, 0, 0);
}

void mmu_init()
{
    // Get the L1 page table base address.
    uint32_t * table = (uint32_t *)&__l1_page_table_start;
    uint32_t share_attr = kShareable;

    // write table address to TTBR0
    _ARM_MCR(15, 0, table, 2, 0, 0);

    // set Client mode for all Domains
    uint32_t dacr = 0x55555555;
    _ARM_MCR(15, 0, dacr, 3, 0, 0); // MCR p15, 0, <Rd>, c3, c0, 0 ; Write DACR

    // Clear the L1 table.
    bzero(table, MMU_L1_PAGE_TABLE_SIZE);

    // Create default mappings.
    mmu_map_l1_range(0x00000000, 0x00000000, 0x00900000, kStronglyOrdered, kShareable, kRWAccess); // ROM and peripherals
    mmu_map_l1_range(0x00900000, 0x00900000, 0x00100000, kStronglyOrdered, kShareable, kRWAccess); // OCRAM
    mmu_map_l1_range(0x00a00000, 0x00a00000, 0x0f600000, kStronglyOrdered, kShareable, kRWAccess); // More peripherals

    // Check whether SMP is enabled. If it is not, then we don't want to make SDRAM shareable.
    uint32_t actlr = 0x0;
    _ARM_MRC(15, 0, actlr, 1, 0, 1);
    if (actlr & BM_ACTLR_SMP)
    {
        share_attr = kShareable;
    }
    else
    {
        share_attr = kNonshareable;
    }

#if defined(CHIP_MX6DQ) || defined(CHIP_MX6SDL)
    mmu_map_l1_range(0x10000000, 0x10000000, 0x80000000, kOuterInner_WB_WA, share_attr, kRWAccess); // 2GB DDR
#elif defined(CHIP_MX6SL) || defined(CHIP_MX6UL)
    mmu_map_l1_range(0x80000000, 0x80000000, 0x0ff00000, kOuterInner_WB_WA, share_attr, kRWAccess); // 255M DDR
    mmu_map_l1_range(0x8ff00000, 0x8ff00000, 0x00100000, kNoncacheable, share_attr, kRWAccess);     // 1M DDR
#else
#error Unknown chip type!
#endif
}

void mmu_map_l1_range(uint32_t pa, uint32_t va, uint32_t length, mmu_memory_type_t memoryType, mmu_shareability_t isShareable, mmu_access_t access)
{
    register mmu_l1_section_t entry;
    entry.u = 0;

    // Set constant attributes.
    entry.id = kMMU_L1_Section_ID;
    entry.xn = 0; // Allow execution
    entry.domain = 0; // Domain 0
    entry.ng = 0; // Global
    entry.ns = 0; // Secure

    // Set attributes based on the selected memory type.
    switch (memoryType)
    {
        case kStronglyOrdered:
            entry.c = 0;
            entry.b = 0;
            entry.tex = 0;
            entry.s = 1; // Ignored
            break;
        case kDevice:
            if (isShareable)
            {
                entry.c = 0;
                entry.b = 1;
                entry.tex = 0;
                entry.s = 1; // Ignored
            }
            else
            {
                entry.c = 0;
                entry.b = 0;
                entry.tex = 2;
                entry.s = 0; // Ignored
            }
            break;
        case kOuterInner_WB_WA:
            entry.c = 1;
            entry.b = 1;
            entry.tex = 1;
            entry.s = isShareable;
            break;
        case kOuterInner_WT:
            entry.c = 1;
            entry.b = 0;
            entry.tex = 0;
            entry.s = isShareable;
            break;
        case kNoncacheable:
            entry.c = 0;
            entry.b = 0;
            entry.tex = 1;
            entry.s = isShareable;
            break;
    }

    // Set attributes from specified access mode.
    switch (access)
    {
        case kNoAccess:
            entry.ap2 = 0;
            entry.ap1_0 = 0;
            break;
        case kROAccess:
            entry.ap2 = 1;
            entry.ap1_0 = 3;
            break;
        case kRWAccess:
            entry.ap2 = 0;
            entry.ap1_0 = 3;
            break;
    }

    // Get the L1 page table base address.
    uint32_t * table = (uint32_t *)&__l1_page_table_start;

    // Convert addresses to 12-bit bases.
    uint32_t vbase = va >> kMMU_L1_Section_Address_Shift;
    uint32_t pbase = pa >> kMMU_L1_Section_Address_Shift;
    uint32_t entries = length >> kMMU_L1_Section_Address_Shift;

    // Fill in L1 page table entries.
    for (; entries > 0; ++pbase, ++vbase, --entries)
    {
        entry.address = pbase;
        table[vbase] = entry.u;
    }

    // Invalidate TLB
    arm_unified_tlb_invalidate();
}

bool mmu_virtual_to_physical(uint32_t virtualAddress, uint32_t * physicalAddress)
{
    uint32_t pa = 0;

    // VA to PA translation with privileged read permission check
    _ARM_MCR(15, 0, virtualAddress & 0xfffffc00, 7, 8, 0);

    // Read PA register
    _ARM_MRC(15, 0, pa, 7, 4, 0);

    // First bit of returned value is Result of conversion (0 is successful translation)
    if (pa & 1)
    {
        // We can try write permission also
        // VA to PA translation with privileged write permission check
        _ARM_MCR(15, 0, virtualAddress & 0xfffffc00, 7, 8, 1);

        // Read PA register
        _ARM_MRC(15, 0, pa, 7, 4, 0);

        // First bit of returned value is Result of conversion (0 is successful translation)
        if (pa & 1)
        {
            return false;
        }
    }

    if (physicalAddress)
    {
        // complete address returning base + offset
        pa = (pa & 0xfffff000) | (virtualAddress & 0x00000fff);
        *physicalAddress = pa;
    }

    return true;
}

//! @brief Check if dcache is enabled or disabled
int arm_dcache_state_query()
{
    uint32_t sctlr; // System Control Register

    // read sctlr
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);

    if (sctlr & BM_SCTLR_C)
    {
        return 1;
    }

    return 0;
}

void arm_dcache_enable()
{
    uint32_t sctlr; // System Control Register

    // read sctlr
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);

    if (!(sctlr & BM_SCTLR_C))
    {
        // set  C bit (data caching enable)
        sctlr |= BM_SCTLR_C;

        // write modified sctlr
        _ARM_MCR(15, 0, sctlr, 1, 0, 0);

        // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
        _ARM_DSB();
    }
}

void arm_dcache_disable()
{
    uint32_t sctlr; // System Control Register

    // read sctlr
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);

    // set  C bit (data caching enable)
    sctlr &= ~BM_SCTLR_C;

    // write modified sctlr
    _ARM_MCR(15, 0, sctlr, 1, 0, 0);

    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void arm_dcache_invalidate()
{
    uint32_t csid;    // Cache Size ID
    uint32_t wayset;  // wayset parameter
    int num_sets; // number of sets
    int num_ways; // number of ways

    _ARM_MRC(15, 1, csid, 0, 0, 0);    // Read Cache Size ID

    // Fill number of sets  and number of ways from csid register  This walues are decremented by 1
    num_ways = (csid >> 0x03) & 0x3FFu; //((csid& csid_ASSOCIATIVITY_MASK) >> csid_ASSOCIATIVITY_SHIFT)

    // Invalidation all lines (all Sets in all ways)
    while (num_ways >= 0)
    {
        num_sets = (csid >> 0x0D) & 0x7FFFu; //((csid & csid_NUMSETS_MASK) >> csid_NUMSETS_SHIFT)
        while (num_sets >= 0 )
        {
            wayset = (num_sets << 5u) | (num_ways << 30u); //(num_sets << SETWAY_SET_SHIFT) | (num_sets << 3SETWAY_WAY_SHIFT)
            // invalidate line if we know set and way
            _ARM_MCR(15, 0, wayset, 7, 6, 2);
            num_sets--;
        }
        num_ways--;
    }

    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void arm_dcache_invalidate_line(const void * addr)
{
    uint32_t csidr = 0, line_size = 0;
    uint32_t va;

    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);
    va = (uint32_t) addr & (~(line_size - 1)); //addr & va_VIRTUAL_ADDRESS_MASK

    // Invalidate data cache line by va to PoC (Point of Coherency).
    _ARM_MCR(15, 0, va, 7, 6, 1);

    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void arm_dcache_invalidate_mlines(const void * addr, size_t length)
{
    uint32_t va;
    uint32_t csidr = 0, line_size = 0;

    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);

    // align the address with line
    const void * end_addr = (const void *)((uint32_t)addr + length);

    do
    {
        // Clean data cache line to PoC (Point of Coherence) by va.
        va = (uint32_t) ((uint32_t)addr & (~(line_size - 1))); //addr & va_VIRTUAL_ADDRESS_MASK
        _ARM_MCR(15, 0, va, 7, 6, 1);
        // increment addres to next line and decrement lenght
        addr = (const void *) ((uint32_t)addr + line_size);
    } while (addr < end_addr);

    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void arm_dcache_flush()
{
    uint32_t csid;    // Cache Size ID
    uint32_t wayset;  // wayset parameter
    int num_sets; // number of sets
    int num_ways; // number of ways

    _ARM_MRC(15, 1, csid, 0, 0, 0);    // Read Cache Size ID

    // Fill number of sets  and number of ways from csid register  This walues are decremented by 1
    num_ways = (csid >> 0x03) & 0x3FFu; //((csid& csid_ASSOCIATIVITY_MASK) >> csid_ASSOCIATIVITY_SHIFT`)
    while (num_ways >= 0)
    {
        num_sets = (csid >> 0x0D) & 0x7FFFu; //((csid & csid_NUMSETS_MASK)      >> csid_NUMSETS_SHIFT       )
        while (num_sets >= 0 )
        {
            wayset = (num_sets << 5u) | (num_ways << 30u); //(num_sets << SETWAY_SET_SHIFT) | (num_ways << 3SETWAY_WAY_SHIFT)
            // FLUSH (clean) line if we know set and way
            _ARM_MCR(15, 0, wayset, 7, 10, 2);
            num_sets--;
        }
        num_ways--;
    }

    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void arm_dcache_flush_line(const void * addr)
{
    uint32_t csidr = 0, line_size = 0;
    uint32_t va;

    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);
    va = (uint32_t) addr & (~(line_size - 1)); //addr & va_VIRTUAL_ADDRESS_MASK

    // Clean data cache line to PoC (Point of Coherence) by va.
    _ARM_MCR(15, 0, va, 7, 10, 1);

    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

void arm_dcache_flush_mlines(const void * addr, size_t length)
{
    uint32_t va;
    uint32_t csidr = 0, line_size = 0;
    const void * end_addr = (const void *)((uint32_t)addr + length);

    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);

    do
    {
        // Clean data cache line to PoC (Point of Coherence) by va.
        va = (uint32_t) ((uint32_t)addr & (~(line_size  - 1))); //addr & va_VIRTUAL_ADDRESS_MASK
        _ARM_MCR(15, 0, va, 7, 10, 1);

        // increment addres to next line and decrement lenght
        addr = (const void *) ((uint32_t)addr + line_size);
    } while (addr < end_addr);

    // All Cache, Branch predictor and TLB maintenance operations before followed instruction complete
    _ARM_DSB();
}

int arm_icache_state_query()
{
    uint32_t sctlr; // System Control Register

    // read sctlr
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);

    if (sctlr & BM_SCTLR_I)
    {
        return 1;
    }

    return 0;
}

void arm_icache_enable()
{
    uint32_t sctlr  ;// System Control Register

    // read sctlr
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);

    // ignore the operation if I is enabled already
    if(!(sctlr & BM_SCTLR_I))
    {
        // set  I bit (instruction caching enable)
        sctlr |= BM_SCTLR_I;

        // write modified sctlr
        _ARM_MCR(15, 0, sctlr, 1, 0, 0);

        // synchronize context on this processor
        _ARM_ISB();
    }
}

void arm_icache_disable()
{
    uint32_t sctlr  ;// System Control Register

    // read sctlr
    _ARM_MRC(15, 0, sctlr, 1, 0, 0);

    // Clear  I bit (instruction caching enable)
    sctlr &= ~BM_SCTLR_I;

    // write modified sctlr
    _ARM_MCR(15, 0, sctlr, 1, 0, 0);

    // synchronize context on this processor
    _ARM_ISB();
}

void arm_icache_invalidate()
{
    uint32_t SBZ = 0x0u;

    _ARM_MCR(15, 0, SBZ, 7, 5, 0);

    // synchronize context on this processor
    _ARM_ISB();
}

void arm_icache_invalidate_is()
{
    uint32_t SBZ = 0x0u;

    _ARM_MCR(15, 0, SBZ, 7, 1, 0);

    // synchronize context on this processor
    _ARM_ISB();
}

void arm_icache_invalidate_line(const void * addr)
{
    uint32_t csidr = 0, line_size = 0;
    uint32_t va;

    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);
    va = (uint32_t) addr & (~(line_size - 1)); //addr & va_VIRTUAL_ADDRESS_MASK

    // Invalidate instruction cache by va to PoU (Point of unification).
    _ARM_MCR(15, 0, va, 7, 5, 1);

    // synchronize context on this processor
    _ARM_ISB();
}

void arm_icache_invalidate_mlines(const void * addr, size_t length)
{
    uint32_t va;
    uint32_t csidr = 0, line_size = 0;
    const void * end_addr = (const void *)((uint32_t)addr + length);

    // get the cache line size
    _ARM_MRC(15, 1, csidr, 0, 0, 0);
    line_size = 1 << ((csidr & 0x7) + 4);

    do
    {
        // Clean data cache line to PoC (Point of Coherence) by va.
        va = (uint32_t) ((uint32_t)addr & (~(line_size - 1))); //addr & va_VIRTUAL_ADDRESS_MASK
        _ARM_MCR(15, 0, va, 7, 5, 1);
        // increment addres to next line and decrement lenght
        addr = (const void *) ((uint32_t)addr + line_size);
    } while (addr < end_addr);

    // synchronize context on this processor
    _ARM_ISB();
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
