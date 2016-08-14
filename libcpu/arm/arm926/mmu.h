/*
 * File      : mmu.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 */

#ifndef __MMU_H__
#define __MMU_H__

#include <rtthread.h>

#define HEAP_BEGIN        0x40E00000
#define HEAP_END          0x43E00000

#define PROCESS_MAX       16
#define PROCESS_BASE      0x10000000
#define PROCESS_IN        2
#define PROCESS_MEM       16

#define CACHE_LINE_SIZE     32

#define DESC_SEC            (0x2|(1<<4))
#define DESC_PET            (0x1|(1<<4))
#define DESC_SMALL          (0x2)

#define CB                  (3<<2)  //cache_on, write_back
#define CNB                 (2<<2)  //cache_on, write_through
#define NCB                 (1<<2)  //cache_off,WR_BUF on
#define NCNB                (0<<2)  //cache_off,WR_BUF off

#define AP_RW               (3<<10) //supervisor=RW, user=RW
#define AP_RO               (2<<10) //supervisor=RW, user=RO
#define AP_NA               (1<<10) //supervisor=RW, user=NA
#define PET_RW              (0xff<<4) //supervisor=RW, user=RW
#define PET_RO              (0xaa<<4) //supervisor=RW, user=RO
#define PET_NA              (0x55<<4) //supervisor=RW, user=NA

#define DOMAIN_FAULT        (0x0)
#define DOMAIN_CHK          (0x1)
#define DOMAIN_NOTCHK       (0x3)
#define DOMAIN0             (0x0<<5)
#define DOMAIN1             (0x1<<5)

#define DOMAIN0_ATTR        (DOMAIN_CHK<<0)
#define DOMAIN1_ATTR        (DOMAIN_FAULT<<2)

#define RW_CBE      (AP_RO|DOMAIN0|CB|DESC_SEC)     /* Read/Write, cache, write back */
#define RW_CBD      (AP_NA|DOMAIN0|CB|DESC_SEC)     /* Read/Write, cache, write back */
#define RW_CNB      (AP_NA|DOMAIN0|CNB|DESC_SEC)    /* Read/Write, cache, write through */
#define RW_NCNB     (AP_NA|DOMAIN0|NCNB|DESC_SEC)   /* Read/Write without cache and write buffer */

#define PET_RW_CB       (PET_RW|CB|DESC_SMALL)     /* Read/Write, cache, write back */
#define PET_RW_CNB      (PET_RW|CNB|DESC_SMALL)    /* Read/Write, cache, write through */
#define PET_RW_NCNB     (PET_RW|NCNB|DESC_SMALL)   /* Read/Write without cache and write buffer */

struct mem_desc
{
    rt_uint32_t vaddr_start;
    rt_uint32_t vaddr_end;
    rt_uint32_t paddr_start;
    rt_uint32_t attr;
};

void rt_hw_mmu_init(struct mem_desc *mdesc, rt_uint32_t size);
void mmu_maketlb(rt_uint32_t pid);
void mmu_freetlb(rt_uint32_t pid);
void mmu_switchtlb(rt_uint32_t pid);
void mmu_usermap(rt_uint32_t pid, rt_uint32_t base, rt_uint32_t map, rt_uint32_t size, rt_uint32_t flush);
void mmu_userunmap(rt_uint32_t pid, rt_uint32_t map, rt_uint32_t size, rt_uint32_t flush);
int mmu_check_ptr(rt_uint32_t pid, rt_uint32_t map, rt_uint32_t size);

#endif
