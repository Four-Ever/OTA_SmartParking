/**********************************************************************************************************************
 * \file Lcf_Tasking_Tricore_Tc.lsl
 * \brief Linker command file for Tasking compiler.
 * \copyright Copyright (C) Infineon Technologies AG 2019
 * 
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of 
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 * 
 * Boost Software License - Version 1.0 - August 17th, 2003
 * 
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and 
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 * 
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all 
 * derivative works of the Software, unless such copies or derivative works are solely in the form of 
 * machine-executable object code generated by a source language processor.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN 
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 * IN THE SOFTWARE.
 *********************************************************************************************************************/
 
#define LCF_CSA0_SIZE       8k
#define LCF_USTACK0_SIZE    2k
#define LCF_ISTACK0_SIZE    1k

#define LCF_CSA1_SIZE       8k
#define LCF_USTACK1_SIZE    2k
#define LCF_ISTACK1_SIZE    1k

#define LCF_CSA2_SIZE       8k
#define LCF_USTACK2_SIZE    2k
#define LCF_ISTACK2_SIZE    1k

#define LCF_HEAP_SIZE       4k

#define LCF_CPU0            0
#define LCF_CPU1            1
#define LCF_CPU2            2

/*Un comment one of the below statements to enable CpuX DMI RAM to hold global variables*/
/*#define LCF_DEFAULT_HOST  LCF_CPU0*/
#define LCF_DEFAULT_HOST    LCF_CPU1
/*#define LCF_DEFAULT_HOST  LCF_CPU2*/

#define LCF_DSPR2_START     0x50000000
#define LCF_DSPR2_SIZE      120k

#define LCF_DSPR1_START     0x60000000
#define LCF_DSPR1_SIZE      120k

#define LCF_DSPR0_START     0x70000000
#define LCF_DSPR0_SIZE      112k

#define LCF_CSA2_OFFSET     (LCF_DSPR2_SIZE - 1k - LCF_CSA2_SIZE)
#define LCF_ISTACK2_OFFSET  (LCF_CSA2_OFFSET - 256 - LCF_ISTACK2_SIZE)
#define LCF_USTACK2_OFFSET  (LCF_ISTACK2_OFFSET - 256 - LCF_USTACK2_SIZE)

#define LCF_CSA1_OFFSET     (LCF_DSPR1_SIZE - 1k - LCF_CSA1_SIZE)
#define LCF_ISTACK1_OFFSET  (LCF_CSA1_OFFSET - 256 - LCF_ISTACK1_SIZE)
#define LCF_USTACK1_OFFSET  (LCF_ISTACK1_OFFSET - 256 - LCF_USTACK1_SIZE)

#define LCF_CSA0_OFFSET     (LCF_DSPR0_SIZE - 1k - LCF_CSA0_SIZE)
#define LCF_ISTACK0_OFFSET  (LCF_CSA0_OFFSET - 256 - LCF_ISTACK0_SIZE)
#define LCF_USTACK0_OFFSET  (LCF_ISTACK0_OFFSET - 256 - LCF_USTACK0_SIZE)

#define LCF_HEAP0_OFFSET    (LCF_USTACK0_OFFSET - LCF_HEAP_SIZE)
#define LCF_HEAP1_OFFSET    (LCF_USTACK1_OFFSET - LCF_HEAP_SIZE)
#define LCF_HEAP2_OFFSET    (LCF_USTACK2_OFFSET - LCF_HEAP_SIZE)

#define LCF_INTVEC0_START   0x800F4000
#define LCF_INTVEC1_START   0x800F5000
#define LCF_INTVEC2_START   0x800F3000

#define LCF_TRAPVEC0_START  0x80000100
#define LCF_TRAPVEC1_START  0x800F6200
#define LCF_TRAPVEC2_START  0x800F6000

#define INTTAB0             (LCF_INTVEC0_START)
#define INTTAB1             (LCF_INTVEC1_START)
#define INTTAB2             (LCF_INTVEC2_START)

#define TRAPTAB0            (LCF_TRAPVEC0_START)
#define TRAPTAB1            (LCF_TRAPVEC1_START)
#define TRAPTAB2            (LCF_TRAPVEC2_START)

#define RESET 0x80000020

#include "tc1v1_6_x.lsl"

// Specify a multi-core processor environment (mpe)

processor mpe
{
    derivative = tc27D;
}

derivative tc27D
{
    core tc0
    {
        architecture = TC1V1.6.X;
        space_id_offset = 100;          // add 100 to all space IDs in the architecture definition
        copytable_space = vtc:linear;   // use the copy table in the virtual core for 'bss' and initialized data sections
    }
    
    core tc1 // core 1 TC16E
    {
        architecture = TC1V1.6.X;
        space_id_offset = 200;          // add 200 to all space IDs in the architecture definition
        copytable_space = vtc:linear;   // use the copy table in the virtual core for 'bss' and initialized data sections
    }
    
    core tc2 // core 2 TC16P
    {
        architecture = TC1V1.6.X;
        space_id_offset = 300;          // add 300 to all space IDs in the architecture definition
        copytable_space = vtc:linear;   // use the copy table in the virtual core for 'bss' and initialized data sections
    }
    
    core vtc
    {
        architecture = TC1V1.6.X;
        import tc0;                     // add all address spaces of core tc0 to core vtc for linking and locating
        import tc1;                     //                                tc1
        import tc2;                     //                                tc2
    }
    
    bus sri
    {
        mau = 8;
        width = 32;
        
        // map shared addresses one-to-one to real cores and virtual cores
        map (dest=bus:tc0:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:tc1:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:tc2:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:vtc:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
    }
    
    memory dsram2 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 120k;
        type = ram;
        map (dest=bus:tc2:fpi_bus, dest_offset=0xd0000000, size=120k, priority=8);
        map (dest=bus:sri, dest_offset=0x50000000, size=120k);
    }
    
    memory psram2 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 32k;
        type = ram;
        map (dest=bus:tc2:fpi_bus, dest_offset=0xc0000000, size=32k, priority=8);
        map (dest=bus:sri, dest_offset=0x50100000, size=32k);
    }
    
    memory dsram1 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 120k;
        type = ram;
        map (dest=bus:tc1:fpi_bus, dest_offset=0xd0000000, size=120k, priority=8);
        map (dest=bus:sri, dest_offset=0x60000000, size=120k);
    }
    
    memory psram1 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 32k;
        type = ram;
        map (dest=bus:tc1:fpi_bus, dest_offset=0xc0000000, size=32k, priority=8);
        map (dest=bus:sri, dest_offset=0x60100000, size=32k);
    }
     
    memory dsram0 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 112k;
        type = ram;
        map (dest=bus:tc0:fpi_bus, dest_offset=0xd0000000, size=112k, priority=8);
        map (dest=bus:sri, dest_offset=0x70000000, size=112k);
    }
    
    memory psram0 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 24k;
        type = ram;
        map (dest=bus:tc0:fpi_bus, dest_offset=0xc0000000, size=24k, priority=8);
        map (dest=bus:sri, dest_offset=0x70100000, size=24k);
    }
    
    memory pfls0
    {
        mau = 8;
        size = 2M;
        type = rom;
        map     cached (dest=bus:sri, dest_offset=0x80000000,           size=1M);
        map not_cached (dest=bus:sri, dest_offset=0xa0000000, reserved, size=1M);
    }
    
    memory pfls1
    {
        mau = 8;
        size = 2M;
        type = rom;
        map     cached (dest=bus:sri, dest_offset=0x80200000,           size=2M);
        map not_cached (dest=bus:sri, dest_offset=0xa0200000, reserved, size=2M);
    }
    
    memory dfls0
    {
        mau = 8;
        size = 1m+16k;
        type = reserved nvram;
        map (dest=bus:sri, dest_offset=0xaf000000, size=384k  );
    }
    
    memory lmuram
    {
        mau = 8;
        size = 32k;
        type = ram;
        priority = 2;
        map     cached (dest=bus:sri, dest_offset=0x90000000,           size=32k);
        map not_cached (dest=bus:sri, dest_offset=0xb0000000, reserved, size=32k);
    }
    
    memory edmem
    {
        mau = 8;
        size = 1M;
        type = ram;
        map (dest=bus:sri, dest_offset=0x9f000000, size=1M);
        map (dest=bus:sri, dest_offset=0xbf000000, reserved, size=1M);
    }

#if (__VERSION__ >= 6003)
    section_setup :vtc:linear
    {
        heap "heap" (min_size = (1k), fixed, align = 8);
    }    
#endif
    
    section_setup :vtc:linear
    {
        start_address
        (
            symbol = "_START"
        );
    }
    
    section_setup :vtc:linear
    {
        stack "ustack_tc0" (min_size = 1k, fixed, align = 8);
        stack "istack_tc0" (min_size = 1k, fixed, align = 8);
        stack "ustack_tc1" (min_size = 1k, fixed, align = 8);
        stack "istack_tc1" (min_size = 1k, fixed, align = 8);
        stack "ustack_tc2" (min_size = 1k, fixed, align = 8);
        stack "istack_tc2" (min_size = 1k, fixed, align = 8);
    }
    
    /*Section setup for the copy table*/
    section_setup :vtc:linear
    {
        copytable
        (
            align = 4,
            dest = linear,
            table
            {
                symbol = "_lc_ub_table_tc0";
                space = :tc0:linear, :tc0:abs24, :tc0:abs18, :tc0:csa;
            },
            table
            {
                symbol = "_lc_ub_table_tc1";
                space = :tc1:linear, :tc1:abs24, :tc1:abs18, :tc1:csa;
            },
            table
            {
                symbol = "_lc_ub_table_tc2";
                space = :tc2:linear, :tc2:abs24, :tc2:abs18, :tc2:csa;
            }
        );
    }
    
    /*Near data sections*/
    section_layout :vtc:abs18
    {   
        group  (ordered, run_addr=mem:lmuram)
        {
            select "(.zdata.zlmubss|.zdata.zlmubss*)";
            select "(.zdata.zlmudata|.zdata.zlmudata*)";
            select "(.zdata.zdata_lmu|.zdata.zdata_lmu*)";
            select "(.zdata.zbss_lmu|.zdata.zbss_lmu*)";
        }
        
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram2)
        {
            select "(.zdata.zdata_cpu2|.zdata.zdata_cpu2*)";
            select "(.zbss.zbss_cpu2|.zbss.zbss_cpu2*)";
        }
        
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram1)
        {
            select "(.zdata.zdata_cpu1|.zdata.zdata_cpu1*)";
            select "(.zbss.zbss_cpu1|.zbss.zbss_cpu1*)";
        }
        
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram0)
        {
            select "(.zdata.zdata_cpu0|.zdata.zdata_cpu0*)";
            select "(.zbss.zbss_cpu0|.zbss.zbss_cpu0*)";
        }
#       if LCF_DEFAULT_HOST == LCF_CPU2
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram2)
#       endif
#       if LCF_DEFAULT_HOST == LCF_CPU1
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram1)
#       endif
#       if LCF_DEFAULT_HOST == LCF_CPU0
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram0)
#       endif
        {
            select "(.zdata|.zdata*)";
            select "(.zbss|.zbss*)";
        }
    }

    section_layout :vtc:linear 
    {
        group a9 (ordered, run_addr=mem:lmuram)
        {
            select "(.data_a9.a9sdata|.data_a9.a9sdata*)";
            select "(.data_a9.sdata_lmu|.data_a9.sdata_lmu*)";
            select "(.data_a9|.data_a9*)";
            select "(.bss_a9.a9sbss|.bss_a9.a9sbss*)";
            select "(.bss_a9.sbss_lmu|.sbss.sbss_lmu*)";
            select "(.bss_a9|.bss_a9*)";
        }
        "_A9_DATA_" := sizeof(group:a9) > 0 ? addressof(group:a9) + 32k : addressof(group:a9) & 0xF0000000 + 32k;
        "_A9_MEM" = "_A9_DATA_";

/*Small data sections, No option given for CPU specific user sections to make generated code portable across Cpus*/
#       if LCF_DEFAULT_HOST == LCF_CPU2
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram2)
#       endif
#       if LCF_DEFAULT_HOST == LCF_CPU1
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram1)
#       endif
#       if LCF_DEFAULT_HOST == LCF_CPU0
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram0)
#       endif
        {
            select "(.sdata |.sdata*)";
            select "(.sbss |.sbss*)";
        }
        "_SMALL_DATA_" := sizeof(group:a0) > 0 ? addressof(group:a0) + 32k : addressof(group:a0) & 0xF0000000 + 32k;

        /*Far data sections*/
        group (ordered, run_addr=mem:lmuram)
        {
            select "(.data.data_lmu|.data.data_lmu*)";
            select "(.bss.bss_lmu|.bss.bss_lmu*)";
            select "(.data.lmudata|.data.lmudata*)";
            select "(.bss.lmubss|.bss.lmubss*)";
        }
        
        group (ordered, contiguous, align = 4, run_addr = mem:edmem)
        {
            select "(.data.edmemdata|.data.edmemdata*)";
            select "(.bss.edmembss|.bss.edmembss*)";
        }
                
        group (ordered, contiguous, align = 4, run_addr = mem:dsram2)
        {
            select "(.data.data_cpu2|.data.data_cpu2*)";
            select "(.bss.bss_cpu2|.bss.bss_cpu2*)";
        }
        
        group (ordered, contiguous, align = 4, run_addr = mem:dsram1)
        {
            select "(.data.data_cpu1|.data.data_cpu1*)";
            select "(.bss.bss_cpu1|.bss.bss_cpu1*)";
        }
        
        group (ordered, contiguous, align = 4, run_addr = mem:dsram0)
        {
            select "(.data.data_cpu0|.data.data_cpu0*)";
            select "(.bss.bss_cpu0|.bss.bss_cpu0*)";
        }

#       if LCF_DEFAULT_HOST == LCF_CPU2
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram2)
#       endif
#       if LCF_DEFAULT_HOST == LCF_CPU1
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram1)
#       endif
#       if LCF_DEFAULT_HOST == LCF_CPU0
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram0)
#       endif
        {
            select "(.data|.data*)";
            select "(.bss|.bss*)";
        }

/*Heap sections*/
#       if LCF_DEFAULT_HOST == LCF_CPU2
        group (ordered, align = 4, run_addr = mem:dsram2[LCF_HEAP2_OFFSET])
#       endif
#       if LCF_DEFAULT_HOST == LCF_CPU1
        group (ordered, align = 4, run_addr = mem:dsram1[LCF_HEAP1_OFFSET])
#       endif
#       if LCF_DEFAULT_HOST == LCF_CPU0
        group (ordered, align = 4, run_addr = mem:dsram0[LCF_HEAP0_OFFSET])
#       endif
        {
            heap "heap" (size = LCF_HEAP_SIZE);
        }
    
        group (ordered, align = 8, run_addr = mem:dsram2[LCF_USTACK2_OFFSET])
        {
            stack "ustack_tc2" (size = LCF_USTACK2_SIZE);
        }
        "__USTACK2":=   "_lc_ue_ustack_tc2";
        "__USTACK2_END":=   "_lc_ub_ustack_tc2";
        
        group (ordered, align = 8, run_addr = mem:dsram2[LCF_ISTACK2_OFFSET])
        {
            stack "istack_tc2" (size = LCF_ISTACK2_SIZE);
        }
        "__ISTACK2":=   "_lc_ue_istack_tc2";
        "__ISTACK2_END":=   "_lc_ub_istack_tc2";
        
        group (ordered, align = 64, attributes=rw, run_addr=mem:dsram2[LCF_CSA2_OFFSET]) 
            reserved "csa_tc2" (size = LCF_CSA2_SIZE);
        "__CSA2":=      "_lc_ub_csa_tc2";
        "__CSA2_END":=  "_lc_ue_csa_tc2";
        
        group (ordered, align = 8, run_addr = mem:dsram1[LCF_USTACK1_OFFSET])
        {
            stack "ustack_tc1" (size = LCF_USTACK1_SIZE);
        }
        "__USTACK1":=   "_lc_ue_ustack_tc1";
        "__USTACK1_END":=   "_lc_ub_ustack_tc1";
        
        group (ordered, align = 8, run_addr = mem:dsram1[LCF_ISTACK1_OFFSET])
        {
            stack "istack_tc1" (size = LCF_ISTACK1_SIZE);
        }
        "__ISTACK1":=   "_lc_ue_istack_tc1";
        "__ISTACK1_END":=   "_lc_ub_istack_tc1";
        
        group  (ordered, align = 64, attributes=rw, run_addr=mem:dsram1[LCF_CSA1_OFFSET]) 
                    reserved "csa_tc1" (size = LCF_CSA1_SIZE);
        "__CSA1":=      "_lc_ub_csa_tc1";
        "__CSA1_END":=  "_lc_ue_csa_tc1";

        group (ordered, align = 8, run_addr = mem:dsram0[LCF_USTACK0_OFFSET])
        {
            stack "ustack_tc0" (size = LCF_USTACK0_SIZE);
        }
        "__USTACK0":=   "_lc_ue_ustack_tc0";
        "__USTACK0_END":=   "_lc_ub_ustack_tc0";
            
        group (ordered, align = 8, run_addr = mem:dsram0[LCF_ISTACK0_OFFSET])
        {
            stack "istack_tc0" (size = LCF_ISTACK0_SIZE);
        }
        "__ISTACK0":=   "_lc_ue_istack_tc0";
        "__ISTACK0_END":=   "_lc_ub_istack_tc0";
        
        group  (ordered, align = 64, attributes=rw, run_addr=mem:dsram0[LCF_CSA0_OFFSET]) 
            reserved "csa_tc0" (size = LCF_CSA0_SIZE);
        "__CSA0":=      "_lc_ub_csa_tc0";
        "__CSA0_END":=  "_lc_ue_csa_tc0";
    }
    
    
    section_layout :vtc:linear
    {
        "_lc_u_int_tab" = (LCF_INTVEC0_START);
        "__INTTAB_CPU0" = (LCF_INTVEC0_START);
        "__INTTAB_CPU1" = (LCF_INTVEC1_START);
        "__INTTAB_CPU2" = (LCF_INTVEC2_START);
        
        // interrupt vector tables for tc0, tc1, tc2 
        group int_tab_tc0 (ordered)
        {
#           include "inttab0.lsl"
        }
        
        group int_tab_tc1 (ordered)
        {
#           include "inttab1.lsl"
        }
        
        group int_tab_tc2 (ordered)
        {
#           include "inttab2.lsl"
        }
        
        group trapvec_tc0 (ordered, run_addr=LCF_TRAPVEC0_START)
        {
            select "(.text.traptab_cpu0*)";
        }
        
        group trapvec_tc2 (ordered, run_addr=LCF_TRAPVEC2_START)
        {
            select "(.text.traptab_cpu2*)";
        }
        
        group trapvec_tc1 (ordered, run_addr=LCF_TRAPVEC1_START)
        {
            select "(.text.traptab_cpu1*)";
        }
        
        group code_psram0 (ordered, attributes=rwx, copy, run_addr=mem:psram0)
        {
            select "(.text.psram_cpu0*)";
            select "(.text.cpu0_psram*)";
        }       

        group code_psram1 (ordered, attributes=rwx, copy, run_addr=mem:psram1)
        {
            select "(.text.psram_cpu1*)";
            select "(.text.cpu1_psram*)";
        }
        
        group code_psram2 (ordered, attributes=rwx, copy, run_addr=mem:psram2)
        {
            select "(.text.psram_cpu2*)";
            select "(.text.cpu2_psram*)";
        }       
    }
    
    section_layout :vtc:abs18
    {
        group  (ordered, run_addr=mem:pfls0)
        {
            select ".zrodata*";
        }
    }
    
    section_layout :vtc:linear
    {       
        group  bmh_0 (ordered, run_addr=0x80000000)
        {
            select "*.bmhd_0";
        }
        group  bmh_1 (ordered, run_addr=0x80020000)
        {
            select "*.bmhd_1";
        }
        group  reset (ordered, run_addr=0x80000020)
        {
            select "*.start";
        }
        group  interface_const (ordered, run_addr=0x80000040)
        {
            select "*.interface_const";
        }
        "__IF_CONST" := addressof(group:ainterface_const);
        group  a1 (ordered, run_addr=mem:pfls0)
        {
            select ".srodata*";
            select ".ldata*";
        }
        "_LITERAL_DATA_" := sizeof(group:a1) > 0 ? addressof(group:a1) + 32k : addressof(group:a1) & 0xF0000000 + 32k;
        "_A1_MEM" = "_LITERAL_DATA_";
        
        group  (ordered, run_addr=mem:pfls0)
        {
            select ".rodata*";
        }
        group  (ordered, run_addr=mem:pfls0)
        {
            select ".text*";
        }
        group a8 (ordered, run_addr=mem:pfls0)
        {
            select "(.rodata_a8|.rodata_a8*)";
        }
        "_A8_DATA_" := sizeof(group:a8) > 0 ? addressof(group:a8) + 32k : addressof(group:a8) & 0xF0000000 + 32k;
        "_A8_MEM" := "_A8_DATA_";
    
        "__TRAPTAB_CPU0" := TRAPTAB0;
        "__TRAPTAB_CPU1" := TRAPTAB1;
        "__TRAPTAB_CPU2" := TRAPTAB2;
    }
}
