#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#undef PARITY_NONE
#undef PARITY_ODD
#undef PARITY_EVEN
#include <Windows.h>

#define OOB_SIZE        64
#define PAGE_DATA_SIZE  2048
#define PAGE_PER_BLOCK  64
#define ECC_SIZE       ((PAGE_DATA_SIZE) * 3 / 256)
#define BLOCK_NUM       1024

#define BLOCK_SIZE      (PAGE_SIZE * PAGE_PER_BLOCK)
#define PAGE_SIZE       (PAGE_DATA_SIZE + OOB_SIZE)

static unsigned char block_data[BLOCK_SIZE];
static struct rt_mtd_nand_device _nanddrv_file_device;
static HANDLE file = INVALID_HANDLE_VALUE;

static rt_uint8_t CountBitsInByte(rt_uint8_t byte)
{
    rt_uint8_t count = 0;

    while (byte > 0)
    {
        if (byte & 1)
        {
            count++;
        }
        byte >>= 1;
    }

    return count;
}

static void Compute256(const rt_uint8_t *data, rt_uint8_t *code)
{
    rt_uint32_t i;
    rt_uint8_t columnSum = 0;
    rt_uint8_t evenLineCode = 0;
    rt_uint8_t oddLineCode = 0;
    rt_uint8_t evenColumnCode = 0;
    rt_uint8_t oddColumnCode = 0;

    // Xor all bytes together to get the column sum;
    // At the same time, calculate the even and odd line codes
    for (i = 0; i < 256; i++)
    {
        columnSum ^= data[i];

        // If the xor sum of the byte is 0, then this byte has no incidence on
        // the computed code; so check if the sum is 1.
        if ((CountBitsInByte(data[i]) & 1) == 1)
        {
            // Parity groups are formed by forcing a particular index bit to 0
            // (even) or 1 (odd).
            // Example on one byte:
            //
            // bits (dec)  7   6   5   4   3   2   1   0
            //      (bin) 111 110 101 100 011 010 001 000
            //                            '---'---'---'----------.
            //                                                   |
            // groups P4' ooooooooooooooo eeeeeeeeeeeeeee P4     |
            //        P2' ooooooo eeeeeee ooooooo eeeeeee P2     |
            //        P1' ooo eee ooo eee ooo eee ooo eee P1     |
            //                                                   |
            // We can see that:                                  |
            //  - P4  -> bit 2 of index is 0 --------------------'
            //  - P4' -> bit 2 of index is 1.
            //  - P2  -> bit 1 of index if 0.
            //  - etc...
            // We deduce that a bit position has an impact on all even Px if
            // the log2(x)nth bit of its index is 0
            //     ex: log2(4) = 2, bit2 of the index must be 0 (-> 0 1 2 3)
            // and on all odd Px' if the log2(x)nth bit of its index is 1
            //     ex: log2(2) = 1, bit1 of the index must be 1 (-> 0 1 4 5)
            //
            // As such, we calculate all the possible Px and Px' values at the
            // same time in two variables, evenLineCode and oddLineCode, such as
            //     evenLineCode bits: P128  P64  P32  P16  P8  P4  P2  P1
            //     oddLineCode  bits: P128' P64' P32' P16' P8' P4' P2' P1'
            //
            evenLineCode ^= (255 - i);
            oddLineCode ^= i;
        }
    }

    // At this point, we have the line parities, and the column sum. First, We
    // must caculate the parity group values on the column sum.
    for (i = 0; i < 8; i++)
    {
        if (columnSum & 1)
        {
            evenColumnCode ^= (7 - i);
            oddColumnCode ^= i;
        }
        columnSum >>= 1;
    }

    // Now, we must interleave the parity values, to obtain the following layout:
    // Code[0] = Line1
    // Code[1] = Line2
    // Code[2] = Column
    // Line = Px' Px P(x-1)- P(x-1) ...
    // Column = P4' P4 P2' P2 P1' P1 PadBit PadBit
    code[0] = 0;
    code[1] = 0;
    code[2] = 0;

    for (i = 0; i < 4; i++)
    {
        code[0] <<= 2;
        code[1] <<= 2;
        code[2] <<= 2;

        // Line 1
        if ((oddLineCode & 0x80) != 0)
        {
            code[0] |= 2;
        }

        if ((evenLineCode & 0x80) != 0)
        {
            code[0] |= 1;
        }

        // Line 2
        if ((oddLineCode & 0x08) != 0)
        {
            code[1] |= 2;
        }

        if ((evenLineCode & 0x08) != 0)
        {
            code[1] |= 1;
        }

        // Column
        if ((oddColumnCode & 0x04) != 0)
        {
            code[2] |= 2;
        }

        if ((evenColumnCode & 0x04) != 0)
        {
            code[2] |= 1;
        }

        oddLineCode <<= 1;
        evenLineCode <<= 1;
        oddColumnCode <<= 1;
        evenColumnCode <<= 1;
    }

    // Invert codes (linux compatibility)
    code[0] = (~(rt_uint32_t)code[0]);
    code[1] = (~(rt_uint32_t)code[1]);
    code[2] = (~(rt_uint32_t)code[2]);
}

void ecc_hamming_compute256x(const rt_uint8_t *pucData, rt_uint32_t dwSize, rt_uint8_t *puCode)
{
    while (dwSize > 0)
    {
        Compute256(pucData, puCode) ;

        pucData += 256;
        puCode += 3;
        dwSize -= 256;
    }
}

/* read chip id */
static rt_uint32_t nanddrv_file_read_id(struct rt_mtd_nand_device *device)
{
    return 0x00;
}

/* read/write/move page */
static rt_err_t nanddrv_file_read_page(struct rt_mtd_nand_device *device,
    rt_off_t page,
    rt_uint8_t *data, rt_uint32_t data_len,
    rt_uint8_t *spare, rt_uint32_t spare_len)
{
    rt_uint32_t offset,code;
    rt_uint8_t data_ecc [PAGE_DATA_SIZE];
    rt_uint8_t oob_ecc [ECC_SIZE];
    rt_uint8_t ecc [ECC_SIZE];

    offset = page * PAGE_SIZE;
    SetFilePointer(file, offset, NULL, FILE_BEGIN);
    ReadFile(file, data_ecc, PAGE_DATA_SIZE, &code, 0);
    if (data && data_len)
        memcpy(data,data_ecc,data_len);
    ReadFile(file, spare, spare_len, &code, 0);

    /* verify ECC */
    ReadFile(file, oob_ecc, ECC_SIZE, &code, 0);
    ecc_hamming_compute256x(data_ecc, PAGE_DATA_SIZE, &ecc[0]);
    if (memcmp(&oob_ecc[0], &ecc[0], ECC_SIZE) != 0)
        return -RT_ERROR;

    return RT_EOK;
}

static rt_err_t nanddrv_file_write_page(struct rt_mtd_nand_device *device,
    rt_off_t page,
    const rt_uint8_t *data, rt_uint32_t data_len,
    const rt_uint8_t *oob, rt_uint32_t spare_len)
{
    rt_uint32_t offset,code;
    rt_uint8_t ecc[ECC_SIZE];

    /* write page */
    offset = page * PAGE_SIZE;
    SetFilePointer(file, offset, NULL, FILE_BEGIN);
    WriteFile(file, data, data_len, &code, 0);
    WriteFile(file, oob, spare_len, &code, 0);

    /* verify ECC */
    ecc_hamming_compute256x(data, PAGE_DATA_SIZE, ecc);
    WriteFile(file, ecc, ECC_SIZE, &code, 0);
    FlushFileBuffers(file);

    return RT_EOK;
}

static rt_err_t nanddrv_file_move_page(struct rt_mtd_nand_device *device, rt_off_t from, rt_off_t to)
{
    return -RT_EIO;
}

/* erase block */
static rt_err_t nanddrv_file_erase_block(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    rt_uint32_t code;

    SetFilePointer(file, block * BLOCK_SIZE, NULL, FILE_BEGIN);
    WriteFile(file, block_data, sizeof(block_data), &code, 0);

    return RT_EOK;
}

static rt_err_t nanddrv_file_check_block(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    rt_uint8_t state = 0;
    rt_uint32_t code;

    SetFilePointer(file, block * BLOCK_SIZE + PAGE_DATA_SIZE, NULL, FILE_BEGIN);
    ReadFile(file, &state, 1, &code, 0);

    return (state==0xff)?RT_EOK:-RT_EIO;
}

static rt_err_t nanddrv_file_mark_block(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    rt_uint8_t state = 0;
    rt_uint32_t code;

    SetFilePointer(file, block * BLOCK_SIZE + PAGE_DATA_SIZE, NULL, FILE_BEGIN);
    WriteFile(file, &state, 1, &code, 0);

    return RT_EOK;;
}

const static struct rt_mtd_nand_driver_ops _ops =
{
    nanddrv_file_read_id,
    nanddrv_file_read_page,
    nanddrv_file_write_page,
    nanddrv_file_move_page,
    nanddrv_file_erase_block,
    nanddrv_file_check_block,
    nanddrv_file_mark_block,
};

void nand_eraseall(void);
void nand_init(void){}

void rt_hw_mtd_nand_init(void)
{
    /* open file */
    memset(block_data, 0xff, sizeof(block_data));
    file = CreateFile("nand.bin",
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ,
        NULL,
        OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        0);
    if (file != INVALID_HANDLE_VALUE)
    {
        int i,page_addr = 0,code;
        int count = BLOCK_NUM;

        SetFilePointer(file, 0, NULL, FILE_END);
        SetFilePointer(file, 0, NULL, FILE_BEGIN);
        code = GetFileSize(file, NULL);

        if (code < count*PAGE_SIZE)
        {
            for (i=0; i<count; i++)
            {
                WriteFile(file, block_data, sizeof(block_data), &code, 0);
                page_addr += sizeof(block_data);
            }
            FlushFileBuffers(file);
        }
    }

    _nanddrv_file_device.plane_num = 2;
    _nanddrv_file_device.oob_size = OOB_SIZE;
    _nanddrv_file_device.oob_free = OOB_SIZE-2-ECC_SIZE;
    _nanddrv_file_device.page_size = PAGE_DATA_SIZE;
    _nanddrv_file_device.pages_per_block = PAGE_PER_BLOCK;
    _nanddrv_file_device.block_start = 160;
    _nanddrv_file_device.block_end = BLOCK_NUM / 2;
    _nanddrv_file_device.block_total = _nanddrv_file_device.block_end - _nanddrv_file_device.block_start;
    _nanddrv_file_device.ops = &_ops;

    rt_mtd_nand_register_device("nand0", &_nanddrv_file_device);
}

#if defined(RT_USING_FINSH)
#include <finsh.h>
void nand_eraseall()
{
    int index;
    for (index = _nanddrv_file_device.block_start; index < _nanddrv_file_device.block_end; index ++)
    {
        if (nanddrv_file_erase_block(&_nanddrv_file_device, index) != 0)
            rt_kprintf("Nand Erase %d BadBlock\n", index);
    }
}
FINSH_FUNCTION_EXPORT(nand_eraseall, erase all of block in the nand flash);
FINSH_FUNCTION_EXPORT_ALIAS(nand_eraseall, __cmd_nand_eraseall,  erase all of block in the nand flash)

#endif //RT_USING_FINSH
