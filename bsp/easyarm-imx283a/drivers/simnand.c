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
#define BLOCK_NUM       1024

#define BLOCK_SIZE      (PAGE_SIZE * PAGE_PER_BLOCK)
#define PAGE_SIZE       (PAGE_DATA_SIZE + OOB_SIZE)

static unsigned char block_data[BLOCK_SIZE];
static struct rt_mtd_nand_device _nanddrv_file_device;
static HANDLE file = INVALID_HANDLE_VALUE;

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
    page = page + device->block_start * device->pages_per_block;

    if (page / device->pages_per_block > device->block_end)
    {
        return -RT_EIO;
    }

    /* write page */
    if (data != NULL && data_len != 0)
    {
        offset = page * PAGE_SIZE;
        SetFilePointer(file, offset, NULL, FILE_BEGIN);
        ReadFile(file, data, data_len, &code, 0);
    }

    if (spare != NULL && spare_len)
    {
        offset = page * PAGE_SIZE + PAGE_DATA_SIZE;
        SetFilePointer(file, offset, NULL, FILE_BEGIN);
        ReadFile(file, spare, spare_len, &code, 0);
    }

    return RT_EOK;
}

static rt_err_t nanddrv_file_write_page(struct rt_mtd_nand_device *device,
    rt_off_t page,
    const rt_uint8_t *data, rt_uint32_t data_len,
    const rt_uint8_t *oob, rt_uint32_t spare_len)
{
    rt_uint32_t offset,code;

    page = page + device->block_start * device->pages_per_block;
    if (page / device->pages_per_block > device->block_end)
    {
        return -RT_EIO;
    }

    /* write page */
    if (data != RT_NULL && data_len != 0)
    {
        offset = page * PAGE_SIZE;
        SetFilePointer(file, offset, NULL, FILE_BEGIN);
        WriteFile(file, data, data_len, &code, 0);
    }

    if (oob != RT_NULL && spare_len != 0)
    {
        offset = page * PAGE_SIZE + PAGE_DATA_SIZE;
        SetFilePointer(file, offset, NULL, FILE_BEGIN);
        WriteFile(file, oob, spare_len, &code, 0);
    }
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

    SetFilePointer(file, block * BLOCK_SIZE, NULL, FILE_BEGIN);
    ReadFile(file, &state, 1, &code, 0);

    return (state==0xff)?RT_EOK:-RT_EIO;
}

static rt_err_t nanddrv_file_mark_block(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    rt_uint8_t state = 0;
    rt_uint32_t code;

    SetFilePointer(file, block * BLOCK_SIZE, NULL, FILE_BEGIN);
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
    _nanddrv_file_device.oob_free = 20;
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
