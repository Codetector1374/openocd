#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "../common.h"
#include "../../target/target.h"
#include "../../helper/command.h"
#include "../../helper/log.h"

#include "imp.h"
#include <target/armv7m.h>

#define FLASH_BASE      0x40062000
#define FLASH_LPCTRL    (FLASH_BASE + 0x0)
#define FLASH_STATUS    (FLASH_BASE + 0x4)
#define FLASH_CTRL      (FLASH_BASE + 0x8)
#define FLASH_DATA      (FLASH_BASE + 0xC)
#define FLASH_ADDR      (FLASH_BASE + 0x10)
#define FLASH_CHKSUM    (FLASH_BASE + 0x14)

#define CODE_OPTION     0x1FFF2000
#define CS0             0
#define CS1             0x5A5A0000
#define CS2             0xA5A50000

#define PAGE_SIZE       64

#define FLASH_CTRL_PG       1
#define FLASH_CTRL_PER      1 << 1
#define FLASH_CTRL_START    1 << 6

// flash bank sn32f260 <base> <size> 0 0 <target#>
FLASH_BANK_COMMAND_HANDLER(sn32f260_flash_bank_command)
{
    if (CMD_ARGC < 6)
        return ERROR_COMMAND_SYNTAX_ERROR;

    bank->driver_priv = NULL;

    return ERROR_OK;
}

static int sn32f260_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
    struct target *target = bank->target;
    return target_read_u32(target, FLASH_STATUS, status);
}

static int sn32f260_wait_status_busy(struct flash_bank *bank, int timeout)
{
    struct target *target = bank->target;
    uint32_t status;

    // wait for flash operation finished
    for (;;) {
        const int retval = sn32f260_get_flash_status(bank, &status);
        if (retval == ERROR_OK) {
            if (status & 0x4) { // ERR FLAG
                target_write_u32(target, FLASH_STATUS, 0x4);  // Clear Error
                LOG_ERROR("FMC returned ERROR bit");
                return ERROR_FAIL;
            }

            if ((status & 1) == 0) {
                return ERROR_OK;
            }
        }

        if (timeout-- <= 0) {
            LOG_ERROR("Timed out waiting for flash: 0x%04x", status);
            return ERROR_FAIL;
        }
        alive_sleep(10);
    }
}

static int sn32f260_erase(struct flash_bank *bank, unsigned first, unsigned last)
{
    struct target *target = bank->target;

    LOG_DEBUG("sn32f260 erase: %d - %d", first, last);

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    for(unsigned i = first ; i <= last; ++i){
        // flash memory page erase
        int retval;
        retval = target_write_u32(target, FLASH_CTRL, FLASH_CTRL_PER);
        if (retval != ERROR_OK)
            return retval;
        retval = target_write_u32(target, FLASH_ADDR, bank->sectors[i].offset);
        if (retval != ERROR_OK)
            return retval;
        retval = target_write_u32(target, FLASH_CTRL, FLASH_CTRL_PER | FLASH_CTRL_START);
        if (retval != ERROR_OK)
            return retval;

        // wait
        retval = sn32f260_wait_status_busy(bank, 100);
        if (retval != ERROR_OK)
            return retval;

        LOG_DEBUG("sn32f260 erased page %d", i);
        bank->sectors[i].is_erased = 1;
    }

    return ERROR_OK;
}

static int sn32f260_protect(struct flash_bank *bank, int set,
    unsigned first, unsigned last)
{
    return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int sn32f260_write(struct flash_bank *bank, const uint8_t *buffer,
                           uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;

    LOG_DEBUG("sn32f260 flash write: 0x%x 0x%x", offset, count);

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if(offset & 0x3){
        LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
        return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
    }

    if(count & 0x3){
        LOG_ERROR("size 0x%" PRIx32 " breaks required 4-byte alignment", count);
        return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
    }

    uint32_t addr = offset;
    for(uint32_t i = 0; i < count; i += 4){
        uint32_t word = (((uint32_t)buffer[i+0]) <<  0U) |
                        (((uint32_t)buffer[i+1]) <<  8U) |
                        (((uint32_t)buffer[i+2]) << 16U) |
                        (((uint32_t)buffer[i+3]) << 24U) ;

        LOG_DEBUG("sn32f260 flash write word 0x%x 0x%x 0x%08x", i, addr, word);

        // flash memory word program
        int retval;
        retval = target_write_u32(target, FLASH_CTRL, FLASH_CTRL_PG);
        if (retval != ERROR_OK)
            return retval;
        retval = target_write_u32(target, FLASH_ADDR, addr);
        if (retval != ERROR_OK)
            return retval;
        retval = target_write_u32(target, FLASH_DATA, word);
        if (retval != ERROR_OK)
            return retval;
        retval = target_write_u32(target, FLASH_CTRL, FLASH_CTRL_START | FLASH_CTRL_PG);
        if (retval != ERROR_OK)
            return retval;

        // wait
        retval = sn32f260_wait_status_busy(bank, 500);
        if (retval != ERROR_OK)
            return retval;
        addr += 4;
    }

    LOG_DEBUG("sn32f260 flash write success");
    return ERROR_OK;
}

static int sn32f260_protect_check(struct flash_bank *bank);

static int sn32f260_probe(struct flash_bank *bank)
{
    int num_pages = bank->size / PAGE_SIZE;

    LOG_INFO("sn32f260 probe: %d pages, 0x%x bytes, 0x%x total", num_pages, PAGE_SIZE, bank->size);

    if(bank->sectors)
        free(bank->sectors);

    bank->base = 0x0;
    bank->num_sectors = num_pages;
    bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

    for(int i = 0; i < num_pages; ++i){
        bank->sectors[i].offset = i * PAGE_SIZE;
        bank->sectors[i].size = PAGE_SIZE;
        bank->sectors[i].is_erased = -1;
        bank->sectors[i].is_protected = 1;
    }

    sn32f260_protect_check(bank);

    return ERROR_OK;
}

static int sn32f260_auto_probe(struct flash_bank *bank)
{
    return sn32f260_probe(bank);
}

static int sn32f260_protect_check(struct flash_bank *bank)
{
    struct target *target = bank->target;
    uint32_t code_security;

    // Read page protection
    target_read_u32(target, CODE_OPTION, &code_security);

    LOG_INFO("sn32f260 CS: %04x", code_security);


    for (size_t i = 0; i < (size_t)bank->num_sectors; ++i) {
        bank->sectors[i].is_protected = (code_security != 0);
    }

    return ERROR_OK;
}

static int sn32f260_info(struct flash_bank *bank, char *buf, int buf_size)
{
    struct target *target = bank->target;
    sn32f260_probe(bank);

    const char *info = "sn32f260 flash (CS0)";
    const size_t info_len = strlen(info);
    const size_t copy_size = MIN((int)info_len + 1, buf_size);
    memcpy(buf, info, copy_size);

    // Read page protection
    uint32_t code_security;
    target_read_u32(target, CODE_OPTION, &code_security);

    if (copy_size > 18) {
        if ((code_security & 0xFFFF0000) == 0) {
            buf[18] = '0';
        } else if ((code_security & 0xFFFF0000) == CS1) {
            buf[18] = '1';
        } else {
            buf[18] = '2';
        }
    }

    buf[copy_size-1] = '\0';
    return ERROR_OK;
}

static int sn32f260_mass_erase(struct flash_bank *bank)
{
    struct target *target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    uint32_t code_security;
    target_read_u32(target, CODE_OPTION, &code_security);
    int retval;
    if ((code_security & 0xFFFF0000) == 0) {
        // Set Code Security First
        target_write_u32(target, CODE_OPTION, CS1);
        alive_sleep(300);
        retval = sn32f260_wait_status_busy(bank, 1000);
        if (retval != ERROR_OK)
            return retval;
    }

    target_write_u32(target, CODE_OPTION, CS0);
    alive_sleep(300);
    retval = sn32f260_wait_status_busy(bank, 1000);
    if (retval != ERROR_OK)
        return retval;


    return ERROR_OK;
}

COMMAND_HANDLER(sn32f260_handle_mass_erase_command)
{
    if (CMD_ARGC < 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    retval = sn32f260_mass_erase(bank);
    if (retval == ERROR_OK) {
        // set all sectors as erased
        for (unsigned i = 0; i < bank->num_sectors; i++)
            bank->sectors[i].is_erased = 1;

        command_print(CMD, "sn32f260 mass erase complete");
    } else {
        command_print(CMD, "sn32f260 mass erase failed");
    }

    return retval;
}

COMMAND_HANDLER(sn32f260_handle_test_write)
{
    if (CMD_ARGC < 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    uint8_t buffer[32];
    for(int i = 0; i < 32; ++i){
        buffer[i] = i;
    }

    retval = sn32f260_erase(bank, 0, 0);
    if (retval != ERROR_OK)
        return retval;

    retval = sn32f260_write(bank, buffer, 0, 32);
    if (retval == ERROR_OK) {
        command_print(CMD, "sn32f260 test write complete");
    } else {
        command_print(CMD, "sn32f260 test write failed");
    }

    return retval;
}


static const struct command_registration sn32f260_exec_command_handlers[] = {
    {
        .name = "mass_erase",
        .handler = sn32f260_handle_mass_erase_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "erase entire flash device",
    },
    {
        .name = "test_write",
        .handler = sn32f260_handle_test_write,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = " test flash write",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration sn32f260_command_handlers[] = {
    {
        .name = "sn32f260",
        .mode = COMMAND_ANY,
        .help = "sn32f260 flash command group",
        .usage = "",
        .chain = sn32f260_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

struct flash_driver sn32f260_flash = {
    .name = "sn32f260",
    .commands = sn32f260_command_handlers,
    .flash_bank_command = sn32f260_flash_bank_command,

    .erase          = sn32f260_erase,
    .protect        = sn32f260_protect,
    .write          = sn32f260_write,
    .read           = default_flash_read,
    .probe          = sn32f260_probe,
    .auto_probe     = sn32f260_auto_probe,
    .erase_check    = default_flash_blank_check,
    .protect_check  = sn32f260_protect_check,
    .info           = sn32f260_info,
};
