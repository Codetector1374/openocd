// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

#include <helper/bits.h>

#define HT32F165X_FLASH_SIZE	0x10000
#define HT32F165X_PAGE_SIZE		1024

#define HT32F165X_FMC_BASE		0x40080000
#define HT32F165X_FMC_TADR		(HT32F165X_FMC_BASE + 0x00)
#define HT32F165X_FMC_WRDR		(HT32F165X_FMC_BASE + 0x04)
#define HT32F165X_FMC_OCMR		(HT32F165X_FMC_BASE + 0x0c)
#define HT32F165X_FMC_OPCR		(HT32F165X_FMC_BASE + 0x10)
#define HT32F165X_FMC_OISR		(HT32F165X_FMC_BASE + 0x18)
#define HT32F165X_FMC_PPSR0		(HT32F165X_FMC_BASE + 0x20)
#define HT32F165X_FMC_CPSR		(HT32F165X_FMC_BASE + 0x30)

#define HT32F165X_CMD_WORD_PROGRAM	0x4
#define HT32F165X_CMD_PAGE_ERASE	0x8
#define HT32F165X_CMD_MASS_ERASE	0xa

#define HT32F165X_WRITE_ALIGNMENT	4
#define HT32F165X_OPTION_BASE		0x1ff00000

#define HT32F165X_OPM_MASK		0x1e
#define HT32F165X_OPM_IDLE		0x0c
#define HT32F165X_OPM_COMMIT	0x14
#define HT32F165X_OPM_FINISHED	0x1c

#define HT32F165X_OISR_ORFF		BIT(0)
#define HT32F165X_OISR_ITADF	BIT(1)
#define HT32F165X_OISR_OBEF		BIT(2)
#define HT32F165X_OISR_IOCMF	BIT(3)
#define HT32F165X_OISR_OREF		BIT(4)
#define HT32F165X_OISR_PPEF		BIT(17)
#define HT32F165X_OISR_CLEAR_MASK \
	(HT32F165X_OISR_ORFF | HT32F165X_OISR_ITADF | HT32F165X_OISR_OBEF | \
	 HT32F165X_OISR_IOCMF | HT32F165X_OISR_OREF)

#define HT32F165X_CPSR_CPSB		BIT(0)
#define HT32F165X_CPSR_OBPSB	BIT(1)

#define HT32F165X_WORD_TIMEOUT_MS	10
#define HT32F165X_ERASE_TIMEOUT_MS	100

struct ht32f165x_flash_bank {
	bool probed;
	uint32_t cpsr;
};

static int ht32f165x_read_opcr(struct flash_bank *bank, uint32_t *opcr)
{
	return target_read_u32(bank->target, HT32F165X_FMC_OPCR, opcr);
}

static int ht32f165x_wait_ready(struct flash_bank *bank, int timeout_ms)
{
	uint32_t opcr;

	for (;;) {
		int retval = ht32f165x_read_opcr(bank, &opcr);
		if (retval != ERROR_OK)
			return retval;

		uint32_t opm = opcr & HT32F165X_OPM_MASK;
		if (opm == HT32F165X_OPM_IDLE || opm == HT32F165X_OPM_FINISHED)
			return ERROR_OK;

		if (timeout_ms-- <= 0) {
			LOG_ERROR("timed out waiting for HT32 flash ready, OPCR 0x%08" PRIx32, opcr);
			return ERROR_FLASH_BUSY;
		}

		alive_sleep(1);
	}
}

static int ht32f165x_wait_finished(struct flash_bank *bank, int timeout_ms)
{
	uint32_t opcr;

	for (;;) {
		int retval = ht32f165x_read_opcr(bank, &opcr);
		if (retval != ERROR_OK)
			return retval;

		if ((opcr & HT32F165X_OPM_MASK) == HT32F165X_OPM_FINISHED)
			return ERROR_OK;

		if (timeout_ms-- <= 0) {
			LOG_ERROR("timed out waiting for HT32 flash operation, OPCR 0x%08" PRIx32, opcr);
			return ERROR_FLASH_BUSY;
		}

		alive_sleep(1);
	}
}

static int ht32f165x_clear_oisr(struct flash_bank *bank)
{
	return target_write_u32(bank->target, HT32F165X_FMC_OISR, HT32F165X_OISR_CLEAR_MASK);
}

static int ht32f165x_check_oisr(struct flash_bank *bank)
{
	uint32_t oisr;

	int retval = target_read_u32(bank->target, HT32F165X_FMC_OISR, &oisr);
	if (retval != ERROR_OK)
		return retval;

	uint32_t clear = oisr & HT32F165X_OISR_CLEAR_MASK;
	if (clear) {
		retval = target_write_u32(bank->target, HT32F165X_FMC_OISR, clear);
		if (retval != ERROR_OK)
			return retval;
	}

	if (oisr & HT32F165X_OISR_PPEF) {
		LOG_ERROR("HT32 flash page is erase/program protected, OISR 0x%08" PRIx32, oisr);
		return ERROR_FLASH_PROTECTED;
	}

	if (oisr & HT32F165X_OISR_ITADF) {
		LOG_ERROR("HT32 flash invalid target address, OISR 0x%08" PRIx32, oisr);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (oisr & HT32F165X_OISR_IOCMF) {
		LOG_ERROR("HT32 flash invalid operation command, OISR 0x%08" PRIx32, oisr);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (oisr & HT32F165X_OISR_OBEF) {
		LOG_ERROR("HT32 flash option byte checksum error, OISR 0x%08" PRIx32, oisr);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (oisr & HT32F165X_OISR_OREF) {
		LOG_ERROR("HT32 flash operation failed, OISR 0x%08" PRIx32, oisr);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int ht32f165x_run_command(struct flash_bank *bank, uint32_t command,
		uint32_t timeout_ms)
{
	struct target *target = bank->target;

	int retval = ht32f165x_wait_ready(bank, timeout_ms);
	if (retval != ERROR_OK)
		return retval;

	retval = ht32f165x_clear_oisr(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, HT32F165X_FMC_OCMR, command);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, HT32F165X_FMC_OPCR, HT32F165X_OPM_COMMIT);
	if (retval != ERROR_OK)
		return retval;

	retval = ht32f165x_wait_finished(bank, timeout_ms);
	if (retval != ERROR_OK)
		return retval;

	return ht32f165x_check_oisr(bank);
}

FLASH_BANK_COMMAND_HANDLER(ht32f165x_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct ht32f165x_flash_bank *ht32f165x_info = calloc(1, sizeof(*ht32f165x_info));
	if (!ht32f165x_info)
		return ERROR_FAIL;

	bank->driver_priv = ht32f165x_info;
	bank->write_start_alignment = HT32F165X_WRITE_ALIGNMENT;
	bank->write_end_alignment = HT32F165X_WRITE_ALIGNMENT;

	return ERROR_OK;
}

static int ht32f165x_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct ht32f165x_flash_bank *ht32f165x_info = bank->driver_priv;
	uint32_t ppsr[4];

	for (unsigned int i = 0; i < ARRAY_SIZE(ppsr); i++) {
		int retval = target_read_u32(target, HT32F165X_FMC_PPSR0 + i * 4, &ppsr[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	int retval = target_read_u32(target, HT32F165X_FMC_CPSR, &ht32f165x_info->cpsr);
	if (retval != ERROR_OK)
		return retval;

	if (bank->num_sectors > ARRAY_SIZE(ppsr) * 32) {
		LOG_ERROR("HT32 flash protection status supports only %u pages, bank has %u",
				(unsigned int)(ARRAY_SIZE(ppsr) * 32), bank->num_sectors);
		return ERROR_FLASH_BANK_INVALID;
	}

	if ((ht32f165x_info->cpsr & HT32F165X_CPSR_CPSB) == 0)
		LOG_WARNING("HT32 flash security protection is enabled");

	if ((ht32f165x_info->cpsr & HT32F165X_CPSR_OBPSB) == 0)
		LOG_INFO("HT32 option byte erase/program protection is enabled");

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		unsigned int index = i / 32;
		unsigned int bit = i % 32;
		bool protected = (ppsr[index] & BIT(bit)) == 0;

		bank->sectors[i].is_protected = protected;
		if (bank->prot_blocks && i < bank->num_prot_blocks)
			bank->prot_blocks[i].is_protected = protected;
	}

	return ERROR_OK;
}

static int ht32f165x_probe(struct flash_bank *bank)
{
	struct ht32f165x_flash_bank *ht32f165x_info = bank->driver_priv;
	uint32_t flash_size = bank->size ? bank->size : HT32F165X_FLASH_SIZE;

	if (flash_size % HT32F165X_PAGE_SIZE) {
		LOG_ERROR("HT32 flash size 0x%08" PRIx32 " is not page aligned", flash_size);
		return ERROR_FLASH_BANK_INVALID;
	}

	free(bank->sectors);
	bank->sectors = NULL;
	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	bank->base = 0;
	bank->size = flash_size;
	bank->num_sectors = flash_size / HT32F165X_PAGE_SIZE;
	bank->sectors = alloc_block_array(0, HT32F165X_PAGE_SIZE, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	bank->num_prot_blocks = bank->num_sectors;
	bank->prot_blocks = alloc_block_array(0, HT32F165X_PAGE_SIZE, bank->num_prot_blocks);
	if (!bank->prot_blocks) {
		free(bank->sectors);
		bank->sectors = NULL;
		bank->num_sectors = 0;
		return ERROR_FAIL;
	}

	ht32f165x_info->probed = true;

	return ht32f165x_protect_check(bank);
}

static int ht32f165x_auto_probe(struct flash_bank *bank)
{
	struct ht32f165x_flash_bank *ht32f165x_info = bank->driver_priv;

	if (ht32f165x_info->probed)
		return ERROR_OK;

	return ht32f165x_probe(bank);
}

static int ht32f165x_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = ht32f165x_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = first; i <= last; i++) {
		retval = ht32f165x_wait_ready(bank, HT32F165X_ERASE_TIMEOUT_MS);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, HT32F165X_FMC_TADR,
				bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK)
			return retval;

		retval = ht32f165x_run_command(bank, HT32F165X_CMD_PAGE_ERASE,
				HT32F165X_ERASE_TIMEOUT_MS);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int ht32f165x_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int ht32f165x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("offset 0x%08" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x3) {
		LOG_ERROR("write count 0x%08" PRIx32 " breaks required 4-byte alignment", count);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	int retval = ht32f165x_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	for (uint32_t i = 0; i < count; i += 4) {
		retval = ht32f165x_wait_ready(bank, HT32F165X_WORD_TIMEOUT_MS);
		if (retval != ERROR_OK)
			return retval;

		uint32_t word = buffer[i] |
			((uint32_t)buffer[i + 1] << 8) |
			((uint32_t)buffer[i + 2] << 16) |
			((uint32_t)buffer[i + 3] << 24);

		retval = target_write_u32(target, HT32F165X_FMC_TADR, bank->base + offset + i);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, HT32F165X_FMC_WRDR, word);
		if (retval != ERROR_OK)
			return retval;

		retval = ht32f165x_run_command(bank, HT32F165X_CMD_WORD_PROGRAM,
				HT32F165X_WORD_TIMEOUT_MS);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int ht32f165x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = ht32f165x_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ht32f165x_run_command(bank, HT32F165X_CMD_MASS_ERASE,
			HT32F165X_ERASE_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;

	return ERROR_OK;
}

COMMAND_HANDLER(ht32f165x_handle_mass_erase_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ht32f165x_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "ht32f165x mass erase complete");
	else
		command_print(CMD, "ht32f165x mass erase failed");

	return retval;
}

static int ht32f165x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct ht32f165x_flash_bank *ht32f165x_info = bank->driver_priv;

	int retval = ht32f165x_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	command_print(cmd, "HT32F165x flash: %" PRIu32 " KiB, %" PRIu32 " byte pages",
			bank->size / 1024, (uint32_t)HT32F165X_PAGE_SIZE);
	command_print(cmd, "security protection: %s",
			(ht32f165x_info->cpsr & HT32F165X_CPSR_CPSB) ? "disabled" : "enabled");
	command_print(cmd, "option byte protection: %s",
			(ht32f165x_info->cpsr & HT32F165X_CPSR_OBPSB) ? "disabled" : "enabled");

	return ERROR_OK;
}

static const struct command_registration ht32f165x_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = ht32f165x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ht32f165x_command_handlers[] = {
	{
		.name = "ht32f165x",
		.mode = COMMAND_ANY,
		.help = "ht32f165x flash command group",
		.usage = "",
		.chain = ht32f165x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver ht32f165x_flash = {
	.name = "ht32f165x",
	.commands = ht32f165x_command_handlers,
	.flash_bank_command = ht32f165x_flash_bank_command,
	.erase = ht32f165x_erase,
	.protect = ht32f165x_protect,
	.write = ht32f165x_write,
	.read = default_flash_read,
	.probe = ht32f165x_probe,
	.auto_probe = ht32f165x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = ht32f165x_protect_check,
	.info = ht32f165x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
