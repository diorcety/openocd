
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "helper/binarybuffer.h"
#include "helper/time_support.h"

#include <target/cortex_m.h>

#define SAML_L10_NUM_PROT_REGIONS	3
#define SAML_L11_NUM_PROT_REGIONS	6
#define SAML_PAGE_SIZE_MAX	1024

#define SAML_DATA_FLASH		((uint32_t)0x00400000)	/* physical Data Flash memory */
#define SAML_FLASH			((uint32_t)0x00000000)	/* physical Flash memory */
#define SAML_USER_ROW		((uint32_t)0x00804000)	/* User Row of Flash */
#define SAML_BOCOR			((uint32_t)0x0080C000)	/* NVM Boot Configuration Row */
#define SAML_PAC1			0x41000000	/* Peripheral Access Control 1 */
#define SAML_DSU			0x41002000	/* Device Service Unit */
#define SAML_NVMCTRL		0x41004000	/* Non-volatile memory controller */

#define SAML_DSU_STATUSA        1               /* DSU status register */
#define SAML_DSU_DID		0x18		/* Device ID register */
#define SAML_DSU_CTRL_EXT	0x100		/* CTRL register, external access */

#define SAML_NVMCTRL_CTRLA		0x00	/* NVM control A register */
#define SAML_NVMCTRL_CTRLB		0x04	/* NVM control B register */
#define SAML_NVMCTRL_CTRLC		0x08	/* NVM control B register */
#define SAML_NVMCTRL_EVCTRL		0x0A	/* NVM control B register */
#define SAML_NVMCTRL_INTENCLR	0x0C	/* NVM Interrupt Flag Status & Clear */
#define SAML_NVMCTRL_INTENSET	0x10	/* NVM Interrupt Flag Status & Clear */
#define SAML_NVMCTRL_INTFLAG	0x14	/* NVM Interrupt Flag Status & Clear */
#define SAML_NVMCTRL_STATUS		0x18	/* NVM status register */
#define SAML_NVMCTRL_ADDR		0x1C	/* NVM address register */
#define SAML_NVMCTRL_SULCK		0x20	/* NVM address register */
#define SAML_NVMCTRL_NSULCK		0x22	/* NVM address register */
#define SAML_NVMCTRL_PARAM		0x24	/* NVM parameters register */

#define SAML_NVMCTRL_SULCK_BS	(1u << 0)
#define SAML_NVMCTRL_SULCK_AS	(1u << 1)
#define SAML_NVMCTRL_SULCK_DS	(1u << 2)

#define SAML_L10_BOCOR_BOOTPROT	0x04
#define SAML_L10_BOCOR_CRCKEY	0x40

#define SAML_L11_BOCOR_BS		0x01
#define SAML_L11_BOCOR_BNSC		0x02
#define SAML_L11_BOCOR_BOOTOPT	0x03
#define SAML_L11_BOCOR_BOOTPROT	0x04
#define SAML_L11_BOCOR_BCR		0x06
#define SAML_L11_BOCOR_BOCORCRC	0x08
#define SAML_L11_BOCOR_CEKEY0	0x10
#define SAML_L11_BOCOR_CEKEY1	0x20
#define SAML_L11_BOCOR_CEKEY2	0x30
#define SAML_L11_BOCOR_CRCKEY	0x40
#define SAML_L11_BOCOR_BOOTKEY	0x50
#define SAML_L11_BOCOR_BOCORHASH	0xE0

#define SAML_L11_UROW_AS	0x08
#define SAML_L11_UROW_ANSC	0x09
#define SAML_L11_UROW_DS	0x0A

#define SAML_CMDEX_KEY		0xA5UL
#define SAML_NVM_CMD(n)		((SAML_CMDEX_KEY << 8) | (n & 0x7F))

/* NVMCTRL commands.  See Table 20-4 in 42129F–SAM–10/2013 */
#define SAML_NVM_CMD_ER		0x02		/* Erase Row */
#define SAML_NVM_CMD_WP		0x04		/* Write Page */
#define SAML_NVM_CMD_EAR	0x05		/* Erase Auxiliary Row */
#define SAML_NVM_CMD_WAP	0x06		/* Write Auxiliary Page */
#define SAML_NVM_CMD_LR		0x40		/* Lock Region */
#define SAML_NVM_CMD_UR		0x41		/* Unlock Region */
#define SAML_NVM_CMD_SPRM	0x42		/* Set Power Reduction Mode */
#define SAML_NVM_CMD_CPRM	0x43		/* Clear Power Reduction Mode */
#define SAML_NVM_CMD_PBC	0x44		/* Page Buffer Clear */
#define SAML_NVM_CMD_SSB	0x45		/* Set Security Bit */
#define SAML_NVM_CMD_INVALL	0x46		/* Invalidate all caches */

#define SAML_NVMCTL_ADDR_ARRAY_FLASH	(0u << 22)
#define SAML_NVMCTL_ADDR_ARRAY_DATA		(1u << 22)
#define SAML_NVMCTL_ADDR_ARRAY_NVM		(2u << 22)

/* NVMCTRL bits */
#define SAML_NVMCTL_CTRLC_MANW		(1u << 0)
#define SAML_NVMCTL_INTFLAG_DONE	(1u << 0)
#define SAML_NVMCTL_INTFLAG_PROGE	(1u << 1)
#define SAML_NVMCTL_INTFLAG_LOCKE	(1u << 2)
#define SAML_NVMCTL_INTFLAG_NVME	(1u << 3)
#define SAML_NVMCTL_INTFLAG_KEYE	(1u << 4)
#define SAML_NVMCTL_INTFLAG_NSCHK	(1u << 5)

/* Known identifiers */
#define SAML_PROCESSOR_M23	0x02
#define SAML_FAMILY_L		0x01
#define SAML_SERIES_10		0x04
#define SAML_SERIES_11		0x03

/* Device ID macros */
#define SAML_GET_PROCESSOR(id) (id >> 28)
#define SAML_GET_FAMILY(id) (((id >> 23) & 0x1F))
#define SAML_GET_SERIES(id) (((id >> 16) & 0x3F))
#define SAML_GET_DEVSEL(id) (id & 0xFF)

/* Bits to mask out lockbits in user row */
#define NVMUSERROW_LOCKBIT_MASK ((uint64_t)0x0000FFFFFFFFFFFF)

struct saml_part {
	uint8_t id;
	const char *name;
	uint32_t flash_kb;
	uint32_t ram_kb;
	uint32_t data_flash_kb;
};

static const struct saml_part saml10_parts[] = {
	{ 0x00, "SAML10E16A", 64, 16, 2 },
	{ 0x01, "SAML10E15A", 32, 8, 2 },
	{ 0x02, "SAML10E14A", 16, 4, 2 },
	{ 0x03, "SAML10D16A", 64, 16, 2 },
	{ 0x04, "SAML10D15A", 32, 8, 2 },
	{ 0x05, "SAML10D14A", 16, 4, 2 },
};

static const struct saml_part saml11_parts[] = {
	{ 0x00, "SAML11E16A", 64, 16, 2 },
	{ 0x01, "SAML11E15A", 32, 8, 2 },
	{ 0x02, "SAML11E14A", 16, 8, 2 },
	{ 0x03, "SAML11D16A", 64, 16, 2 },
	{ 0x04, "SAML11D15A", 32, 8, 2 },
	{ 0x05, "SAML11D14A", 16, 8, 2 },
};


/* Each family of parts contains a parts table in the DEVSEL field of DID.  The
 * processor ID, family ID, and series ID are used to determine which exact
 * family this is and then we can use the corresponding table. */
struct saml_family {
	uint8_t processor;
	uint8_t family;
	uint8_t series;
	const struct saml_part *parts;
	size_t num_parts;
	uint64_t nvm_userrow_res_mask; /* protect bits which are reserved, 0 -> protect */
};

/* Known SAMD families */
static const struct saml_family saml_families[] = {
	{ SAML_PROCESSOR_M23, SAML_FAMILY_L, SAML_SERIES_10,
		saml10_parts, ARRAY_SIZE(saml10_parts),
		(uint64_t)0xFFFF03FFFC01FF77 },
	{ SAML_PROCESSOR_M23, SAML_FAMILY_L, SAML_SERIES_11,
		saml11_parts, ARRAY_SIZE(saml11_parts),
		(uint64_t)0xFFFF03FFFC01FF77 },
};

struct saml_info {
	uint32_t id;
	uint32_t page_size;
	int num_pages;
	uint32_t sector_size;

	bool probed;
	struct target *target;
};


/**
 * Gives the family structure to specific device id.
 * @param id The id of the device.
 * @return On failure NULL, otherwise a pointer to the structure.
 */
static const struct saml_family *saml_find_family(uint32_t id)
{
	uint8_t processor = SAML_GET_PROCESSOR(id);
	uint8_t family = SAML_GET_FAMILY(id);
	uint8_t series = SAML_GET_SERIES(id);

	for (unsigned i = 0; i < ARRAY_SIZE(saml_families); i++) {
		if (saml_families[i].processor == processor &&
			saml_families[i].series == series &&
			saml_families[i].family == family)
			return &saml_families[i];
	}

	return NULL;
}

/**
 * Gives the part structure to specific device id.
 * @param id The id of the device.
 * @return On failure NULL, otherwise a pointer to the structure.
 */
static const struct saml_part *saml_find_part(uint32_t id)
{
	uint8_t devsel = SAML_GET_DEVSEL(id);
	const struct saml_family *family = saml_find_family(id);
	if (family == NULL)
		return NULL;

	for (unsigned i = 0; i < family->num_parts; i++) {
		if (family->parts[i].id == devsel)
			return &family->parts[i];
	}

	return NULL;
}

static int saml_protect_check(struct flash_bank *bank)
{
	int res;
	uint16_t slock, nslock; /* Lock bits are active-low */
	uint8_t prot_size;
	struct saml_info *chip = (struct saml_info *)bank->driver_priv;
	const struct saml_part *part = saml_find_part(chip->id);

	/* Read the non-secure lock bits regargless of series */
	res = target_read_u16(bank->target,
			SAML_NVMCTRL + SAML_NVMCTRL_NSULCK, &nslock);
	if (res != ERROR_OK) {
		return res;
	}

	LOG_DEBUG("Region Offset     Size       Protected");
	LOG_DEBUG("------ ---------- ---------- ---------");
	if (SAML_GET_SERIES(chip->id) == SAML_SERIES_10) {
		/* Read protection region size from BOCOR register */
		target_read_u8(chip->target, SAML_BOCOR + SAML_L10_BOCOR_BOOTPROT,
					   &prot_size);

		bank->prot_blocks[0].offset = 0;
		bank->prot_blocks[0].size = prot_size * chip->sector_size;
		bank->prot_blocks[0].is_protected = !(nslock & SAML_NVMCTRL_SULCK_BS);
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "BNS", 
				bank->prot_blocks[0].offset, 
				bank->prot_blocks[0].size,
				bank->prot_blocks[0].is_protected ? "Yes" : "No");

		bank->prot_blocks[1].offset = bank->prot_blocks[0].size;
		bank->prot_blocks[1].size = bank->size - bank->prot_blocks[1].offset;
		bank->prot_blocks[1].is_protected = !(nslock & SAML_NVMCTRL_SULCK_AS);
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "ANS", 
				bank->prot_blocks[1].offset, 
				bank->prot_blocks[1].size,
				bank->prot_blocks[1].is_protected ? "Yes" : "No");

		bank->prot_blocks[2].offset = SAML_DATA_FLASH;
		bank->prot_blocks[2].size = part->data_flash_kb * 1024;
		bank->prot_blocks[2].is_protected = !(nslock & SAML_NVMCTRL_SULCK_DS);
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "DNS", 
				bank->prot_blocks[2].offset, 
				bank->prot_blocks[2].size,
				bank->prot_blocks[2].is_protected ? "Yes" : "No");
	}

	if (SAML_GET_SERIES(chip->id) == SAML_SERIES_11) {
		/* Read the secure lock bits */
		res = target_read_u16(bank->target,
				SAML_NVMCTRL + SAML_NVMCTRL_SULCK, &slock);
		if (res != ERROR_OK) {
			return res;
		}

		/* secure boot protection size is BS * row size */
		target_read_u8(chip->target, SAML_BOCOR + SAML_L11_BOCOR_BS, 
				&prot_size);
		bank->prot_blocks[0].offset = 0;
		bank->prot_blocks[0].size = prot_size * chip->sector_size;
		bank->prot_blocks[0].is_protected = !(slock & SAML_NVMCTRL_SULCK_BS);
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "BS", 
				bank->prot_blocks[0].offset, 
				bank->prot_blocks[0].size,
				bank->prot_blocks[0].is_protected ? "Yes" : "No");

		/* non-secure boot protection size is BOOTPROT - BS */
		target_read_u8(chip->target, SAML_BOCOR + SAML_L11_BOCOR_BOOTPROT,
				&prot_size);
		bank->prot_blocks[1].offset = bank->prot_blocks[0].size;
		bank->prot_blocks[1].size = bank->prot_blocks[0].size
			- (prot_size * chip->sector_size);
		bank->prot_blocks[1].is_protected = !(nslock & SAML_NVMCTRL_SULCK_BS);
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "BNS", 
				bank->prot_blocks[1].offset, 
				bank->prot_blocks[1].size,
				bank->prot_blocks[1].is_protected ? "Yes" : "No");
		
		/* secure app protection region size is AS * row size */
		/* Set up secure application flash offset as just after BOOTPROT. */
		bank->prot_blocks[2].offset = prot_size * chip->sector_size;
		target_read_u8(chip->target, SAML_USER_ROW + SAML_L11_UROW_AS, 
				&prot_size);
		bank->prot_blocks[2].size = prot_size * chip->sector_size;
		bank->prot_blocks[2].is_protected = !(slock & SAML_NVMCTRL_SULCK_AS);
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "AS", 
				bank->prot_blocks[2].offset, 
				bank->prot_blocks[2].size,
				bank->prot_blocks[2].is_protected ? "Yes" : "No");
		
		/* Remaining flash size is used in non-secure application region */
		bank->prot_blocks[3].offset = bank->prot_blocks[2].offset
			+ bank->prot_blocks[2].size;
		bank->prot_blocks[3].size = bank->size - bank->prot_blocks[3].offset;
		bank->prot_blocks[3].is_protected = !(nslock & SAML_NVMCTRL_SULCK_AS);
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "ANS", 
				bank->prot_blocks[3].offset, 
				bank->prot_blocks[3].size,
				bank->prot_blocks[3].is_protected ? "Yes" : "No");
		
		/* secure data region protection size is DS * row size. */
		target_read_u8(chip->target, SAML_USER_ROW + SAML_L11_UROW_DS, 
				&prot_size);
		bank->prot_blocks[4].offset = SAML_DATA_FLASH;
		bank->prot_blocks[4].size = (prot_size & 0x0F) * chip->sector_size;
		bank->prot_blocks[4].is_protected = !(slock & SAML_NVMCTRL_SULCK_DS);
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "DS", 
				bank->prot_blocks[4].offset, 
				bank->prot_blocks[4].size,
				bank->prot_blocks[4].is_protected ? "Yes" : "No");

		/* non-secure data region protection size is remaining space in data 
		 * region.
		 */
		bank->prot_blocks[5].offset = bank->prot_blocks[4].offset
			+ bank->prot_blocks[4].size;
		bank->prot_blocks[5].size = (part->data_flash_kb * 1024)
			- bank->prot_blocks[4].size;
		bank->prot_blocks[5].is_protected = !(nslock & SAML_NVMCTRL_SULCK_DS);			
		LOG_DEBUG("%-6s 0x%08X 0x%08X %-s", "DNS", 
				bank->prot_blocks[5].offset, 
				bank->prot_blocks[5].size,
				bank->prot_blocks[5].is_protected ? "Yes" : "No");
	}

	return ERROR_OK;
}

static int saml_get_flash_page_info(struct target *target,
		uint32_t *sizep, int *nump)
{
	int res;
	uint32_t param;

	res = target_read_u32(target, SAML_NVMCTRL + SAML_NVMCTRL_PARAM, &param);
	if (res == ERROR_OK) {
		/* The PSZ field (bits 18:16) indicate the page size bytes as 2^(3+n)
		 * so 0 is 8KB and 7 is 1024KB. */
		if (sizep)
			*sizep = (8 << ((param >> 16) & 0x7));
		/* The NVMP field (bits 15:0) indicates the total number of pages */
		if (nump)
			*nump = param & 0xFFFF;
	} else {
		LOG_ERROR("Couldn't read NVM Parameters register");
	}

	return res;
}

static int saml_probe(struct flash_bank *bank)
{
	int res;
	struct saml_info *chip = (struct saml_info *)bank->driver_priv;
	const struct saml_part *part;

	if (chip->probed)
		return ERROR_OK;

	res = target_read_u32(bank->target, SAML_DSU + SAML_DSU_DID, &chip->id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Device ID register");
		return res;
	}

	part = saml_find_part(chip->id);
	if (part == NULL) {
		LOG_ERROR("Couldn't find part corresponding to DID %08" PRIx32, chip->id);
		return ERROR_FAIL;
	}

	bank->size = part->flash_kb * 1024;

	res = saml_get_flash_page_info(bank->target, &chip->page_size,
			&chip->num_pages);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	LOG_INFO("Flash page size: %dB", chip->page_size);
	LOG_INFO("Flash pages: %d", chip->num_pages);
	LOG_INFO("Flash size %dB", bank->size);

	/* Sanity check: the total flash size in the DSU should match the page size
	 * multiplied by the number of pages. */
	if (bank->size != chip->num_pages * chip->page_size) {
		LOG_WARNING("SAML: bank size doesn't match NVM parameters. "
				"Identified %" PRIu32 "KB Flash but NVMCTRL reports %u %" PRIu32 "B pages",
				part->flash_kb, chip->num_pages, chip->page_size);
	}

	/* Erase granularity = 1 row = 4 pages */
	chip->sector_size = chip->page_size * 4;

	/* Allocate the sector table */
	bank->num_sectors = chip->num_pages / 4;
	LOG_INFO("Bank sectors: %d", bank->num_sectors);
	bank->sectors = alloc_block_array(0, chip->sector_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* Read protection size from BOCOR register */
	if (SAML_GET_SERIES(chip->id) == SAML_SERIES_10) {
		bank->num_prot_blocks = SAML_L10_NUM_PROT_REGIONS;
	}

	if (SAML_GET_SERIES(chip->id) == SAML_SERIES_11) {
		bank->num_prot_blocks = SAML_L11_NUM_PROT_REGIONS;
	}

	/* Allocate the table of protection blocks 
	 * Note: protection region sizes and offsets are updated in 
	 * saml_protect_check().
	 */
	bank->prot_blocks = alloc_block_array(0, 0, bank->num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	saml_protect_check(bank);

	/* Done */
	chip->probed = true;

	LOG_INFO("SAML MCU: %s (%" PRIu32 "KB Flash, %" PRIu32 "KB RAM)", part->name,
			part->flash_kb, part->ram_kb);

	return ERROR_OK;
}

static int saml_check_error(struct target *target)
{
	int ret, ret2;
	uint8_t status;

	ret = target_read_u8(target,
			SAML_NVMCTRL + SAML_NVMCTRL_INTFLAG, &status);
	if (ret != ERROR_OK) {
		LOG_ERROR("Can't read NVM status");
		return ret;
	}

	/* DONE flag is also included so it also gets cleared at the end of this 
	 * function. */
	if ((status & 0x001F) == 0)
		return ERROR_OK;

	if (status & SAML_NVMCTL_INTFLAG_NVME) {
		LOG_ERROR("SAML: NVM Error");
		ret = ERROR_FLASH_OPERATION_FAILED;
	}

	if (status & SAML_NVMCTL_INTFLAG_LOCKE) {
		LOG_ERROR("SAML: NVM lock error");
		ret = ERROR_FLASH_PROTECTED;
	}

	if (status & SAML_NVMCTL_INTFLAG_PROGE) {
		LOG_ERROR("SAML: NVM programming error");
		ret = ERROR_FLASH_OPER_UNSUPPORTED;
	}

	/* Clear the error conditions by writing a one to them */
	ret2 = target_write_u16(target,
			SAML_NVMCTRL + SAML_NVMCTRL_INTFLAG, status);
	if (ret2 != ERROR_OK)
		LOG_ERROR("Can't clear NVM error conditions");

	return ret;
}

static int saml_check_flash_done(struct target *target, unsigned int timeout_ms)
{
	uint8_t flags = 0;

	int64_t endtime = timeval_ms() + timeout_ms;
	while (1) {
		target_read_u8(target, SAML_NVMCTRL + SAML_NVMCTRL_INTFLAG, &flags);
		if (flags & SAML_NVMCTL_INTFLAG_DONE)
			break;
		alive_sleep(1);
		if (timeval_ms() >= endtime)
			break;
	}

	return saml_check_error(target);
}

static int saml_issue_nvmctrl_command(struct target *target, uint16_t cmd)
{
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Issue the NVM command */
	res = target_write_u16(target,
			SAML_NVMCTRL + SAML_NVMCTRL_CTRLA, SAML_NVM_CMD(cmd));
	if (res != ERROR_OK)
		return res;

	/* Wait up to 10ms for the erase operation to complete */
	return saml_check_flash_done(target, 100);
}

/**
 * Erases a flash-row at the given address.
 * @param target Pointer to the target structure.
 * @param address The address of the row.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int saml_erase_row(struct target *target, uint32_t address)
{
	int res;

	/* Set an address contained in the row to be erased */
	res = target_write_u32(target,
			SAML_NVMCTRL + SAML_NVMCTRL_ADDR, address & 0x00C0FFFF);

	/* Issue the Erase Row command to erase that row. */
	if (res == ERROR_OK) {
		res = saml_issue_nvmctrl_command(target, SAML_NVM_CMD_ER);
	}

	if (res != ERROR_OK)  {
		LOG_ERROR("Failed to erase row containing address 0x%08" PRIX32, address);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/**
 * Returns the bitmask of reserved bits in register.
 * @param target Pointer to the target structure.
 * @param mask Bitmask, 0 -> value stays untouched.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int saml_get_reservedmask(struct target *target, uint64_t *mask)
{
	int res;
	/* Get the devicetype */
	uint32_t id;
	res = target_read_u32(target, SAML_DSU + SAML_DSU_DID, &id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Device ID register");
		return res;
	}
	const struct saml_family *family;
	family = saml_find_family(id);
	if (family == NULL) {
		LOG_ERROR("Couldn't determine device family");
		return ERROR_FAIL;
	}
	*mask = family->nvm_userrow_res_mask;
	return ERROR_OK;
}

static int read_userrow(struct target *target, uint64_t *userrow)
{
	int res;
	uint8_t buffer[8];

	res = target_read_memory(target, SAML_USER_ROW, 4, 2, buffer);
	if (res != ERROR_OK)
		return res;

	*userrow = target_buffer_get_u64(target, buffer);
	return ERROR_OK;
}

/**
 * Modify the contents of the User Row in Flash. The User Row itself
 * has a size of one page and contains a combination of "fuses" and
 * calibration data. Bits which have a value of zero in the mask will
 * not be changed. Up to now devices only use the first 64 bits.
 * @param target Pointer to the target structure.
 * @param value_input The value to write.
 * @param value_mask Bitmask, 0 -> value stays untouched.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int saml_modify_user_row_masked(struct target *target,
		uint64_t value_input, uint64_t value_mask)
{
	int res;
	uint32_t nvm_ctrlc;
	bool manual_wp = true;

	/* Retrieve the MCU's page size, in bytes. This is also the size of the
	 * entire User Row. */
	uint32_t page_size;
	res = saml_get_flash_page_info(target, &page_size, NULL);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	/* Make sure the size is sane. */
	assert(page_size <= SAML_PAGE_SIZE_MAX &&
		page_size >= sizeof(value_input));

	uint8_t buf[SAML_PAGE_SIZE_MAX];
	/* Read the user row (comprising one page) by words. */
	res = target_read_memory(target, SAML_USER_ROW, 4, page_size / 4, buf);
	if (res != ERROR_OK)
		return res;

	uint64_t value_device;
	res = read_userrow(target, &value_device);
	if (res != ERROR_OK)
		return res;
	uint64_t value_new = (value_input & value_mask) | (value_device & ~value_mask);

	/* We will need to erase before writing if the new value needs a '1' in any
	 * position for which the current value had a '0'.  Otherwise we can avoid
	 * erasing. */
	if ((~value_device) & value_new) {
		res = saml_erase_row(target, SAML_USER_ROW);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't erase user row");
			return res;
		}
	}

	/* Modify */
	target_buffer_set_u64(target, buf, value_new);

	/* Write the page buffer back out to the target. */
	res = target_write_memory(target, SAML_USER_ROW, 4, page_size / 4, buf);
	if (res != ERROR_OK)
		return res;

	/* Check if we need to do manual page write commands */
	res = target_read_u32(target, SAML_NVMCTRL + SAML_NVMCTRL_CTRLC, &nvm_ctrlc);
	if (res == ERROR_OK)
		manual_wp = (nvm_ctrlc & SAML_NVMCTL_CTRLC_MANW) != 0;
	else {
		LOG_ERROR("Read of NVM register CTRKB failed.");
		return ERROR_FAIL;
	}
	if (manual_wp) {
		/* Trigger flash write */
		res = saml_issue_nvmctrl_command(target, SAML_NVM_CMD_WAP);
	} else {
		res = saml_check_error(target);
	}

	return res;
}

/**
 * Modifies the user row register to the given value.
 * @param target Pointer to the target structure.
 * @param value The value to write.
 * @param startb The bit-offset by which the given value is shifted.
 * @param endb The bit-offset of the last bit in value to write.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int saml_modify_user_row(struct target *target, uint64_t value,
		uint8_t startb, uint8_t endb)
{
	uint64_t mask = 0;
	int i;
	for (i = startb ; i <= endb ; i++)
		mask |= ((uint64_t)1) << i;

	return saml_modify_user_row_masked(target, value << startb, mask);
}

static int saml_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	int res = ERROR_OK;

	/* We can issue lock/unlock region commands with the target running but
	 * the settings won't persist unless we're able to modify the LOCK regions
	 * and that requires the target to be halted. */
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (unsigned int prot_block = first; prot_block <= last; prot_block++) {
		if (set != bank->prot_blocks[prot_block].is_protected) {
			/* Load an address that is within this protection block (we use offset 0) */
			res = target_write_u32(bank->target,
							SAML_NVMCTRL + SAML_NVMCTRL_ADDR,
							bank->prot_blocks[prot_block].offset >> 1);
			if (res != ERROR_OK)
				goto exit;

			/* Tell the controller to lock that block */
			res = saml_issue_nvmctrl_command(bank->target,
					set ? SAML_NVM_CMD_LR : SAML_NVM_CMD_UR);
			if (res != ERROR_OK)
				goto exit;
		}
	}

	/* We've now applied our changes, however they will be undone by the next
	 * reset unless we also apply them to the LOCK bits in the User Page.  The
	 * LOCK bits start at bit 48, corresponding to Sector 0 and end with bit 63,
	 * corresponding to Sector 15.  A '1' means unlocked and a '0' means
	 * locked.  See Table 9-3 in the SAMD20 datasheet for more details. */

	res = saml_modify_user_row(bank->target,
			set ? (uint64_t)0 : (uint64_t)UINT64_MAX,
			48 + first, 48 + last);
	if (res != ERROR_OK)
		LOG_WARNING("SAML: protect settings were not made persistent!");

	res = ERROR_OK;

exit:
	saml_protect_check(bank);

	return res;
}

static int saml_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int res;
	struct saml_info *chip = (struct saml_info *)bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (saml_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* For each sector to be erased */
	for (unsigned int s = first; s <= last; s++) {
		res = saml_erase_row(bank->target, bank->sectors[s].offset);
		if (res != ERROR_OK) {
			LOG_ERROR("SAML: failed to erase sector %d at 0x%08" PRIx32, s, bank->sectors[s].offset);
			return res;
		}
	}

	return ERROR_OK;
}


static int saml_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int res;
	uint32_t nvm_ctrlc;
	uint32_t address;
	uint32_t pg_offset;
	uint32_t nb;
	uint32_t nw;
	struct saml_info *chip = (struct saml_info *)bank->driver_priv;
	uint8_t *pb = NULL;
	bool manual_wp;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (saml_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Check if we need to do manual page write commands */
	res = target_read_u32(bank->target, SAML_NVMCTRL + SAML_NVMCTRL_CTRLC, &nvm_ctrlc);

	if (res != ERROR_OK)
		return res;

	if (nvm_ctrlc & SAML_NVMCTL_CTRLC_MANW)
		manual_wp = true;
	else
		manual_wp = false;

	res = saml_issue_nvmctrl_command(bank->target, SAML_NVM_CMD_PBC);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: %d", __func__, __LINE__);
		return res;
	}

	while (count) {
		nb = chip->page_size - offset % chip->page_size;
		if (count < nb)
			nb = count;

		address = bank->base + offset;
		pg_offset = offset % chip->page_size;

		if (offset % 4 || (offset + nb) % 4) {
			/* Either start or end of write is not word aligned */
			if (!pb) {
				pb = malloc(chip->page_size);
				if (!pb)
					return ERROR_FAIL;
			}

			/* Set temporary page buffer to 0xff and overwrite the relevant part */
			memset(pb, 0xff, chip->page_size);
			memcpy(pb + pg_offset, buffer, nb);

			/* Align start address to a word boundary */
			address -= offset % 4;
			pg_offset -= offset % 4;
			assert(pg_offset % 4 == 0);

			/* Extend length to whole words */
			nw = (nb + offset % 4 + 3) / 4;
			assert(pg_offset + 4 * nw <= chip->page_size);

			/* Now we have original data extended by 0xff bytes
			 * to the nearest word boundary on both start and end */
			res = target_write_memory(bank->target, address, 4, nw, pb + pg_offset);
		} else {
			assert(nb % 4 == 0);
			nw = nb / 4;
			assert(pg_offset + 4 * nw <= chip->page_size);

			/* Word aligned data, use direct write from buffer */
			res = target_write_memory(bank->target, address, 4, nw, buffer);
		}
		if (res != ERROR_OK) {
			LOG_ERROR("%s: %d", __func__, __LINE__);
			goto free_pb;
		}

		/* Devices with errata 13134 have automatic page write enabled by default
		 * For other devices issue a write page CMD to the NVM
		 * If the page has not been written up to the last word
		 * then issue CMD_WP always */
		if (manual_wp || pg_offset + 4 * nw < chip->page_size) {
			res = saml_issue_nvmctrl_command(bank->target, SAML_NVM_CMD_WP);
		} else {
			/* Access through AHB is stalled while flash is being programmed */
			usleep(200);

			res = saml_check_error(bank->target);
		}

		if (res != ERROR_OK) {
			LOG_ERROR("%s: write failed at address 0x%08" PRIx32, __func__, address);
			goto free_pb;
		}

		/* We're done with the page contents */
		count -= nb;
		offset += nb;
		buffer += nb;
	}

free_pb:
	if (pb)
		free(pb);

	return res;
}

FLASH_BANK_COMMAND_HANDLER(saml_flash_bank_command)
{
	if (bank->base != SAML_FLASH) {
		LOG_ERROR("Address " TARGET_ADDR_FMT
				" invalid bank address (try 0x%08" PRIx32
				"[atsaml series] )",
				bank->base, SAML_FLASH);
		return ERROR_FAIL;
	}

	struct saml_info *chip;
	chip = calloc(1, sizeof(*chip));
	if (!chip) {
		LOG_ERROR("No memory for flash bank chip info");
		return ERROR_FAIL;
	}

	chip->target = bank->target;
	chip->probed = false;

	bank->driver_priv = chip;

	return ERROR_OK;
}

COMMAND_HANDLER(saml_handle_info_command)
{
	return ERROR_OK;
}

COMMAND_HANDLER(saml_handle_chip_erase_command)
{
	struct target *target = get_current_target(CMD_CTX);
	int res = ERROR_FAIL;

	if (target) {
		/* Enable access to the DSU by disabling the write protect bit */
		target_write_u32(target, SAML_PAC1, (1<<1));
		/* intentionally without error checking - not accessible on secured chip */

		/* Tell the DSU to perform a full chip erase.  It takes about 240ms to
		 * perform the erase. */
		res = target_write_u8(target, SAML_DSU + SAML_DSU_CTRL_EXT, (1<<4));
		if (res == ERROR_OK)
			command_print(CMD, "chip erase started");
		else
			command_print(CMD, "write to DSU CTRL failed");
	}

	return res;
}

COMMAND_HANDLER(saml_handle_set_security_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC < 1 || (CMD_ARGC >= 1 && (strcmp(CMD_ARGV[0], "enable")))) {
		command_print(CMD, "supply the \"enable\" argument to proceed.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		res = saml_issue_nvmctrl_command(target, SAML_NVM_CMD_SSB);

		/* Check (and clear) error conditions */
		if (res == ERROR_OK)
			command_print(CMD, "chip secured on next power-cycle");
		else
			command_print(CMD, "failed to secure chip");
	}

	return res;
}

COMMAND_HANDLER(saml_handle_eeprom_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		if (CMD_ARGC >= 1) {
			int val = atoi(CMD_ARGV[0]);
			uint32_t code;

			if (val == 0)
				code = 7;
			else {
				/* Try to match size in bytes with corresponding size code */
				for (code = 0; code <= 6; code++) {
					if (val == (2 << (13 - code)))
						break;
				}

				if (code > 6) {
					command_print(CMD, "Invalid EEPROM size.  Please see "
							"datasheet for a list valid sizes.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}

			res = saml_modify_user_row(target, code, 4, 6);
		} else {
			uint16_t val;
			res = target_read_u16(target, SAML_USER_ROW, &val);
			if (res == ERROR_OK) {
				uint32_t size = ((val >> 4) & 0x7); /* grab size code */

				if (size == 0x7)
					command_print(CMD, "EEPROM is disabled");
				else {
					/* Otherwise, 6 is 256B, 0 is 16KB */
					command_print(CMD, "EEPROM size is %u bytes",
							(2 << (13 - size)));
				}
			}
		}
	}

	return res;
}

static COMMAND_HELPER(get_u64_from_hexarg, unsigned int num, uint64_t *value)
{
	if (num >= CMD_ARGC) {
		command_print(CMD, "Too few Arguments.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (strlen(CMD_ARGV[num]) >= 3 &&
		CMD_ARGV[num][0] == '0' &&
		CMD_ARGV[num][1] == 'x') {
		char *check = NULL;
		*value = strtoull(&(CMD_ARGV[num][2]), &check, 16);
		if ((value == 0 && errno == ERANGE) ||
			check == NULL || *check != 0) {
			command_print(CMD, "Invalid 64-bit hex value in argument %d.",
				num + 1);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	} else {
		command_print(CMD, "Argument %d needs to be a hex value.", num + 1);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(saml_handle_nvmuserrow_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (CMD_ARGC > 2) {
			command_print(CMD, "Too much Arguments given.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (CMD_ARGC > 0) {
			if (target->state != TARGET_HALTED) {
				LOG_ERROR("Target not halted.");
				return ERROR_TARGET_NOT_HALTED;
			}

			uint64_t mask;
			res = saml_get_reservedmask(target, &mask);
			if (res != ERROR_OK) {
				LOG_ERROR("Couldn't determine the mask for reserved bits.");
				return ERROR_FAIL;
			}
			mask &= NVMUSERROW_LOCKBIT_MASK;

			uint64_t value;
			res = CALL_COMMAND_HANDLER(get_u64_from_hexarg, 0, &value);
			if (res != ERROR_OK)
				return res;
			if (CMD_ARGC == 2) {
				uint64_t mask_temp;
				res = CALL_COMMAND_HANDLER(get_u64_from_hexarg, 1, &mask_temp);
				if (res != ERROR_OK)
					return res;
				mask &= mask_temp;
			}
			res = saml_modify_user_row_masked(target, value, mask);
			if (res != ERROR_OK)
				return res;
		}

		/* read register */
		uint64_t value;
		res = read_userrow(target, &value);
		if (res == ERROR_OK)
			command_print(CMD, "NVMUSERROW: 0x%016"PRIX64, value);
		else
			LOG_ERROR("NVMUSERROW could not be read.");
	}
	return res;
}

COMMAND_HANDLER(saml_handle_bootloader_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		/* Retrieve the MCU's page size, in bytes. */
		uint32_t page_size;
		res = saml_get_flash_page_info(target, &page_size, NULL);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't determine Flash page size");
			return res;
		}

		if (CMD_ARGC >= 1) {
			int val = atoi(CMD_ARGV[0]);
			uint32_t code;

			if (val == 0)
				code = 7;
			else {
				/* Try to match size in bytes with corresponding size code */
				for (code = 0; code <= 6; code++) {
					if ((unsigned int)val == (2UL << (8UL - code)) * page_size)
						break;
				}

				if (code > 6) {
					command_print(CMD, "Invalid bootloader size.  Please "
							"see datasheet for a list valid sizes.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

			}

			res = saml_modify_user_row(target, code, 0, 2);
		} else {
			uint16_t val;
			res = target_read_u16(target, SAML_USER_ROW, &val);
			if (res == ERROR_OK) {
				uint32_t size = (val & 0x7); /* grab size code */
				uint32_t nb;

				if (size == 0x7)
					nb = 0;
				else
					nb = (2 << (8 - size)) * page_size;

				/* There are 4 pages per row */
				command_print(CMD, "Bootloader size is %" PRIu32 " bytes (%" PRIu32 " rows)",
					   nb, (uint32_t)(nb / (page_size * 4)));
			}
		}
	}

	return res;
}



COMMAND_HANDLER(saml_handle_reset_deassert)
{
	struct target *target = get_current_target(CMD_CTX);
	int retval = ERROR_OK;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	/* If the target has been unresponsive before, try to re-establish
	 * communication now - CPU is held in reset by DSU, DAP is working */
	if (!target_was_examined(target))
		target_examine_one(target);
	target_poll(target);

	/* In case of sysresetreq, debug retains state set in cortex_m_assert_reset()
	 * so we just release reset held by DSU
	 *
	 * n_RESET (srst) clears the DP, so reenable debug and set vector catch here
	 *
	 * After vectreset DSU release is not needed however makes no harm
	 */
	if (target->reset_halt && (jtag_reset_config & RESET_HAS_SRST)) {
		retval = target_write_u32(target, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, DCB_DEMCR,
				TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
		/* do not return on error here, releasing DSU reset is more important */
	}

	/* clear CPU Reset Phase Extension bit */
	int retval2 = target_write_u8(target, SAML_DSU + SAML_DSU_STATUSA, (1<<1));
	if (retval2 != ERROR_OK)
		return retval2;

	return retval;
}

static const struct command_registration atsaml_exec_command_handlers[] = {
	{
		.name = "dsu_reset_deassert",
		.handler = saml_handle_reset_deassert,
		.mode = COMMAND_EXEC,
		.help = "Deassert internal reset held by DSU.",
		.usage = "",
	},
	{
		.name = "info",
		.handler = saml_handle_info_command,
		.mode = COMMAND_EXEC,
		.help = "Print information about the current atsaml chip "
			"and its flash configuration.",
		.usage = "",
	},
	{
		.name = "chip-erase",
		.handler = saml_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase the entire Flash by using the Chip-"
			"Erase feature in the Device Service Unit (DSU).",
		.usage = "",
	},
	{
		.name = "set-security",
		.handler = saml_handle_set_security_command,
		.mode = COMMAND_EXEC,
		.help = "Secure the chip's Flash by setting the Security Bit. "
			"This makes it impossible to read the Flash contents. "
			"The only way to undo this is to issue the chip-erase "
			"command.",
		.usage = "'enable'",
	},
	{
		.name = "eeprom",
		.usage = "[size_in_bytes]",
		.handler = saml_handle_eeprom_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the EEPROM size setting, stored in the User Row. "
			"Please see Table 20-3 of the SAMD20 datasheet for allowed values. "
			"Changes are stored immediately but take affect after the MCU is "
			"reset.",
	},
	{
		.name = "bootloader",
		.usage = "[size_in_bytes]",
		.handler = saml_handle_bootloader_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the bootloader size, stored in the User Row. "
			"Please see Table 20-2 of the SAMD20 datasheet for allowed values. "
			"Changes are stored immediately but take affect after the MCU is "
			"reset.",
	},
	{
		.name = "nvmuserrow",
		.usage = "[value] [mask]",
		.handler = saml_handle_nvmuserrow_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the nvmuserrow register. It is 64 bit wide "
			"and located at address 0x804000. Use the optional mask argument "
			"to prevent changes at positions where the bitvalue is zero. "
			"For security reasons the lock- and reserved-bits are masked out "
			"in background and therefore cannot be changed.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration atsaml_command_handlers[] = {
	{
		.name = "atsaml",
		.mode = COMMAND_ANY,
		.help = "atsaml flash command group",
		.usage = "",
		.chain = atsaml_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver atsaml_flash = {
	.name = "atsaml",
	.commands = atsaml_command_handlers,
	.flash_bank_command = saml_flash_bank_command,
	.erase = saml_erase,
	.protect = saml_protect,
	.write = saml_write,
	.read = default_flash_read,
	.probe = saml_probe,
	.auto_probe = saml_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = saml_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
