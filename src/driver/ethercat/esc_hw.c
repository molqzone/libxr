#include "esc_hw.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "ecat_options.h"
#include "ecat_slv.h"
#include "hpm_clock_drv.h"
#include "hpm_esc_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
#include "hpm_ioc_regs.h"

#define ESCREG_ESC_CONFIG 0x0141U
#define DC_SYNC_OUT 0x04U
#define ESCREG_CYCLIC_UNIT_CONTROL 0x0980U
#define SYNC_OUT_PDI_CONTROL 0x01U
#define ESCREG_SYNC_START_TIME 0x0990U
#define ESCREG_DC_SYNC_STATUS 0x098EU
#define SYNC_START_OFFSET (2342840UL * 2UL)
#define ESC_EEPROM_SIZE 2048U

#define JL11X1_PAGESEL_REG_OFFSET 31U
#define JL11X1_WOLEN_REG_OFFSET 0xF3U
#define JL11X1_WOLEN_REG_VALUE 0U
#define JL11X1_RESET_HOLD_TIME_MS 1U
#define JL11X1_RESET_WAIT_TIME_MS 5U
#define JL11X1_RESET_LEVEL 0U
#define RMGO_ECAT_SUPPORT_PORT1 BOARD_ECAT_SUPPORT_PORT1
#define RMGO_ECAT_SUPPORT_PORT2 BOARD_ECAT_SUPPORT_PORT2

static bool s_hw_initialized;

static uint8_t s_eeprom[ESC_EEPROM_SIZE];

volatile uint32_t g_rmgo_ecat_debug_stage;
volatile uint32_t g_rmgo_ecat_debug_hw_status;
volatile uint32_t g_rmgo_ecat_debug_poll_count;
volatile uint32_t g_rmgo_ecat_debug_pdi_irq_count;
volatile uint32_t g_rmgo_ecat_debug_sync0_irq_count;
volatile uint32_t g_rmgo_ecat_debug_eep_count;
volatile uint32_t g_rmgo_ecat_debug_eep_cmd;
volatile uint32_t g_rmgo_ecat_debug_eep_addr;
volatile uint32_t g_rmgo_ecat_debug_alevent;
volatile uint32_t g_rmgo_ecat_debug_alcontrol;
volatile uint32_t g_rmgo_ecat_debug_alstatus;
volatile uint32_t g_rmgo_ecat_debug_dlstatus;
volatile uint32_t g_rmgo_ecat_debug_phy_status;
volatile uint32_t g_rmgo_ecat_debug_phy0;
volatile uint32_t g_rmgo_ecat_debug_phy1;
volatile uint32_t g_rmgo_ecat_debug_phy2;
volatile uint32_t g_rmgo_ecat_debug_phy0_id;
volatile uint32_t g_rmgo_ecat_debug_phy1_id;
volatile uint32_t g_rmgo_ecat_debug_phy2_id;
volatile uint32_t g_rmgo_ecat_debug_sm2_sml;
volatile uint32_t g_rmgo_ecat_debug_sm3_sml;
volatile uint32_t g_rmgo_ecat_debug_sm2_mappings;
volatile uint32_t g_rmgo_ecat_debug_sm3_mappings;
volatile uint32_t g_rmgo_ecat_debug_app_state;

static void ecat_debug_snapshot(uint32_t stage)
{
    g_rmgo_ecat_debug_stage = stage;
    g_rmgo_ecat_debug_alevent = ESCvar.ALevent;
    g_rmgo_ecat_debug_alcontrol = ESCvar.ALcontrol;
    g_rmgo_ecat_debug_alstatus = ESCvar.ALstatus;
    g_rmgo_ecat_debug_dlstatus = ESCvar.DLstatus;
    g_rmgo_ecat_debug_sm2_sml = ESCvar.ESC_SM2_sml;
    g_rmgo_ecat_debug_sm3_sml = ESCvar.ESC_SM3_sml;
    g_rmgo_ecat_debug_sm2_mappings = (uint32_t) ESCvar.sm2mappings;
    g_rmgo_ecat_debug_sm3_mappings = (uint32_t) ESCvar.sm3mappings;
    g_rmgo_ecat_debug_app_state = (uint32_t) ESCvar.App.state;
}

static uint32_t ecat_debug_read_phy(uint8_t phy_addr, volatile uint32_t *id_out)
{
    uint16_t bmsr = 0;
    uint16_t bmsr_latched = 0;
    uint16_t id1 = 0;
    uint16_t id2 = 0;
    uint16_t vendor_status = 0;
    uint32_t status = 0;

    if (esc_mdio_read(HPM_ESC, phy_addr, 1, &bmsr_latched) != status_success) {
        status |= 0x01U;
    }
    (void) bmsr_latched;
    if (esc_mdio_read(HPM_ESC, phy_addr, 1, &bmsr) != status_success) {
        status |= 0x02U;
    }
    if (esc_mdio_read(HPM_ESC, phy_addr, 2, &id1) != status_success) {
        status |= 0x04U;
    }
    if (esc_mdio_read(HPM_ESC, phy_addr, 3, &id2) != status_success) {
        status |= 0x08U;
    }
    if (esc_mdio_read(HPM_ESC, phy_addr, 17, &vendor_status) != status_success) {
        status |= 0x10U;
    }

    g_rmgo_ecat_debug_phy_status =
        (g_rmgo_ecat_debug_phy_status & ~(0xFFU << (phy_addr * 8U))) |
        (status << (phy_addr * 8U));
    *id_out = ((uint32_t) id1 << 16U) | id2;

    return ((uint32_t) bmsr << 16U) | vendor_status;
}

static void ecat_debug_sample_phys(void)
{
    g_rmgo_ecat_debug_phy0 =
        ecat_debug_read_phy(BOARD_ECAT_PORT0_PHY_ADDR, &g_rmgo_ecat_debug_phy0_id);
#if RMGO_ECAT_SUPPORT_PORT1
    g_rmgo_ecat_debug_phy1 =
        ecat_debug_read_phy(BOARD_ECAT_PORT1_PHY_ADDR, &g_rmgo_ecat_debug_phy1_id);
#else
    g_rmgo_ecat_debug_phy1 = 0;
    g_rmgo_ecat_debug_phy1_id = 0;
#endif
#if RMGO_ECAT_SUPPORT_PORT2
    g_rmgo_ecat_debug_phy2 =
        ecat_debug_read_phy(BOARD_ECAT_PORT2_PHY_ADDR, &g_rmgo_ecat_debug_phy2_id);
#else
    g_rmgo_ecat_debug_phy2 = 0;
    g_rmgo_ecat_debug_phy2_id = 0;
#endif
}

static volatile uint8_t *esc_mem8(void)
{
    return (volatile uint8_t *) HPM_ESC_BASE;
}

static void rmgo_board_init_ethercat_2port(void)
{
    board_init_ethercat(HPM_ESC);
}

static void esc_read_bytes(uint16_t address, void *buf, uint16_t len)
{
    uint8_t *dst = (uint8_t *) buf;
#if defined(HPM_IP_FEATURE_ESC_BYTE_READ) && HPM_IP_FEATURE_ESC_BYTE_READ
    volatile uint8_t *src = &esc_mem8()[address];
    for (uint16_t i = 0; i < len; ++i) {
        dst[i] = src[i];
    }
#else
    uint16_t offset = address & 0x3U;
    uint32_t aligned_address = (uint32_t) address & ~0x3UL;
    uint16_t copied = 0;

    while (copied < len) {
        volatile uint32_t *src =
            (volatile uint32_t *) (HPM_ESC_BASE + aligned_address);
        uint32_t word = *src;
        uint8_t *word_bytes = (uint8_t *) &word;

        while ((offset < 4U) && (copied < len)) {
            dst[copied++] = word_bytes[offset++];
        }

        offset = 0;
        aligned_address += 4U;
    }
#endif
}

static void esc_write_bytes(uint16_t address, const void *buf, uint16_t len)
{
    const uint8_t *src = (const uint8_t *) buf;
    volatile uint8_t *dst = &esc_mem8()[address];

    for (uint16_t i = 0; i < len; ++i) {
        dst[i] = src[i];
    }
}

static void ecat_phy_reset(void)
{
    gpio_write_pin(BOARD_ECAT_PHY0_RESET_GPIO, BOARD_ECAT_PHY0_RESET_GPIO_PORT_INDEX,
                   BOARD_ECAT_PHY0_RESET_PIN_INDEX, JL11X1_RESET_LEVEL);
#if RMGO_ECAT_SUPPORT_PORT1
    gpio_write_pin(BOARD_ECAT_PHY1_RESET_GPIO, BOARD_ECAT_PHY1_RESET_GPIO_PORT_INDEX,
                   BOARD_ECAT_PHY1_RESET_PIN_INDEX, JL11X1_RESET_LEVEL);
#endif
#if RMGO_ECAT_SUPPORT_PORT2
    gpio_write_pin(BOARD_ECAT_PHY2_RESET_GPIO, BOARD_ECAT_PHY2_RESET_GPIO_PORT_INDEX,
                   BOARD_ECAT_PHY2_RESET_PIN_INDEX, JL11X1_RESET_LEVEL);
#endif

    board_delay_ms(JL11X1_RESET_HOLD_TIME_MS);

    gpio_write_pin(BOARD_ECAT_PHY0_RESET_GPIO, BOARD_ECAT_PHY0_RESET_GPIO_PORT_INDEX,
                   BOARD_ECAT_PHY0_RESET_PIN_INDEX, !JL11X1_RESET_LEVEL);
#if RMGO_ECAT_SUPPORT_PORT1
    gpio_write_pin(BOARD_ECAT_PHY1_RESET_GPIO, BOARD_ECAT_PHY1_RESET_GPIO_PORT_INDEX,
                   BOARD_ECAT_PHY1_RESET_PIN_INDEX, !JL11X1_RESET_LEVEL);
#endif
#if RMGO_ECAT_SUPPORT_PORT2
    gpio_write_pin(BOARD_ECAT_PHY2_RESET_GPIO, BOARD_ECAT_PHY2_RESET_GPIO_PORT_INDEX,
                   BOARD_ECAT_PHY2_RESET_PIN_INDEX, !JL11X1_RESET_LEVEL);
#endif

    board_delay_ms(JL11X1_RESET_WAIT_TIME_MS);
}

static hpm_stat_t ecat_jl1111_phy_config_led_mode(uint8_t phy_addr)
{
    hpm_stat_t stat = esc_mdio_write(HPM_ESC, phy_addr, JL11X1_PAGESEL_REG_OFFSET,
                                     JL11X1_WOLEN_REG_OFFSET / 32U);
    if (stat != status_success) {
        return stat;
    }

    stat = esc_mdio_write(HPM_ESC, phy_addr, JL11X1_WOLEN_REG_OFFSET % 32U,
                          JL11X1_WOLEN_REG_VALUE);
    if (stat != status_success) {
        return stat;
    }

    return esc_mdio_write(HPM_ESC, phy_addr, JL11X1_PAGESEL_REG_OFFSET, 0);
}

static hpm_stat_t ecat_jl1111_phy_disable_broadcast(uint8_t phy_addr)
{
    if ((phy_addr + BOARD_ECAT_PHY_ADDR_OFFSET) == 0U) {
        return status_success;
    }

    hpm_stat_t stat = esc_mdio_write(HPM_ESC, phy_addr, JL11X1_PAGESEL_REG_OFFSET, 128);
    if (stat != status_success) {
        return stat;
    }

    uint16_t value = (uint16_t) (((phy_addr + BOARD_ECAT_PHY_ADDR_OFFSET) << 5U) | 0x1FU);
    stat = esc_mdio_write(HPM_ESC, phy_addr, 19, value);
    if (stat != status_success) {
        return stat;
    }

    return esc_mdio_write(HPM_ESC, phy_addr, JL11X1_PAGESEL_REG_OFFSET, 0);
}

static hpm_stat_t ecat_phy_config(void)
{
    hpm_stat_t stat = ecat_jl1111_phy_config_led_mode(BOARD_ECAT_PORT0_PHY_ADDR);
    if (stat != status_success) {
        return stat;
    }

    stat = ecat_jl1111_phy_disable_broadcast(BOARD_ECAT_PORT0_PHY_ADDR);
    if (stat != status_success) {
        return stat;
    }

#if RMGO_ECAT_SUPPORT_PORT1
    stat = ecat_jl1111_phy_config_led_mode(BOARD_ECAT_PORT1_PHY_ADDR);
    if (stat != status_success) {
        return stat;
    }

    stat = ecat_jl1111_phy_disable_broadcast(BOARD_ECAT_PORT1_PHY_ADDR);
    if (stat != status_success) {
        return stat;
    }
#endif

#if RMGO_ECAT_SUPPORT_PORT2
    stat = ecat_jl1111_phy_config_led_mode(BOARD_ECAT_PORT2_PHY_ADDR);
    if (stat != status_success) {
        return stat;
    }

    stat = ecat_jl1111_phy_disable_broadcast(BOARD_ECAT_PORT2_PHY_ADDR);
    if (stat != status_success) {
        return stat;
    }
#endif

    return status_success;
}

static void hpm_ecat_enable_interrupts(void)
{
    esc_enable_irq(HPM_ESC, esc_irq_mask_all);
    intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, 3);
    intc_m_enable_irq_with_priority(IRQn_ESC_SYNC1, 3);
    intc_m_enable_irq_with_priority(IRQn_ESC, 4);
    intc_m_enable_irq_with_priority(IRQn_ESC_RESET, 2);
}

static uint8_t ecat_eeprom_checksum(const uint8_t *data, uint32_t size)
{
    uint8_t remainder = 0xFFU;
    const uint8_t polynomial = 0x07U;

    for (uint32_t i = 0; i < size; ++i) {
        remainder ^= data[i];

        for (uint8_t bit = 0; bit < 8U; ++bit) {
            if ((remainder & 0x80U) != 0U) {
                remainder = (uint8_t) ((remainder << 1U) ^ polynomial);
            } else {
                remainder = (uint8_t) (remainder << 1U);
            }
        }
    }

    return remainder;
}

static void eeprom_put_u16(uint32_t word, uint16_t value)
{
    s_eeprom[word * 2U] = (uint8_t) (value & 0xFFU);
    s_eeprom[word * 2U + 1U] = (uint8_t) (value >> 8U);
}

static void eeprom_put_u32(uint32_t word, uint32_t value)
{
    eeprom_put_u16(word, (uint16_t) (value & 0xFFFFU));
    eeprom_put_u16(word + 1U, (uint16_t) (value >> 16U));
}

static void hpm_ecat_prepare_eeprom(void)
{
    memset(s_eeprom, 0xFF, sizeof(s_eeprom));

    /* ESC configuration area. Word 7 is checksum over words 0-6. */
    eeprom_put_u16(0, 0x0C80U);
    eeprom_put_u16(1, 0x6681U);
    eeprom_put_u16(2, 0x0000U);
    eeprom_put_u16(3, 0x0000U);
    eeprom_put_u16(4, 0x3412U);
    eeprom_put_u16(5, 0x0000U);
    eeprom_put_u16(6, 0x0000U);
    eeprom_put_u16(7, ecat_eeprom_checksum(s_eeprom, 14U));

    eeprom_put_u32(8, 0x00000000UL);   /* Vendor ID */
    eeprom_put_u32(10, 0x000AB123UL);  /* Product code */
    eeprom_put_u32(12, 0x00000002UL);  /* Revision */
    eeprom_put_u32(14, 0x00000001UL);  /* Serial */
    eeprom_put_u16(0x18U, MBX0_sma);   /* Standard receive mailbox offset */
    eeprom_put_u16(0x19U, MBX0_sml);   /* Standard receive mailbox size */
    eeprom_put_u16(0x1AU, MBX1_sma);   /* Standard send mailbox offset */
    eeprom_put_u16(0x1BU, MBX1_sml);   /* Standard send mailbox size */
    eeprom_put_u16(0x1CU, 0x0004U);    /* Mailbox protocol: CoE */

    uint32_t word = 0x40U;

    eeprom_put_u16(word++, 0x000AU); /* Strings */
    eeprom_put_u16(word++, 0x0010U); /* 32 bytes */
    const uint8_t strings[] = {
        4,
        8, 'D', 'y', 'n', 'a', 'm', 'i', 'c', 'X',
        5, 'R', 'M', 'G', 'O', 'H',
        8, 'R', 'M', 'G', 'O', ' ', 'H', 'P', 'M',
        7, 'R', 'M', 'G', 'O', ' ', 'F', 'O',
    };
    memcpy(&s_eeprom[word * 2U], strings, sizeof(strings));
    word += 16U;

    eeprom_put_u16(word++, 0x001EU); /* General */
    eeprom_put_u16(word++, 0x0009U); /* 18 bytes */
    const uint8_t general[] = {
        1, 0, 2, 3, 0,
        0x33, 0, 0, 0, 0,
        0x00,
        0x00, 0x00,
        0x01, 0x00,
        0x00, 0x00,
        0x00,
    };
    memcpy(&s_eeprom[word * 2U], general, sizeof(general));
    word += 9U;

    eeprom_put_u16(word++, 0x0028U); /* FMMU */
    eeprom_put_u16(word++, 0x0001U); /* 2 bytes */
    s_eeprom[word * 2U] = 0x01U;     /* FMMU0: outputs */
    s_eeprom[word * 2U + 1U] = 0x02U; /* FMMU1: inputs */
    word += 1U;

    eeprom_put_u16(word++, 0x0029U); /* Sync Manager */
    eeprom_put_u16(word++, 0x0010U); /* 4 entries * 8 bytes */
    const uint8_t sms[] = {
        0x00, 0x10, 0x00, 0x02, 0x26, 0x00, 0x01, 0x01,
        0x00, 0x12, 0x00, 0x02, 0x22, 0x00, 0x01, 0x02,
        0x00, 0x16, 0x00, 0x02, 0x24, 0x00, 0x01, 0x03,
        0x00, 0x1A, 0x00, 0x02, 0x20, 0x00, 0x01, 0x04,
    };
    memcpy(&s_eeprom[word * 2U], sms, sizeof(sms));
    word += 16U;

    eeprom_put_u16(word++, 0xFFFFU); /* End */
    eeprom_put_u16(word++, 0x0000U);
}

static hpm_stat_t hpm_ecat_hw_init(void)
{
    esc_eeprom_clock_config_t esc_config = {
        .eeprom_emulation = true,
        .eeprom_size_over_16kbit = false,
        .core_clock_en = true,
        .phy_refclk_en = true,
    };

    ecat_debug_snapshot(0x1000U);
    hpm_ecat_prepare_eeprom();
    rmgo_board_init_ethercat_2port();
    esc_config_eeprom_and_clock(HPM_ESC, &esc_config);
    ecat_debug_snapshot(0x1100U);

    uint16_t config_data[8];
    for (uint32_t i = 0; i < 8U; ++i) {
        config_data[i] = (uint16_t) s_eeprom[i * 2U] |
                         ((uint16_t) s_eeprom[i * 2U + 1U] << 8U);
    }
    uint32_t reload_data = config_data[4];
    reload_data += ESC_ESC_CFG_ELDAP_GET(config_data[0] >> 8U) << 16U;
    reload_data += ESC_ESC_CFG_ELP0_GET(config_data[0] >> 8U) << 17U;
    reload_data += ESC_ESC_CFG_ELP1_GET(config_data[0] >> 8U) << 18U;
    reload_data += ESC_ESC_CFG_ELP2_GET(config_data[0] >> 8U) << 19U;
    reload_data += ESC_ESC_CFG_ELP3_GET(config_data[0] >> 8U) << 20U;
    esc_write_eeprom_data(HPM_ESC, reload_data);
    esc_eeprom_emulation_ack(HPM_ESC, esc_eeprom_reload_cmd, false, false);
    ecat_debug_snapshot(0x1200U);

    hpm_stat_t stat = esc_check_eeprom_loading(HPM_ESC);
    g_rmgo_ecat_debug_hw_status = (uint32_t) stat;
    ecat_debug_snapshot(0x1300U);
    if (stat == status_esc_eeprom_checksum_error) {
        printf("ESC EEPROM checksum error, continue for bring-up.\r\n");
    } else if (stat != status_success) {
        printf("ESC EEPROM loading failed: %d\r\n", stat);
        return stat;
    }

    ecat_phy_reset();
    ecat_debug_snapshot(0x1400U);
    esc_set_phy_offset(HPM_ESC, BOARD_ECAT_PHY_ADDR_OFFSET);

    stat = ecat_phy_config();
    if (stat != status_success) {
        printf("ESC PHY configuration failed: %d\r\n", stat);
        return stat;
    }
    ecat_debug_sample_phys();

#if defined(HPM_IP_FEATURE_ESC_SYNC_IRQ_MASK) && HPM_IP_FEATURE_ESC_SYNC_IRQ_MASK
    esc_enable_sync_irq_to_pdi_irq(HPM_ESC, false, false);
#endif

    esc_config_ctrl_signal_function(HPM_ESC, BOARD_ECAT_NMII_LINK0_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link0,
                                    BOARD_ECAT_PORT0_LINK_INVERT);
#if RMGO_ECAT_SUPPORT_PORT1
    esc_config_ctrl_signal_function(HPM_ESC, BOARD_ECAT_NMII_LINK1_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link1,
                                    BOARD_ECAT_PORT1_LINK_INVERT);
#endif
#if RMGO_ECAT_SUPPORT_PORT2
    esc_config_ctrl_signal_function(HPM_ESC, BOARD_ECAT_NMII_LINK2_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link2,
                                    BOARD_ECAT_PORT2_LINK_INVERT);
#endif
    esc_config_nmii_link_source(HPM_ESC, true, RMGO_ECAT_SUPPORT_PORT1,
                                RMGO_ECAT_SUPPORT_PORT2);

    hpm_ecat_enable_interrupts();
    s_hw_initialized = true;
    ecat_debug_snapshot(0x1F00U);
    return status_success;
}

void ESC_eeprom_emulation_handler(void)
{
    g_rmgo_ecat_debug_eep_count++;
    if ((HPM_ESC->EEPROM_CTRL_STAT & ESC_EEPROM_CTRL_STAT_EE_EMU_MASK) == 0U) {
        g_rmgo_ecat_debug_eep_cmd = 0xEE00U;
        return;
    }

    if ((HPM_ESC->EEPROM_CTRL_STAT & ESC_EEPROM_CTRL_STAT_BUSY_MASK) == 0U) {
        g_rmgo_ecat_debug_eep_cmd = 0xEE01U;
        return;
    }

    const esc_eeprom_cmd_t cmd = (esc_eeprom_cmd_t) esc_get_eeprom_cmd(HPM_ESC);
    const uint32_t wordaddr = esc_get_eeprom_word_address(HPM_ESC);
    g_rmgo_ecat_debug_eep_cmd = (uint32_t) cmd;
    g_rmgo_ecat_debug_eep_addr = wordaddr;

    if (cmd == esc_eeprom_read_cmd) {
        uint64_t data = 0;
        uint32_t words = ESC_EEPROM_CTRL_STAT_NUM_RD_BYTE_GET(HPM_ESC->EEPROM_CTRL_STAT) ? 4U : 2U;

        for (uint32_t i = 0; i < words; ++i) {
            uint32_t addr = (wordaddr + i) * 2U;
            uint16_t word = 0xFFFFU;

            if ((addr + 1U) < sizeof(s_eeprom)) {
                word = (uint16_t) s_eeprom[addr] | ((uint16_t) s_eeprom[addr + 1U] << 8U);
            }

            data |= ((uint64_t) word) << (16U * i);
        }

        esc_write_eeprom_data(HPM_ESC, data);
        esc_eeprom_emulation_ack(HPM_ESC, cmd, false, false);
    } else if (cmd == esc_eeprom_write_cmd) {
        uint32_t addr = wordaddr * 2U;

        if ((addr + 1U) < sizeof(s_eeprom)) {
            uint16_t word = (uint16_t) (esc_read_eeprom_data(HPM_ESC) & 0xFFFFU);
            s_eeprom[addr] = (uint8_t) (word & 0xFFU);
            s_eeprom[addr + 1U] = (uint8_t) (word >> 8U);
            esc_eeprom_emulation_ack(HPM_ESC, cmd, false, false);
        } else {
            esc_eeprom_emulation_ack(HPM_ESC, cmd, true, false);
        }
    } else if (cmd == esc_eeprom_reload_cmd) {
        uint16_t config_data[8];
        for (uint32_t i = 0; i < 8U; ++i) {
            config_data[i] = (uint16_t) s_eeprom[i * 2U] |
                             ((uint16_t) s_eeprom[i * 2U + 1U] << 8U);
        }

        uint8_t checksum = ecat_eeprom_checksum(s_eeprom, 14U);
        if (checksum != (uint8_t) config_data[7]) {
            esc_eeprom_emulation_ack(HPM_ESC, cmd, false, true);
            return;
        }

        uint32_t reload_data = config_data[4];
        reload_data += ESC_ESC_CFG_ELDAP_GET(config_data[0] >> 8U) << 16U;
        reload_data += ESC_ESC_CFG_ELP0_GET(config_data[0] >> 8U) << 17U;
        reload_data += ESC_ESC_CFG_ELP1_GET(config_data[0] >> 8U) << 18U;
        reload_data += ESC_ESC_CFG_ELP2_GET(config_data[0] >> 8U) << 19U;
        reload_data += ESC_ESC_CFG_ELP3_GET(config_data[0] >> 8U) << 20U;
        esc_write_eeprom_data(HPM_ESC, reload_data);
        esc_eeprom_emulation_ack(HPM_ESC, cmd, false, false);
    } else {
        esc_eeprom_emulation_ack(HPM_ESC, cmd, true, false);
    }
}

void ESC_read(uint16_t address, void *buf, uint16_t len)
{
    esc_read_bytes(address, buf, len);
    esc_read_bytes(ESCREG_ALEVENT, (void *) &ESCvar.ALevent, sizeof(ESCvar.ALevent));
    ESCvar.ALevent = etohs(ESCvar.ALevent);
}

void ESC_write(uint16_t address, void *buf, uint16_t len)
{
    esc_write_bytes(address, buf, len);
    esc_read_bytes(ESCREG_ALEVENT, (void *) &ESCvar.ALevent, sizeof(ESCvar.ALevent));
    ESCvar.ALevent = etohs(ESCvar.ALevent);
}

void ESC_init(const esc_cfg_t *config)
{
    (void) config;

    if (!s_hw_initialized) {
        (void) hpm_ecat_hw_init();
    }

    ecat_debug_snapshot(0x2000U);
}

void ESC_interrupt_enable(uint32_t mask)
{
    esc_irq_mask_t irq_mask = esc_irq_mask_none;

    if ((mask & ESCREG_ALEVENT_DC_SYNC0) != 0U) {
        irq_mask = (esc_irq_mask_t) (irq_mask | esc_sync0_irq_mask);
    }
    if ((mask & ESCREG_ALEVENT_DC_SYNC1) != 0U) {
        irq_mask = (esc_irq_mask_t) (irq_mask | esc_sync1_irq_mask);
    }

    esc_enable_irq(HPM_ESC, irq_mask);
    ESC_ALeventmaskwrite(mask);
}

void ESC_interrupt_disable(uint32_t mask)
{
    esc_irq_mask_t irq_mask = esc_irq_mask_none;

    if ((mask & ESCREG_ALEVENT_DC_SYNC0) != 0U) {
        irq_mask = (esc_irq_mask_t) (irq_mask | esc_sync0_irq_mask);
    }
    if ((mask & ESCREG_ALEVENT_DC_SYNC1) != 0U) {
        irq_mask = (esc_irq_mask_t) (irq_mask | esc_sync1_irq_mask);
    }

    esc_disable_irq(HPM_ESC, irq_mask);
    ESC_ALeventmaskwrite(ESC_ALeventmaskread() & ~mask);
}

uint32_t ESC_enable_DC(void)
{
    uint8_t data = 0;

    ESC_read(ESCREG_ESC_CONFIG, &data, sizeof(data));
    if ((data & DC_SYNC_OUT) == 0U) {
        return 0;
    }

    uint32_t sync0_cycle_time = 0;
    ESC_read(ESCREG_SYNC0_CYCLE_TIME, &sync0_cycle_time, sizeof(sync0_cycle_time));
    sync0_cycle_time = etohl(sync0_cycle_time);

    ESC_read(ESCREG_CYCLIC_UNIT_CONTROL, &data, sizeof(data));
    if (data == SYNC_OUT_PDI_CONTROL) {
        ESC_read(ESCREG_LOCALTIME, (void *) &ESCvar.Time, sizeof(ESCvar.Time));
        ESCvar.Time = etohl(ESCvar.Time);

        uint32_t start_time = ESCvar.Time + SYNC_START_OFFSET;
        ESC_write(ESCREG_SYNC_START_TIME, &start_time, sizeof(start_time));

        ESC_read(ESCREG_SYNC_ACT, &data, sizeof(data));
        data = data | ESCREG_SYNC_ACT_ACTIVATED | ESCREG_SYNC_SYNC0_EN;
        ESC_write(ESCREG_SYNC_ACT, &data, sizeof(data));

        data = 0;
        while ((data & (ESCREG_SYNC_ACT_ACTIVATED | ESCREG_SYNC_SYNC0_EN)) == 0U) {
            ESC_read(ESCREG_SYNC_ACT, &data, sizeof(data));
        }
    }

    printf("ESC DC sync0 cycle: %lu ns\r\n", (unsigned long) sync0_cycle_time);
    return sync0_cycle_time;
}

int ESC_dc_watchdog_init(void)
{
    ESCvar.dcsync = 1;
    ESCvar.synccounterlimit = 10000;

    uint32_t sync0_cycle_time = ESC_enable_DC();
    return (int) (2U * sync0_cycle_time);
}

uint16_t ESC_check_dc(void)
{
    (void) ESC_dc_watchdog_init();
    return 0;
}

SDK_DECLARE_EXT_ISR_M(IRQn_ESC, ecat_pdi_isr)
void ecat_pdi_isr(void)
{
    const uint32_t worker_events = ESCREG_ALEVENT_CONTROL |
                                   ESCREG_ALEVENT_SMCHANGE |
                                   ESCREG_ALEVENT_SM0 |
                                   ESCREG_ALEVENT_SM1 |
                                   ESCREG_ALEVENT_EEP;
    const uint32_t pdo_events = ESCREG_ALEVENT_SM2 |
                                ESCREG_ALEVENT_SM3 |
                                ESCREG_ALEVENT_DC_SYNC0;

    g_rmgo_ecat_debug_pdi_irq_count++;
    ESC_updateALevent();
    ecat_debug_snapshot(0x3000U);

    if ((ESCvar.ALevent & worker_events) != 0U) {
        ecat_slv_worker(worker_events);
        ecat_debug_snapshot(0x3010U);
    }

    if ((ESCvar.ALevent & pdo_events) != 0U) {
        if (ESCvar.dcsync == 0) {
            DIG_process(DIG_PROCESS_OUTPUTS_FLAG | DIG_PROCESS_APP_HOOK_FLAG |
                        DIG_PROCESS_INPUTS_FLAG);
        } else {
            DIG_process(DIG_PROCESS_OUTPUTS_FLAG);
        }
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_ESC_SYNC0, ecat_sync0_isr)
void ecat_sync0_isr(void)
{
    g_rmgo_ecat_debug_sync0_irq_count++;
    uint32_t sync_state = 0;
    ESC_read(ESCREG_DC_SYNC_STATUS, &sync_state, sizeof(sync_state));
    (void) sync_state;

    ESC_updateALevent();
    ecat_debug_snapshot(0x3100U);
    DIG_process(DIG_PROCESS_APP_HOOK_FLAG | DIG_PROCESS_INPUTS_FLAG);
}

SDK_DECLARE_EXT_ISR_M(IRQn_ESC_SYNC1, ecat_sync1_isr)
void ecat_sync1_isr(void)
{
    uint32_t sync_state = 0;
    ESC_read(ESCREG_DC_SYNC_STATUS, &sync_state, sizeof(sync_state));
    (void) sync_state;
}

SDK_DECLARE_EXT_ISR_M(IRQn_ESC_RESET, ecat_reset_isr)
void ecat_reset_isr(void)
{
    printf("ESC reset request received.\r\n");
}

void rmgo_ecat_debug_mark_poll(void)
{
    g_rmgo_ecat_debug_poll_count++;
    if ((g_rmgo_ecat_debug_poll_count & 0xFFU) == 0U) {
        ecat_debug_sample_phys();
    }
    ecat_debug_snapshot(0x4000U);
}
