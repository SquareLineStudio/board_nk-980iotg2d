/**************************************************************************//**
*
* @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*
* SPDX-License-Identifier: Apache-2.0
*
* Change Logs:
* Date            Author       Notes
* 2020-12-12      Wayne        First version
*
******************************************************************************/

#include "board.h"

static void nu_pin_uart_init(void)
{
#if defined(BSP_USING_UART0)
    /* UART0: GPF11, GPF12 */
    outpw(REG_SYS_GPF_MFPH, (inpw(REG_SYS_GPF_MFPH) & ~0x000FF000) | 0x00011000);
#endif

#if !defined(BOARD_USING_LCD_ILI9341)
#if defined(BSP_USING_UART1)
    /* UART1: GPF9, GPF10 */
    outpw(REG_SYS_GPF_MFPH, (inpw(REG_SYS_GPF_MFPH) & ~0x00000FF0) | 0x00000220);
#endif
#endif

#if defined(BSP_USING_UART4)
    /* UART4: PD12, PD13 */
    outpw(REG_SYS_GPD_MFPH, (inpw(REG_SYS_GPD_MFPH) & ~0x00FF0000) | 0x00110000);
#endif

#if defined(BSP_USING_UART5)
    /* UART5: PG6, PG7 */
    outpw(REG_SYS_GPG_MFPL, (inpw(REG_SYS_GPG_MFPL) & ~0xFF000000) | 0x22000000);
#endif

#if defined(BSP_USING_UART6)
    /* UART6: PD10, PD11 */
    outpw(REG_SYS_GPD_MFPH, (inpw(REG_SYS_GPD_MFPH) & ~0x0000FF00) | 0x00002200);
#endif

#if !defined(BSP_USING_ADC_TOUCH)
#if defined(BSP_USING_UART7)
    /* UART7: PB4, PB6 */
    outpw(REG_SYS_GPB_MFPL, (inpw(REG_SYS_GPB_MFPL) & ~0x0F0F0000) | 0x05050000);
#endif
#endif

#if defined(BSP_USING_UART9)
    /* UART9: PB1, PB3 */
    outpw(REG_SYS_GPB_MFPL, (inpw(REG_SYS_GPB_MFPL) & ~0x0000F0F0) | 0x00007070);
#endif
}

static void nu_pin_emac_init(void)
{
#if defined(BSP_USING_EMAC0)
    /* EMAC0  */
    outpw(REG_SYS_GPE_MFPL, 0x11111111);
    outpw(REG_SYS_GPE_MFPH, (inpw(REG_SYS_GPE_MFPH) & ~0x000000FF) | 0x00000011);
#endif
}

static void nu_pin_sdh_init(void)
{
#if defined(BSP_USING_SDH1)
    /* SDH1: PF[0, 6]  */
    outpw(REG_SYS_GPF_MFPL, (inpw(REG_SYS_GPF_MFPL) & ~0x0FFFFFFF) | 0x02222222);
#endif
}

static void nu_pin_qspi_init(void)
{
#if defined(BSP_USING_QSPI0)
    /* QSPI0: PD[2, 7]  */
    outpw(REG_SYS_GPD_MFPL, (inpw(REG_SYS_GPD_MFPL) & ~0xFFFFFF00) | 0x11111100);
#endif
}

static void nu_pin_spi_init(void)
{
#if defined(BSP_USING_SPI0)
    /* SPI0: PD[8, 11]  */
    outpw(REG_SYS_GPD_MFPH, (inpw(REG_SYS_GPD_MFPH) & ~0x0000FFFF) | 0x00001111);
#endif
}

static void nu_pin_i2c_init(void)
{
#if defined(BSP_USING_I2C0)
    /* I2C0: PA[0, 1]  */
    outpw(REG_SYS_GPA_MFPL, (inpw(REG_SYS_GPA_MFPL) & ~0x000000FF) | 0x00000033);
#endif

#if defined(BSP_USING_I2C1)
    /* I2C1: PB4, PB6 */
    outpw(REG_SYS_GPB_MFPL, (inpw(REG_SYS_GPB_MFPL) & ~0x0F0F0000) | 0x02020000);
#endif

#if !defined(BSP_USING_ADC_TOUCH)
#if defined(BSP_USING_I2C2)
    /* I2C2: PB5, PB7  */
    outpw(REG_SYS_GPB_MFPL, (inpw(REG_SYS_GPB_MFPL) & ~0xF0F00000) | 0x20200000);
#endif
#endif

#if defined(BSP_USING_I2C3)
    /* I2C3: PD14, PD15 */
    outpw(REG_SYS_GPD_MFPH, (inpw(REG_SYS_GPD_MFPH) & ~0x0FF00000) | 0x03300000);
#endif

}


static void nu_pin_pwm_init(void)
{
    /* PWM02, PWM03: PF[7, 8]  */
    outpw(REG_SYS_GPF_MFPL, (inpw(REG_SYS_GPF_MFPL) & ~0xF0000000) | 0x40000000);
    outpw(REG_SYS_GPF_MFPH, (inpw(REG_SYS_GPF_MFPH) & ~0x0000000F) | 0x00000004);
}

static void nu_pin_i2s_init(void)
{
    /* I2S A[2, 6] */
    outpw(REG_SYS_GPA_MFPL, (inpw(REG_SYS_GPA_MFPL) & ~0x0FFFFF00) | 0x02222200);
}

static void nu_pin_can_init(void)
{
#if defined(BSP_USING_CAN0)
    /* CAN0: PC3, PC4  */
    outpw(REG_SYS_GPC_MFPL, (inpw(REG_SYS_GPC_MFPL) & ~0x000FF000) | 0x00077000);
#endif

#if defined(BSP_USING_CAN1)
    /* CAN1: PG13, PG14  */
    outpw(REG_SYS_GPG_MFPH, (inpw(REG_SYS_GPG_MFPH) & ~0x0FF00000) | 0x04400000);
#endif

#if defined(BSP_USING_CAN2)
    /* CAN2: PB8, PC0  */
    outpw(REG_SYS_GPB_MFPH, (inpw(REG_SYS_GPB_MFPH) & ~0x0000000F) | 0x00000003);
    outpw(REG_SYS_GPC_MFPL, (inpw(REG_SYS_GPC_MFPL) & ~0x0000000F) | 0x00000003);
#endif

#if defined(BSP_USING_CAN3)
    /* CAN3: PA0, PA1  */
    outpw(REG_SYS_GPA_MFPL, (inpw(REG_SYS_GPA_MFPL) & ~0x000000FF) | 0x00000077);
#endif
}

static void nu_pin_adc_init(void)
{
#if defined(BSP_USING_ADC_TOUCH)
    GPIO_SetMode(PB, BIT4 | BIT5 | BIT6 | BIT7, GPIO_MODE_INPUT);
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT4 | BIT5 | BIT6 | BIT7);
#endif
}

static void nu_pin_usbd_init(void)
{
#if defined(BSP_USING_USBD)
    /* USB0_VBUSVLD, PE.11  */
    outpw(REG_SYS_GPE_MFPH, (inpw(REG_SYS_GPE_MFPH) & ~0x0000F000) | 0x00001000);
#endif
}

static void nu_pin_usbh_init(void)
{
}

void nu_pin_init(void)
{
    nu_pin_uart_init();
    nu_pin_emac_init();
    nu_pin_sdh_init();
    nu_pin_qspi_init();
    nu_pin_spi_init();
    nu_pin_i2c_init();
    nu_pin_pwm_init();
    nu_pin_i2s_init();
    nu_pin_can_init();
    nu_pin_adc_init();
    nu_pin_usbd_init();
    nu_pin_usbh_init();
}

void nu_pin_deinit(void)
{

}
