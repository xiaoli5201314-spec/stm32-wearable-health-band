/*
 * stm32f405_regs.h -- STM32F405 外设寄存器定义（原创，不依赖 CMSIS / HAL）
 *
 * 只声明本工程真正用到的外设，寄存器地址与 RM0090 参考手册一致。
 * 用带 offset 的宏而不是完整结构体，是为了避免结构体填充差异带来的地址错位风险。
 */
#ifndef STM32F405_REGS_H
#define STM32F405_REGS_H

#include <stdint.h>

#define REG32(addr)  (*(volatile uint32_t *)(uintptr_t)(addr))

/* ---------------- 外设基址 ---------------- */
#define PERIPH_RCC_BASE      0x40023800u
#define PERIPH_GPIOA_BASE    0x40020000u
#define PERIPH_GPIOB_BASE    0x40020400u
#define PERIPH_GPIOC_BASE    0x40020800u
#define PERIPH_I2C1_BASE     0x40005400u
#define PERIPH_USART1_BASE   0x40011000u
#define PERIPH_USART2_BASE   0x40004400u
#define PERIPH_ADC1_BASE     0x40012000u
#define PERIPH_ADC_COMMON    0x40012300u
#define PERIPH_PWR_BASE      0x40007000u
#define PERIPH_RTC_BASE      0x40002800u
#define PERIPH_IWDG_BASE     0x40003000u
#define PERIPH_EXTI_BASE     0x40013C00u
#define PERIPH_SYSCFG_BASE   0x40013800u
#define PERIPH_FLASH_BASE    0x40023C00u
#define PERIPH_TIM2_BASE     0x40000000u

/* ---------------- RCC ---------------- */
#define RCC_CR         (PERIPH_RCC_BASE + 0x00u)
#define RCC_PLLCFGR    (PERIPH_RCC_BASE + 0x04u)
#define RCC_CFGR       (PERIPH_RCC_BASE + 0x08u)
#define RCC_AHB1ENR    (PERIPH_RCC_BASE + 0x30u)
#define RCC_APB1ENR    (PERIPH_RCC_BASE + 0x40u)
#define RCC_APB2ENR    (PERIPH_RCC_BASE + 0x44u)
#define RCC_BDCR       (PERIPH_RCC_BASE + 0x70u)
#define RCC_CSR        (PERIPH_RCC_BASE + 0x74u)

/* RCC_CR：HSEON bit16, HSERDY bit17, PLLON bit24, PLLRDY bit25 */
#define RCC_CR_HSEON      0x00010000u
#define RCC_CR_HSERDY     0x00020000u
#define RCC_CR_PLLON      0x01000000u
#define RCC_CR_PLLRDY     0x02000000u

/* RCC_CFGR：SW[1:0] bit0-1, SWS[3:2], HPRE[7:4], PPRE1[12:10], PPRE2[15:13] */
#define RCC_CFGR_SW_PLL   0x00000002u
#define RCC_CFGR_SWS_MASK 0x0000000Cu
#define RCC_CFGR_SWS_PLL  0x00000008u

/* RCC_BDCR：LSEON bit0, LSERDY bit1, RTCSEL[9:8], RTCEN bit15, BDRST bit16 */
#define RCC_BDCR_LSEON    0x00000001u
#define RCC_BDCR_LSERDY   0x00000002u
#define RCC_BDCR_RTCSEL_LSE 0x00000100u
#define RCC_BDCR_RTCEN    0x00008000u
#define RCC_BDCR_BDRST    0x00010000u

/* RCC_CSR：LSION bit0, LSIRDY bit1, IWDGRSTF bit29, SFTRSTF bit28, PINRSTF bit26, PORRSTF bit27 */
#define RCC_CSR_LSION     0x00000001u
#define RCC_CSR_LSIRDY    0x00000002u
#define RCC_CSR_IWDGRSTF  0x20000000u
#define RCC_CSR_RMVF      0x01000000u

/* RCC_AHB1ENR 时钟使能位 */
#define RCC_AHB1EN_GPIOA  0x00000001u
#define RCC_AHB1EN_GPIOB  0x00000002u
#define RCC_AHB1EN_GPIOC  0x00000004u
#define RCC_APB1EN_I2C1   0x00200000u
#define RCC_APB1EN_PWR    0x10000000u
#define RCC_APB2EN_USART1 0x00000010u
#define RCC_APB1EN_USART2 0x00020000u
#define RCC_APB2EN_ADC1   0x00000100u
#define RCC_APB1EN_TIM2   0x00000001u

/* ---------------- GPIO ---------------- */
#define GPIO_MODER(base)    ((base) + 0x00u)
#define GPIO_OTYPER(base)   ((base) + 0x04u)
#define GPIO_OSPEEDR(base)  ((base) + 0x08u)
#define GPIO_PUPDR(base)    ((base) + 0x0Cu)
#define GPIO_IDR(base)      ((base) + 0x10u)
#define GPIO_ODR(base)      ((base) + 0x14u)
#define GPIO_BSRR(base)     ((base) + 0x18u)
#define GPIO_AFRL(base)     ((base) + 0x20u)
#define GPIO_AFRH(base)     ((base) + 0x24u)

#define GPIO_MODE_INPUT     0x0u
#define GPIO_MODE_OUTPUT    0x1u
#define GPIO_MODE_AF        0x2u
#define GPIO_MODE_ANALOG    0x3u
#define GPIO_PUPD_PULLUP    0x1u
#define GPIO_OTYPE_OD       0x1u   /* I2C 必须开漏 */
#define GPIO_OSPEED_VHIGH   0x3u
#define GPIO_AF_I2C1        0x4u
#define GPIO_AF_USART1      0x7u

/* ---------------- I2C (RM0090 §27) ---------------- */
#define I2C_CR1        (PERIPH_I2C1_BASE + 0x00u)
#define I2C_CR2        (PERIPH_I2C1_BASE + 0x04u)
#define I2C_OAR1       (PERIPH_I2C1_BASE + 0x08u)
#define I2C_DR         (PERIPH_I2C1_BASE + 0x10u)
#define I2C_SR1        (PERIPH_I2C1_BASE + 0x14u)
#define I2C_SR2        (PERIPH_I2C1_BASE + 0x18u)
#define I2C_CCR        (PERIPH_I2C1_BASE + 0x1Cu)
#define I2C_TRISE      (PERIPH_I2C1_BASE + 0x20u)
#define I2C_FLTR       (PERIPH_I2C1_BASE + 0x24u)

#define I2C_CR1_PE     0x00000001u
#define I2C_CR1_START  0x00000100u
#define I2C_CR1_STOP   0x00000200u
#define I2C_CR1_ACK    0x00000400u
#define I2C_CR1_POS    0x00000800u
#define I2C_CR1_SWRST  0x00008000u
#define I2C_SR1_SB     0x00000001u
#define I2C_SR1_ADDR   0x00000002u
#define I2C_SR1_BTF    0x00000004u
#define I2C_SR1_RXNE   0x00000040u
#define I2C_SR1_TXE    0x00000080u
#define I2C_SR1_AF     0x00000400u
#define I2C_SR2_MSL    0x00000001u
#define I2C_SR2_BUSY   0x00000002u

/* ---------------- USART ---------------- */
#define USART_SR       (PERIPH_USART1_BASE + 0x00u)
#define USART_DR       (PERIPH_USART1_BASE + 0x04u)
#define USART_BRR      (PERIPH_USART1_BASE + 0x08u)
#define USART_CR1      (PERIPH_USART1_BASE + 0x0Cu)
#define USART_CR3      (PERIPH_USART1_BASE + 0x14u)
#define USART_SR_RXNE  0x00000020u
#define USART_SR_TXE   0x00000080u
#define USART_SR_TC    0x00000040u
#define USART_SR_ORE   0x00000008u
#define USART_CR1_UE   0x00002000u
#define USART_CR1_TE   0x00000008u
#define USART_CR1_RE   0x00000004u
#define USART_CR1_RXNEIE 0x00000020u

/* ---------------- ADC ---------------- */
#define ADC_SR         (PERIPH_ADC1_BASE + 0x00u)
#define ADC_CR1        (PERIPH_ADC1_BASE + 0x04u)
#define ADC_CR2        (PERIPH_ADC1_BASE + 0x08u)
#define ADC_SMPR1      (PERIPH_ADC1_BASE + 0x0Cu)
#define ADC_SMPR2      (PERIPH_ADC1_BASE + 0x10u)
#define ADC_SQR1       (PERIPH_ADC1_BASE + 0x2Cu)
#define ADC_SQR3       (PERIPH_ADC1_BASE + 0x34u)
#define ADC_DR         (PERIPH_ADC1_BASE + 0x4Cu)
#define ADC_CCR        (PERIPH_ADC_COMMON + 0x04u)

#define ADC_CR2_ADON    0x00000001u
#define ADC_CR2_SWSTART 0x40000000u
#define ADC_CR2_EOCS    0x00000200u
#define ADC_CR1_RES_12B 0x00000000u
#define ADC_SR_EOC      0x00000002u
#define ADC_CCR_TSVREFE 0x00800000u   /* 使能内部温度/基准通道 */

/* ---------------- PWR / EXTI / SYSCFG / FLASH ---------------- */
#define PWR_CR         (PERIPH_PWR_BASE + 0x00u)
#define PWR_CSR        (PERIPH_PWR_BASE + 0x04u)
#define PWR_CR_LPDS    0x00000001u   /* 深度睡眠下调节器低功耗 */
#define PWR_CR_PDDS    0x00000002u   /* 1 = 进入 STANDBY（本工程用 STOP，置 0） */
#define PWR_CR_FPDS    0x00000002u
#define PWR_CR_CWUF    0x00000004u
#define PWR_CR_VOS     0x00004000u

#define EXTI_IMR       (PERIPH_EXTI_BASE + 0x00u)
#define EXTI_EMR       (PERIPH_EXTI_BASE + 0x04u)
#define EXTI_RTSR      (PERIPH_EXTI_BASE + 0x08u)
#define EXTI_FTSR      (PERIPH_EXTI_BASE + 0x0Cu)
#define EXTI_PR        (PERIPH_EXTI_BASE + 0x14u)

#define SYSCFG_EXTICR1 (PERIPH_SYSCFG_BASE + 0x08u)
#define SYSCFG_EXTICR2 (PERIPH_SYSCFG_BASE + 0x0Cu)
#define SYSCFG_MEMRMP  (PERIPH_SYSCFG_BASE + 0x00u)

#define FLASH_ACR      (PERIPH_FLASH_BASE + 0x00u)
#define FLASH_ACR_ICEN 0x00000200u
#define FLASH_ACR_DCEN 0x00000400u
#define FLASH_ACR_PRFTEN 0x00000100u

/* ---------------- TIM2（调度器节拍源） ---------------- */
#define TIM2_CR1       (PERIPH_TIM2_BASE + 0x00u)
#define TIM2_DIER      (PERIPH_TIM2_BASE + 0x0Cu)
#define TIM2_SR        (PERIPH_TIM2_BASE + 0x10u)
#define TIM2_CNT       (PERIPH_TIM2_BASE + 0x24u)
#define TIM2_PSC       (PERIPH_TIM2_BASE + 0x28u)
#define TIM2_ARR       (PERIPH_TIM2_BASE + 0x2Cu)
#define TIM2_CR1_CEN   0x00000001u
#define TIM2_DIER_UIE  0x00000001u
#define TIM2_SR_UIF    0x00000001u

/* ---------------- Cortex-M4 内核指令 ---------------- */
static inline void cpu_wfi(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile ("wfi");
#endif
}

static inline void cpu_dsb(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile ("dsb" ::: "memory");
#else
    __asm volatile ("" ::: "memory");
#endif
}

#endif /* STM32F405_REGS_H */
