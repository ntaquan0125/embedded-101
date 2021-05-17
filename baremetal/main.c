#define __IO volatile
typedef unsigned int uint32_t;

#define SRAM_START  0x20000000U
#define SRAM_SIZE   (20U * 1024)
#define SRAM_END    (SRAM_START + SRAM_SIZE)

// Base Addresses For Peripherals
#define PERIPH_BASE 0x40000000U
#define RCC_BASE    0x40021000U
#define GPIOC_BASE  0x40011000U

// Peripheral registers structures
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;

typedef struct
{
    __IO uint32_t 	CRL;
    __IO uint32_t 	CRH;
    __IO uint32_t 	IDR;
    __IO uint32_t 	ODR;
    __IO uint32_t 	BSRR;
    __IO uint32_t 	BRR;
    __IO uint32_t 	LCKR;
} GPIO_TypeDef;

// Peripheral declaration
#define RCC     ((RCC_TypeDef *) RCC_BASE)
#define GPIOC   ((GPIO_TypeDef *) GPIOC_BASE)

// Bit definition for RCC
#define RCC_APB2ENR_IOPCEN  (1 << 4)

// Bit definition for GPIO
#define GPIO_CRH_MODE13_0   (1 << 20)
#define GPIO_ODR_ODR13      (1 << 13)
#define GPIO_BSRR_BS13      (1 << 13)
#define GPIO_BSRR_BR13      (1 << 29)

void delay(uint32_t t)
{
    while (t)
    {
        asm("nop");
        --t;
    }
}

int main (void)
{
    // GPIO init
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH |= GPIO_CRH_MODE13_0;

    while (1) {
        GPIOC->ODR ^= GPIO_ODR_ODR13;
        delay(1000000U);
    }
}

uint32_t vectors[] __attribute__ ((section(".isr_vector"))) = {
    SRAM_END,
    (uint32_t)main,
};
