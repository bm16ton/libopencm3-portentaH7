

#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/fsmc.h>
#include <libopencm3/cm3/systick.h>

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "cdc.h"
#include "portenta.h"

#define USART_CONSOLE USART1
volatile uint32_t systick = 0;
void get_buffered_line(void);

uint32_t SysTick_Config(uint32_t ticks);
static void usart_setup(void);
static void SCB_EnableDCache (void);
static void SCB_EnableICache (void);
int sdramsetup( void );

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _siitcm, _sidtcm, _sitcm, _sdtcm, _eitcm, _edtcm, _estack;

uint32_t SystemCoreClock = 400000000;
void SysTick_IRQn_handler( void ) {
  ++systick;
}

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART1, '\r');
			}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

static void usart_setup(void)
{
	/* Setup USART1 parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static inline __attribute__((always_inline)) void __WFI(void)
{
	__asm volatile ("wfi");
}

#define SCB_CCR_IC_Msk                     (1UL << SCB_CCR_IC_Pos)  
#define SCB_CCR_IC_Pos                      17U                                           //!< SCB CCR: Instruction cache enable bit Position 
#define CCSIDR_SETS(x)         (((x) & SCB_CCSIDR_NUMSETS_Msk      ) >> SCB_CCSIDR_NUMSETS_Pos      )
#define CCSIDR_WAYS(x)         (((x) & SCB_CCSIDR_ASSOCIATIVITY_Msk) >> SCB_CCSIDR_ASSOCIATIVITY_Pos)
#define SCB_CCSIDR_NUMSETS_Msk             (0x7FFFUL << SCB_CCSIDR_NUMSETS_Pos)           //!< SCB CCSIDR: NumSets Mask 
#define SCB_CCSIDR_NUMSETS_Pos             13U                                            //!< SCB CCSIDR: NumSets Position 
#define SCB_CCR_DC_Pos                      16U                                           //!< SCB CCR: Cache enable bit Position 
#define SCB_CCR_DC_Msk                     (1UL << SCB_CCR_DC_Pos)                        //!< SCB CCR: DC Mask 

#define SCB_CCSIDR_ASSOCIATIVITY_Pos        3U                                            //!< SCB CCSIDR: Associativity Position 
#define SCB_CCSIDR_ASSOCIATIVITY_Msk       (0x3FFUL << SCB_CCSIDR_ASSOCIATIVITY_Pos)      //!< SCB CCSIDR: Associativity Mask 
#define CCSIDR_WAYS(x)         (((x) & SCB_CCSIDR_ASSOCIATIVITY_Msk) >> SCB_CCSIDR_ASSOCIATIVITY_Pos)

#define SCB_DCISW_SET_Pos                   5U                                            //!< SCB DCISW: Set Position 
#define SCB_DCISW_SET_Msk                  (0x1FFUL << SCB_DCISW_SET_Pos)                 //!< SCB DCISW: Set Mask 

#define SCB_DCISW_WAY_Pos                  30U                                            //!< SCB DCISW: Way Position 
#define SCB_DCISW_WAY_Msk                  (3UL << SCB_DCISW_WAY_Pos)  

static void __DSB(void)
{
	__asm volatile ("dsb 0xF":::"memory");
}

static void __ISB(void)
{
	__asm volatile ("isb 0xF":::"memory");
}


static void SCB_EnableICache (void)
{
	__DSB();
	__ISB();
	SCB_ICIALLU = 0UL;                     /* invalidate I-Cache */
	__DSB();
	__ISB();
	SCB_CCR |=  (uint32_t)SCB_CCR_IC_Msk;  /* enable I-Cache */
	__DSB();
	__ISB();
}

static void SCB_EnableDCache (void)
{
	volatile uint32_t *SCB_CSSELR = (volatile uint32_t *)(SCB_BASE +  0x84);

	uint32_t ccsidr;
	uint32_t sets;
	uint32_t ways;

	*SCB_CSSELR = 0U; /*(0U << 1U) | 0U;*/  /* Level 1 data cache */
	__DSB();

	ccsidr = SCB_CCSIDR;

	sets = (uint32_t)(CCSIDR_SETS(ccsidr));
	do {
		ways = (uint32_t)(CCSIDR_WAYS(ccsidr));
		do {
			SCB_DCISW = (((sets << SCB_DCISW_SET_Pos) & SCB_DCISW_SET_Msk) |
					((ways << SCB_DCISW_WAY_Pos) & SCB_DCISW_WAY_Msk)  );
#if defined ( __CC_ARM )
			__schedule_barrier();
#endif
		} while (ways-- != 0U);
	} while(sets-- != 0U);
	__DSB();

	SCB_CCR |=  (uint32_t)SCB_CCR_DC_Msk;  /* enable D-Cache */

	__DSB();
	__ISB();
}


static void ulpi_pins(uint32_t gpioport, uint16_t gpiopins)
{
	gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, gpiopins);
	gpio_set_output_options(gpioport, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, gpiopins);
	gpio_set_af(gpioport, GPIO_AF10, gpiopins);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO5 on GPIO port K for LED. */
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

	/* Setup GPIO pins for USART1 rx. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

	/* Setup USART1 RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO10);

	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO5);
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO6);
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO7);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);
    gpio_set(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_clear(GPIOK, GPIO7);

    //ulpi

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO3);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO3);

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO0);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO1);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO1);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO10);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO10);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO11);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO12);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO12);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO13);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO13);

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO0);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0);

	gpio_mode_setup(GPIOH, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO4);
	gpio_set_output_options(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO4);

	gpio_mode_setup(GPIOI, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO11);
	gpio_set_output_options(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO11);

	gpio_set_af(GPIOA, GPIO_AF10, GPIO3);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO5);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO0);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO1);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO5);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO10);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO11);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO12);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO13);
	gpio_set_af(GPIOC, GPIO_AF10, GPIO0);
	gpio_set_af(GPIOH, GPIO_AF10, GPIO4);
	gpio_set_af(GPIOI, GPIO_AF10, GPIO11);

	ulpi_pins(GPIOA, GPIO3 | GPIO5);
	ulpi_pins(GPIOB, GPIO0 | GPIO1 | GPIO5  | GPIO10 | GPIO11 | GPIO12 | GPIO13);
	ulpi_pins(GPIOC, GPIO0);
	ulpi_pins(GPIOH, GPIO4);
	ulpi_pins(GPIOI, GPIO11);

}

static void InitLdoAndPll(void) {
  // Clock configuration for this platform.
  // clang-format off
  static const struct rcc_pll_config pll_config = {
    .sysclock_source  = RCC_PLL,
    .pll_source       = RCC_PLLCKSELR_PLLSRC_HSE,
    .hse_frequency    = 25000000UL,
    .pll1 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 160U,   // 800MHz for PLL1 to drive the sysclk throu post-div of 2.
        .divp = 2U,     // PLL1P post-divider gives 400MHz output.
        .divq = 16U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 2U,                // PLL1R are disabled for now.
    },
    .pll2 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 80U,   // 800MHz for PLL1 to drive the sysclk throu post-div of 2.
        .divp = 2U,     // PLL1P post-divider gives 400MHz output.
        .divq = 2U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 2U,    // MAY NEED TO BE 4 FOR SDRAM          // PLL1R are disabled for now.
    },
    .pll3 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 160U,   // 800MHz for PLL1 to drive the sysclk throu post-div of 2.
        .divp = 2U,     // PLL1P post-divider gives 400MHz output.
        .divq = 5U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 2U,                // PLL1R are disabled for now.
    },
    // Set CPU to PLL1P output at 480MHz.
    .core_pre  = RCC_D1CFGR_D1CPRE_BYP,
    .hpre = RCC_D1CFGR_D1HPRE_DIV2,   // Constrain HCLK below 240MHz by dividing core clock by 2.
    .ppre1 = RCC_D2CFGR_D2PPRE_DIV2,  // Constrain APB1 below 120MHz by dividing HCLK3 by 2.
    .ppre2 = RCC_D2CFGR_D2PPRE_DIV2,  // Constrain APB2 below 120MHz by dividing HCLK3 by 2.
    .ppre3 = RCC_D1CFGR_D1PPRE_DIV2,  // Constrain APB3 below 120MHz by dividing HCLK3 by 2.
    .ppre4 = RCC_D3CFGR_D3PPRE_DIV2,  // Constrain APB4 below 120MHz by dividing HCLK3 by 2.
    .flash_waitstates = 4,
    .voltage_scale = PWR_VOS_SCALE_0, // Highest setting, should support 480MHz operation.
  };
  // clang-format on
rcc_clock_setup_pll(&pll_config);

  // Setup SPI buses to use the HSI clock @ 64MHz for SPI log2 based dividers.
rcc_set_spi123_clksel(RCC_D2CCIP1R_SPI123SEL_PERCK);  // PERCLK is defaulted to HSI.
  rcc_set_spi45_clksel(RCC_D2CCIP1R_SPI45SEL_HSI);

  // Set CANFD to use PLL1Q set at 80MHz. 
  rcc_set_fdcan_clksel(RCC_D2CCIP1R_FDCANSEL_PLL1Q);
  
  rcc_periph_clock_enable(RCC_USB1OTGHSEN);
  rcc_periph_clock_enable(RCC_USB1OTGHSULPIEN);
  
}

static struct {
	uint32_t	gpio;
	uint16_t	pins;
} sdram_pins[5] = {
	{GPIOD, GPIO0 | GPIO1 | GPIO8 | GPIO9 | GPIO10 | GPIO14 | GPIO15},
	{GPIOE, GPIO0 | GPIO1 | GPIO7 | GPIO8 | GPIO9 | GPIO10 |
			GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15 },
	{GPIOF, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO11 |
			GPIO12 | GPIO13 | GPIO14 | GPIO15 },
	{GPIOG, GPIO0 | GPIO1 | GPIO2 | GPIO4 | GPIO5 | GPIO8 | GPIO15},
	{GPIOH, GPIO2 | GPIO5 | GPIO3}
};

static struct sdram_timing timing = {
	.trcd = 2,		/* RCD Delay */
	.trp = 2,		/* RP Delay */
	.twr = 2,		/* Write Recovery Time */
	.trc = 7,		/* Row Cycle Delay */
	.tras = 4,		/* Self Refresh Time */
	.txsr = 7,		/* Exit Self Refresh Time */
	.tmrd = 2,		/* Load to Active Delay */
};

uint32_t SysTick_Config(uint32_t ticks)
{
//  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
//  {
//    return (1UL);                                                   /* Reload value impossible */
//  }
(void)ticks;
systick_set_reload(400000000-1);
//  SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
//  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
 // SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  	systick_counter_enable();
  	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  		systick_interrupt_enable();
//  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
//                   SysTick_CTRL_TICKINT_Msk   |
//                   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}


int sdramsetup( void ) {

	uint32_t cr_tmp, tr_tmp; /* control, timing registers */

   rcc_periph_clock_enable(RCC_FMC);

  SCB_CPACR  |=  ( 0xF << 20 );
  SysTick_Config( SystemCoreClock / 1000 );

	for (int i = 0; i < 5; i++) {
		gpio_mode_setup(sdram_pins[i].gpio, GPIO_MODE_AF,
				GPIO_PUPD_NONE, sdram_pins[i].pins);
		gpio_set_output_options(sdram_pins[i].gpio, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, sdram_pins[i].pins);
		gpio_set_af(sdram_pins[i].gpio, GPIO_AF12, sdram_pins[i].pins);
	}
  printf( "Configure FMC...\r\n" );

  // Enable the FMC peripheral.
  FSMC_BCR(0)   |=  ( FSMC_BCR_FMCEN );

	cr_tmp  = FMC_SDCR_RPIPE_1CLK;
	cr_tmp |= FMC_SDCR_SDCLK_2HCLK;
	cr_tmp |= FMC_SDCR_CAS_3CYC;
	cr_tmp |= FMC_SDCR_NB4;
	cr_tmp |= FMC_SDCR_MWID_16b;
	cr_tmp |= FMC_SDCR_NR_12;
	cr_tmp |= FMC_SDCR_NC_8;

	FMC_SDCR1 |= (cr_tmp & FMC_SDCR_DNC_MASK);
	FMC_SDCR2 = cr_tmp;

	tr_tmp = sdram_timing(&timing);
	FMC_SDTR1 |= (tr_tmp & FMC_SDTR_DNC_MASK);
	FMC_SDTR2 = tr_tmp;
                                 
    FMC_SDCMR     =  ( ( 1 << FMC_SDCMR_MODE_MASK ) |
                       FMC_SDCMR_CTB1 );

	for (unsigned i = 0; i < 20; i++)
	  {
		__asm__("nop");
	  }
	  
	    while( FMC_SDSR & 0x00000020 ) {};
	    
	      FMC_SDCMR     =  ( ( 2 << FMC_SDCMR_MODE_MASK ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_SDSR & 0x00000020 ) {};

  FMC_SDCMR     =  ( ( 1 << FMC_SDCMR_NRFS_SHIFT ) |
                                ( 3 << FMC_SDCMR_MODE_SHIFT ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_SDSR & 0x00000020 ) {};

  FMC_SDCMR     =  ( ( 0x020 << FMC_SDCMR_MRD_SHIFT ) |
                                ( 4 << FMC_SDCMR_MODE_SHIFT ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_SDSR & 0x00000020 ) {};
 
   // Configure SDRTR Refresh Timing Register:
  // 7.8us * 120MHz - 20 = 916.
  FMC_SDRTR     |=  ( 916 << FMC_SDRTR_COUNT_SHIFT );

  // Make sure writes are enabled.
  FMC_SDCR1 &= ~( FMC_SDCR_WP_ENABLE );

  printf( "Done.\r\n" );
  return 0;
}	
	
int ramtest(void) {
	  // Test external RAM reads and writes.
  uint32_t* sdram  = ( uint32_t* )0x60000000;
  uint16_t* sdramh = ( uint16_t* )0x60000000;
  uint8_t*  sdramb = ( uint8_t*  )0x60000000;
  printf( "RAM[0]: 0x%08lX (Uninitialized)\r\n", sdram[ 0 ] );
  sdram[ 0 ] = 0x01234567;
  sdram[ 1 ] = 0x89ABCDEF;
  printf( "RAM[0]: 0x%02X (Byte)\r\n", sdramb[ 0 ] );
  printf( "RAM[0]: 0x%04X (Halfword)\r\n", sdramh[ 0 ] );
  printf( "RAM[0]: 0x%08lX (Word)\r\n", sdram[ 0 ] );
  printf( "RAM[4]: 0x%08lX\r\n", sdram[ 1 ] );

  // Done; infinite loop.
 //  while( 1 ) {};
  return 0; // lol
}
	
void startportenta(void) {
    InitLdoAndPll();
	/* Enable clocks for LED & USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_GPIOG);
	rcc_periph_clock_enable(RCC_GPIOH);
	rcc_periph_clock_enable(RCC_GPIOI);
	rcc_periph_clock_enable(RCC_GPIOJ);
	rcc_periph_clock_enable(RCC_GPIOK);
    gpio_setup();
    gpio_set(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_clear(GPIOK, GPIO7);
	/* Enable clocks for USART1. */
	
	SCB_EnableICache();
	SCB_EnableDCache();
    rcc_periph_clock_enable(RCC_USART1);
    usart_setup();
    sdramsetup();
    ramtest();
}
