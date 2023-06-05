
#define SCB_DCCSW_SET_Pos                   5U                                            /*!< SCB DCCSW: Set Position */
#define SCB_DCCSW_SET_Msk                  (0x1FFUL << SCB_DCCSW_SET_Pos)                 /*!< SCB DCCSW: Set Mask */
#define SCB_CSSELR_LEVEL_Pos                1U                                            /*!< SCB CSSELR: Level Position */
#define SCB_CSSELR_LEVEL_Msk               (7UL << SCB_CSSELR_LEVEL_Pos)                  /*!< SCB CSSELR: Level Mask */
#define SCB_CCR_IC_Msk                     (1UL << SCB_CCR_IC_Pos)
#define SCB_CCR_IC_Pos                      17U
#define SCB_CSSELR_IND_Pos                  0U                                            /*!< SCB CSSELR: InD Position */
#define SCB_CSSELR_IND_Msk                 (1UL /*<< SCB_CSSELR_IND_Pos*/)                /*!< SCB CSSELR: InD Mask */
#define SCB_DCCSW_WAY_Pos                  30U                                            /*!< SCB DCCSW: Way Position */
#define SCB_DCCSW_WAY_Msk                  (3UL << SCB_DCCSW_WAY_Pos)                     /*!< SCB DCCSW: Way Mask */

#define SCB_CCR_DC_Pos                      16U                                           /*!< SCB CCR: Cache enable bit Position */
#define SCB_CCR_DC_Msk                     (1UL << SCB_CCR_DC_Pos)                        /*!< SCB CCR: Cache enable bit Mask */
#define SCB_CCSIDR_NUMSETS_Pos             13U                                            /*!< SCB CCSIDR: NumSets Position */
#define SCB_CCSIDR_NUMSETS_Msk             (0x7FFFUL << SCB_CCSIDR_NUMSETS_Pos)           /*!< SCB CCSIDR: NumSets Mask */
#define SCB_CCSIDR_ASSOCIATIVITY_Pos        3U                                            /*!< SCB CCSIDR: Associativity Position */
#define SCB_CCSIDR_ASSOCIATIVITY_Msk       (0x3FFUL << SCB_CCSIDR_ASSOCIATIVITY_Pos)      /*!< SCB CCSIDR: Associativity Mask */
#define CCSIDR_WAYS(x)         (((x) & SCB_CCSIDR_ASSOCIATIVITY_Msk) >> SCB_CCSIDR_ASSOCIATIVITY_Pos)
#define CCSIDR_SETS(x)         (((x) & SCB_CCSIDR_NUMSETS_Msk      ) >> SCB_CCSIDR_NUMSETS_Pos      )

#define SCB_DCCISW_WAY_Pos                 30U                                            /*!< SCB DCCISW: Way Position */
#define SCB_DCCISW_WAY_Msk                 (3UL << SCB_DCCISW_WAY_Pos)                    /*!< SCB DCCISW: Way Mask */
#define SCB_DCCISW_SET_Pos                  5U                                            /*!< SCB DCCISW: Set Position */
#define SCB_DCCISW_SET_Msk                 (0x1FFUL << SCB_DCCISW_SET_Pos)                /*!< SCB DCCISW: Set Mask */


static inline __attribute__((always_inline))  void __ISB(void)
{
	__asm volatile ("isb 0xF":::"memory");
}

static inline __attribute__((always_inline)) void __DSB(void)
{
  __asm volatile ("dsb 0xF":::"memory");
}


static inline __attribute__((always_inline)) void SCB_CleanDCache (void)
{

    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

     SCB_CCSELR = 0U; /*(0U << 1U) | 0U;*/  /* Level 1 data cache */
   __DSB();

    ccsidr = SCB_CCSELR;

                                            /* clean D-Cache */
    sets = (uint32_t)(CCSIDR_SETS(ccsidr));
    do {
      ways = (uint32_t)(CCSIDR_WAYS(ccsidr));
      do {
        SCB_DCCSW = (((sets << SCB_DCCSW_SET_Pos) & SCB_DCCSW_SET_Msk) |
                      ((ways << SCB_DCCSW_WAY_Pos) & SCB_DCCSW_WAY_Msk)  );
        #if defined ( __CC_ARM )
          __schedule_barrier();
        #endif
      } while (ways-- != 0U);
    } while(sets-- != 0U);

    __DSB();
    __ISB();

}

static inline __attribute__((always_inline)) void SCB_CleanInvalidateDCache (void)
{

    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

    SCB_CCSELR = 0U; /*(0U << 1U) | 0U;*/  /* Level 1 data cache */
    __DSB();

    ccsidr = SCB_CCSIDR;

                                            /* clean & invalidate D-Cache */
    sets = (uint32_t)(CCSIDR_SETS(ccsidr));
    do {
      ways = (uint32_t)(CCSIDR_WAYS(ccsidr));
      do {
        SCB_DCCISW = (((sets << SCB_DCCISW_SET_Pos) & SCB_DCCISW_SET_Msk) |
                       ((ways << SCB_DCCISW_WAY_Pos) & SCB_DCCISW_WAY_Msk)  );
        #if defined ( __CC_ARM )
          __schedule_barrier();
        #endif
      } while (ways-- != 0U);
    } while(sets-- != 0U);

    __DSB();
    __ISB();

}

static inline __attribute__((always_inline)) void SCB_CleanDCache_by_Addr (uint32_t *addr, int32_t dsize)
{

     int32_t op_size = dsize;
    uint32_t op_addr = (uint32_t) addr;
     int32_t linesize = 32;                /* in Cortex-M7 size of cache line is fixed to 8 words (32 bytes) */

    __DSB();

    while (op_size > 0) {
      SCB_DCCMVAC = op_addr;
      op_addr += (uint32_t)linesize;
      op_size -=           linesize;
    }

    __DSB();
    __ISB();

}

static inline __attribute__((always_inline)) void SCB_EnableICache(void)
{
	__asm volatile ("dsb");
	__asm volatile ("isb");

	SCB_ICIALLU = 0UL;                     // invalidate I-Cache

	__asm volatile ("dsb");
	__asm volatile ("isb");

	SCB_CCR |=  (uint32_t)SCB_CCR_IC_Msk;  // enable I-Cache

	__asm volatile ("dsb");
	__asm volatile ("isb");
}

static inline __attribute__((always_inline)) void SCB_EnableDCache(void)
{
	__asm volatile ("dsb");
	__asm volatile ("isb");
 SCB_CCR |=  (uint32_t)SCB_CCR_DC;  // enable D-Cache

	__asm volatile ("dsb");
	__asm volatile ("isb");
}

static inline __attribute__((always_inline))  void SCB_DisableDCache (void)
{

    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

    SCB_CCSELR = 0U; /*(0U << 1U) | 0U;*/  /* Level 1 data cache */
    __DSB();

    SCB_CCR &= ~(uint32_t)SCB_CCR_DC_Msk;  /* disable D-Cache */
    __DSB();

    ccsidr = SCB_CCSIDR;

                                            /* clean & invalidate D-Cache */
    sets = (uint32_t)(CCSIDR_SETS(ccsidr));
    do {
      ways = (uint32_t)(CCSIDR_WAYS(ccsidr));
      do {
        SCB_DCCISW = (((sets << SCB_DCCISW_SET_Pos) & SCB_DCCISW_SET_Msk) |
                       ((ways << SCB_DCCISW_WAY_Pos) & SCB_DCCISW_WAY_Msk)  );
        #if defined ( __CC_ARM )
          __schedule_barrier();
        #endif
      } while (ways-- != 0U);
    } while(sets-- != 0U);

    __DSB();
    __ISB();

}

