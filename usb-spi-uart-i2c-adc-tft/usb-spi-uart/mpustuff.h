
#define MPU_RASR_S_Pos                     18U                                            /*!< MPU RASR: ATTRS.S Position */
#define MPU_RASR_S_Msk                     (1UL << MPU_RASR_S_Pos)                        /*!< MPU RASR: ATTRS.S Mask */

#define ARM_MPU_ACCESS_(TypeExtField, IsShareable, IsCacheable, IsBufferable)   \
  ((((TypeExtField ) << MPU_RASR_TEX_Pos) & MPU_RASR_TEX_Msk)                 | \
   (((IsShareable ) << MPU_RASR_S_Pos) & MPU_RASR_S_Msk)                      | \
   (((IsCacheable ) << MPU_RASR_C_Pos) & MPU_RASR_C_Msk)                      | \
   (((IsBufferable ) << MPU_RASR_B_Pos) & MPU_RASR_B_Msk))

#define MPU_RASR_TEX_Pos                   19U                                            /*!< MPU RASR: ATTRS.TEX Position */
#define MPU_RASR_TEX_Msk                   (0x7UL << MPU_RASR_TEX_Pos)                    /*!< MPU RASR: ATTRS.TEX Mask */
#define MPU_RASR_C_Pos                     17U                                            /*!< MPU RASR: ATTRS.C Position */
#define MPU_RASR_C_Msk                     (1UL << MPU_RASR_C_Pos)                        /*!< MPU RASR: ATTRS.C Mask */
#define MPU_RASR_B_Pos                     16U                                            /*!< MPU RASR: ATTRS.B Position */
#define MPU_RASR_B_Msk                     (1UL << MPU_RASR_B_Pos)                        /*!< MPU RASR: ATTRS.B Mask */

#define MPU_RASR_AP_Pos                    24U                                            /*!< MPU RASR: ATTRS.AP Position */
#define MPU_RASR_AP_Msk                    (0x7UL << MPU_RASR_AP_Pos)                     /*!< MPU RASR: ATTRS.AP Mask */
#define ARM_MPU_AP_NONE 0U ///!< MPU Access Permission no access
#define ARM_MPU_AP_PRIV 1U ///!< MPU Access Permission privileged access only
#define ARM_MPU_AP_URO  2U ///!< MPU Access Permission unprivileged access read-only
#define ARM_MPU_AP_FULL 3U ///!< MPU Access Permission full access
#define ARM_MPU_AP_PRO  5U ///!< MPU Access Permission privileged access read-only
#define ARM_MPU_AP_RO   6U ///!< MPU Access Permission read-only access
#define MPU_RASR_XN_Pos                    28U                                            /*!< MPU RASR: ATTRS.XN Position */
#define MPU_RASR_XN_Msk                    (1UL << MPU_RASR_XN_Pos)                       /*!< MPU RASR: ATTRS.XN Mask */
#define ARM_MPU_ACCESS_DEVICE(IsShareable) ((IsShareable) ? ARM_MPU_ACCESS_(0U, 1U, 0U, 1U) : ARM_MPU_ACCESS_(2U, 0U, 0U, 0U))
#define MPU_TYPE			MMIO32(MPU_BASE + 0x00)
//#define MPU               (((MPU_TYPE       *)     MPU_BASE      ))   /*!< Memory Protection Unit */
#define ARM_MPU_CACHEP_NOCACHE 0U
#define ARM_MPU_RASR_EX(DisableExec, AccessPermission, AccessAttributes, SubRegionDisable, Size)      \
  ((((DisableExec ) << MPU_RASR_XN_Pos) & MPU_RASR_XN_Msk)                                          | \
   (((AccessPermission) << MPU_RASR_AP_Pos) & MPU_RASR_AP_Msk)                                      | \
   (((AccessAttributes) ) & (MPU_RASR_TEX_Msk | MPU_RASR_S_Msk | MPU_RASR_C_Msk | MPU_RASR_B_Msk)))
#define ARM_MPU_RASR(DisableExec, AccessPermission, TypeExtField, IsShareable, IsCacheable, IsBufferable, SubRegionDisable, Size) \
ARM_MPU_RASR_EX(DisableExec, AccessPermission, ARM_MPU_ACCESS_(TypeExtField, IsShareable, IsCacheable, IsBufferable), SubRegionDisable, Size)
#define ARM_MPU_ACCESS_DEVICE(IsShareable) ((IsShareable) ? ARM_MPU_ACCESS_(0U, 1U, 0U, 1U) : ARM_MPU_ACCESS_(2U, 0U, 0U, 0U))
#define MPU_CTRL_PRIVDEFENA_Pos             2U                                            /*!< MPU CTRL: PRIVDEFENA Position */
#define MPU_CTRL_PRIVDEFENA_Msk            (1UL << MPU_CTRL_PRIVDEFENA_Pos)               /*!< MPU CTRL: PRIVDEFENA Mask */

#define VALID 0x10
#define REGION_Enabled  (0x01)
#define REGION_32K      (14 << 1)      // 2**15 == 32k
#define NORMAL          (8 << 16)      // TEX:0b001 S:0b0 C:0b0 B:0b0
#define FULL_ACCESS     (0x03 << 24)   // Privileged Read Write, Unprivileged Read Write
#define NOT_EXEC        (0x01 << 28)   // All Instruction fetches abort
#define MPU_RGNSZ_32B      ((uint8_t)0x04U)
#define MPU_RGNSZ_64B      ((uint8_t)0x05U)
#define MPU_RGNSZ_128B     ((uint8_t)0x06U)
#define MPU_RGNSZ_256B     ((uint8_t)0x07U)
#define MPU_RGNSZ_512B     ((uint8_t)0x08U)
#define MPU_RGNSZ_1KB      ((uint8_t)0x09U)
#define MPU_RGNSZ_2KB      ((uint8_t)0x0AU)
#define MPU_RGNSZ_4KB      ((uint8_t)0x0BU)
#define MPU_RGNSZ_8KB      ((uint8_t)0x0CU)
#define MPU_RGNSZ_16KB     ((uint8_t)0x0DU)
#define MPU_RGNSZ_32KB     ((uint8_t)0x0EU)
#define MPU_RGNSZ_64KB     ((uint8_t)0x0FU)
#define MPU_RGNSZ_128KB    ((uint8_t)0x10U)
#define MPU_RGNSZ_256KB    ((uint8_t)0x11U)
#define MPU_RGNSZ_512KB    ((uint8_t)0x12U)
#define MPU_RGNSZ_1MB      ((uint8_t)0x13U)
#define MPU_RGNSZ_2MB      ((uint8_t)0x14U)
#define MPU_RGNSZ_4MB      ((uint8_t)0x15U)
#define MPU_RGNSZ_8MB      ((uint8_t)0x16U)
#define MPU_RGNSZ_16MB     ((uint8_t)0x17U)
#define MPU_RGNSZ_32MB     ((uint8_t)0x18U)
#define MPU_RGNSZ_64MB     ((uint8_t)0x19U)
#define MPU_RGNSZ_128MB    ((uint8_t)0x1AU)
#define MPU_RGNSZ_256MB    ((uint8_t)0x1BU)
#define MPU_RGNSZ_512MB    ((uint8_t)0x1CU)
#define MPU_RGNSZ_1GB      ((uint8_t)0x1DU)
#define MPU_RGNSZ_2GB      ((uint8_t)0x1EU)
#define MPU_RGNSZ_4GB      ((uint8_t)0x1FU)
#define ARM_MPU_CACHEP_NOCACHE 0U
#define ARM_MPU_AP_RO   6U
#define MPU_CTRL_ENABLE_Msk                (1UL /*<< MPU_CTRL_ENABLE_Pos*/)               /*!< MPU CTRL: ENABLE Mask */

static inline void ARM_MPU_SetRegionEx(uint32_t rnr, uint32_t rbar, uint32_t rasr)
{
  MPU_RNR = rnr;
  MPU_RBAR = rbar;
  MPU_RASR = rasr;
}

static inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  __DSB();
  __ISB();
  MPU_CTRL = MPU_Control | MPU_CTRL_ENABLE_Msk;
#ifdef SCB_SHCSR_MEMFAULTENA_Msk
  SCB_SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
#endif
}

/*
void enable_mpu(void);

#define VALID 0x10
#define REGION_Enabled  (0x01)
#define REGION_32K      (14 << 1)      // 2**15 == 32k
#define NORMAL          (8 << 16)      // TEX:0b001 S:0b0 C:0b0 B:0b0
#define FULL_ACCESS     (0x03 << 24)   // Privileged Read Write, Unprivileged Read Write
#define NOT_EXEC        (0x01 << 28)   // All Instruction fetches abort
#define ARM_MPU_REGION_SIZE_16KB     ((uint8_t)0x0DU) ///!< MPU Region Size 16 KBytes
#define ARM_MPU_REGION_SIZE_32KB     ((uint8_t)0x0EU) ///!< MPU Region Size 32 KBytes
#define ARM_MPU_REGION_SIZE_64KB     ((uint8_t)0x0FU) ///!< MPU Region Size 64 KBytes
#define ARM_MPU_REGION_SIZE_128KB    ((uint8_t)0x10U) ///!< MPU Region Size 128 KBytes
#define ARM_MPU_REGION_SIZE_256KB    ((uint8_t)0x11U) ///!< MPU Region Size 256 KBytes
#define ARM_MPU_REGION_SIZE_512KB    ((uint8_t)0x12U) ///!< MPU Region Size 512 KBytes
#define ARM_MPU_REGION_SIZE_1MB      ((uint8_t)0x13U) ///!< MPU Region Size 1 MByte
#define ARM_MPU_REGION_SIZE_2MB      ((uint8_t)0x14U) ///!< MPU Region Size 2 MBytes
#define ARM_MPU_REGION_SIZE_4MB      ((uint8_t)0x15U) ///!< MPU Region Size 4 MBytes
#define ARM_MPU_REGION_SIZE_8MB      ((uint8_t)0x16U) ///!< MPU Region Size 8 MBytes
#define ARM_MPU_REGION_SIZE_16MB     ((uint8_t)0x17U) ///!< MPU Region Size 16 MBytes
#define ARM_MPU_CACHEP_NOCACHE 0U
#define ARM_MPU_AP_RO   6U
//SCB->RBAR = 0x10000000 | VALID | 1;
//SCB->RASR = (REGION_Enabled | NOT_EXEC | NORMAL | REGION_32K | FULL_ACCESS);

void enable_mpu(void) {
// disable mpu
	__asm volatile ("dsb");
	__asm volatile ("isb");
MPU_CTRL |= 0;
	__asm volatile ("dsb");
	__asm volatile ("isb");
	MPU_RNR |= 4;

   // Configure region 0 to cover 512KB Flash (Normal, Non-Shared, Executable, Read-only)
   MPU_RBAR |= (0x60000000 | VALID | 4);
   MPU_RASR |= (REGION_Enabled | NORMAL | ARM_MPU_REGION_SIZE_8MB | FULL_ACCESS | ARM_MPU_CACHEP_NOCACHE);
   // Enable MPU
   MPU_CTRL |= (1 | MPU_CTRL_HFNMIENA);
   __ISB();
   __DSB();
}

#define VALID 0x10
#define FULL_ACCESS     (0x03 << 24)   // Privileged Read Write, Unprivileged Read Write
#define SRAM_NOCACHE_START_ADDRESS (0x60000000UL)
#define NOCACHE_SRAM_REGION_SIZE 0x800000
#define MPU_NOCACHE_SRAM_REGION (4)
#define INNER_OUTER_NORMAL_NOCACHE_TYPE(x) ((0x01 << MPU_RASR_TEX_Pos ) | ( 0 << MPU_RASR_C_Pos ) | ( 0 << MPU_RASR_B_Pos ) | ( x << MPU_RASR_S_Pos))

void enable_mpu(void) {
	__asm volatile ("dsb");
	__asm volatile ("isb");
MPU_CTRL = 0;
long unsigned int dw_region_base_addr = SRAM_NOCACHE_START_ADDRESS | VALID | MPU_NOCACHE_SRAM_REGION;
long unsigned int dw_region_attr = FULL_ACCESS | INNER_OUTER_NORMAL_NOCACHE_TYPE( 1 ) | NOCACHE_SRAM_REGION_SIZE | 0x01;
MPU_RBAR = dw_region_base_addr;
MPU_RASR = dw_region_attr;

MPU_CTRL = (1 | MPU_CTRL_PRIVDEFENA);
__DSB();
__ISB();
}
*/
/** Disable the MPU.
*/
static inline void ARM_MPU_Disable(void)
{
  __DSB();
  __ISB();
#ifdef SCB_SHCSR_MEMFAULTENA_Msk
  SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;
#endif
  MPU_CTRL  &= ~MPU_CTRL_ENABLE_Msk;
}

/*
Parameters
    DisableExec	Instruction access disable bit. 1 = disable instruction fetches.
    AccessPermission	Data access permission configures read/write access for User and Privileged mode. Possible values see ARM_MPU_AP_xxx.
    TypeExtField	Type extension field, allows you to configure memory access type, for example strongly ordered, peripheral.
    IsShareable	1 = region is shareable between multiple bus masters.
    IsCacheable	1 = region is cacheable (values may be kept in cache).
    IsBufferable	1 = region is bufferable (when using write-back caching). Cacheable but non-bufferable regions use write-through policy.
    SubRegionDisable	Sub-region disable field (8 bits).
    Size	Region size with values defined under ARM_MPU_REGION_SIZE_xxx.
    */
/*
#define ARM_MPU_RASR 	( 	  	DisableExec,    \
		  	AccessPermission,   \
		  	TypeExtField,   \
		  	IsShareable,    \
		  	IsCacheable,    \
		  	IsBufferable,   \
		  	SubRegionDisable,   \
		  	Size    \
	)
*/
