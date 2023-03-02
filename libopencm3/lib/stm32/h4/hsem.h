// HSEM_R register 
#define HSEM_R_PROCID_SHIFT        (0U)
#define HSEM_R_PROCID_MASK         (0xFFUL << HSEM_R_PROCID_MASK)                //  0x000000FF 
#define HSEM_R_PROCID             HSEM_R_PROCID_MASK                            // Semaphore ProcessID 
#define HSEM_R_COREID_SHIFT        (8U)
#define HSEM_R_COREID_MASK         (0xFFUL << HSEM_R_COREID_MASK)                //  0x0000FF00 
#define HSEM_R_COREID             HSEM_R_COREID_MASK                            // Semaphore CoreID.   
#define HSEM_R_LOCK_SHIFT          (31U)
#define HSEM_R_LOCK_MASK           (0x1UL << HSEM_R_LOCK_MASK)                   //  0x80000000 
#define HSEM_R_LOCK               HSEM_R_LOCK_MASK                              // Lock indication.    

// HSEM_RLR register 
#define HSEM_RLR_PROCID_SHIFT      (0U)
#define HSEM_RLR_PROCID_MASK       (0xFFUL << HSEM_RLR_PROCID_MASK)              //  0x000000FF 
#define HSEM_RLR_PROCID           HSEM_RLR_PROCID_MASK                          // Semaphore ProcessID 
#define HSEM_RLR_COREID_SHIFT      (8U)
#define HSEM_RLR_COREID_MASK       (0xFFUL << HSEM_RLR_COREID_MASK)              //  0x0000FF00 
#define HSEM_RLR_COREID           HSEM_RLR_COREID_MASK                          // Semaphore CoreID.   
#define HSEM_RLR_LOCK_SHIFT        (31U)
#define HSEM_RLR_LOCK_MASK         (0x1UL << HSEM_RLR_LOCK_MASK)                 //  0x80000000 
#define HSEM_RLR_LOCK             HSEM_RLR_LOCK_MASK                            // Lock indication.    

// HSEM_C1IER register
#define HSEM_C1IER_ISE0_SHIFT      (0U)
#define HSEM_C1IER_ISE0_MASK       (0x1UL << HSEM_C1IER_ISE0_MASK)               //  0x00000001 
#define HSEM_C1IER_ISE0           HSEM_C1IER_ISE0_MASK                          // semaphore 0 , interrupt 0 enable bit.  
#define HSEM_C1IER_ISE1_SHIFT      (1U)
#define HSEM_C1IER_ISE1_MASK       (0x1UL << HSEM_C1IER_ISE1_MASK)               //  0x00000002 
#define HSEM_C1IER_ISE1           HSEM_C1IER_ISE1_MASK                          // semaphore 1 , interrupt 0 enable bit.  
#define HSEM_C1IER_ISE2_SHIFT      (2U)
#define HSEM_C1IER_ISE2_MASK       (0x1UL << HSEM_C1IER_ISE2_MASK)               //  0x00000004 
#define HSEM_C1IER_ISE2           HSEM_C1IER_ISE2_MASK                          // semaphore 2 , interrupt 0 enable bit.  
#define HSEM_C1IER_ISE3_SHIFT      (3U)
#define HSEM_C1IER_ISE3_MASK       (0x1UL << HSEM_C1IER_ISE3_MASK)               //  0x00000008 
#define HSEM_C1IER_ISE3           HSEM_C1IER_ISE3_MASK                          // semaphore 3 , interrupt 0 enable bit.  
#define HSEM_C1IER_ISE4_SHIFT      (4U)
#define HSEM_C1IER_ISE4_MASK       (0x1UL << HSEM_C1IER_ISE4_MASK)               //  0x00000010 
#define HSEM_C1IER_ISE4           HSEM_C1IER_ISE4_MASK                          // semaphore 4 , interrupt 0 enable bit.  
#define HSEM_C1IER_ISE5_SHIFT      (5U)
#define HSEM_C1IER_ISE5_MASK       (0x1UL << HSEM_C1IER_ISE5_MASK)               //  0x00000020 
#define HSEM_C1IER_ISE5           HSEM_C1IER_ISE5_MASK                          // semaphore 5 interrupt 0 enable bit.    
#define HSEM_C1IER_ISE6_SHIFT      (6U)
#define HSEM_C1IER_ISE6_MASK       (0x1UL << HSEM_C1IER_ISE6_MASK)               //  0x00000040 
#define HSEM_C1IER_ISE6           HSEM_C1IER_ISE6_MASK                          // semaphore 6 interrupt 0 enable bit.    
#define HSEM_C1IER_ISE7_SHIFT      (7U)
#define HSEM_C1IER_ISE7_MASK       (0x1UL << HSEM_C1IER_ISE7_MASK)               //  0x00000080 
#define HSEM_C1IER_ISE7           HSEM_C1IER_ISE7_MASK                          // semaphore 7 interrupt 0 enable bit.    
#define HSEM_C1IER_ISE8_SHIFT      (8U)
#define HSEM_C1IER_ISE8_MASK       (0x1UL << HSEM_C1IER_ISE8_MASK)               //  0x00000100 
#define HSEM_C1IER_ISE8           HSEM_C1IER_ISE8_MASK                          // semaphore 8 interrupt 0 enable bit.    
#define HSEM_C1IER_ISE9_SHIFT      (9U)
#define HSEM_C1IER_ISE9_MASK       (0x1UL << HSEM_C1IER_ISE9_MASK)               //  0x00000200 
#define HSEM_C1IER_ISE9           HSEM_C1IER_ISE9_MASK                          // semaphore 9 interrupt 0 enable bit.    
#define HSEM_C1IER_ISE10_SHIFT     (10U)
#define HSEM_C1IER_ISE10_MASK      (0x1UL << HSEM_C1IER_ISE10_MASK)              //  0x00000400 
#define HSEM_C1IER_ISE10          HSEM_C1IER_ISE10_MASK                         // semaphore 10 interrupt 0 enable bit.   
#define HSEM_C1IER_ISE11_SHIFT     (11U)
#define HSEM_C1IER_ISE11_MASK      (0x1UL << HSEM_C1IER_ISE11_MASK)              //  0x00000800 
#define HSEM_C1IER_ISE11          HSEM_C1IER_ISE11_MASK                         // semaphore 11 interrupt 0 enable bit.   
#define HSEM_C1IER_ISE12_SHIFT     (12U)
#define HSEM_C1IER_ISE12_MASK      (0x1UL << HSEM_C1IER_ISE12_MASK)              //  0x00001000 
#define HSEM_C1IER_ISE12          HSEM_C1IER_ISE12_MASK                         // semaphore 12 interrupt 0 enable bit.   
#define HSEM_C1IER_ISE13_SHIFT     (13U)
#define HSEM_C1IER_ISE13_MASK      (0x1UL << HSEM_C1IER_ISE13_MASK)              //  0x00002000 
#define HSEM_C1IER_ISE13          HSEM_C1IER_ISE13_MASK                         // semaphore 13 interrupt 0 enable bit.   
#define HSEM_C1IER_ISE14_SHIFT     (14U)
#define HSEM_C1IER_ISE14_MASK      (0x1UL << HSEM_C1IER_ISE14_MASK)              //  0x00004000 
#define HSEM_C1IER_ISE14          HSEM_C1IER_ISE14_MASK                         // semaphore 14 interrupt 0 enable bit.   
#define HSEM_C1IER_ISE15_SHIFT     (15U)
#define HSEM_C1IER_ISE15_MASK      (0x1UL << HSEM_C1IER_ISE15_MASK)              //  0x00008000 
#define HSEM_C1IER_ISE15          HSEM_C1IER_ISE15_MASK                         // semaphore 15 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE16_SHIFT     (16U)
#define HSEM_C1IER_ISE16_MASK      (0x1UL << HSEM_C1IER_ISE16_MASK)              //  0x00010000 
#define HSEM_C1IER_ISE16          HSEM_C1IER_ISE16_MASK                         // semaphore 16 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE17_SHIFT     (17U)
#define HSEM_C1IER_ISE17_MASK      (0x1UL << HSEM_C1IER_ISE17_MASK)              //  0x00020000 
#define HSEM_C1IER_ISE17          HSEM_C1IER_ISE17_MASK                         // semaphore 17 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE18_SHIFT     (18U)
#define HSEM_C1IER_ISE18_MASK      (0x1UL << HSEM_C1IER_ISE18_MASK)              //  0x00040000 
#define HSEM_C1IER_ISE18          HSEM_C1IER_ISE18_MASK                         // semaphore 18 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE19_SHIFT     (19U)
#define HSEM_C1IER_ISE19_MASK      (0x1UL << HSEM_C1IER_ISE19_MASK)              //  0x00080000 
#define HSEM_C1IER_ISE19          HSEM_C1IER_ISE19_MASK                         // semaphore 19 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE20_SHIFT     (20U)
#define HSEM_C1IER_ISE20_MASK      (0x1UL << HSEM_C1IER_ISE20_MASK)              //  0x00100000 
#define HSEM_C1IER_ISE20          HSEM_C1IER_ISE20_MASK                         // semaphore 20 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE21_SHIFT     (21U)
#define HSEM_C1IER_ISE21_MASK      (0x1UL << HSEM_C1IER_ISE21_MASK)              //  0x00200000 
#define HSEM_C1IER_ISE21          HSEM_C1IER_ISE21_MASK                         // semaphore 21 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE22_SHIFT     (22U)
#define HSEM_C1IER_ISE22_MASK      (0x1UL << HSEM_C1IER_ISE22_MASK)              //  0x00400000 
#define HSEM_C1IER_ISE22          HSEM_C1IER_ISE22_MASK                         // semaphore 22 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE23_SHIFT     (23U)
#define HSEM_C1IER_ISE23_MASK      (0x1UL << HSEM_C1IER_ISE23_MASK)              //  0x00800000 
#define HSEM_C1IER_ISE23          HSEM_C1IER_ISE23_MASK                         // semaphore 23 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE24_SHIFT     (24U)
#define HSEM_C1IER_ISE24_MASK      (0x1UL << HSEM_C1IER_ISE24_MASK)              //  0x01000000 
#define HSEM_C1IER_ISE24          HSEM_C1IER_ISE24_MASK                         // semaphore 24 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE25_SHIFT     (25U)
#define HSEM_C1IER_ISE25_MASK      (0x1UL << HSEM_C1IER_ISE25_MASK)              //  0x02000000 
#define HSEM_C1IER_ISE25          HSEM_C1IER_ISE25_MASK                         // semaphore 25 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE26_SHIFT     (26U)
#define HSEM_C1IER_ISE26_MASK      (0x1UL << HSEM_C1IER_ISE26_MASK)              //  0x04000000 
#define HSEM_C1IER_ISE26          HSEM_C1IER_ISE26_MASK                         // semaphore 26 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE27_SHIFT     (27U)
#define HSEM_C1IER_ISE27_MASK      (0x1UL << HSEM_C1IER_ISE27_MASK)              //  0x08000000 
#define HSEM_C1IER_ISE27          HSEM_C1IER_ISE27_MASK                         // semaphore 27 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE28_SHIFT     (28U)
#define HSEM_C1IER_ISE28_MASK      (0x1UL << HSEM_C1IER_ISE28_MASK)              //  0x10000000 
#define HSEM_C1IER_ISE28          HSEM_C1IER_ISE28_MASK                         // semaphore 28 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE29_SHIFT     (29U)
#define HSEM_C1IER_ISE29_MASK      (0x1UL << HSEM_C1IER_ISE29_MASK)              //  0x20000000 
#define HSEM_C1IER_ISE29          HSEM_C1IER_ISE29_MASK                         // semaphore 29 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE30_SHIFT     (30U)
#define HSEM_C1IER_ISE30_MASK      (0x1UL << HSEM_C1IER_ISE30_MASK)              //  0x40000000 
#define HSEM_C1IER_ISE30          HSEM_C1IER_ISE30_MASK                         // semaphore 30 interrupt 0 enable bit. 
#define HSEM_C1IER_ISE31_SHIFT     (31U)
#define HSEM_C1IER_ISE31_MASK      (0x1UL << HSEM_C1IER_ISE31_MASK)              //  0x80000000 
#define HSEM_C1IER_ISE31          HSEM_C1IER_ISE31_MASK                         // semaphore 31 interrupt 0 enable bit. 

// HSEM_C1ICR register
#define HSEM_C1ICR_ISC0_SHIFT      (0U)
#define HSEM_C1ICR_ISC0_MASK       (0x1UL << HSEM_C1ICR_ISC0_MASK)               //  0x00000001 
#define HSEM_C1ICR_ISC0           HSEM_C1ICR_ISC0_MASK                          // semaphore 0 , interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC1_SHIFT      (1U)
#define HSEM_C1ICR_ISC1_MASK       (0x1UL << HSEM_C1ICR_ISC1_MASK)               //  0x00000002 
#define HSEM_C1ICR_ISC1           HSEM_C1ICR_ISC1_MASK                          // semaphore 1 , interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC2_SHIFT      (2U)
#define HSEM_C1ICR_ISC2_MASK       (0x1UL << HSEM_C1ICR_ISC2_MASK)               //  0x00000004 
#define HSEM_C1ICR_ISC2           HSEM_C1ICR_ISC2_MASK                          // semaphore 2 , interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC3_SHIFT      (3U)
#define HSEM_C1ICR_ISC3_MASK       (0x1UL << HSEM_C1ICR_ISC3_MASK)               //  0x00000008 
#define HSEM_C1ICR_ISC3           HSEM_C1ICR_ISC3_MASK                          // semaphore 3 , interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC4_SHIFT      (4U)
#define HSEM_C1ICR_ISC4_MASK       (0x1UL << HSEM_C1ICR_ISC4_MASK)               //  0x00000010 
#define HSEM_C1ICR_ISC4           HSEM_C1ICR_ISC4_MASK                          // semaphore 4 , interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC5_SHIFT      (5U)
#define HSEM_C1ICR_ISC5_MASK       (0x1UL << HSEM_C1ICR_ISC5_MASK)               //  0x00000020 
#define HSEM_C1ICR_ISC5           HSEM_C1ICR_ISC5_MASK                          // semaphore 5 interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC6_SHIFT      (6U)
#define HSEM_C1ICR_ISC6_MASK       (0x1UL << HSEM_C1ICR_ISC6_MASK)               //  0x00000040 
#define HSEM_C1ICR_ISC6           HSEM_C1ICR_ISC6_MASK                          // semaphore 6 interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC7_SHIFT      (7U)
#define HSEM_C1ICR_ISC7_MASK       (0x1UL << HSEM_C1ICR_ISC7_MASK)               //  0x00000080 
#define HSEM_C1ICR_ISC7           HSEM_C1ICR_ISC7_MASK                          // semaphore 7 interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC8_SHIFT      (8U)
#define HSEM_C1ICR_ISC8_MASK       (0x1UL << HSEM_C1ICR_ISC8_MASK)               //  0x00000100 
#define HSEM_C1ICR_ISC8           HSEM_C1ICR_ISC8_MASK                          // semaphore 8 interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC9_SHIFT      (9U)
#define HSEM_C1ICR_ISC9_MASK       (0x1UL << HSEM_C1ICR_ISC9_MASK)               //  0x00000200 
#define HSEM_C1ICR_ISC9           HSEM_C1ICR_ISC9_MASK                          // semaphore 9 interrupt 0 clear bit.  
#define HSEM_C1ICR_ISC10_SHIFT     (10U)
#define HSEM_C1ICR_ISC10_MASK      (0x1UL << HSEM_C1ICR_ISC10_MASK)              //  0x00000400 
#define HSEM_C1ICR_ISC10          HSEM_C1ICR_ISC10_MASK                         // semaphore 10 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC11_SHIFT     (11U)
#define HSEM_C1ICR_ISC11_MASK      (0x1UL << HSEM_C1ICR_ISC11_MASK)              //  0x00000800 
#define HSEM_C1ICR_ISC11          HSEM_C1ICR_ISC11_MASK                         // semaphore 11 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC12_SHIFT     (12U)
#define HSEM_C1ICR_ISC12_MASK      (0x1UL << HSEM_C1ICR_ISC12_MASK)              //  0x00001000 
#define HSEM_C1ICR_ISC12          HSEM_C1ICR_ISC12_MASK                         // semaphore 12 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC13_SHIFT     (13U)
#define HSEM_C1ICR_ISC13_MASK      (0x1UL << HSEM_C1ICR_ISC13_MASK)              //  0x00002000 
#define HSEM_C1ICR_ISC13          HSEM_C1ICR_ISC13_MASK                         // semaphore 13 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC14_SHIFT     (14U)
#define HSEM_C1ICR_ISC14_MASK      (0x1UL << HSEM_C1ICR_ISC14_MASK)              //  0x00004000 
#define HSEM_C1ICR_ISC14          HSEM_C1ICR_ISC14_MASK                         // semaphore 14 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC15_SHIFT     (15U)
#define HSEM_C1ICR_ISC15_MASK      (0x1UL << HSEM_C1ICR_ISC15_MASK)              //  0x00008000 
#define HSEM_C1ICR_ISC15          HSEM_C1ICR_ISC15_MASK                         // semaphore 15 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC16_SHIFT     (16U)
#define HSEM_C1ICR_ISC16_MASK      (0x1UL << HSEM_C1ICR_ISC16_MASK)              //  0x00010000 
#define HSEM_C1ICR_ISC16          HSEM_C1ICR_ISC16_MASK                         // semaphore 16 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC17_SHIFT     (17U)
#define HSEM_C1ICR_ISC17_MASK      (0x1UL << HSEM_C1ICR_ISC17_MASK)              //  0x00020000 
#define HSEM_C1ICR_ISC17          HSEM_C1ICR_ISC17_MASK                         // semaphore 17 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC18_SHIFT     (18U)
#define HSEM_C1ICR_ISC18_MASK      (0x1UL << HSEM_C1ICR_ISC18_MASK)              //  0x00040000 
#define HSEM_C1ICR_ISC18          HSEM_C1ICR_ISC18_MASK                         // semaphore 18 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC19_SHIFT     (19U)
#define HSEM_C1ICR_ISC19_MASK      (0x1UL << HSEM_C1ICR_ISC19_MASK)              //  0x00080000 
#define HSEM_C1ICR_ISC19          HSEM_C1ICR_ISC19_MASK                         // semaphore 19 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC20_SHIFT     (20U)
#define HSEM_C1ICR_ISC20_MASK      (0x1UL << HSEM_C1ICR_ISC20_MASK)              //  0x00100000 
#define HSEM_C1ICR_ISC20          HSEM_C1ICR_ISC20_MASK                         // semaphore 20 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC21_SHIFT     (21U)
#define HSEM_C1ICR_ISC21_MASK      (0x1UL << HSEM_C1ICR_ISC21_MASK)              //  0x00200000 
#define HSEM_C1ICR_ISC21          HSEM_C1ICR_ISC21_MASK                         // semaphore 21 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC22_SHIFT     (22U)
#define HSEM_C1ICR_ISC22_MASK      (0x1UL << HSEM_C1ICR_ISC22_MASK)              //  0x00400000 
#define HSEM_C1ICR_ISC22          HSEM_C1ICR_ISC22_MASK                         // semaphore 22 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC23_SHIFT     (23U)
#define HSEM_C1ICR_ISC23_MASK      (0x1UL << HSEM_C1ICR_ISC23_MASK)              //  0x00800000 
#define HSEM_C1ICR_ISC23          HSEM_C1ICR_ISC23_MASK                         // semaphore 23 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC24_SHIFT     (24U)
#define HSEM_C1ICR_ISC24_MASK      (0x1UL << HSEM_C1ICR_ISC24_MASK)              //  0x01000000 
#define HSEM_C1ICR_ISC24          HSEM_C1ICR_ISC24_MASK                         // semaphore 24 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC25_SHIFT     (25U)
#define HSEM_C1ICR_ISC25_MASK      (0x1UL << HSEM_C1ICR_ISC25_MASK)              //  0x02000000 
#define HSEM_C1ICR_ISC25          HSEM_C1ICR_ISC25_MASK                         // semaphore 25 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC26_SHIFT     (26U)
#define HSEM_C1ICR_ISC26_MASK      (0x1UL << HSEM_C1ICR_ISC26_MASK)              //  0x04000000 
#define HSEM_C1ICR_ISC26          HSEM_C1ICR_ISC26_MASK                         // semaphore 26 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC27_SHIFT     (27U)
#define HSEM_C1ICR_ISC27_MASK      (0x1UL << HSEM_C1ICR_ISC27_MASK)              //  0x08000000 
#define HSEM_C1ICR_ISC27          HSEM_C1ICR_ISC27_MASK                         // semaphore 27 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC28_SHIFT     (28U)
#define HSEM_C1ICR_ISC28_MASK      (0x1UL << HSEM_C1ICR_ISC28_MASK)              //  0x10000000 
#define HSEM_C1ICR_ISC28          HSEM_C1ICR_ISC28_MASK                         // semaphore 28 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC29_SHIFT     (29U)
#define HSEM_C1ICR_ISC29_MASK      (0x1UL << HSEM_C1ICR_ISC29_MASK)              //  0x20000000 
#define HSEM_C1ICR_ISC29          HSEM_C1ICR_ISC29_MASK                         // semaphore 29 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC30_SHIFT     (30U)
#define HSEM_C1ICR_ISC30_MASK      (0x1UL << HSEM_C1ICR_ISC30_MASK)              //  0x40000000 
#define HSEM_C1ICR_ISC30          HSEM_C1ICR_ISC30_MASK                         // semaphore 30 interrupt 0 clear bit. 
#define HSEM_C1ICR_ISC31_SHIFT     (31U)
#define HSEM_C1ICR_ISC31_MASK      (0x1UL << HSEM_C1ICR_ISC31_MASK)              //  0x80000000 
#define HSEM_C1ICR_ISC31          HSEM_C1ICR_ISC31_MASK                         // semaphore 31 interrupt 0 clear bit. 

// HSEM_C1ISR register
#define HSEM_C1ISR_ISF0_SHIFT      (0U)
#define HSEM_C1ISR_ISF0_MASK       (0x1UL << HSEM_C1ISR_ISF0_MASK)               //  0x00000001 
#define HSEM_C1ISR_ISF0           HSEM_C1ISR_ISF0_MASK                          // semaphore 0 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF1_SHIFT      (1U)
#define HSEM_C1ISR_ISF1_MASK       (0x1UL << HSEM_C1ISR_ISF1_MASK)               //  0x00000002 
#define HSEM_C1ISR_ISF1           HSEM_C1ISR_ISF1_MASK                          // semaphore 1 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF2_SHIFT      (2U)
#define HSEM_C1ISR_ISF2_MASK       (0x1UL << HSEM_C1ISR_ISF2_MASK)               //  0x00000004 
#define HSEM_C1ISR_ISF2           HSEM_C1ISR_ISF2_MASK                          // semaphore 2 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF3_SHIFT      (3U)
#define HSEM_C1ISR_ISF3_MASK       (0x1UL << HSEM_C1ISR_ISF3_MASK)               //  0x00000008 
#define HSEM_C1ISR_ISF3           HSEM_C1ISR_ISF3_MASK                          // semaphore 3 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF4_SHIFT      (4U)
#define HSEM_C1ISR_ISF4_MASK       (0x1UL << HSEM_C1ISR_ISF4_MASK)               //  0x00000010 
#define HSEM_C1ISR_ISF4           HSEM_C1ISR_ISF4_MASK                          // semaphore 4 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF5_SHIFT      (5U)
#define HSEM_C1ISR_ISF5_MASK       (0x1UL << HSEM_C1ISR_ISF5_MASK)               //  0x00000020 
#define HSEM_C1ISR_ISF5           HSEM_C1ISR_ISF5_MASK                          // semaphore 5 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF6_SHIFT      (6U)
#define HSEM_C1ISR_ISF6_MASK       (0x1UL << HSEM_C1ISR_ISF6_MASK)               //  0x00000040 
#define HSEM_C1ISR_ISF6           HSEM_C1ISR_ISF6_MASK                          // semaphore 6 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF7_SHIFT      (7U)
#define HSEM_C1ISR_ISF7_MASK       (0x1UL << HSEM_C1ISR_ISF7_MASK)               //  0x00000080 
#define HSEM_C1ISR_ISF7           HSEM_C1ISR_ISF7_MASK                          // semaphore 7 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF8_SHIFT      (8U)
#define HSEM_C1ISR_ISF8_MASK       (0x1UL << HSEM_C1ISR_ISF8_MASK)               //  0x00000100 
#define HSEM_C1ISR_ISF8           HSEM_C1ISR_ISF8_MASK                          // semaphore 8 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF9_SHIFT      (9U)
#define HSEM_C1ISR_ISF9_MASK       (0x1UL << HSEM_C1ISR_ISF9_MASK)               //  0x00000200 
#define HSEM_C1ISR_ISF9           HSEM_C1ISR_ISF9_MASK                          // semaphore 9 interrupt 0 status bit.  
#define HSEM_C1ISR_ISF10_SHIFT     (10U)
#define HSEM_C1ISR_ISF10_MASK      (0x1UL << HSEM_C1ISR_ISF10_MASK)              //  0x00000400 
#define HSEM_C1ISR_ISF10          HSEM_C1ISR_ISF10_MASK                         // semaphore 10 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF11_SHIFT     (11U)
#define HSEM_C1ISR_ISF11_MASK      (0x1UL << HSEM_C1ISR_ISF11_MASK)              //  0x00000800 
#define HSEM_C1ISR_ISF11          HSEM_C1ISR_ISF11_MASK                         // semaphore 11 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF12_SHIFT     (12U)
#define HSEM_C1ISR_ISF12_MASK      (0x1UL << HSEM_C1ISR_ISF12_MASK)              //  0x00001000 
#define HSEM_C1ISR_ISF12          HSEM_C1ISR_ISF12_MASK                         // semaphore 12 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF13_SHIFT     (13U)
#define HSEM_C1ISR_ISF13_MASK      (0x1UL << HSEM_C1ISR_ISF13_MASK)              //  0x00002000 
#define HSEM_C1ISR_ISF13          HSEM_C1ISR_ISF13_MASK                         // semaphore 13 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF14_SHIFT     (14U)
#define HSEM_C1ISR_ISF14_MASK      (0x1UL << HSEM_C1ISR_ISF14_MASK)              //  0x00004000 
#define HSEM_C1ISR_ISF14          HSEM_C1ISR_ISF14_MASK                         // semaphore 14 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF15_SHIFT     (15U)
#define HSEM_C1ISR_ISF15_MASK      (0x1UL << HSEM_C1ISR_ISF15_MASK)              //  0x00008000 
#define HSEM_C1ISR_ISF15          HSEM_C1ISR_ISF15_MASK                         // semaphore 15 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF16_SHIFT     (16U)
#define HSEM_C1ISR_ISF16_MASK      (0x1UL << HSEM_C1ISR_ISF16_MASK)              //  0x00010000 
#define HSEM_C1ISR_ISF16          HSEM_C1ISR_ISF16_MASK                         // semaphore 16 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF17_SHIFT     (17U)
#define HSEM_C1ISR_ISF17_MASK      (0x1UL << HSEM_C1ISR_ISF17_MASK)              //  0x00020000 
#define HSEM_C1ISR_ISF17          HSEM_C1ISR_ISF17_MASK                         // semaphore 17 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF18_SHIFT     (18U)
#define HSEM_C1ISR_ISF18_MASK      (0x1UL << HSEM_C1ISR_ISF18_MASK)              //  0x00040000 
#define HSEM_C1ISR_ISF18          HSEM_C1ISR_ISF18_MASK                         // semaphore 18 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF19_SHIFT     (19U)
#define HSEM_C1ISR_ISF19_MASK      (0x1UL << HSEM_C1ISR_ISF19_MASK)              //  0x00080000 
#define HSEM_C1ISR_ISF19          HSEM_C1ISR_ISF19_MASK                         // semaphore 19 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF20_SHIFT     (20U)
#define HSEM_C1ISR_ISF20_MASK      (0x1UL << HSEM_C1ISR_ISF20_MASK)              //  0x00100000 
#define HSEM_C1ISR_ISF20          HSEM_C1ISR_ISF20_MASK                         // semaphore 20 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF21_SHIFT     (21U)
#define HSEM_C1ISR_ISF21_MASK      (0x1UL << HSEM_C1ISR_ISF21_MASK)              //  0x00200000 
#define HSEM_C1ISR_ISF21          HSEM_C1ISR_ISF21_MASK                         // semaphore 21 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF22_SHIFT     (22U)
#define HSEM_C1ISR_ISF22_MASK      (0x1UL << HSEM_C1ISR_ISF22_MASK)              //  0x00400000 
#define HSEM_C1ISR_ISF22          HSEM_C1ISR_ISF22_MASK                         // semaphore 22 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF23_SHIFT     (23U)
#define HSEM_C1ISR_ISF23_MASK      (0x1UL << HSEM_C1ISR_ISF23_MASK)              //  0x00800000 
#define HSEM_C1ISR_ISF23          HSEM_C1ISR_ISF23_MASK                         // semaphore 23 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF24_SHIFT     (24U)
#define HSEM_C1ISR_ISF24_MASK      (0x1UL << HSEM_C1ISR_ISF24_MASK)              //  0x01000000 
#define HSEM_C1ISR_ISF24          HSEM_C1ISR_ISF24_MASK                         // semaphore 24 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF25_SHIFT     (25U)
#define HSEM_C1ISR_ISF25_MASK      (0x1UL << HSEM_C1ISR_ISF25_MASK)              //  0x02000000 
#define HSEM_C1ISR_ISF25          HSEM_C1ISR_ISF25_MASK                         // semaphore 25 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF26_SHIFT     (26U)
#define HSEM_C1ISR_ISF26_MASK      (0x1UL << HSEM_C1ISR_ISF26_MASK)              //  0x04000000 
#define HSEM_C1ISR_ISF26          HSEM_C1ISR_ISF26_MASK                         // semaphore 26 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF27_SHIFT     (27U)
#define HSEM_C1ISR_ISF27_MASK      (0x1UL << HSEM_C1ISR_ISF27_MASK)              //  0x08000000 
#define HSEM_C1ISR_ISF27          HSEM_C1ISR_ISF27_MASK                         // semaphore 27 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF28_SHIFT     (28U)
#define HSEM_C1ISR_ISF28_MASK      (0x1UL << HSEM_C1ISR_ISF28_MASK)              //  0x10000000 
#define HSEM_C1ISR_ISF28          HSEM_C1ISR_ISF28_MASK                         // semaphore 28 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF29_SHIFT     (29U)
#define HSEM_C1ISR_ISF29_MASK      (0x1UL << HSEM_C1ISR_ISF29_MASK)              //  0x20000000 
#define HSEM_C1ISR_ISF29          HSEM_C1ISR_ISF29_MASK                         // semaphore 29 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF30_SHIFT     (30U)
#define HSEM_C1ISR_ISF30_MASK      (0x1UL << HSEM_C1ISR_ISF30_MASK)              //  0x40000000 
#define HSEM_C1ISR_ISF30          HSEM_C1ISR_ISF30_MASK                         // semaphore 30 interrupt 0 status bit. 
#define HSEM_C1ISR_ISF31_SHIFT     (31U)
#define HSEM_C1ISR_ISF31_MASK      (0x1UL << HSEM_C1ISR_ISF31_MASK)              //  0x80000000 
#define HSEM_C1ISR_ISF31          HSEM_C1ISR_ISF31_MASK                         // semaphore 31 interrupt 0 status bit. 

// HSEM_C1MISR register
#define HSEM_C1MISR_MISF0_SHIFT    (0U)
#define HSEM_C1MISR_MISF0_MASK     (0x1UL << HSEM_C1MISR_MISF0_MASK)             //  0x00000001 
#define HSEM_C1MISR_MISF0         HSEM_C1MISR_MISF0_MASK                        // semaphore 0 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF1_SHIFT    (1U)
#define HSEM_C1MISR_MISF1_MASK     (0x1UL << HSEM_C1MISR_MISF1_MASK)             //  0x00000002 
#define HSEM_C1MISR_MISF1         HSEM_C1MISR_MISF1_MASK                        // semaphore 1 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF2_SHIFT    (2U)
#define HSEM_C1MISR_MISF2_MASK     (0x1UL << HSEM_C1MISR_MISF2_MASK)             //  0x00000004 
#define HSEM_C1MISR_MISF2         HSEM_C1MISR_MISF2_MASK                        // semaphore 2 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF3_SHIFT    (3U)
#define HSEM_C1MISR_MISF3_MASK     (0x1UL << HSEM_C1MISR_MISF3_MASK)             //  0x00000008 
#define HSEM_C1MISR_MISF3         HSEM_C1MISR_MISF3_MASK                        // semaphore 3 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF4_SHIFT    (4U)
#define HSEM_C1MISR_MISF4_MASK     (0x1UL << HSEM_C1MISR_MISF4_MASK)             //  0x00000010 
#define HSEM_C1MISR_MISF4         HSEM_C1MISR_MISF4_MASK                        // semaphore 4 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF5_SHIFT    (5U)
#define HSEM_C1MISR_MISF5_MASK     (0x1UL << HSEM_C1MISR_MISF5_MASK)             //  0x00000020 
#define HSEM_C1MISR_MISF5         HSEM_C1MISR_MISF5_MASK                        // semaphore 5 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF6_SHIFT    (6U)
#define HSEM_C1MISR_MISF6_MASK     (0x1UL << HSEM_C1MISR_MISF6_MASK)             //  0x00000040 
#define HSEM_C1MISR_MISF6         HSEM_C1MISR_MISF6_MASK                        // semaphore 6 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF7_SHIFT    (7U)
#define HSEM_C1MISR_MISF7_MASK     (0x1UL << HSEM_C1MISR_MISF7_MASK)             //  0x00000080 
#define HSEM_C1MISR_MISF7         HSEM_C1MISR_MISF7_MASK                        // semaphore 7 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF8_SHIFT    (8U)
#define HSEM_C1MISR_MISF8_MASK     (0x1UL << HSEM_C1MISR_MISF8_MASK)             //  0x00000100 
#define HSEM_C1MISR_MISF8         HSEM_C1MISR_MISF8_MASK                        // semaphore 8 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF9_SHIFT    (9U)
#define HSEM_C1MISR_MISF9_MASK     (0x1UL << HSEM_C1MISR_MISF9_MASK)             //  0x00000200 
#define HSEM_C1MISR_MISF9         HSEM_C1MISR_MISF9_MASK                        // semaphore 9 interrupt 0 masked status bit.  
#define HSEM_C1MISR_MISF10_SHIFT   (10U)
#define HSEM_C1MISR_MISF10_MASK    (0x1UL << HSEM_C1MISR_MISF10_MASK)            //  0x00000400 
#define HSEM_C1MISR_MISF10        HSEM_C1MISR_MISF10_MASK                       // semaphore 10 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF11_SHIFT   (11U)
#define HSEM_C1MISR_MISF11_MASK    (0x1UL << HSEM_C1MISR_MISF11_MASK)            //  0x00000800 
#define HSEM_C1MISR_MISF11        HSEM_C1MISR_MISF11_MASK                       // semaphore 11 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF12_SHIFT   (12U)
#define HSEM_C1MISR_MISF12_MASK    (0x1UL << HSEM_C1MISR_MISF12_MASK)            //  0x00001000 
#define HSEM_C1MISR_MISF12        HSEM_C1MISR_MISF12_MASK                       // semaphore 12 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF13_SHIFT   (13U)
#define HSEM_C1MISR_MISF13_MASK    (0x1UL << HSEM_C1MISR_MISF13_MASK)            //  0x00002000 
#define HSEM_C1MISR_MISF13        HSEM_C1MISR_MISF13_MASK                       // semaphore 13 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF14_SHIFT   (14U)
#define HSEM_C1MISR_MISF14_MASK    (0x1UL << HSEM_C1MISR_MISF14_MASK)            //  0x00004000 
#define HSEM_C1MISR_MISF14        HSEM_C1MISR_MISF14_MASK                       // semaphore 14 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF15_SHIFT   (15U)
#define HSEM_C1MISR_MISF15_MASK    (0x1UL << HSEM_C1MISR_MISF15_MASK)            //  0x00008000 
#define HSEM_C1MISR_MISF15        HSEM_C1MISR_MISF15_MASK                       // semaphore 15 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF16_SHIFT   (16U)
#define HSEM_C1MISR_MISF16_MASK    (0x1UL << HSEM_C1MISR_MISF16_MASK)            //  0x00010000 
#define HSEM_C1MISR_MISF16        HSEM_C1MISR_MISF16_MASK                       // semaphore 16 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF17_SHIFT   (17U)
#define HSEM_C1MISR_MISF17_MASK    (0x1UL << HSEM_C1MISR_MISF17_MASK)            //  0x00020000 
#define HSEM_C1MISR_MISF17        HSEM_C1MISR_MISF17_MASK                       // semaphore 17 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF18_SHIFT   (18U)
#define HSEM_C1MISR_MISF18_MASK    (0x1UL << HSEM_C1MISR_MISF18_MASK)            //  0x00040000 
#define HSEM_C1MISR_MISF18        HSEM_C1MISR_MISF18_MASK                       // semaphore 18 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF19_SHIFT   (19U)
#define HSEM_C1MISR_MISF19_MASK    (0x1UL << HSEM_C1MISR_MISF19_MASK)            //  0x00080000 
#define HSEM_C1MISR_MISF19        HSEM_C1MISR_MISF19_MASK                       // semaphore 19 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF20_SHIFT   (20U)
#define HSEM_C1MISR_MISF20_MASK    (0x1UL << HSEM_C1MISR_MISF20_MASK)            //  0x00100000 
#define HSEM_C1MISR_MISF20        HSEM_C1MISR_MISF20_MASK                       // semaphore 20 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF21_SHIFT   (21U)
#define HSEM_C1MISR_MISF21_MASK    (0x1UL << HSEM_C1MISR_MISF21_MASK)            //  0x00200000 
#define HSEM_C1MISR_MISF21        HSEM_C1MISR_MISF21_MASK                       // semaphore 21 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF22_SHIFT   (22U)
#define HSEM_C1MISR_MISF22_MASK    (0x1UL << HSEM_C1MISR_MISF22_MASK)            //  0x00400000 
#define HSEM_C1MISR_MISF22        HSEM_C1MISR_MISF22_MASK                       // semaphore 22 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF23_SHIFT   (23U)
#define HSEM_C1MISR_MISF23_MASK    (0x1UL << HSEM_C1MISR_MISF23_MASK)            //  0x00800000 
#define HSEM_C1MISR_MISF23        HSEM_C1MISR_MISF23_MASK                       // semaphore 23 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF24_SHIFT   (24U)
#define HSEM_C1MISR_MISF24_MASK    (0x1UL << HSEM_C1MISR_MISF24_MASK)            //  0x01000000 
#define HSEM_C1MISR_MISF24        HSEM_C1MISR_MISF24_MASK                       // semaphore 24 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF25_SHIFT   (25U)
#define HSEM_C1MISR_MISF25_MASK    (0x1UL << HSEM_C1MISR_MISF25_MASK)            //  0x02000000 
#define HSEM_C1MISR_MISF25        HSEM_C1MISR_MISF25_MASK                       // semaphore 25 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF26_SHIFT   (26U)
#define HSEM_C1MISR_MISF26_MASK    (0x1UL << HSEM_C1MISR_MISF26_MASK)            //  0x04000000 
#define HSEM_C1MISR_MISF26        HSEM_C1MISR_MISF26_MASK                       // semaphore 26 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF27_SHIFT   (27U)
#define HSEM_C1MISR_MISF27_MASK    (0x1UL << HSEM_C1MISR_MISF27_MASK)            //  0x08000000 
#define HSEM_C1MISR_MISF27        HSEM_C1MISR_MISF27_MASK                       // semaphore 27 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF28_SHIFT   (28U)
#define HSEM_C1MISR_MISF28_MASK    (0x1UL << HSEM_C1MISR_MISF28_MASK)            //  0x10000000 
#define HSEM_C1MISR_MISF28        HSEM_C1MISR_MISF28_MASK                       // semaphore 28 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF29_SHIFT   (29U)
#define HSEM_C1MISR_MISF29_MASK    (0x1UL << HSEM_C1MISR_MISF29_MASK)            //  0x20000000 
#define HSEM_C1MISR_MISF29        HSEM_C1MISR_MISF29_MASK                       // semaphore 29 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF30_SHIFT   (30U)
#define HSEM_C1MISR_MISF30_MASK    (0x1UL << HSEM_C1MISR_MISF30_MASK)            //  0x40000000 
#define HSEM_C1MISR_MISF30        HSEM_C1MISR_MISF30_MASK                       // semaphore 30 interrupt 0 masked status bit. 
#define HSEM_C1MISR_MISF31_SHIFT   (31U)
#define HSEM_C1MISR_MISF31_MASK    (0x1UL << HSEM_C1MISR_MISF31_MASK)            //  0x80000000 
#define HSEM_C1MISR_MISF31        HSEM_C1MISR_MISF31_MASK                       // semaphore 31 interrupt 0 masked status bit. 

// HSEM_C2IER register
#define HSEM_C2IER_ISE0_SHIFT      (0U)
#define HSEM_C2IER_ISE0_MASK       (0x1UL << HSEM_C2IER_ISE0_MASK)               //  0x00000001 
#define HSEM_C2IER_ISE0           HSEM_C2IER_ISE0_MASK                          // semaphore 0 , interrupt 1 enable bit.  
#define HSEM_C2IER_ISE1_SHIFT      (1U)
#define HSEM_C2IER_ISE1_MASK       (0x1UL << HSEM_C2IER_ISE1_MASK)               //  0x00000002 
#define HSEM_C2IER_ISE1           HSEM_C2IER_ISE1_MASK                          // semaphore 1 , interrupt 1 enable bit.  
#define HSEM_C2IER_ISE2_SHIFT      (2U)
#define HSEM_C2IER_ISE2_MASK       (0x1UL << HSEM_C2IER_ISE2_MASK)               //  0x00000004 
#define HSEM_C2IER_ISE2           HSEM_C2IER_ISE2_MASK                          // semaphore 2 , interrupt 1 enable bit.  
#define HSEM_C2IER_ISE3_SHIFT      (3U)
#define HSEM_C2IER_ISE3_MASK       (0x1UL << HSEM_C2IER_ISE3_MASK)               //  0x00000008 
#define HSEM_C2IER_ISE3           HSEM_C2IER_ISE3_MASK                          // semaphore 3 , interrupt 1 enable bit.  
#define HSEM_C2IER_ISE4_SHIFT      (4U)
#define HSEM_C2IER_ISE4_MASK       (0x1UL << HSEM_C2IER_ISE4_MASK)               //  0x00000010 
#define HSEM_C2IER_ISE4           HSEM_C2IER_ISE4_MASK                          // semaphore 4 , interrupt 1 enable bit.  
#define HSEM_C2IER_ISE5_SHIFT      (5U)
#define HSEM_C2IER_ISE5_MASK       (0x1UL << HSEM_C2IER_ISE5_MASK)               //  0x00000020 
#define HSEM_C2IER_ISE5           HSEM_C2IER_ISE5_MASK                          // semaphore 5 interrupt 1 enable bit.  
#define HSEM_C2IER_ISE6_SHIFT      (6U)
#define HSEM_C2IER_ISE6_MASK       (0x1UL << HSEM_C2IER_ISE6_MASK)               //  0x00000040 
#define HSEM_C2IER_ISE6           HSEM_C2IER_ISE6_MASK                          // semaphore 6 interrupt 1 enable bit.  
#define HSEM_C2IER_ISE7_SHIFT      (7U)
#define HSEM_C2IER_ISE7_MASK       (0x1UL << HSEM_C2IER_ISE7_MASK)               //  0x00000080 
#define HSEM_C2IER_ISE7           HSEM_C2IER_ISE7_MASK                          // semaphore 7 interrupt 1 enable bit.  
#define HSEM_C2IER_ISE8_SHIFT      (8U)
#define HSEM_C2IER_ISE8_MASK       (0x1UL << HSEM_C2IER_ISE8_MASK)               //  0x00000100 
#define HSEM_C2IER_ISE8           HSEM_C2IER_ISE8_MASK                          // semaphore 8 interrupt 1 enable bit.  
#define HSEM_C2IER_ISE9_SHIFT      (9U)
#define HSEM_C2IER_ISE9_MASK       (0x1UL << HSEM_C2IER_ISE9_MASK)               //  0x00000200 
#define HSEM_C2IER_ISE9           HSEM_C2IER_ISE9_MASK                          // semaphore 9 interrupt 1 enable bit.  
#define HSEM_C2IER_ISE10_SHIFT     (10U)
#define HSEM_C2IER_ISE10_MASK      (0x1UL << HSEM_C2IER_ISE10_MASK)              //  0x00000400 
#define HSEM_C2IER_ISE10          HSEM_C2IER_ISE10_MASK                         // semaphore 10 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE11_SHIFT     (11U)
#define HSEM_C2IER_ISE11_MASK      (0x1UL << HSEM_C2IER_ISE11_MASK)              //  0x00000800 
#define HSEM_C2IER_ISE11          HSEM_C2IER_ISE11_MASK                         // semaphore 11 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE12_SHIFT     (12U)
#define HSEM_C2IER_ISE12_MASK      (0x1UL << HSEM_C2IER_ISE12_MASK)              //  0x00001000 
#define HSEM_C2IER_ISE12          HSEM_C2IER_ISE12_MASK                         // semaphore 12 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE13_SHIFT     (13U)
#define HSEM_C2IER_ISE13_MASK      (0x1UL << HSEM_C2IER_ISE13_MASK)              //  0x00002000 
#define HSEM_C2IER_ISE13          HSEM_C2IER_ISE13_MASK                         // semaphore 13 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE14_SHIFT     (14U)
#define HSEM_C2IER_ISE14_MASK      (0x1UL << HSEM_C2IER_ISE14_MASK)              //  0x00004000 
#define HSEM_C2IER_ISE14          HSEM_C2IER_ISE14_MASK                         // semaphore 14 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE15_SHIFT     (15U)
#define HSEM_C2IER_ISE15_MASK      (0x1UL << HSEM_C2IER_ISE15_MASK)              //  0x00008000 
#define HSEM_C2IER_ISE15          HSEM_C2IER_ISE15_MASK                         // semaphore 15 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE16_SHIFT     (16U)
#define HSEM_C2IER_ISE16_MASK      (0x1UL << HSEM_C2IER_ISE16_MASK)              //  0x00010000 
#define HSEM_C2IER_ISE16          HSEM_C2IER_ISE16_MASK                         // semaphore 16 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE17_SHIFT     (17U)
#define HSEM_C2IER_ISE17_MASK      (0x1UL << HSEM_C2IER_ISE17_MASK)              //  0x00020000 
#define HSEM_C2IER_ISE17          HSEM_C2IER_ISE17_MASK                         // semaphore 17 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE18_SHIFT     (18U)
#define HSEM_C2IER_ISE18_MASK      (0x1UL << HSEM_C2IER_ISE18_MASK)              //  0x00040000 
#define HSEM_C2IER_ISE18          HSEM_C2IER_ISE18_MASK                         // semaphore 18 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE19_SHIFT     (19U)
#define HSEM_C2IER_ISE19_MASK      (0x1UL << HSEM_C2IER_ISE19_MASK)              //  0x00080000 
#define HSEM_C2IER_ISE19          HSEM_C2IER_ISE19_MASK                         // semaphore 19 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE20_SHIFT     (20U)
#define HSEM_C2IER_ISE20_MASK      (0x1UL << HSEM_C2IER_ISE20_MASK)              //  0x00100000 
#define HSEM_C2IER_ISE20          HSEM_C2IER_ISE20_MASK                         // semaphore 20 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE21_SHIFT     (21U)
#define HSEM_C2IER_ISE21_MASK      (0x1UL << HSEM_C2IER_ISE21_MASK)              //  0x00200000 
#define HSEM_C2IER_ISE21          HSEM_C2IER_ISE21_MASK                         // semaphore 21 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE22_SHIFT     (22U)
#define HSEM_C2IER_ISE22_MASK      (0x1UL << HSEM_C2IER_ISE22_MASK)              //  0x00400000 
#define HSEM_C2IER_ISE22          HSEM_C2IER_ISE22_MASK                         // semaphore 22 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE23_SHIFT     (23U)
#define HSEM_C2IER_ISE23_MASK      (0x1UL << HSEM_C2IER_ISE23_MASK)              //  0x00800000 
#define HSEM_C2IER_ISE23          HSEM_C2IER_ISE23_MASK                         // semaphore 23 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE24_SHIFT     (24U)
#define HSEM_C2IER_ISE24_MASK      (0x1UL << HSEM_C2IER_ISE24_MASK)              //  0x01000000 
#define HSEM_C2IER_ISE24          HSEM_C2IER_ISE24_MASK                         // semaphore 24 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE25_SHIFT     (25U)
#define HSEM_C2IER_ISE25_MASK      (0x1UL << HSEM_C2IER_ISE25_MASK)              //  0x02000000 
#define HSEM_C2IER_ISE25          HSEM_C2IER_ISE25_MASK                         // semaphore 25 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE26_SHIFT     (26U)
#define HSEM_C2IER_ISE26_MASK      (0x1UL << HSEM_C2IER_ISE26_MASK)              //  0x04000000 
#define HSEM_C2IER_ISE26          HSEM_C2IER_ISE26_MASK                         // semaphore 26 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE27_SHIFT     (27U)
#define HSEM_C2IER_ISE27_MASK      (0x1UL << HSEM_C2IER_ISE27_MASK)              //  0x08000000 
#define HSEM_C2IER_ISE27          HSEM_C2IER_ISE27_MASK                         // semaphore 27 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE28_SHIFT     (28U)
#define HSEM_C2IER_ISE28_MASK      (0x1UL << HSEM_C2IER_ISE28_MASK)              //  0x10000000 
#define HSEM_C2IER_ISE28          HSEM_C2IER_ISE28_MASK                         // semaphore 28 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE29_SHIFT     (29U)
#define HSEM_C2IER_ISE29_MASK      (0x1UL << HSEM_C2IER_ISE29_MASK)              //  0x20000000 
#define HSEM_C2IER_ISE29          HSEM_C2IER_ISE29_MASK                         // semaphore 29 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE30_SHIFT     (30U)
#define HSEM_C2IER_ISE30_MASK      (0x1UL << HSEM_C2IER_ISE30_MASK)              //  0x40000000 
#define HSEM_C2IER_ISE30          HSEM_C2IER_ISE30_MASK                         // semaphore 30 interrupt 1 enable bit. 
#define HSEM_C2IER_ISE31_SHIFT     (31U)
#define HSEM_C2IER_ISE31_MASK      (0x1UL << HSEM_C2IER_ISE31_MASK)              //  0x80000000 
#define HSEM_C2IER_ISE31          HSEM_C2IER_ISE31_MASK                         // semaphore 31 interrupt 1 enable bit. 

// HSEM_C2ICR register
#define HSEM_C2ICR_ISC0_SHIFT      (0U)
#define HSEM_C2ICR_ISC0_MASK       (0x1UL << HSEM_C2ICR_ISC0_MASK)               //  0x00000001 
#define HSEM_C2ICR_ISC0           HSEM_C2ICR_ISC0_MASK                          // semaphore 0 , interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC1_SHIFT      (1U)
#define HSEM_C2ICR_ISC1_MASK       (0x1UL << HSEM_C2ICR_ISC1_MASK)               //  0x00000002 
#define HSEM_C2ICR_ISC1           HSEM_C2ICR_ISC1_MASK                          // semaphore 1 , interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC2_SHIFT      (2U)
#define HSEM_C2ICR_ISC2_MASK       (0x1UL << HSEM_C2ICR_ISC2_MASK)               //  0x00000004 
#define HSEM_C2ICR_ISC2           HSEM_C2ICR_ISC2_MASK                          // semaphore 2 , interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC3_SHIFT      (3U)
#define HSEM_C2ICR_ISC3_MASK       (0x1UL << HSEM_C2ICR_ISC3_MASK)               //  0x00000008 
#define HSEM_C2ICR_ISC3           HSEM_C2ICR_ISC3_MASK                          // semaphore 3 , interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC4_SHIFT      (4U)
#define HSEM_C2ICR_ISC4_MASK       (0x1UL << HSEM_C2ICR_ISC4_MASK)               //  0x00000010 
#define HSEM_C2ICR_ISC4           HSEM_C2ICR_ISC4_MASK                          // semaphore 4 , interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC5_SHIFT      (5U)
#define HSEM_C2ICR_ISC5_MASK       (0x1UL << HSEM_C2ICR_ISC5_MASK)               //  0x00000020 
#define HSEM_C2ICR_ISC5           HSEM_C2ICR_ISC5_MASK                          // semaphore 5 interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC6_SHIFT      (6U)
#define HSEM_C2ICR_ISC6_MASK       (0x1UL << HSEM_C2ICR_ISC6_MASK)               //  0x00000040 
#define HSEM_C2ICR_ISC6           HSEM_C2ICR_ISC6_MASK                          // semaphore 6 interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC7_SHIFT      (7U)
#define HSEM_C2ICR_ISC7_MASK       (0x1UL << HSEM_C2ICR_ISC7_MASK)               //  0x00000080 
#define HSEM_C2ICR_ISC7           HSEM_C2ICR_ISC7_MASK                          // semaphore 7 interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC8_SHIFT      (8U)
#define HSEM_C2ICR_ISC8_MASK       (0x1UL << HSEM_C2ICR_ISC8_MASK)               //  0x00000100 
#define HSEM_C2ICR_ISC8           HSEM_C2ICR_ISC8_MASK                          // semaphore 8 interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC9_SHIFT      (9U)
#define HSEM_C2ICR_ISC9_MASK       (0x1UL << HSEM_C2ICR_ISC9_MASK)               //  0x00000200 
#define HSEM_C2ICR_ISC9           HSEM_C2ICR_ISC9_MASK                          // semaphore 9 interrupt 1 clear bit.  
#define HSEM_C2ICR_ISC10_SHIFT     (10U)
#define HSEM_C2ICR_ISC10_MASK      (0x1UL << HSEM_C2ICR_ISC10_MASK)              //  0x00000400 
#define HSEM_C2ICR_ISC10          HSEM_C2ICR_ISC10_MASK                         // semaphore 10 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC11_SHIFT     (11U)
#define HSEM_C2ICR_ISC11_MASK      (0x1UL << HSEM_C2ICR_ISC11_MASK)              //  0x00000800 
#define HSEM_C2ICR_ISC11          HSEM_C2ICR_ISC11_MASK                         // semaphore 11 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC12_SHIFT     (12U)
#define HSEM_C2ICR_ISC12_MASK      (0x1UL << HSEM_C2ICR_ISC12_MASK)              //  0x00001000 
#define HSEM_C2ICR_ISC12          HSEM_C2ICR_ISC12_MASK                         // semaphore 12 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC13_SHIFT     (13U)
#define HSEM_C2ICR_ISC13_MASK      (0x1UL << HSEM_C2ICR_ISC13_MASK)              //  0x00002000 
#define HSEM_C2ICR_ISC13          HSEM_C2ICR_ISC13_MASK                         // semaphore 13 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC14_SHIFT     (14U)
#define HSEM_C2ICR_ISC14_MASK      (0x1UL << HSEM_C2ICR_ISC14_MASK)              //  0x00004000 
#define HSEM_C2ICR_ISC14          HSEM_C2ICR_ISC14_MASK                         // semaphore 14 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC15_SHIFT     (15U)
#define HSEM_C2ICR_ISC15_MASK      (0x1UL << HSEM_C2ICR_ISC15_MASK)              //  0x00008000 
#define HSEM_C2ICR_ISC15          HSEM_C2ICR_ISC15_MASK                         // semaphore 15 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC16_SHIFT     (16U)
#define HSEM_C2ICR_ISC16_MASK      (0x1UL << HSEM_C2ICR_ISC16_MASK)              //  0x00010000 
#define HSEM_C2ICR_ISC16          HSEM_C2ICR_ISC16_MASK                         // semaphore 16 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC17_SHIFT     (17U)
#define HSEM_C2ICR_ISC17_MASK      (0x1UL << HSEM_C2ICR_ISC17_MASK)              //  0x00020000 
#define HSEM_C2ICR_ISC17          HSEM_C2ICR_ISC17_MASK                         // semaphore 17 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC18_SHIFT     (18U)
#define HSEM_C2ICR_ISC18_MASK      (0x1UL << HSEM_C2ICR_ISC18_MASK)              //  0x00040000 
#define HSEM_C2ICR_ISC18          HSEM_C2ICR_ISC18_MASK                         // semaphore 18 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC19_SHIFT     (19U)
#define HSEM_C2ICR_ISC19_MASK      (0x1UL << HSEM_C2ICR_ISC19_MASK)              //  0x00080000 
#define HSEM_C2ICR_ISC19          HSEM_C2ICR_ISC19_MASK                         // semaphore 19 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC20_SHIFT     (20U)
#define HSEM_C2ICR_ISC20_MASK      (0x1UL << HSEM_C2ICR_ISC20_MASK)              //  0x00100000 
#define HSEM_C2ICR_ISC20          HSEM_C2ICR_ISC20_MASK                         // semaphore 20 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC21_SHIFT     (21U)
#define HSEM_C2ICR_ISC21_MASK      (0x1UL << HSEM_C2ICR_ISC21_MASK)              //  0x00200000 
#define HSEM_C2ICR_ISC21          HSEM_C2ICR_ISC21_MASK                         // semaphore 21 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC22_SHIFT     (22U)
#define HSEM_C2ICR_ISC22_MASK      (0x1UL << HSEM_C2ICR_ISC22_MASK)              //  0x00400000 
#define HSEM_C2ICR_ISC22          HSEM_C2ICR_ISC22_MASK                         // semaphore 22 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC23_SHIFT     (23U)
#define HSEM_C2ICR_ISC23_MASK      (0x1UL << HSEM_C2ICR_ISC23_MASK)              //  0x00800000 
#define HSEM_C2ICR_ISC23          HSEM_C2ICR_ISC23_MASK                         // semaphore 23 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC24_SHIFT     (24U)
#define HSEM_C2ICR_ISC24_MASK      (0x1UL << HSEM_C2ICR_ISC24_MASK)              //  0x01000000 
#define HSEM_C2ICR_ISC24          HSEM_C2ICR_ISC24_MASK                         // semaphore 24 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC25_SHIFT     (25U)
#define HSEM_C2ICR_ISC25_MASK      (0x1UL << HSEM_C2ICR_ISC25_MASK)              //  0x02000000 
#define HSEM_C2ICR_ISC25          HSEM_C2ICR_ISC25_MASK                         // semaphore 25 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC26_SHIFT     (26U)
#define HSEM_C2ICR_ISC26_MASK      (0x1UL << HSEM_C2ICR_ISC26_MASK)              //  0x04000000 
#define HSEM_C2ICR_ISC26          HSEM_C2ICR_ISC26_MASK                         // semaphore 26 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC27_SHIFT     (27U)
#define HSEM_C2ICR_ISC27_MASK      (0x1UL << HSEM_C2ICR_ISC27_MASK)              //  0x08000000 
#define HSEM_C2ICR_ISC27          HSEM_C2ICR_ISC27_MASK                         // semaphore 27 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC28_SHIFT     (28U)
#define HSEM_C2ICR_ISC28_MASK      (0x1UL << HSEM_C2ICR_ISC28_MASK)              //  0x10000000 
#define HSEM_C2ICR_ISC28          HSEM_C2ICR_ISC28_MASK                         // semaphore 28 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC29_SHIFT     (29U)
#define HSEM_C2ICR_ISC29_MASK      (0x1UL << HSEM_C2ICR_ISC29_MASK)              //  0x20000000 
#define HSEM_C2ICR_ISC29          HSEM_C2ICR_ISC29_MASK                         // semaphore 29 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC30_SHIFT     (30U)
#define HSEM_C2ICR_ISC30_MASK      (0x1UL << HSEM_C2ICR_ISC30_MASK)              //  0x40000000 
#define HSEM_C2ICR_ISC30          HSEM_C2ICR_ISC30_MASK                         // semaphore 30 interrupt 1 clear bit. 
#define HSEM_C2ICR_ISC31_SHIFT     (31U)
#define HSEM_C2ICR_ISC31_MASK      (0x1UL << HSEM_C2ICR_ISC31_MASK)              //  0x80000000 
#define HSEM_C2ICR_ISC31          HSEM_C2ICR_ISC31_MASK                         // semaphore 31 interrupt 1 clear bit. 

// HSEM_C2ISR register
#define HSEM_C2ISR_ISF0_SHIFT      (0U)
#define HSEM_C2ISR_ISF0_MASK       (0x1UL << HSEM_C2ISR_ISF0_MASK)               //  0x00000001 
#define HSEM_C2ISR_ISF0           HSEM_C2ISR_ISF0_MASK                          // semaphore 0 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF1_SHIFT      (1U)
#define HSEM_C2ISR_ISF1_MASK       (0x1UL << HSEM_C2ISR_ISF1_MASK)               //  0x00000002 
#define HSEM_C2ISR_ISF1           HSEM_C2ISR_ISF1_MASK                          // semaphore 1 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF2_SHIFT      (2U)
#define HSEM_C2ISR_ISF2_MASK       (0x1UL << HSEM_C2ISR_ISF2_MASK)               //  0x00000004 
#define HSEM_C2ISR_ISF2           HSEM_C2ISR_ISF2_MASK                          // semaphore 2 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF3_SHIFT      (3U)
#define HSEM_C2ISR_ISF3_MASK       (0x1UL << HSEM_C2ISR_ISF3_MASK)               //  0x00000008 
#define HSEM_C2ISR_ISF3           HSEM_C2ISR_ISF3_MASK                          // semaphore 3 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF4_SHIFT      (4U)
#define HSEM_C2ISR_ISF4_MASK       (0x1UL << HSEM_C2ISR_ISF4_MASK)               //  0x00000010 
#define HSEM_C2ISR_ISF4           HSEM_C2ISR_ISF4_MASK                          // semaphore 4 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF5_SHIFT      (5U)
#define HSEM_C2ISR_ISF5_MASK       (0x1UL << HSEM_C2ISR_ISF5_MASK)               //  0x00000020 
#define HSEM_C2ISR_ISF5           HSEM_C2ISR_ISF5_MASK                          // semaphore 5 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF6_SHIFT      (6U)
#define HSEM_C2ISR_ISF6_MASK       (0x1UL << HSEM_C2ISR_ISF6_MASK)               //  0x00000040 
#define HSEM_C2ISR_ISF6           HSEM_C2ISR_ISF6_MASK                          // semaphore 6 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF7_SHIFT      (7U)
#define HSEM_C2ISR_ISF7_MASK       (0x1UL << HSEM_C2ISR_ISF7_MASK)               //  0x00000080 
#define HSEM_C2ISR_ISF7           HSEM_C2ISR_ISF7_MASK                          // semaphore 7 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF8_SHIFT      (8U)
#define HSEM_C2ISR_ISF8_MASK       (0x1UL << HSEM_C2ISR_ISF8_MASK)               //  0x00000100 
#define HSEM_C2ISR_ISF8           HSEM_C2ISR_ISF8_MASK                          // semaphore 8 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF9_SHIFT      (9U)
#define HSEM_C2ISR_ISF9_MASK       (0x1UL << HSEM_C2ISR_ISF9_MASK)               //  0x00000200 
#define HSEM_C2ISR_ISF9           HSEM_C2ISR_ISF9_MASK                          // semaphore 9 interrupt 1 status bit.  
#define HSEM_C2ISR_ISF10_SHIFT     (10U)
#define HSEM_C2ISR_ISF10_MASK      (0x1UL << HSEM_C2ISR_ISF10_MASK)              //  0x00000400 
#define HSEM_C2ISR_ISF10          HSEM_C2ISR_ISF10_MASK                         // semaphore 10 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF11_SHIFT     (11U)
#define HSEM_C2ISR_ISF11_MASK      (0x1UL << HSEM_C2ISR_ISF11_MASK)              //  0x00000800 
#define HSEM_C2ISR_ISF11          HSEM_C2ISR_ISF11_MASK                         // semaphore 11 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF12_SHIFT     (12U)
#define HSEM_C2ISR_ISF12_MASK      (0x1UL << HSEM_C2ISR_ISF12_MASK)              //  0x00001000 
#define HSEM_C2ISR_ISF12          HSEM_C2ISR_ISF12_MASK                         // semaphore 12 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF13_SHIFT     (13U)
#define HSEM_C2ISR_ISF13_MASK      (0x1UL << HSEM_C2ISR_ISF13_MASK)              //  0x00002000 
#define HSEM_C2ISR_ISF13          HSEM_C2ISR_ISF13_MASK                         // semaphore 13 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF14_SHIFT     (14U)
#define HSEM_C2ISR_ISF14_MASK      (0x1UL << HSEM_C2ISR_ISF14_MASK)              //  0x00004000 
#define HSEM_C2ISR_ISF14          HSEM_C2ISR_ISF14_MASK                         // semaphore 14 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF15_SHIFT     (15U)
#define HSEM_C2ISR_ISF15_MASK      (0x1UL << HSEM_C2ISR_ISF15_MASK)              //  0x00008000 
#define HSEM_C2ISR_ISF15          HSEM_C2ISR_ISF15_MASK                         // semaphore 15 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF16_SHIFT     (16U)
#define HSEM_C2ISR_ISF16_MASK      (0x1UL << HSEM_C2ISR_ISF16_MASK)              //  0x00010000 
#define HSEM_C2ISR_ISF16          HSEM_C2ISR_ISF16_MASK                         // semaphore 16 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF17_SHIFT     (17U)
#define HSEM_C2ISR_ISF17_MASK      (0x1UL << HSEM_C2ISR_ISF17_MASK)              //  0x00020000 
#define HSEM_C2ISR_ISF17          HSEM_C2ISR_ISF17_MASK                         // semaphore 17 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF18_SHIFT     (18U)
#define HSEM_C2ISR_ISF18_MASK      (0x1UL << HSEM_C2ISR_ISF18_MASK)              //  0x00040000 
#define HSEM_C2ISR_ISF18          HSEM_C2ISR_ISF18_MASK                         // semaphore 18 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF19_SHIFT     (19U)
#define HSEM_C2ISR_ISF19_MASK      (0x1UL << HSEM_C2ISR_ISF19_MASK)              //  0x00080000 
#define HSEM_C2ISR_ISF19          HSEM_C2ISR_ISF19_MASK                         // semaphore 19 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF20_SHIFT     (20U)
#define HSEM_C2ISR_ISF20_MASK      (0x1UL << HSEM_C2ISR_ISF20_MASK)              //  0x00100000 
#define HSEM_C2ISR_ISF20          HSEM_C2ISR_ISF20_MASK                         // semaphore 20 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF21_SHIFT     (21U)
#define HSEM_C2ISR_ISF21_MASK      (0x1UL << HSEM_C2ISR_ISF21_MASK)              //  0x00200000 
#define HSEM_C2ISR_ISF21          HSEM_C2ISR_ISF21_MASK                         // semaphore 21 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF22_SHIFT     (22U)
#define HSEM_C2ISR_ISF22_MASK      (0x1UL << HSEM_C2ISR_ISF22_MASK)              //  0x00400000 
#define HSEM_C2ISR_ISF22          HSEM_C2ISR_ISF22_MASK                         // semaphore 22 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF23_SHIFT     (23U)
#define HSEM_C2ISR_ISF23_MASK      (0x1UL << HSEM_C2ISR_ISF23_MASK)              //  0x00800000 
#define HSEM_C2ISR_ISF23          HSEM_C2ISR_ISF23_MASK                         // semaphore 23 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF24_SHIFT     (24U)
#define HSEM_C2ISR_ISF24_MASK      (0x1UL << HSEM_C2ISR_ISF24_MASK)              //  0x01000000 
#define HSEM_C2ISR_ISF24          HSEM_C2ISR_ISF24_MASK                         // semaphore 24 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF25_SHIFT     (25U)
#define HSEM_C2ISR_ISF25_MASK      (0x1UL << HSEM_C2ISR_ISF25_MASK)              //  0x02000000 
#define HSEM_C2ISR_ISF25          HSEM_C2ISR_ISF25_MASK                         // semaphore 25 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF26_SHIFT     (26U)
#define HSEM_C2ISR_ISF26_MASK      (0x1UL << HSEM_C2ISR_ISF26_MASK)              //  0x04000000 
#define HSEM_C2ISR_ISF26          HSEM_C2ISR_ISF26_MASK                         // semaphore 26 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF27_SHIFT     (27U)
#define HSEM_C2ISR_ISF27_MASK      (0x1UL << HSEM_C2ISR_ISF27_MASK)              //  0x08000000 
#define HSEM_C2ISR_ISF27          HSEM_C2ISR_ISF27_MASK                         // semaphore 27 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF28_SHIFT     (28U)
#define HSEM_C2ISR_ISF28_MASK      (0x1UL << HSEM_C2ISR_ISF28_MASK)              //  0x10000000 
#define HSEM_C2ISR_ISF28          HSEM_C2ISR_ISF28_MASK                         // semaphore 28 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF29_SHIFT     (29U)
#define HSEM_C2ISR_ISF29_MASK      (0x1UL << HSEM_C2ISR_ISF29_MASK)              //  0x20000000 
#define HSEM_C2ISR_ISF29          HSEM_C2ISR_ISF29_MASK                         // semaphore 29 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF30_SHIFT     (30U)
#define HSEM_C2ISR_ISF30_MASK      (0x1UL << HSEM_C2ISR_ISF30_MASK)              //  0x40000000 
#define HSEM_C2ISR_ISF30          HSEM_C2ISR_ISF30_MASK                         // semaphore 30 interrupt 1 status bit. 
#define HSEM_C2ISR_ISF31_SHIFT     (31U)
#define HSEM_C2ISR_ISF31_MASK      (0x1UL << HSEM_C2ISR_ISF31_MASK)              //  0x80000000 
#define HSEM_C2ISR_ISF31          HSEM_C2ISR_ISF31_MASK                         // semaphore 31 interrupt 1 status bit. 

// HSEM_C2MISR register
#define HSEM_C2MISR_MISF0_SHIFT    (0U)
#define HSEM_C2MISR_MISF0_MASK     (0x1UL << HSEM_C2MISR_MISF0_MASK)             //  0x00000001 
#define HSEM_C2MISR_MISF0         HSEM_C2MISR_MISF0_MASK                        // semaphore 0 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF1_SHIFT    (1U)
#define HSEM_C2MISR_MISF1_MASK     (0x1UL << HSEM_C2MISR_MISF1_MASK)             //  0x00000002 
#define HSEM_C2MISR_MISF1         HSEM_C2MISR_MISF1_MASK                        // semaphore 1 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF2_SHIFT    (2U)
#define HSEM_C2MISR_MISF2_MASK     (0x1UL << HSEM_C2MISR_MISF2_MASK)             //  0x00000004 
#define HSEM_C2MISR_MISF2         HSEM_C2MISR_MISF2_MASK                        // semaphore 2 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF3_SHIFT    (3U)
#define HSEM_C2MISR_MISF3_MASK     (0x1UL << HSEM_C2MISR_MISF3_MASK)             //  0x00000008 
#define HSEM_C2MISR_MISF3         HSEM_C2MISR_MISF3_MASK                        // semaphore 3 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF4_SHIFT    (4U)
#define HSEM_C2MISR_MISF4_MASK     (0x1UL << HSEM_C2MISR_MISF4_MASK)             //  0x00000010 
#define HSEM_C2MISR_MISF4         HSEM_C2MISR_MISF4_MASK                        // semaphore 4 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF5_SHIFT    (5U)
#define HSEM_C2MISR_MISF5_MASK     (0x1UL << HSEM_C2MISR_MISF5_MASK)             //  0x00000020 
#define HSEM_C2MISR_MISF5         HSEM_C2MISR_MISF5_MASK                        // semaphore 5 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF6_SHIFT    (6U)
#define HSEM_C2MISR_MISF6_MASK     (0x1UL << HSEM_C2MISR_MISF6_MASK)             //  0x00000040 
#define HSEM_C2MISR_MISF6         HSEM_C2MISR_MISF6_MASK                        // semaphore 6 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF7_SHIFT    (7U)
#define HSEM_C2MISR_MISF7_MASK     (0x1UL << HSEM_C2MISR_MISF7_MASK)             //  0x00000080 
#define HSEM_C2MISR_MISF7         HSEM_C2MISR_MISF7_MASK                        // semaphore 7 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF8_SHIFT    (8U)
#define HSEM_C2MISR_MISF8_MASK     (0x1UL << HSEM_C2MISR_MISF8_MASK)             //  0x00000100 
#define HSEM_C2MISR_MISF8         HSEM_C2MISR_MISF8_MASK                        // semaphore 8 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF9_SHIFT    (9U)
#define HSEM_C2MISR_MISF9_MASK     (0x1UL << HSEM_C2MISR_MISF9_MASK)             //  0x00000200 
#define HSEM_C2MISR_MISF9         HSEM_C2MISR_MISF9_MASK                        // semaphore 9 interrupt 1 masked status bit.  
#define HSEM_C2MISR_MISF10_SHIFT   (10U)
#define HSEM_C2MISR_MISF10_MASK    (0x1UL << HSEM_C2MISR_MISF10_MASK)            //  0x00000400 
#define HSEM_C2MISR_MISF10        HSEM_C2MISR_MISF10_MASK                       // semaphore 10 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF11_SHIFT   (11U)
#define HSEM_C2MISR_MISF11_MASK    (0x1UL << HSEM_C2MISR_MISF11_MASK)            //  0x00000800 
#define HSEM_C2MISR_MISF11        HSEM_C2MISR_MISF11_MASK                       // semaphore 11 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF12_SHIFT   (12U)
#define HSEM_C2MISR_MISF12_MASK    (0x1UL << HSEM_C2MISR_MISF12_MASK)            //  0x00001000 
#define HSEM_C2MISR_MISF12        HSEM_C2MISR_MISF12_MASK                       // semaphore 12 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF13_SHIFT   (13U)
#define HSEM_C2MISR_MISF13_MASK    (0x1UL << HSEM_C2MISR_MISF13_MASK)            //  0x00002000 
#define HSEM_C2MISR_MISF13        HSEM_C2MISR_MISF13_MASK                       // semaphore 13 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF14_SHIFT   (14U)
#define HSEM_C2MISR_MISF14_MASK    (0x1UL << HSEM_C2MISR_MISF14_MASK)            //  0x00004000 
#define HSEM_C2MISR_MISF14        HSEM_C2MISR_MISF14_MASK                       // semaphore 14 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF15_SHIFT   (15U)
#define HSEM_C2MISR_MISF15_MASK    (0x1UL << HSEM_C2MISR_MISF15_MASK)            //  0x00008000 
#define HSEM_C2MISR_MISF15        HSEM_C2MISR_MISF15_MASK                       // semaphore 15 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF16_SHIFT   (16U)
#define HSEM_C2MISR_MISF16_MASK    (0x1UL << HSEM_C2MISR_MISF16_MASK)            //  0x00010000 
#define HSEM_C2MISR_MISF16        HSEM_C2MISR_MISF16_MASK                       // semaphore 16 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF17_SHIFT   (17U)
#define HSEM_C2MISR_MISF17_MASK    (0x1UL << HSEM_C2MISR_MISF17_MASK)            //  0x00020000 
#define HSEM_C2MISR_MISF17        HSEM_C2MISR_MISF17_MASK                       // semaphore 17 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF18_SHIFT   (18U)
#define HSEM_C2MISR_MISF18_MASK    (0x1UL << HSEM_C2MISR_MISF18_MASK)            //  0x00040000 
#define HSEM_C2MISR_MISF18        HSEM_C2MISR_MISF18_MASK                       // semaphore 18 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF19_SHIFT   (19U)
#define HSEM_C2MISR_MISF19_MASK    (0x1UL << HSEM_C2MISR_MISF19_MASK)            //  0x00080000 
#define HSEM_C2MISR_MISF19        HSEM_C2MISR_MISF19_MASK                       // semaphore 19 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF20_SHIFT   (20U)
#define HSEM_C2MISR_MISF20_MASK    (0x1UL << HSEM_C2MISR_MISF20_MASK)            //  0x00100000 
#define HSEM_C2MISR_MISF20        HSEM_C2MISR_MISF20_MASK                       // semaphore 20 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF21_SHIFT   (21U)
#define HSEM_C2MISR_MISF21_MASK    (0x1UL << HSEM_C2MISR_MISF21_MASK)            //  0x00200000 
#define HSEM_C2MISR_MISF21        HSEM_C2MISR_MISF21_MASK                       // semaphore 21 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF22_SHIFT   (22U)
#define HSEM_C2MISR_MISF22_MASK    (0x1UL << HSEM_C2MISR_MISF22_MASK)            //  0x00400000 
#define HSEM_C2MISR_MISF22        HSEM_C2MISR_MISF22_MASK                       // semaphore 22 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF23_SHIFT   (23U)
#define HSEM_C2MISR_MISF23_MASK    (0x1UL << HSEM_C2MISR_MISF23_MASK)            //  0x00800000 
#define HSEM_C2MISR_MISF23        HSEM_C2MISR_MISF23_MASK                       // semaphore 23 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF24_SHIFT   (24U)
#define HSEM_C2MISR_MISF24_MASK    (0x1UL << HSEM_C2MISR_MISF24_MASK)            //  0x01000000 
#define HSEM_C2MISR_MISF24        HSEM_C2MISR_MISF24_MASK                       // semaphore 24 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF25_SHIFT   (25U)
#define HSEM_C2MISR_MISF25_MASK    (0x1UL << HSEM_C2MISR_MISF25_MASK)            //  0x02000000 
#define HSEM_C2MISR_MISF25        HSEM_C2MISR_MISF25_MASK                       // semaphore 25 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF26_SHIFT   (26U)
#define HSEM_C2MISR_MISF26_MASK    (0x1UL << HSEM_C2MISR_MISF26_MASK)            //  0x04000000 
#define HSEM_C2MISR_MISF26        HSEM_C2MISR_MISF26_MASK                       // semaphore 26 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF27_SHIFT   (27U)
#define HSEM_C2MISR_MISF27_MASK    (0x1UL << HSEM_C2MISR_MISF27_MASK)            //  0x08000000 
#define HSEM_C2MISR_MISF27        HSEM_C2MISR_MISF27_MASK                       // semaphore 27 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF28_SHIFT   (28U)
#define HSEM_C2MISR_MISF28_MASK    (0x1UL << HSEM_C2MISR_MISF28_MASK)            //  0x10000000 
#define HSEM_C2MISR_MISF28        HSEM_C2MISR_MISF28_MASK                       // semaphore 28 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF29_SHIFT   (29U)
#define HSEM_C2MISR_MISF29_MASK    (0x1UL << HSEM_C2MISR_MISF29_MASK)            //  0x20000000 
#define HSEM_C2MISR_MISF29        HSEM_C2MISR_MISF29_MASK                       // semaphore 29 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF30_SHIFT   (30U)
#define HSEM_C2MISR_MISF30_MASK    (0x1UL << HSEM_C2MISR_MISF30_MASK)            //  0x40000000 
#define HSEM_C2MISR_MISF30        HSEM_C2MISR_MISF30_MASK                       // semaphore 30 interrupt 1 masked status bit. 
#define HSEM_C2MISR_MISF31_SHIFT   (31U)
#define HSEM_C2MISR_MISF31_MASK    (0x1UL << HSEM_C2MISR_MISF31_MASK)            //  0x80000000 
#define HSEM_C2MISR_MISF31        HSEM_C2MISR_MISF31_MASK                       // semaphore 31 interrupt 1 masked status bit. 
// HSEM_CR register
#define HSEM_CR_COREID_SHIFT       (8U)
#define HSEM_CR_COREID_MASK        (0xFFUL << HSEM_CR_COREID_MASK)               //  0x0000FF00 
#define HSEM_CR_COREID            HSEM_CR_COREID_MASK                           // CoreID of semaphores to be cleared. 
#define HSEM_CR_KEY_SHIFT          (16U)
#define HSEM_CR_KEY_MASK           (0xFFFFUL << HSEM_CR_KEY_MASK)                //  0xFFFF0000 
#define HSEM_CR_KEY               HSEM_CR_KEY_MASK                              // semaphores clear key. 

// HSEM_KEYR register
#define HSEM_KEYR_KEY_SHIFT        (16U)
#define HSEM_KEYR_KEY_MASK         (0xFFFFUL << HSEM_KEYR_KEY_MASK)              //  0xFFFF0000 
#define HSEM_KEYR_KEY             HSEM_KEYR_KEY_MASK                            // semaphores clear key. 

// HSEM_CR register
#define HSEM_CPU1_COREID    (0x00000003U)					 // Semaphore Core H7 ID 
#define HSEM_CPU2_COREID    (0x00000001U) 					// Semaphore Core $4 ID 
#define HSEM_CR_COREID_CPU1      (HSEM_CPU1_COREID << HSEM_CR_COREID_Pos)
#define HSEM_CR_COREID_CPU2      (HSEM_CPU2_COREID << HSEM_CR_COREID_Pos)
#if defined(STM32H4)
#define HSEM_CR_COREID_CURRENT   (HSEM_CPU2_COREID << HSEM_CR_COREID_Pos)
#else  // M7 
#define HSEM_CR_COREID_CURRENT   (HSEM_CPU1_COREID << HSEM_CR_COREID_Pos)
#endif //M4 

#define HSEM_SEMEPHORE_ID_MIN     (0U)       // Min id
#define HSEM_SEMEPHORE_ID_MAX     (31U)      // Max id

#define HSEM_PROCESS_ID_MIN (0U)       //  Min Process ID
#define HSEM_PROCESS_ID_MAX (255U)     // Max Process ID 

#define HSEM_CLEAR_KEY_MIN (0U)       // clear Min Key
#define HSEM_CLEAR_KEY_MAX (0xFFFFU)  // clear Max Key
