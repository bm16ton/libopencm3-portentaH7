#pragma once
#ifndef __ITCM_STARTUP_H__
#define __ITCM_STARTIP_H__

/* declare our function in assembly file as C-function */

	void ITCM_Startup(void);		//ATT: it uses all registers R0..R3!
    void SDRAM_Startup(void);

#endif
