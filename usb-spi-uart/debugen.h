#ifndef DEBUGEN_H
#define DEBUGEN_H


int ramtest(void);
void usbdebug(void);
void printplls(void);
void dma_status(char *m);
void spi_status(unsigned int spibase, char *m);
#endif
