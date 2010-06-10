/* Platform data for hi358x ARINC 429 receiver */

#ifndef _SPI_HI358X_
#define _SPI_HI358X_

enum {
	HI3585 = 1,
	HI3587,
	HI3588,
};

struct hi358x_platform_data {
	int model;
	int tx_irq;
	int rx_irq;
};

#endif /* _SPI_HI358X_ */

