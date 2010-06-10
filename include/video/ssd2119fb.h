/*
 * Simple SPI SSD2119 display driver
 *
 * Author: Carl Smith <carl@bluewatersys.com>
 *
 * Copyright (c) 2009, Bluewater Systems Ltd
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __SSD2119FB_H__
#define __SSD2119FB_H__

struct ssd2119_platform_data {
	int gpio_reset;
	int gpio_dc;
        int refresh_speed;
};

#endif /* __SSD2119FB_H__ */
