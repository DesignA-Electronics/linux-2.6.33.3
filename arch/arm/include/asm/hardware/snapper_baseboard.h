/*
 *  Snapper baseboard handling
 *
 *  Copyright (C) 2010 Bluewater Systems Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _SNAPPER_BASEBOARD_H
#define _SNAPPER_BASEBOARD_H

/* Magic Snapper baseboard macro */
#define SNAPPER_BASEBOARD(name, init_func)				\
	static int __##name##_baseboard = 0;				\
	static int __init __##name##_setup(char *str)			\
	{								\
		int len;						\
									\
		len = strlen(str);					\
		if (str && len > 0)					\
			if (strnicmp(#name, str, len) == 0)		\
				__##name##_baseboard = 1;		\
		return 0;						\
	}								\
	static int __init __##name##_init(void)				\
	{								\
		return init_func();					\
	}								\
	__setup("baseboard=", __##name##_setup);			\
	arch_initcall(__##name##_init);

#endif /* _SNAPPER_BASEBOARD_H */

