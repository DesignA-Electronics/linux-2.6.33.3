#ifndef _AT91_HAMMERHEAD_H
#define _AT91_HAMMERHEAD_H

enum {
	HAMMERHEAD_BOARD_UNKNOWN,
	HAMMERHEAD_BOARD_REV0,
	HAMMERHEAD_BOARD_REV1,
};

extern const char *hammerhead_variant_id_string(void);
extern int hammerhead_variant_id(void);

#endif /* _AT91_HAMMERHEAD_H */
