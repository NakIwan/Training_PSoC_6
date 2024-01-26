#ifndef INTERFACE_H_
#define IINTRFACE_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "GUI.h"
#include "mtb_ssd1306.h"

#ifndef INTERFACE_MAXCHAR
#define INTERFACE_MAXCHAR 22
#endif

typedef struct interface {
	cyhal_i2c_t *i2c;

	uint8_t total_menu;
	uint8_t position;
	uint8_t offset;

	const char **menu;
	unsigned char set_title;
	char title [INTERFACE_MAXCHAR];

	uint8_t cursor;
}interface_t;

void interface_construct(interface_t *interface_menu, cyhal_i2c_t *i2c);
void interface_begin(interface_t *interface_menu);
void interface_next(interface_t *interface_menu);
void interface_previous(interface_t *interface_menu);
void interface_reset_position(interface_t *interface_menu);

void interface_set_menu(interface_t *interface_menu, const char **menu, uint8_t total_menu);

uint8_t interface_getPosition(interface_t *interface_menu);

void interface_draw_menu(interface_t *interface_menu);
void interface_draw(interface_t *interface_menu);
void interface_clear(interface_t *interface_menu);

void interface_cursor(interface_t *interface_menu, uint8_t cursor);
void interface_setTitle(interface_t *interface_menu, const char *title);

#endif
