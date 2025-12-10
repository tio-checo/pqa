/*********************************************************************
 *
 * Support for MCP3913 six-channel Analog Front End (AFE).
 *
 *********************************************************************/

#ifndef __MCP3913_H
#define __MCP3913_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* constants */
/* number of analog channels */
#define MCP3913_NUM_CHANNELS	6

/* types */

/* global variables */
extern SemaphoreHandle_t	mcp3913_semaphore;

/* public functions */
/* Initialization routine. */
extern int mcp3913_init(void);
/*
 * Read content of the register.
 */
extern int mcp3913_read(uint8_t, uint32_t *);
/*
 * Read all six ADC channels at once.
 * Expects data space allocated by caller.
 */
extern int mcp3913_read_all_channels(int32_t *);
/*
 * Write register.
 */
extern int mcp3913_write(uint8_t, uint32_t);

#endif /* __MCP3913_H */

