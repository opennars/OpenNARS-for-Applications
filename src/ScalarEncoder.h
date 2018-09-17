#ifndef H_SCALARENCODER
#define H_SCALARENCODER

#include "SDR.h"

/**
 * encodes a scalar integer value as a SDR
 *
 * /param w width of encoding of a single bucket
 * /param min minimum of encoded range
 * /param max maximum of encoded range
 * /param value encoded value
 */
SDR encoder_scalar(int w, int min, int max, int value);

#endif
