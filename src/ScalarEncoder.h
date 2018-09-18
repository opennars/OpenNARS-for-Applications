#ifndef H_SCALARENCODER
#define H_SCALARENCODER

////////////////////
//  Scalar encoder//
////////////////////
//Supports to encode a value in HTM way:
//https://www.youtube.com/watch?v=V3Yqtpytif0&list=PL3yXMgtrZmDqhsFQzwUC9V8MeeVOQ7eZ9&index=6

//References//
//-----------//
#include "SDR.h"

//Methods//
//-------//
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
