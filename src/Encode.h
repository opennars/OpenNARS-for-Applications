#ifndef H_SCALARENCODER
#define H_SCALARENCODER

//////////////////////////////
//  Scalar and term encoder //
//////////////////////////////
//Supports to encode a value in HTM way:
//https://www.youtube.com/watch?v=V3Yqtpytif0&list=PL3yXMgtrZmDqhsFQzwUC9V8MeeVOQ7eZ9&index=6

//References//
//-----------//
#include <string.h>
#include <math.h>
#include "SDR.h"
#include "Globals.h"

//Parameters//
//----------//
#define TERMS_MAX 500
#define TERM_ONES 5

//Data structure//
//--------------//
SDR input_terms[TERMS_MAX];
bool input_terms_used[TERMS_MAX];

//Methods//
//-------//
//encodes an integer value as a SDR
//https://www.youtube.com/watch?v=V3Yqtpytif0&list=PL3yXMgtrZmDqhsFQzwUC9V8MeeVOQ7eZ9&index=6
SDR Encode_Scalar(int min, int max, int value);
//Encodes a term 
SDR Encode_Term(char *name);

#endif
