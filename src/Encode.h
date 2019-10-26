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
#include "Term.h"
#include "Globals.h"

//Parameters//
//----------//
#define TERMS_MAX 255
#define TERMS_LEN 255
#define TERM_ONES 5

//Data structure//
//--------------//
char *terms[TERMS_MAX];

//Methods//
//-------//
//Encodes a term 
SDR Encode_Term(char *name);

#endif
