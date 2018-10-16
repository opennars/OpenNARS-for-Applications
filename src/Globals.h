#define DEBUG 1
#define DEBUG_INFO(x) {if(DEBUG == 1){}}

#ifdef DEBUG
#define PRINTD printf
#else
#define PRINTD(format, args...) ((void)0)
#endif

#define IN_DEBUG(x) {if(DEBUG){ x } }
