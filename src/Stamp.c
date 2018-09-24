#include "Stamp.h"

void stamp_init(Stamp *stamp) {
    for (int i=0;i<STAMP_SIZE;i++) {
        stamp->ids[i] = STAMP_FREE;
    }
}

bool stamp_checkOverlap(Stamp *a, Stamp *b) {
    for (int i=0;i<STAMP_SIZE;i++) {
        if (a->ids[i] == STAMP_FREE) {
            continue;
        }

        for (int j=0;j<STAMP_SIZE;j++) {
            if (b->ids[j] == STAMP_FREE) {
                continue;
            }

            if (a->ids[i] == b->ids[j]) {
                return 1;
            }
        }
    }

    return 0;
}
