/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Usage.h"

double Usage_usefulness(Usage usage, long currentTime)
{
    double recency = MAX(0, currentTime - usage.lastUsed);
    double usefulnessToNormalize = ((double) usage.useCount) / (recency + 1.0);
    return usefulnessToNormalize / (usefulnessToNormalize + 1.0);
}

Usage Usage_use(Usage usage, long currentTime, bool eternalInput)
{
    return (Usage) { .useCount = usage.useCount+(eternalInput ? ETERNAL_INPUT_USAGE_BOOST : 1),
                     .lastUsed = currentTime };
}

void Usage_Print(Usage *usage)
{
    printf("Usage: useCount=%ld lastUsed=%ld\n", usage->useCount, usage->lastUsed);
}
