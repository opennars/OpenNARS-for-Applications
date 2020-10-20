"""
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
 * """

discretization = 100
Midpoint = (500, 500)

def detectionPosition(x, y, w, h):
    xmid = x + w/2 #assuming x and y is at the top left
    ymid = y + h/2 #point of the bounding box, please adjust if not!
    return (xmid, ymid)

def discretizedPosition(xmid, ymid):
    return (xmid / discretization, ymid / discretization)

def discretizedSize(w, h):
    sz = max(w,h)
    if sz < 100:
        return "small"
    elif sz < 500:
        return "average"
    else:
        return "large"
        
(xmid_discrete, ymid_discrete) = discretizedPosition(Midpoint[0], Midpoint[1])

def foveaRelativePosition(x, y):
    (x_discrete, y_discrete) = discretizedPosition(x, y)
    (xLabel, yLabel) = ("centeredX", "centeredY")
    if x_discrete < xmid_discrete:
        xLabel = "lessX"
    elif x_discrete > xmid_discrete:
        xLabel = "moreX"
    if y_discrete < ymid_discrete:
        xLabel = "lessY"
    elif y_discrete > ymid_discrete:
        yLabel = "moreY"
    return (xLabel, yLabel)

def object_narsese(Property, Class, xLabel, yLabel, Confidence=0.9):
    return "<([" + Property + "] & " + Class + ") --> [" + xLabel + " " + yLabel + "]>. :|: {1.0 " + str(Confidence) + "}"

