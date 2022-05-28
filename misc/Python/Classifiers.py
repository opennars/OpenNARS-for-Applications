"""
 * The MIT License
 *
 * Copyright 2021 The OpenNARS authors.
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

import numpy as np

class AdaptiveHypersphereClassifier:
  def __init__(self, spheresize = 0.1, adaptation = 0.01, maxPrototypes=10):
    self.prototypes = []
    self.spheresize = spheresize
    self.adaptation = adaptation
    self.maxPrototypes = maxPrototypes
  def returnedPrototype(self, prototype):
    if len(self.prototypes) == 1:
      return -1 #no way to make distinction with just 1 prototype
    return prototype
  def calc(self, x):
    closest_i = -1
    closest_dist = float("inf")
    for i, prot in enumerate(self.prototypes):
      dist = np.linalg.norm(prot - x)
      if np.linalg.norm(prot - x) < self.spheresize and dist < closest_dist:
        closest_i = i
        closest_dist = dist
    if closest_i != -1:
      self.prototypes[closest_i] = self.prototypes[closest_i] + (x - self.prototypes[closest_i]) * self.adaptation 
      return self.returnedPrototype(closest_i)
    self.prototypes.append(x)
    self.prototypes = self.prototypes[-self.maxPrototypes:]
    return self.returnedPrototype(len(self.prototypes)-1)
  def observe(self, x, name = ""):
    narsese = "%d%s. :|:" if name == "" else "<%d --> [%s]>. :|:"
    return [narsese % (self.calc(x), name)]
