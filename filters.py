"""An assortment of time-series filters."""

from klampt import vectorops
from collections import defaultdict


class ExponentialFilter:
    def __init__(self,rate):
        self.rate = rate
        self.value = None
    def process(self,inputs):
        #do a filter
        if self.value == None:
            self.value = inputs
        else:
            #exponential filter
            x = vectorops.mul(self.value,1.0-self.rate)
            self.value = vectorops.madd(x,inputs,self.rate)
        return self.value

class DeadbandFilter:
    def __init__(self,width=1):
        self.width = width
        self.last = None
    def process(self,inputs):
        if self.last == None:
            self.last = inputs[:]
        for i,(l,v) in enumerate(zip(self.last,inputs)):
            if abs(l-v) > self.width:
                self.last[i] = v
        return self.last

class WeightedMajorityFilter:
    def __init__(self,weights=[1,0.5,0.5,0.5]):
        self.weights = weights
        self.history = []
    def process(self,inputs):
        if len(self.history) >= len(self.weights):
            del self.history[-1]
        self.history = [inputs[:]]+self.history
        res = []
        for i in range(len(inputs)):
            counts = defaultdict(float)
            for v,w in zip(self.history,self.weights[:len(self.history)]):
                counts[v[i]] += w
            majority = max(counts.items(),key=lambda(x):x[1])[0]
            res.append(majority)
        return res

class CompositeFilter:
    def __init__(self,filters):
        self.filters = filters[:]

    def process(self,inputs):
        vals = inputs
        for f in self.filters:
            vals = f.process(vals)
        return vals
