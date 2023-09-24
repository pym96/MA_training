import math
import time
import numpy as np
import torch
from d2l import torch as d2l

n = 1000
a = torch.ones(n)
b = torch.ones(n)

class Timer():
    '''Saving running time'''
    def __init__(self):
        self.times = []
        self.start()

    def start(self):
        self.tik = time.time()

    def stop(self):
        self.times.append(time.time() - self.tik)
        return self.times[-1]
    
    def avf(self):
        return sum(self.times) / len(self.times)
    
    def sum(self):
        return sum(self.times)
    
    def cumsum(self):
        '''Accumulating sum of time'''
        return np.array(self.times).cumsum().tolist()
    
c = torch.zeros(n)
timer = Timer()
for i in range(n):
    c[i] = a[i] + b[i]

# print(f'{timer.stop():5f} sec') 
timer.start()
dd = a + b
print(f'{timer.stop()}:.5f sec')

