import numpy as np

import sys
import qpSWIFT
import scipy.io as sio

data = sio.loadmat('Matrix.mat')

P = np.array(data['P'])
c = np.reshape(np.array(data['c']),(data['c'].size))
A = np.array(data['A'])
b = np.reshape(np.array(data['b']),(data['b'].size))
G = np.array(data['G'],dtype=float)
h = np.reshape(np.array(data['h']),(data['h'].size))

num = 1000

for i in range(0,num):
    reseq = qpSWIFT.run(c,h,P,G,A,b)
    sys.stdout.write('\r')
    sys.stdout.write("Progress: %d %%" % ((i*1.0/num)*0.5*100))
    sys.stdout.flush()


for i in range(0,num):
    reseq = qpSWIFT.run(c,h,P,G)
    sys.stdout.write('\r')
    sys.stdout.write("Progress: %d %%" % ((i*1.0/num)*0.5*100 + 50))
    sys.stdout.flush()

sys.stdout.write('\r')
print("Basic Test Passed")
