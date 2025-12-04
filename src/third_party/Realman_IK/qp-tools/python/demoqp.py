import numpy as np 
import qpSWIFT


###  Solver Options
### For information about Solver options please refer to qpSWIFT
### documentation

opts = {'MAXITER':30,'VERBOSE':1,'OUTPUT':2}

### Cost Function

P = np.array([[5.0,1.0,0.0],
         [1.0, 2.0, 1.0],
         [0.0, 1.0, 4.0]])
    
c = np.array([1.0,2.0,1.0])

### Inequality Constraints
G = np.array([[-4.0,-4.0,0.0],
        [0.0,0.0,-1.0]])

h = np.array([-1.0,-1.0])

### Equality Constraints
A = np.array([[1.0, -2.0, 1.0]])

b = np.array([3.0])

### Equality Constrained QP
reseq = qpSWIFT.run(c,h,P,G,A,b,opts)

### Inequality Constrained QP
res = qpSWIFT.run(c,h,P,G,opts=opts)


### Solution
print(res['sol'])


print(reseq['sol'])
