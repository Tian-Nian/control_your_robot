import numpy as np
import qpSWIFT

opts = {"MAXITER": 10, "VERBOSE": 1}

P = np.array([[65.0, -22, -16],
              [-22.0, 14, 7],
              [-16, 7, 5]])

c = np.array([3.0, 2.0, 3.0])


G = np.array([[1.0, 2.0, 1.0],
              [2.0, 0.0, 1.0],
              [-1.0, 2.0, -1.0]])

h = np.array([3.0, 2.0, -2.0])

A = np.array([[1.0, 1.0, 1.0]])

b = np.array([1.0])

k = qpSWIFT.run(c, h, P, G, A, b, opts)
