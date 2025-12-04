import numpy as np 
import qpSWIFT

### To remove traceback call statements
import sys
sys.tracebacklimit = 0


def error_ctrl(cmd,msg):
    try:
        exec(cmd)
        return False
    except TypeError as err:
        err_str = str(err)
        if(err_str == msg):
            return True
        else:
            return False

def output_ctrl(cmd):
    exec(cmd,globals())
    return res

### Solver Options
### For information about Solver options please refer to qpSWIFT
### documentation


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
note_opts = 'reseq = qpSWIFT.run(c,h,P,G,A,b,opts)\n'
note_no_opts = "reseq = qpSWIFT.run(c,h,P,G,A,b)\n"



### Options Arguments
assert error_ctrl("opts = {'MAXITER':-1}\n"+note_opts,"max iterations must be between 0 and 200, using the default value of 100"),"opts iter _1 test failed"
assert error_ctrl("opts = {'MAXITER':201}\n"+note_opts,"max iterations must be between 0 and 200, using the default value of 100"),"opts iter _2 test failed"

assert error_ctrl("opts = {'VERBOSE':-1}\n"+note_opts,"verbose must be non-negative integer, using the default value"),"opts verbose _1 test failed"
assert error_ctrl("opts = {'VERBOSE':0.2}\n"+note_opts,"verbose must be non-negative integer, using the default value"),"opts verbose _2 test failed"

assert error_ctrl("opts = {'ABSTOL':-1}\n"+note_opts,"absolute tolerance must be between 0 and 1, using the default value"),"opts abstol _1 test failed"
assert error_ctrl("opts = {'ABSTOL':1.1}\n"+note_opts,"absolute tolerance must be between 0 and 1, using the default value"),"opts abstol _2 test failed"
assert error_ctrl("opts = {'ABSTOL':1}\n"+note_opts,"absolute tolerance must be between 0 and 1, using the default value"),"opts abstol _3 test failed"

assert error_ctrl("opts = {'RELTOL':-1}\n"+note_opts,"realtive tolerance must be between 0 and 1, using the default value"),"opts reltol _1 test failed"
assert error_ctrl("opts = {'RELTOL':1.1}\n"+note_opts,"realtive tolerance must be between 0 and 1, using the default value"),"opts reltol _2 test failed"
assert error_ctrl("opts = {'RELTOL':1}\n"+note_opts,"realtive tolerance must be between 0 and 1, using the default value"),"opts reltol_3 test failed"

assert error_ctrl("opts = {'SIGMA':-1}\n"+note_opts,"sigma must be positive, using the default value"),"opts sigma _1 test failed"
assert error_ctrl("opts = {'SIGMA':1}\n"+note_opts,"sigma must be positive, using the default value"),"opts sigma _2 test failed"
### Options Arguments

### Input Checking
## c h and b
assert error_ctrl("c = np.array([1,2,1])\n"+note_no_opts,"c must be a floating array with one dimension"),"c float test failed"
assert error_ctrl("c = np.array([[1,2,1]])\n"+note_no_opts,"c must be a floating array with one dimension"),"c one dim test failed"

assert error_ctrl("h = np.array([-1,-1])\n"+note_no_opts,"h must be a floating array with one dimension"),"h float test failed"
assert error_ctrl("h = np.array([[-1.0,-1.0]])\n"+note_no_opts,"h must be a floating array with one dimension"),"h one dim test failed"

assert error_ctrl("b = np.array([3])\n"+note_no_opts,"b must be a floating array with one dimension"),"b float test failed"
assert error_ctrl("b = np.array([[3.0]])\n"+note_no_opts,"b must be a floating array with one dimension"),"b one dim test failed"
## c h and b

## P A and G
assert error_ctrl("P = np.array([[5,1,0],\n[1, 2, 1],\n[0, 1, 4]])\n"+note_no_opts,"P must be a floating matrix with two dimensions"),"P float test failed"
assert error_ctrl("P = np.array([5,1,0])\n"+note_no_opts,"P must be a floating matrix with two dimensions"),"P matrix test failed"
assert error_ctrl("P = np.array([[5,1,0,2],\n[1, 2.0, 1,2],\n[0, 1, 4,3]])\n"+note_no_opts,"c and P do not have compatible dimensions"),"P float test_1 failed"
assert error_ctrl("P = np.array([[5,1,0.0],\n[0, 1, 4]])\n"+note_no_opts,"c and P do not have compatible dimensions"),"P float test_2 failed"

assert error_ctrl("G = np.array([[-4,-4,0],\n[0,0,-1]])\n"+note_no_opts,"G must be a floating matrix with two dimensions"),"G float test failed"
assert error_ctrl("G = np.array([-4.0,-4,0])\n"+note_no_opts,"G must be a floating matrix with two dimensions"),"G matrix test failed"
assert error_ctrl("G = np.array([[-4,-4,0,2.0],\n[0,0.0,-1,3.0]])\n"+note_no_opts,"h and G do not have compatible dimensions"),"G float test_1 failed"
assert error_ctrl("G = np.array([[-4,-4,0.0]])\n"+note_no_opts,"h and G do not have compatible dimensions"),"G float test_2 failed"

assert error_ctrl("A = np.array([[1, -2, 1]])\n"+note_no_opts,"A must be a floating matrix with two dimensions"),"A float test failed"
assert error_ctrl("A = np.array([1.0, -2.0, 1.0])\n"+note_no_opts,"A must be a floating matrix with two dimensions"),"A matrix test failed"
assert error_ctrl("A = np.array([[2.0,1.0, -2.0, 1.0]])\n"+note_no_opts,"b and A do not have compatible dimensions"),"A float test_1 failed"
assert error_ctrl("A = np.array([[1.0, -2.0, 1.0],[1.0,3.0,4.5]])\n"+note_no_opts,"b and A do not have compatible dimensions"),"A float test_2 failed"
## P A and G

### Input Checking


### Output Arguments
opts_res = 1
assert output_ctrl("opts = {'OUTPUT':10}\n" + note_opts + "sol = reseq['sol']\n" + "if((sol.dtype != np.dtype('float64')) or  (sol.ndim != 1) or (sol.size!=c.size)):\n res = False\nelse:\n res = True\n"),"result _1 sol Test Failed"

assert output_ctrl("opts = {'OUTPUT':1}\n" + note_opts + "sol = reseq['sol']\n" + "if((sol.dtype != np.dtype('float64')) or  (sol.ndim != 1) or (sol.size!=c.size)):\n res = False\nelse:\n res = True\n"),"result _2 sol Test Failed"
assert output_ctrl("opts = {'OUTPUT':1}\n" + note_opts + "sol = reseq['basicInfo']['ExitFlag']\n" + "if((type(sol) != int)):\n res = False\nelse:\n res = True\n"),"result _2 basicinfo ExitFlag Test Failed"
assert output_ctrl("opts = {'OUTPUT':1}\n" + note_opts + "sol = reseq['basicInfo']['Iterations']\n" + "if((type(sol) != int)):\n res = False\nelse:\n res = True\n"),"result _2 basicinfo Iterations Test Failed"
assert output_ctrl("opts = {'OUTPUT':1}\n" + note_opts + "sol = reseq['basicInfo']['Setup_Time']\n" + "if((type(sol) != float)):\n res = False\nelse:\n res = True\n"),"result _2 basicinfo Setup_Time Test Failed"
assert output_ctrl("opts = {'OUTPUT':1}\n" + note_opts + "sol = reseq['basicInfo']['Solve_Time']\n" + "if((type(sol) != float)):\n res = False\nelse:\n res = True\n"),"result _2 basicinfo Solve_Time Test Failed"

assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['sol']\n" + "if((sol.dtype != np.dtype('float64')) or  (sol.ndim != 1) or (sol.size!=c.size)):\n res = False\nelse:\n res = True\n"),"result _3 sol Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['basicInfo']['ExitFlag']\n" + "if((type(sol) != int)):\n res = False\nelse:\n res = True\n"),"result _3 basicinfo ExitFlag Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['basicInfo']['Iterations']\n" + "if((type(sol) != int)):\n res = False\nelse:\n res = True\n"),"result _3 basicinfo Iterations Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['basicInfo']['Setup_Time']\n" + "if((type(sol) != float)):\n res = False\nelse:\n res = True\n"),"result _3 basicinfo Setup_Time Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['basicInfo']['Solve_Time']\n" + "if((type(sol) != float)):\n res = False\nelse:\n res = True\n"),"result _3 basicinfo Solve_Time Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['advInfo']['fval']\n" + "if((type(sol) != float)):\n res = False\nelse:\n res = True\n"),"result _3 advInfo fval Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['advInfo']['kktTime']\n" + "if((type(sol) != float)):\n res = False\nelse:\n res = True\n"),"result _3 advInfo kktTime Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['advInfo']['ldlTime']\n" + "if((type(sol) != float)):\n res = False\nelse:\n res = True\n"),"result _3 advInfo ldlTime Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['advInfo']['y']\n" + "if((sol.dtype != np.dtype('float64')) or  (sol.ndim != 1) or (sol.size!=b.size)):\n res = False\nelse:\n res = True\n"),"result _3 advinfo y Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['advInfo']['z']\n" + "if((sol.dtype != np.dtype('float64')) or  (sol.ndim != 1) or (sol.size!=h.size)):\n res = False\nelse:\n res = True\n"),"result _3 advinfo z Test Failed"
assert output_ctrl("opts = {'OUTPUT':2}\n" + note_opts + "sol = reseq['advInfo']['s']\n" + "if((sol.dtype != np.dtype('float64')) or  (sol.ndim != 1) or (sol.size!=h.size)):\n res = False\nelse:\n res = True\n"),"result _3 advinfo s Test Failed"
### Output Arguments


print('Syntax Test Passed')

