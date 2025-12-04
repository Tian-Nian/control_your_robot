## qpSWIFT - Light Weight Interior Point Solver
# ---------------------------------------------------
# ---------- Installation ---------------------------
# ---------------------------------------------------
#   python3 setup.py install (might need sudo for Linux users)
#   
#   pip import qpSWIFT
# ---------------------------------------------------
# ---------- Usage ----------------------------------
# ---------------------------------------------------
#	General Quadratic Program
#
#   minimize    0.5*x'Px + c'x
#       subject to  Ax = b
#                   Gx <= h
#
#     res = qpSWIFT.run(c,h,P,G,A,b,opts)
#			or
#	  res = qpSWIFT.run(c,h,P,G,opts=opts)
#
#        minimize    0.5*x'Px + c'x
#        subject to  Ax = b
#                    Gx <= h
#
# ---------------------------------------------------
#
# ---------------------------------------------------
#
#    INPUT arguments:
#
#       P is a numpy matrix of dimension (n,n)
#
#       c is a numpy column vector of size n
#
#       A is a numpy matrix of size (p,n); p is number of equality constraints
#
#       b is a numpy column vector of size p
#
#       G is a numpy matrix of size (m,n); m is the number of inequality constraints
#
#       h is a numpy column vector of size m
#
#       opts is a dictionary with the following fields
# 
#           -> MAXITER : maximum number of iterations needed
#           -> ABSTOL  : absolute tolerance
#           -> RELTOL  : relative tolerance
#           -> SIGMA   : maximum centering allowed
#           -> VERBOSE : PRINT LEVELS  ||  0            -- No Print
#                                      || >0            -- Print everything
#           -> OUTPUT  : OUTPUT LEVELS ||  1            -- sol + basicInfo
#                                      ||  2            -- sol + basicInfo + advInfo
#                                      ||  (anything else)   -- sol                  
#
#   Note: Options,A and b are not mandatory
#   Note: All the fileds of Options are also not mandatory
#   Note: All the input Matrices should be numpy objects
# --------------------------------------------------
#
#	OUTPUT arguments:
#
#   res represents a dictionary class with the following key value pairs
#
#   *   [sol] : Basic Solution represented as numpy vector
#
#   *   [basic_info] : Dictionary class with four keyvalue pairs
#           -> Exit Flag : 0 : Optimal Solution Found
#                        : 1 : Failure in factorising KKT matrix
#                        : 2 : Maximum Number of Iterations Reached
#                        : 3 : Unknown Problem in Solver
#           -> Iterations : Number of Iterations
#           -> Setup Time : Invloves setting up QP; solving for initial guess
#           -> Solve Time : Solution Time
#
#   *   [adv_info] : Dictionary class with five keyvalue pairs
#      -> Fval       : Objective Value of the QP
#      -> KKT_Time   : Time needed to solve the KKT system of equations
#      -> LDL_Time   : Time needed to perform LDL' factorization
#      -> y          : Dual Variables 
#      -> z          : Dual Variables
#      -> s          : Primal Slack Variables
# 
# 
# Copyright (C) Abhishek Pandala [agp19@vt.edu], Hae Won Park [haewonpark@kaist.ac.kr], Yanran Ding [yanran@mit.edu]
