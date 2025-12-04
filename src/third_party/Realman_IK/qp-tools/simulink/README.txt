-------------------------------------------------------------------------------
Compilation -> Type Swift_make('qpSWIFT_sfunc_e') in your matlab command window
               to compile qpSWIFT to handle quadratic programs with inequality 
               and equality constraints. The number of output variables of 
               s-function is set to 3. To change this, modify the pre-processor 
               defintion NV (line 13) to required variables in the s-function qpSWIFT_sfunc_e.c
            -> Type Swift_make('qpSWIFT_sfunc') in your matlab command window
               to compile qpSWIFT to handle quadratic programs with only
               inequality constraints. To change this modify the pre-processor 
               defintion NV (line 13) to required variables in the s-function qpSWIFT_sfunc.c
            -> Add the corresponding mex file to your working directory to use qpSWIFT
-------------------------------------------------------------------------------
Usage       -> Instructions on using the qpSWIFT_sfunc.c s-function can be found in the 
               inputData function of demoqp.slx
            -> Instructions on using the qpSWIFT_sfunc_e.c s-function can be found in the
               inputData_e function of demoqp_e.slx 
-------------------------------------------------------------------------------
Demo        -> Demo QP is given in demoqp.slx and demoqp_e.slx
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
Note: Make sure you have compatible C compiler available for your matlab version
-------------------------------------------------------------------------------