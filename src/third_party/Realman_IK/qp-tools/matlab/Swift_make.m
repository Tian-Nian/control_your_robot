function Swift_make(file)


DEBUG = 0;

if DEBUG == 1
    debug = ' -g';
else
    debug = '';
end
    
   
if(isempty(file))
    disp('Enter the name of the s_function to be used');
else
% d = '-largeArrayDims -DDLONG -DLDL_LONG';
% d = '-DDLONG -DLDL_LONG';
%% ldl solver
    fprintf('Compiling LDL...\n');
    cmd = ['mex', debug,' -c -largeArrayDims ','-I../include ','../src/ldl.c'];
    eval(cmd);
    fprintf('LDL Compilation Done\n');
    
%% AMD Solver
    fprintf('Compiling AMD...\n');
    cmd = ['mex -c ', debug,' -largeArrayDims ','-I../include ','../src/amd_order.c ','../src/amd_dump.c ','../src/amd_postorder.c ',...
            '../src/amd_post_tree.c ','../src/amd_aat.c ','../src/amd_2.c ','../src/amd_1.c ','../src/amd_defaults.c ','../src/amd_control.c ', ...
             '../src/amd_info.c ','../src/amd_valid.c ', '../src/amd_global.c ','../src/amd_preprocess.c ' ];
    eval(cmd);
    fprintf('AMD Compilation Done\n');
%% QP (core IPM solver)
    fprintf('Compiling QP...\n');
    cmd = ['mex -c ', debug,' -largeArrayDims ','-I../include ','../src/Auxilary.c ','../src/qpSWIFT.c ','../src/timer.c '];
    eval(cmd);
    fprintf('QP Compilation Done !!!\n');
 
%% QP_mex
    fprintf('Compiling QP_sfunc...\n');
    if (strcmp([file(end-1),file(end)],'.c'))
        cmd = ['mex -c ', debug,' -largeArrayDims ','-I../include ',file];
        file = file(1:end-2);
    else
        cmd = ['mex -c ', debug,' -largeArrayDims ','-I../include ',file,'.c'];
    end
    eval(cmd);
    fprintf('QP_sfunc Compilation Done\n');
    fprintf('Linking Object Files...\n');
    if (ispc) 
        cmd = ['mex ', debug,' -largeArrayDims ','qpSWIFT.obj Auxilary.obj timer.obj ldl.obj ' ...
                     'amd_info.obj amd_1.obj amd_2.obj amd_control.obj amd_defaults.obj amd_aat.obj '...
                     'amd_global.obj amd_post_tree.obj amd_postorder.obj amd_preprocess.obj amd_dump.obj '...
                     'amd_order.obj amd_valid.obj ',file '.obj',' -output ', 'qpSWIFT'];
    end
    if (isunix) 
        cmd = ['mex ', debug,' -largeArrayDims ','qpSWIFT.o Auxilary.o timer.o ldl.o '...
                      'amd_info.o amd_1.o amd_2.o amd_control.o amd_defaults.o amd_aat.o '...
                      'amd_global.o amd_post_tree.o amd_postorder.o amd_preprocess.o amd_dump.o '...
                      'amd_order.o amd_valid.o ',file '.o',' -output ', 'qpSWIFT']; 
    end
    eval(cmd);
    fprintf('Successfully Compiled \n');

%% clean
    if( ispc ), delete('*.obj'); end
    if( isunix), delete('*.o');end
    fprintf('Object Files cleaned \n');
    clear;
end
