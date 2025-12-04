from distutils.core import setup, Extension
import numpy as np

_qpSWIFT = Extension("qpSWIFT",
    sources= [
    "pyqpSWIFT.c",
    "../src/amd_1.c",
    "../src/amd_2.c",
    "../src/amd_aat.c",
    "../src/amd_control.c",
    "../src/amd_defaults.c",
    "../src/amd_dump.c",
    "../src/amd_global.c",
    "../src/amd_info.c",
    "../src/amd_order.c",
    "../src/amd_post_tree.c",
    "../src/amd_postorder.c",
    "../src/amd_preprocess.c",
    "../src/amd_valid.c",
    "../src/ldl.c",
    "../src/timer.c",
    "../src/Auxilary.c",
    "../src/qpSWIFT.c" 
    ],
    include_dirs=["../include/",
    np.get_include(),
    ],
#    extra_compile_args=["-O3"
#    ]
    )

def main():
    setup(
    name="qpSWIFT",
    version="1.0.0",
    description="Python interface for qpSWIFT",
    author="Abhishek Pandala",
    setup_requires=["numpy >= 1.6"],
    install_requires=["numpy >= 1.6"],
    ext_modules=[_qpSWIFT]
    )

if __name__ == "__main__":
    main()
