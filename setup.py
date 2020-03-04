from setuptools import setup
from setuptools import find_packages
from Cython.Build import cythonize
from Cython.Distutils import build_ext
from distutils.extension import Extension


setup(
    name='BRTDP-DS-MPI',
    version='1.0',
    description='',
    author='Instance01',
    packages=find_packages(),
    install_requires=['osmnx', 'networkx', 'numpy'],
    include_dirs=["BRTDP_DS_MPI/"],
    cmdclass={"build_ext": build_ext},
    ext_modules=cythonize([
        Extension(
            "cpp_brtdp",
            ["BRTDP_DS_MPI/algorithm/cpp_brtdp.cpp"],
            include_dirs=["BRTDP_DS_MPI/external/"]
        ),
        Extension(
            "lib",
            ["BRTDP_DS_MPI/cpp_lib.cpp"],
            libraries=["m"]
        ),
        "BRTDP_DS_MPI/*.pyx",
        "BRTDP_DS_MPI/algorithm/*.pyx",
    ],
        language="c++",
        include_path=["BRTDP_DS_MPI/external/"]
    )
)
