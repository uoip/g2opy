from setuptools import setup
from setuptools.command.install import install
from sysconfig import get_config_var, get_paths
import glob
import shutil


# Only pick up the module built for the interpreter running this script: lib/ may
# hold builds made with other Python versions, and their file names differ by ABI
# tag (e.g. g2o.cpython-311-... vs g2o.cpython-312-...).
__library_file__ = './lib/g2o*' + get_config_var('EXT_SUFFIX')
__version__ = '0.0.1'



class CopyLibFile(install):
    """"
    Directly copy library file to python's site-packages directory.
    """

    def run(self):
        install_dir = get_paths()['purelib']
        lib_file = glob.glob(__library_file__)
        assert len(lib_file) == 1, \
            'expected exactly one {}, found {}'.format(__library_file__, lib_file)

        print('copying {} -> {}'.format(lib_file[0], install_dir))
        shutil.copy(lib_file[0], install_dir)




setup(
    name='g2opy',
    version=__version__,
    # Nothing is packaged as a Python module: the binding is a single prebuilt
    # extension module that CopyLibFile drops into site-packages.  An empty list
    # also keeps setuptools from guessing at a flat-layout and aborting with
    # "Multiple top-level packages discovered".
    packages=[],
    description='Python binding of C++ graph optimization framework g2o.',
    url='https://github.com/uoip/g2opy',
    license='BSD',
    cmdclass=dict(
        install=CopyLibFile
    ),
    keywords='g2o, SLAM, BA, ICP, optimization, python, binding',
    long_description="""This is a Python binding for c++ library g2o 
        (https://github.com/RainerKuemmerle/g2o).

        g2o is an open-source C++ framework for optimizing graph-based nonlinear 
        error functions. g2o has been designed to be easily extensible to a wide 
        range of problems and a new problem typically can be specified in a few 
        lines of code. The current implementation provides solutions to several 
        variants of SLAM and BA."""
)
