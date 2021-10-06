import sys
from setuptools import setup, find_packages

if sys.version_info < (3, 7):
    sys.exit('Sorry, Python < 3.7 is not supported.')

with open("README.md", "r") as fh:
    long_description = fh.read()
    
setup(
    name="yamspy",
    packages=[package for package in find_packages()],
    version="0.3.3",
    license="GPL",
    description="Yet Another Implementation of Multiwii Serial Protocol Python Interface for Betaflight, iNAV, etc.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Ricardo de Azambuja",
    author_email="ricardo.azambuja@gmail.com",
    url="https://github.com/thecognifly/YAMSPy",
    download_url="https://github.com/thecognifly/YAMSPy/archive/refs/tags/v0.3.3.tar.gz",
    keywords=['CogniFly', 'Betaflight', 'iNAV', 'drone', 'UAV', 'Multi Wii Serial Protocol', 'MSP'],
    install_requires=['pyserial'],
    classifiers=[
          'Development Status :: 4 - Beta',
          'Intended Audience :: Developers',
          'Intended Audience :: Education',
          'Intended Audience :: Information Technology',
          'Intended Audience :: Science/Research',
          'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
          'Operating System :: Microsoft :: Windows',
          'Operating System :: POSIX :: Linux',
          'Programming Language :: Python',
          'Framework :: Robot Framework :: Library',
          'Topic :: Education',
          'Topic :: Scientific/Engineering :: Artificial Intelligence'
    ]
)