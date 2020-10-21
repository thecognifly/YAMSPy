from setuptools import setup, find_packages
import codecs
import os.path


# from https://packaging.python.org/guides/single-sourcing-package-version/
def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()

def get_version(rel_path):
    for line in read(rel_path).splitlines():
        if line.startswith('__version__'):
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    else:
        raise RuntimeError("Unable to find version string.")

setup(
    name="YAMSPy",
    version=get_version("yamspy/__init__.py"),
    packages=['yamspy'],
    install_requires=['pyserial'],

    # metadata to display on PyPI
    author="Ricardo de Azambuja",
    author_email="ricardo.azambuja@gmail.com",
    description="Yet Another Implementation of Multiwii Serial Protocol Python Interface for Betaflight",
    keywords="Betaflight iNAV drone UAV Multi Wii Serial Protocol MSP",
    url="https://github.com/ricardodeazambuja/YAMSPy",
    classifiers=[
        'Programming Language :: Python :: 3 :: Only' # https://pypi.org/classifiers/
    ]
)

# https://setuptools.readthedocs.io/en/latest/setuptools.html