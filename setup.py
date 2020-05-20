from setuptools import setup, find_packages
setup(
    name="YAMSPy",
    version="0.2",
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