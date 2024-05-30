from setuptools import find_packages
from setuptools import setup

setup(
    name='interface_t1',
    version='0.0.0',
    packages=find_packages(
        include=('interface_t1', 'interface_t1.*')),
)
