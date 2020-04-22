#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'hd_map',
    ],
    package_dir={'': 'src'},
)
setup(**d)
