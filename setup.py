from setuptools import setup, find_packages

setup(
    name='sst_wrapper',
    version='0.0.1',
    description='baselines for planning env',
    long_description='A package to run baseline motion planning',
    author='Jacob Johnson',
    author_email='c_johnson@braincorportion.com',
    url='',
    download_url='',
    license='BSD License 2.0',
    install_requires=['numpy>=1.13.3'],
    dependency_links=[
        'https://github.com/jacobjj/bc-gym-planning-env.git',
        'https://github.com/jacobjj/sparse_rrt.git'
    ],
    package_data={'': ['input']},
    include_package_data=True,
    classifiers=[
        'Development Status :: 3 - Alpha', 'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python :: 3.6',
        'Topic :: Software Development :: Libraries',
        'Topic :: Software Development :: Libraries :: Python Modules'
    ],
    packages=find_packages(),
)
