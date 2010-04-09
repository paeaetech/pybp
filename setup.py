from distutils.core import setup

setup(name='pybp',
	version='0.3',
	description='Interface for BusPirate'
	author='Mikko Sivulainen',
	author_email='sivu@paeae.com',
	py_modules = ['pybp'],
	url = 'http://github.com/paeaetech/pybp',
	requires = ['serial'],
)