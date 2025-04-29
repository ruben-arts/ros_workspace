from setuptools import setup, find_packages
from setuptools.command.install import install as _install

class CustomInstall(_install):
    def finalize_options(self):
        super().finalize_options()
        # Override to install scripts to lib/talker
        self.install_scripts = self.install_base + '/lib/talker'

setup(
    name='talker',
    version='0.0.5',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/talker']),
        ('share/talker', ['package.xml']),
    ],
    zip_safe=True,
    maintainer='rubenarts',
    entry_points={
        'console_scripts': [
            'talker = talker.talker:main',
        ],
    },
    cmdclass={'install': CustomInstall},
)
