from setuptools import find_packages, setup

package_name = 'jaka_zu3_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vision_grasp.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hys',
    maintainer_email='a1502759908@gmail.com',
    description='RGB-D target detection and guarded pick execution for JAKA ZU3',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_detector = jaka_zu3_vision.target_detector:main',
            'target_motion = jaka_zu3_vision.target_motion:main',
        ],
    },
)
