from setuptools import setup, find_packages
package_name = 'humanoid_motion_controller'
setup(
    name=package_name, version='0.2.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='HUMANOID-LAB contributors', maintainer_email='maintainers@users.noreply.github.com',
    description='Safety-gated, kinematics-based 12-DOF joint reference controller.', license='MIT',
    entry_points={'console_scripts': ['motion_controller = humanoid_motion_controller.motion_controller_node:main']},
)
