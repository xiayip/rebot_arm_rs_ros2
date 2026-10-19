from setuptools import setup
setup(
    name='rebot_arm_hmi_bridge', version='0.1.0', packages=['rebot_arm_hmi_bridge'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/rebot_arm_hmi_bridge']),
                ('share/rebot_arm_hmi_bridge', ['package.xml'])],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='reBot ARM RS Developers', maintainer_email='yipeng.xia@hotmail.com',
    description='HMI CAN requests to verified ROS 2 controller switches', license='Apache-2.0',
    entry_points={'console_scripts': ['hmi_bridge = rebot_arm_hmi_bridge.node:main']},
)
