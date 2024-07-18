from setuptools import find_packages, setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = 'autonomous_navigation_challenge_2024'

generate_parameter_module(
  "poly_creator_parameters", # python module name for parameter library
  "autonomous_navigation_challenge_2024/poly_creator_parameters.yaml" # path to input yaml file
)

generate_parameter_module(
    "navigation_parameters", # python module name for parameter library
    "autonomous_navigation_challenge_2024/navigation_parameters.yaml" # path to input yaml file
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.10/site-packages/'+package_name+'/pipelines/yolo1', ['autonomous_navigation_challenge_2024/pipelines/yolo1/best.pt', 'autonomous_navigation_challenge_2024/pipelines/yolo1/best2.pt']),
        ('lib/python3.10/site-packages/'+package_name+'/pipelines/frcnn/faster_rcnn_inception_v2/inference_graph', ['autonomous_navigation_challenge_2024/pipelines/frcnn/faster_rcnn_inception_v2/inference_graph/frozen_inference_graph.pb']),
        ('share/' + package_name +'/config', ['autonomous_navigation_challenge_2024/config/my_navigate_to_pose_w_replanning_and_recovery.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antonio Langella',
    maintainer_email='a.langella31@studenti.unisa.it',
    description='This module contains the code for the 2024 challenge of the "Mobile Robots for Critial Missions" course',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation = autonomous_navigation_challenge_2024.navigation:main',
            'perception = autonomous_navigation_challenge_2024.perception:main',
            'poly_creator = autonomous_navigation_challenge_2024.poly_creator:main',
            'mock_signpost_publisher = autonomous_navigation_challenge_2024.mock_signpost_publisher:main',
            "test_detector = autonomous_navigation_challenge_2024.test_detector:main",
            "image_publisher = autonomous_navigation_challenge_2024.image_publisher:main",
            "view_camera = autonomous_navigation_challenge_2024.view_camera:main",
            "pub_video = autonomous_navigation_challenge_2024.pub_video:main",
            "change_parameters = autonomous_navigation_challenge_2024.change_parameters:main",
            "mock_kidnap = autonomous_navigation_challenge_2024.mock_kidnap:main",
        ],
    },
)
