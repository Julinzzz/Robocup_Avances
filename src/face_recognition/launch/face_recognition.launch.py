from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1) Nodo de cámara USB
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 1280,
                'image_height': 720,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'brightness': -10,
                'contrast': 50,
                'saturation': 70,
                'sharpness': 50,
                'autoexposure': False,
                'exposure': 100,
                'gain': 80,
                'white_balance_temperature_auto': True,
                'white_balance_temperature': 4600,
                'focus_auto': False,
                'focus': 50,
                'camera_name': 'webcam',
                'camera_info_url': ''
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw')
            ]
        ),

        # 2) Nodo de reconocimiento facial
        Node(
            package='face_recognition',
            executable='face_recognition',
            name='face_recognition',
            output='screen',
            remappings=[
                # si tu nodo lee de otro topic, ajusta aquí
                # ('/camera/image_raw','/camera/image_raw')
            ]
        ),

    ])
