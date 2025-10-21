from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[
                {
                    'video_device': '/dev/video0', # ← Cambia este valor para cambiar de dispositivo
                    'image_width': 1280,       # Mayor resolución para mejor detección
                    'image_height': 720,
                    'framerate': 30.0,
                    'pixel_format': 'yuyv',
                    
                    # Ajustes de imagen para aspecto natural (valores típicos)
                    'brightness': -10,          # Rango 0-100 (50 = neutral)
                    'contrast': 50,            # Rango 0-100 (50 = neutral)
                    'saturation': 70,          # Rango 0-100 (60 para colores vivos pero naturales)
                    'sharpness': 50,           # Rango 0-100 (50 = neutral)
                    
                    # Control de exposición/luz
                    'autoexposure': False,     # Desactivar para control manual
                    'exposure': 100,           # Rango 1-10000 (100-200 para interior normal)
                    'gain': 80,                # Rango 1-100 (80 para condiciones normales)
                    
                    # Balance de blancos automático
                    'white_balance_temperature_auto': True,  # Activado para colores naturales
                    'white_balance_temperature': 4600,       # Valor inicial (ajuste automático lo sobreescribirá)
                    
                    # Enfoque (si tu cámara lo soporta)
                    'focus_auto': False,       # Desactivar enfoque automático
                    'focus': 50,               # Rango 0-100 (50 para condiciones normales)
                    
                    'camera_name': 'webcam',
                    'camera_info_url': ''
                }
            ],
            remappings=[
                ('/image_raw', '/camera/image_raw')
            ]
        ),
        Node(
            package='robocup_pose_estimation',
            executable='pose_node',
            name='pose_estimator'
        )
    ])