from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# 導入 Command
from launch.substitutions import LaunchConfiguration, Command 
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_name = 'kilin_description'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # 1. 定義 URDF 檔案路徑 (修復錯誤 1)
    # 假設您的 URDF 檔案位於 install/kilin_description/share/kilin_description/urdf/Kilin.urdf
    robot_description_path = os.path.join(
        pkg_share_dir,
        'urdf',
        'Kilin.urdf'  # <--- 確認檔案名
    )
    
    # ...
    # 2. Robot State Publisher 節點 (修復參數傳遞錯誤)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # 關鍵修復：使用 ParameterValue 聲明 Command 的輸出是一個字串 (str)
        parameters=[{'robot_description': ParameterValue(
            Command(['cat ', robot_description_path]), value_type=str
        )}]
    )
    #...

    # 3. Joint State Publisher GUI 節點 (模擬控制)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # 4. Rviz 2 節點 (可視化)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 5. Static TF 節點 (確保機器人相對於 'world' 是穩定的)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )
    
    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        static_tf_node
    ])
