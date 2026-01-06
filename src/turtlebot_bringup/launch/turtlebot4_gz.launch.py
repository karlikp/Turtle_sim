from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PythonExpression
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # Twoja paczka z zasobami (worlds/models)
    sim_share = get_package_share_path("turtlebot_sim")
    worlds_path = str(sim_share / "worlds")
    models_path = str(sim_share / "models")
    default_world = str(sim_share / "worlds" / "sim_world_1.sdf")

    # TurtleBot4 description (zainstalowane z apt / source)
    tb4_desc_share = get_package_share_path("turtlebot4_description")
    default_xacro = str(tb4_desc_share / "urdf" / "turtlebot4.urdf.xacro")

    # --- Launch args ---
    name = DeclareLaunchArgument("name", default_value="turtlebot4")
    x = DeclareLaunchArgument("x", default_value="0.0")
    y = DeclareLaunchArgument("y", default_value="0.0")
    z = DeclareLaunchArgument("z", default_value="0.05")
    yaw = DeclareLaunchArgument("yaw", default_value="0.0")

    # world: domyślnie Twój świat z turtlebot_sim/worlds
    world = DeclareLaunchArgument("world", default_value=default_world)

    # wariant robota (jeśli Twoje xacro tego nie obsługuje, usuń model:=... poniżej)
    model = DeclareLaunchArgument("model", default_value="standard")

    name_lc = LaunchConfiguration("name")
    x_lc = LaunchConfiguration("x")
    y_lc = LaunchConfiguration("y")
    z_lc = LaunchConfiguration("z")
    yaw_lc = LaunchConfiguration("yaw")
    world_lc = LaunchConfiguration("world")
    model_lc = LaunchConfiguration("model")

     # Wybór ścieżki xacro zależnie od model:=standard|lite
    xacro_file = PythonExpression([
        "'",
        str(tb4_desc_share),
        "/urdf/' + ('standard' if '",
        model_lc,
        "' == 'standard' else 'lite') + '/turtlebot4.urdf.xacro'"
    ])

    robot_description = Command(["xacro ", xacro_file])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # --- Gazebo Fortress ---
    gz_sim = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "4", world_lc],
        output="screen",
)

    # --- Spawn TB4 from robot_description ---
    spawn_tb4 = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-name", name_lc,
            "-x", x_lc, "-y", y_lc, "-z", z_lc, "-Y", yaw_lc,
            "-topic", "robot_description",
        ],
        output="screen",
    )

    # --- Bridges (minimalne) ---
    cmd_vel_map = [
        TextSubstitution(text="/model/"), name_lc,
        TextSubstitution(text="/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"),
    ]
    odom_map = [
        TextSubstitution(text="/model/"), name_lc,
        TextSubstitution(text="/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"),
    ]

    bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            cmd_vel_map,
            odom_map,
        ],
        output="screen",
    )

    return LaunchDescription([
        name, x, y, z, yaw, world, model,

        SetParameter(name="use_sim_time", value=True),

        # Ważne: to muszą być REALNE akcje w LaunchDescription (bez przecinka robiącego tuple)
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", worlds_path),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models_path),

        rsp,
        gz_sim,
        spawn_tb4,
        bridge,
    ])
