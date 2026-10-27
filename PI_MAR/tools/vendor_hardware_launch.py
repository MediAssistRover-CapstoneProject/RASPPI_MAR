"""Generate an isolated vendor launch overlay; never edit Hiwonder's installation."""
import ast
from pathlib import Path


def replace_include(source, variable, leaf, replacement):
    tree=ast.parse(source); matches=[]
    class Replace(ast.NodeTransformer):
        def visit_Call(self,node):
            self.generic_visit(node)
            if (isinstance(node.func,ast.Attribute) and node.func.attr=='join' and len(node.args)==2
                    and isinstance(node.args[0],ast.Name) and node.args[0].id==variable
                    and isinstance(node.args[1],ast.Constant) and node.args[1].value==leaf):
                matches.append(node)
                return ast.copy_location(ast.Constant(str(replacement)),node)
            return node
    tree=Replace().visit(tree)
    if len(matches)!=1:raise RuntimeError('VENDOR_LAUNCH_LAYOUT_CHANGED: '+leaf+'; no vendor file edited')
    return '# Generated MAR overlay; original vendor source preserved.\n'+ast.unparse(ast.fix_missing_locations(tree))+'\n'


def prepare(root, robot_launch, controller_launch):
    root,robot_launch,controller_launch=map(Path,(root,robot_launch,controller_launch))
    folder=root/'pi/state/stack/vendor_overlay'; folder.mkdir(parents=True,exist_ok=True)
    controller=replace_include(controller_launch.read_text(), 'peripherals_package_path',
        'launch/imu_filter.launch.py',root/'pi/launch/mediassist_imu.launch.py')
    robot=replace_include(robot_launch.read_text(),'controller_package_path',
        'launch/controller.launch.py',folder/'controller.launch.py')
    # Validate both sources before writing either overlay.
    for name,code in (('controller.launch.py',controller),('robot.launch.py',robot)):
        compile(code,name,'exec'); (folder/name).write_text(code,encoding='utf-8')
    return folder/'robot.launch.py'


def include(context,root):
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration
    original=LaunchConfiguration('vendor_robot_launch').perform(context)
    controller=(Path(get_package_share_directory('controller')) if os.environ.get('need_compile')=='True'
                else Path('/home/ubuntu/ros2_ws/src/driver/controller'))/'launch/controller.launch.py'
    overlay=prepare(root,original,controller)
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(overlay)),launch_arguments={
        'sim':'false','robot_name':'/','master_name':'/','use_joy':'false'}.items())]
