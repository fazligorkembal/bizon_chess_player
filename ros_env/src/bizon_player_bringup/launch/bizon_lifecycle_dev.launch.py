import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.actions import PushRosNamespace

from collections.abc import Generator
import tempfile
from typing import Optional, TypeAlias, Union

import launch
import yaml

class Colors:
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'

YamlValue: TypeAlias = Union[str, int, float, bool]


class DictItemReference:

    def __init__(self, dictionary: dict[str, YamlValue], key: str):
        self.dictionary = dictionary
        self.dictKey = key

    def key(self) -> str:
        return self.dictKey

    def setValue(self, value: YamlValue) -> None:
        self.dictionary[self.dictKey] = value


class RewrittenYaml(launch.Substitution):
    """
    Substitution that modifies the given YAML file.

    Used in launch system
    """

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: dict[str, launch.SomeSubstitutionsType],
        root_key: Optional[launch.SomeSubstitutionsType] = None,
        key_rewrites: Optional[dict[str, launch.SomeSubstitutionsType]] = None,
        value_rewrites: Optional[dict[str, launch.SomeSubstitutionsType]] = None,
        convert_types: bool = False,
    ) -> None:
        super().__init__()
        """
        Construct the substitution

        :param: source_file the original YAML file to modify
        :param: param_rewrites mappings to replace
        :param: root_key if provided, the contents are placed under this key
        :param: key_rewrites keys of mappings to replace
        :param: value_rewrites values to replace
        :param: convert_types whether to attempt converting the string to a number or boolean
        """

        # import here to avoid loop
        from launch.utilities import normalize_to_list_of_substitutions

        self.__source_file: list[launch.Substitution] = \
            normalize_to_list_of_substitutions(source_file)
        self.__param_rewrites = {}
        self.__key_rewrites = {}
        self.__value_rewrites = {}
        self.__convert_types = convert_types
        self.__root_key = None
        for key in param_rewrites:
            self.__param_rewrites[key] = normalize_to_list_of_substitutions(
                param_rewrites[key]
            )
        if key_rewrites is not None:
            for key in key_rewrites:
                self.__key_rewrites[key] = normalize_to_list_of_substitutions(
                    key_rewrites[key]
                )
        if value_rewrites is not None:
            for value in value_rewrites:
                self.__value_rewrites[value] = normalize_to_list_of_substitutions(
                    value_rewrites[value]
                )
        if root_key is not None:
            self.__root_key = normalize_to_list_of_substitutions(root_key)

    @property
    def name(self) -> list[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> str:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
        param_rewrites, keys_rewrites, value_rewrites = self.resolve_rewrites(context)

        with open(yaml_filename, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)

        self.substitute_params(data, param_rewrites)
        self.add_params(data, param_rewrites)
        self.substitute_keys(data, keys_rewrites)
        self.substitute_values(data, value_rewrites)
        if self.__root_key is not None:
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context: launch.LaunchContext) -> \
            tuple[dict[str, str], dict[str, str], dict[str, str]]:
        resolved_params = {}
        for key in self.__param_rewrites:
            resolved_params[key] = launch.utilities.perform_substitutions(
                context, self.__param_rewrites[key]
            )
        resolved_keys = {}
        for key in self.__key_rewrites:
            resolved_keys[key] = launch.utilities.perform_substitutions(
                context, self.__key_rewrites[key]
            )
        resolved_values = {}
        for value in self.__value_rewrites:
            resolved_values[value] = launch.utilities.perform_substitutions(
                context, self.__value_rewrites[value]
            )
        return resolved_params, resolved_keys, resolved_values

    def substitute_params(self, yaml: dict[str, YamlValue],
                          param_rewrites: dict[str, str]) -> None:
        # substitute leaf-only parameters
        for key in self.getYamlLeafKeys(yaml):
            if key.key() in param_rewrites:
                raw_value = param_rewrites[key.key()]
                key.setValue(self.convert(raw_value))

        # substitute total path parameters
        yaml_paths = self.pathify(yaml)
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                rewrite_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                yaml = self.updateYamlPathVals(yaml, yaml_keys, rewrite_val)

    def add_params(self, yaml: dict[str, YamlValue],
                   param_rewrites: dict[str, str]) -> None:
        # add new total path parameters
        yaml_paths = self.pathify(yaml)
        for path in param_rewrites:
            if not path in yaml_paths:  # noqa: E713
                new_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                if 'ros__parameters' in yaml_keys:
                    yaml = self.updateYamlPathVals(yaml, yaml_keys, new_val)

    def substitute_values(
            self, yaml: dict[str, YamlValue],
            value_rewrites: dict[str, str]) -> None:

        def process_value(value: YamlValue) -> YamlValue:
            if isinstance(value, dict):
                for k, v in list(value.items()):
                    value[k] = process_value(v)
                return value
            elif isinstance(value, list):
                return [process_value(v) for v in value]
            elif str(value) in value_rewrites:
                return self.convert(value_rewrites[str(value)])
            return value

        for key in list(yaml.keys()):
            yaml[key] = process_value(yaml[key])

    def updateYamlPathVals(
            self, yaml: dict[str, YamlValue],
            yaml_key_list: list[str], rewrite_val: YamlValue) -> dict[str, YamlValue]:

        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                yaml[key] = rewrite_val
                break
            key = yaml_key_list.pop(0)
            if isinstance(yaml, list):
                yaml[int(key)] = self.updateYamlPathVals(
                    yaml[int(key)], yaml_key_list, rewrite_val
                )
            else:
                yaml[key] = self.updateYamlPathVals(  # type: ignore[assignment]
                    yaml.get(key, {}),  # type: ignore[arg-type]
                    yaml_key_list,
                    rewrite_val
                )
        return yaml

    def substitute_keys(
            self, yaml: dict[str, YamlValue], key_rewrites: dict[str, str]) -> None:
        if len(key_rewrites) != 0:
            for key in list(yaml.keys()):
                val = yaml[key]
                if key in key_rewrites:
                    new_key = key_rewrites[key]
                    yaml[new_key] = yaml[key]
                    del yaml[key]
                if isinstance(val, dict):
                    self.substitute_keys(val, key_rewrites)

    def getYamlLeafKeys(self, yamlData: dict[str, YamlValue]) -> \
            Generator[DictItemReference, None, None]:
        if not isinstance(yamlData, dict):
            return

        for key in yamlData.keys():
            child = yamlData[key]

            if isinstance(child, dict):
                # Recursively process nested dictionaries
                yield from self.getYamlLeafKeys(child)

            yield DictItemReference(yamlData, key)

    def pathify(
            self, d: Union[dict[str, YamlValue], list[YamlValue], YamlValue],
            p: Optional[str] = None,
            paths: Optional[dict[str, YamlValue]] = None,
            joinchar: str = '.') -> dict[str, YamlValue]:
        if p is None:
            paths = {}
            self.pathify(d, '', paths, joinchar=joinchar)
            return paths

        assert paths is not None
        pn = p
        if p != '':
            pn += joinchar
        if isinstance(d, dict):
            for k in d:
                v = d[k]
                self.pathify(v, str(pn) + str(k), paths, joinchar=joinchar)
        elif isinstance(d, list):
            for idx, e in enumerate(d):
                self.pathify(e, pn + str(idx), paths, joinchar=joinchar)
        else:
            paths[p] = d
        return paths

    def convert(self, text_value: str) -> YamlValue:
        if self.__convert_types:
            # try converting to int or float
            try:
                return float(text_value) if '.' in text_value else int(text_value)
            except ValueError:
                pass

        # try converting to bool
        if text_value.lower() == 'true':
            return True
        if text_value.lower() == 'false':
            return False

        # nothing else worked so fall through and return text
        return text_value


def launch_setup(context, *args, **kwargs):
    #bringup_dir = get_package_share_directory('nav2_bringup')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    #container_name = LaunchConfiguration('container_name')
    #container_name_full = (namespace, '/', container_name)
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')
    debug_session_dir = LaunchConfiguration('debug_session_dir').perform(context)

    lifecycle_nodes = [
        'behavior_server',
    ]

    # 'behavior_server.ros__parameters.debug_session_dir' is a dotted path
    # into the *unwrapped* params_file (root_key namespacing happens after
    # this substitution runs -- see RewrittenYaml.perform() below), so
    # add_params() adds it under behavior_server's ros__parameters the same
    # way autostart is substituted in below it, rather than at the
    # yaml's top level where BoardPlugin/DecisionPlugin would never see it.
    param_substitutions = {
        'autostart': autostart,
        'behavior_server.ros__parameters.debug_session_dir': debug_session_dir,
    }

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    #print(Colors.CYAN + 'Launching with container: ' + container_name.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with namespace: ' + namespace.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with log_level: ' + log_level.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with use_sim_time: ' + use_sim_time.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with autostart: ' + autostart.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with use_composition: ' + use_composition.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with lifecycle_nodes: ' + str(lifecycle_nodes) + Colors.END)
    print(Colors.CYAN + 'Launching with remappings: ' + str(remappings) + Colors.END)
    print(Colors.CYAN + 'Launching with stdout_linebuf_envvar: ' + str(stdout_linebuf_envvar) + Colors.END)
    #print(Colors.CYAN + 'Launching with container_name_full: ' + str(container_name_full) + Colors.END)
    print(Colors.CYAN + 'Launching with param_file: ' + params_file.perform(context) + Colors.END)

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )
    

    load_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            PushRosNamespace(namespace),
            Node(
                package="bizon_behavior_servers",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[configured_params],
                emulate_tty=True,
            ),
            Node(
                package='bizon_lifecycle_manager',
                executable='lifecycle_manager_node',
                name='lifecycle_manager',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
                emulate_tty=True,
            ),
            # Standalone node (not lifecycle-managed -- see
            # rosout_logger_main.cpp's header comment), started alongside
            # behavior_server so MoveIt/OMPL/controller WARN-and-above
            # output lands in the same debug/ folder as everything else
            # instead of only existing on a console nobody is reading.
            Node(
                package="bizon_behavior_servers",
                executable="rosout_logger",
                name="rosout_logger",
                output="screen",
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'debug_session_dir': debug_session_dir,
                    'namespace_prefix': namespace.perform(context),
                }],
                emulate_tty=True,
            ),
        ]
    )

    return [
        load_nodes,
    ]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='bizon1', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    #declare_container_name_cmd = DeclareLaunchArgument(
    #    'container_name',
    #    default_value='bizon_container',
    #    description='the name of conatiner that nodes will load in if use composition',
    #)

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_debug_session_dir_cmd = DeclareLaunchArgument(
        'debug_session_dir',
        default_value='',
        description='Absolute path of this game\'s debug session directory, resolved by '
                    'bizon_player.launch.py; empty disables debug logging',
    )


    bringup_dir = get_package_share_directory('bizon_player_bringup')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'bizon_behavior_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    return LaunchDescription(
        [
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_autostart_cmd,
            declare_use_composition_cmd,
            #declare_container_name_cmd,
            declare_log_level_cmd,
            declare_params_file_cmd,
            declare_debug_session_dir_cmd,
        ] + [OpaqueFunction(function=launch_setup)]
    )
    