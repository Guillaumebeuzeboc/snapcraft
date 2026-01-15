# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""The colcon plugin for ROS 2 parts.

    - colcon-packages:
      (list of strings)
      List of colcon packages to build. If not specified, all packages in the
      workspace will be built. If set to an empty list ([]), no packages will
      be built, which could be useful if you only want ROS debs in the snap.

    - colcon-packages-ignore:
      (list of strings)
      List of colcon packages to ignore. If not specified or set to an empty
      list ([]), no packages will be ignored.

    - colcon-cmake-args:
      (list of strings)
      Arguments to pass to cmake projects. Note that any arguments here which match
      colcon arguments need to be prefixed with a space. This can be done by quoting
      each argument with a leading space.

    - colcon-catkin-cmake-args:
      (list of strings)
      Arguments to pass to catkin packages. Note that any arguments here which match
      colcon arguments need to be prefixed with a space. This can be done by quoting
      each argument with a leading space.

    - colcon-ament-cmake-args:
      (list of strings)
      Arguments to pass to ament_cmake packages. Note that any arguments here which
      match colcon arguments need to be prefixed with a space. This can be done by
      quoting each argument with a leading space.

This plugin expects the build-environment `ROS_VERSION` and `ROS_DISTRO`
to be populated by the `ros2-<distro>` extension.

This plugin also expects certain variables that are specified by the extension,
specific to the ROS distro. If not using the extension, set these in your
    `build-environment`:
      - ROS_VERSION: "2"
      - ROS_DISTRO: "humble"
"""

from pathlib import Path
from typing import cast

from craft_parts.packages.snaps import _get_parsed_snap
from craft_parts.plugins import colcon_plugin
from overrides import overrides

from . import _ros


class ColconPluginProperties(colcon_plugin.ColconPluginProperties, frozen=True):
    """The part properties used by the Colcon plugin."""

    colcon_ament_cmake_args: list[str] = []
    colcon_catkin_cmake_args: list[str] = []
    colcon_ros_build_snaps: list[str] = []


class ColconPlugin(colcon_plugin.ColconPlugin, _ros.RosPlugin):
    """Plugin for the colcon build tool."""

    properties_class = ColconPluginProperties

    def _get_install_path(self) -> Path:
        return super()._get_install_path() / Path("opt/ros/snap")

    @overrides
    def get_build_snaps(self) -> set[str]:
        return colcon_plugin.ColconPlugin.get_build_snaps(
            self
        ) | _ros.RosPlugin.get_build_snaps(self)

    @overrides
    def get_build_packages(self) -> set[str]:
        base = self._part_info.base
        build_packages = {"python3-colcon-common-extensions"}
        if base == "core22":
            build_packages.update({"python3-rosinstall", "python3-wstool"})
        return (
            _ros.RosPlugin.get_build_packages(self)
            | colcon_plugin.ColconPlugin.get_build_packages(self)
            | build_packages
        )

    @overrides
    def get_build_environment(self) -> dict[str, str]:
        env = _ros.RosPlugin.get_build_environment(self)
        env.update(colcon_plugin.ColconPlugin.get_build_environment(self))

        return env

    @overrides
    def _get_source_command(self, path: str) -> list[str]:
        source_commands = colcon_plugin.ColconPlugin._get_source_command(self, path)

        source_commands.extend(
            [
                f'if [ -f "{path}/opt/ros/snap/local_setup.sh" ]; then',
                'COLCON_CURRENT_PREFIX="{wspath}" . "{wspath}/local_setup.sh"'.format(
                    wspath=f"{path}/opt/ros/snap"
                ),
                "fi",
            ]
        )
        return source_commands

    @overrides
    def _get_workspace_activation_commands(self) -> list[str]:
        """Return a list of commands source a ROS 2 workspace.

        The commands returned will be run before doing anything else.
        They will be run in a single shell instance with the rest of
        the build step, so these commands can affect the commands that
        follow.

        snapcraftctl can be used in the script to call out to snapcraft
        specific functionality.
        """

        activation_commands = []

        # Source ROS ws in all build-snaps first
        activation_commands.append("## Sourcing ROS ws in build snaps")
        self._options: ColconPluginProperties
        if self._options.colcon_ros_build_snaps:
            for ros_build_snap in self._options.colcon_ros_build_snaps:
                snap_name = _get_parsed_snap(ros_build_snap)[0]
                activation_commands.extend(
                    self._get_source_command(f"/snap/{snap_name}/current")
                )
            activation_commands.append("")

        activation_commands.extend(
            colcon_plugin.ColconPlugin._get_workspace_activation_commands(self)
        )

        return activation_commands

    @overrides
    def _get_build_commands(self) -> list[str]:
        options = cast(ColconPluginProperties, self._options)

        build_command = colcon_plugin.ColconPlugin._get_build_commands(self)

        if options.colcon_ament_cmake_args:
            build_command.extend(
                ["--ament-cmake-args", *options.colcon_ament_cmake_args]
            )

        if options.colcon_catkin_cmake_args:
            build_command.extend(
                ["--catkin-cmake-args", *options.colcon_catkin_cmake_args]
            )

        return ["## Build command", " ".join(build_command)] + [
            "## Post build command",
            # Remove the COLCON_IGNORE marker so that, at staging,
            # catkin can crawl the entire folder to look up for packages.
            'if [ -f "${CRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE ]; then',
            'rm "${CRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE',
            "fi",
        ]

    @overrides
    def get_build_commands(self) -> list[str]:
        """Return a list of commands to run during the build step."""
        return _ros.RosPlugin.get_build_commands(self)
