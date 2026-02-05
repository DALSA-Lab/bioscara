# Building the ROS2 Workspace
After importing all dependencies the ROS2 workspace *lib/ros2_ws/* will contain a lot of packages, primarily MoveIt and MTC packages along the usual Bioscara packages.
In order to be able to run them, they all have to be built. `colcon` is the ROS2 building tool that is used for this purpose.

:::{note}
 This guide assumes that you are in the *lib/ros2_ws/* workspace.
 If you are not, navigate to it:
 ```bash
    cd lib/ros2_ws/
 ```
:::

## Building the Entire Workspace including MoveIt2 and MTC
Both MoveIt2 and the MTC are huge packages that will make the Raspberry Pi crash under the load. Even for normal computers there is a high chance of freezing.
For this reason the `colcon build` command is invoked with very restrictive parameters to allow the compilation to finish at all.
The build command that should be used to compile the entire workspace at once is:
```bash
MAKEFLAGS="-j1 -l1" colcon build --mixin release --executor sequential --symlink-install --continue-on-error > log.out &
```
```bash
disown
```
The `MAKEFLAGS="-j1 -l1"` restrict the process to a single core, the `--executor sequential` flag ensurs that only one package at the time is built (otherwise up to 8 would be built), the `--continue-on-error` flag will try to keep the compilation going if one package fails. This is very important since the compilation can take up to 10 (!) hours on the Raspberry Pi 4B with 4GB memory. 

The high memory usage will cause the Raspberry Pi to become unresponsive at times, potentially dropping the SSH connection and thus the user session that started the build proces. 
To avoid this the `colcon` output is being piped `> log.out` to a file and the process is being sent to the background `&`. Before disconnecting from the session the process must be disowned by calling `disown`. The build job will now continue even if the user session that started it terminates. 

## Building selected Packages
Luckil as long as you dont modify any of the MoveIt or MTC packages or accidentally delete the *build/* or *install/* directories, you will not have to build the entire workspace again.
To rebuild a selected set of packages simply invoke `colcon` like this:
```bash
colcon build --mixin release --symlink-install --packages-select <package_name> <package_name> <package_name>
```
or select the by a regex expression `--packages-select-regex <package_name_regex>` or any other (selection flag)[https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html]

## Other Notes
Using the `--symlink-install` flag create symlinks instead of copying files from the source and build directories where possible. This way it is not necessary to recompile a package if a non-C++ file has been modified. Changing parameter yaml-files, Pthyon scripts or URDF files is thus simpler and faster. 

The `--mixin release` makes the packages compile in release mode, which is very important for performance reasons. Not using this flag will make path planning very slow.

If compilation fails, in particular if a package's *CMakeLists.txt* has been modified, it can help to selectively clean the *build/* and *install/* directory:
```bash
rm -rf build/<package_name> install/<package_name>
```

