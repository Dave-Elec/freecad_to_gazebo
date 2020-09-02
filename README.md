# Freecad to Gazebo exporter

## Introduction
Freecad to gazebo exporter is a tool for exporting freecad assembly project to gazebo/ros model.

## Design Rules
This project is in its early stage therefore specific design rules must be followed to work with it.
* The assmbly file must be created with `A2Plus` (freecad's prefered assembly workspace).
* Parts of the assembly must be in separate files.
* Imovable parts of the main assembly must be made in separate subasssemblies.
* Joints should be represented by AxisConsident constraints with lock rotation turned off.
* For URDF files to work properly, tree structure must be maintained (ie. parent and childs of constraints must follow tree structure).

## Requirements
* [Freecad][freecad] with [A2Plus][a2plus] workspace installed
* [ROS][ros] *Optional*, If you wish to install it as a package

## Usage

#### Clone the repository to catkin workspace
```console
$ git clone https://github.com/Dave-Elec/freecad_to_gazebo.git ~/catkin_ws/src
```

#### Re-build catkin workspace
```console
$ catkin_make
$ source devel/setup.bash
```

#### To generate only SDF model from freecad assembly:
```console
# freecad2gazebo <assembly_file> <path/to/model_dir> --sdf-only
```

#### To generate SDF and URDF model from freecad assembly:
```console
$ freecad2gazebo <assembly_file> <path/to/model_dir>
```
Note: Only links and joints are generated in the SDF model. to use the model with ros, use the URDF model.

## Options
```console
$ freecad2gazebo <assembly_file> <path/to/model> [--sdf-only] [--noexport] [--config <path/to/config>]
```

**--sdf**: Export only SDF.

**--noexport**: Don't export mesh files.

**--config**: Use other configuration file. (default is `robot_config.json` inside a directory the same as the assembly file).


## Example config file
```json
{
    "name": "robot_name",
    "joints_limits": {"upper": 90, "lower": -90, "effort": 10, "velocity": 5},
    "transmission": {
        "type": "transmission_interface/SimpleTransmission",
        "hardware_interface": "hardware_interface/PositionJointInterface"
    },
    "joints_config": {
        "type": "position_controllers/JointGroupPositionController",
        "grouped": true
    },
    "joints_pid": {"p": 20.0, "i": 10.0, "d": 0.0, "i_clamp": 0.0},
    "root_link": "base_link",
    "ros_package": "humanoid_17dof_description"
}
```

## Future plans
* Extend collada exporter to export materials from assemblies.
* Create a FreeCAD workbench to interactively assign joints and export to gazebo.
* Reduce set of design rules by making it more robust and general perpouse.
* Support any valid structures of assemblies.

[freecad]:https://freecadweb.org
[a2plus]:https://github.com/kbwbe/A2plus
[ros]:https://ros.org
