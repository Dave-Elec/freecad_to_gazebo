import FreeCAD
from xml.etree import ElementTree as ET
from xml.dom.minidom import parseString
import yaml
from freecad_to_gazebo.model import *
from freecad_to_gazebo.mesh_exporter import *
import a2plib
import argparse


def export_gazebo_model(assembly_file, model_dir, configs={}):
    doc = FreeCAD.open(assembly_file)

    robot_name = configs.get('name', doc.Label)
    scale = configs.get('scale', 0.001)
    scale_vec = FreeCAD.Vector([scale]*3)
    density = configs.get('density', 1000)

    export_mesh = configs.get('export', True)

    assembly_dir = os.path.split(doc.FileName)[0]

    bounding_box = FreeCAD.BoundBox()
    for obj in doc.findObjects('Part::Feature'):
        bounding_box.add(obj.Shape.BoundBox)

    bounding_box.scale(*scale_vec)

    global_pose_base = FreeCAD.Vector(bounding_box.XLength/2,
                                 bounding_box.YLength/2,
                                 bounding_box.ZLength/2)
    global_pose_base -= bounding_box.Center
    global_pose = FreeCAD.Placement()
    global_pose.Base = global_pose_base

    model = Model(name=robot_name, pose=global_pose)
    model.self_collide = configs.get('self_collide', False)
    model.sdf_version = '1.5'

    joint_limits = configs.get('joints_limits', {})
    joint_dynamics = configs.get('joints_dynamics', {})

    constraints = []
    for obj in doc.Objects:
        if a2plib.isA2pPart(obj):
            name = obj.Label
            shape = obj.Shape
            mass = shape.Mass * scale**3 * density
            com = shape.CenterOfMass * scale
            inr = shape.MatrixOfInertia
            inr.scale(*scale_vec*(scale**4) * density)
            placement = shape.Placement
            placement.Base.scale(*scale_vec)

            part_file = os.path.join(assembly_dir, obj.sourceFile)
            part_file = os.path.normpath(part_file)
            mesh_file = os.path.join(model_dir,
                                     'meshes',
                                     os.path.relpath(part_file, assembly_dir))
            mesh_file = os.path.splitext(mesh_file)[0] + '.dae'
            mesh_dir = os.path.split(mesh_file)[0]

            if export_mesh:
                os.makedirs(mesh_dir, exist_ok=True)
                export(doc, [obj], mesh_file, scale=scale, offset=com*-1)

            pose = placement.copy()
            pose.Base = com

            pose_rpy = pose.copy()
            pose_rpy.Base=(np.zeros(3))


            inertia = Inertia(inertia=np.array(inr.A)[[0,1,2,5,6,10]])
            inertial = Inertial(pose=pose_rpy,
                                mass=mass,
                                inertia=inertia)

            package = configs.get('ros_package', robot_name)
            mesh_uri = os.path.join(package,
                                    os.path.relpath(mesh_file, model_dir))
            mesh_uri = os.path.normpath(mesh_uri)

            visual = Visual(name=name+'_visual',
                            mesh=mesh_uri)
            collision = Collision(name=name+'_collision',
                                  mesh=mesh_uri)

            link = Link(name=name,
                        pose=pose,
                        inertial=inertial,
                        visual=visual,
                        collision=collision)
            model.links.append(link)

        elif a2plib.isA2pConstraint(obj):
            parent = doc.getObject(obj.Object1)
            child = doc.getObject(obj.Object2)

            if sorted([parent.Label, child.Label]) in constraints:
                continue

            if obj.Type == 'axial' and not obj.lockRotation:
                pose = a2plib.getPos(parent, obj.SubElement1)
                pose = pose - child.Shape.CenterOfMass
                pose.scale(*scale_vec)

                joint_pose = FreeCAD.Placement()
                joint_pose.Base = pose
                axis_pose = a2plib.getAxis(parent, obj.SubElement1)

                axis = Axis(pose=axis_pose,
                            lower_limit=joint_limits.get('lower', -90),
                            upper_limit=joint_limits.get('upper', 90),
                            effort_limit=joint_limits.get('effort', 10),
                            velocity_limit=joint_limits.get('velocity', 10),
                            friction=joint_dynamics.get('friction', 0),
                            damping=joint_dynamics.get('damping', 0))

                joint = Joint(name=parent.Label+'_'+child.Label,
                              pose=joint_pose,
                              parent=parent.Label,
                              child=child.Label,
                              type='revolute',
                              axis=axis)

                model.joints.append(joint)

                constraints.append(sorted([parent.Label, child.Label]))

    os.makedirs(os.path.join(model_dir, 'models'), exist_ok=True)

    with open(os.path.join(model_dir, 'models', robot_name+'.sdf'), 'w') as sdf_file:
        sdf_file.write(model.to_xml_string('sdf'))

    if not configs.get('sdf_only', None):
        with open(os.path.join(model_dir, 'models', robot_name+'.urdf'), 'w') as urdf_file:
            urdf_file.write(model.to_xml_string('urdf'))

        actuators = ET.Element('robot', name=robot_name)
        gazebo = ET.SubElement(actuators, 'gazebo')
        plugin = ET.SubElement(gazebo, 'plugin')
        plugin.set('filename', 'libgazebo_ros_control.so')
        plugin.set('name', 'gazebo_ros_control')
        namespace = ET.SubElement(plugin, 'robotNamespace')
        namespace.text = '/'+robot_name
        simtype = ET.SubElement(plugin, 'robotSimType')
        simtype.text = 'gazebo_ros_control/DefaultRobotHWSim'

        tr_configs = configs.get('transmission', {})
        jt_configs = configs.get('joints_config')
        pid = configs.get('joints_pid')

        joint_names = [joint.name for joint in model.joints]

        for joint in joint_names:
            transmission = ET.SubElement(actuators, 'transmission', name=joint)
            tr_type = ET.SubElement(transmission, 'type')
            tr_type.text = tr_configs.get('type', 'transmission_interface/SimpleTransmission')
            actuator = ET.SubElement(transmission, 'actuator', name=joint)
            hw_interface = ET.SubElement(actuator, 'hardwareInterface')
            hw_interface.text = tr_configs.get('hardware_interface', 'hardware_interface/PositionJointInterface')
            reduction = ET.SubElement(actuator, 'mechanicalReduction')
            reduction.text = '1'

            tr_joint = ET.SubElement(transmission, 'joint', name=joint)
            hw_interface = ET.SubElement(tr_joint, 'hardwareInterface')
            hw_interface.text = tr_configs.get('hardware_interface', 'hardware_interface/PositionJointInterface')

        with open(os.path.join(model_dir, 'models', robot_name+'_actuators.urdf'), 'w') as actuators_file:
            actuators_file.write(parseString(ET.tostring(actuators)).toprettyxml(indent=' '*2))

        control_configs={}
        control_configs[robot_name] = {
            'joint_state_controller':{
                'type': 'joint_state_controller/JointStateController',
                'publish_rate': 50,
            }
        }

        if jt_configs.get('groupped', False):
            for joint in joint_names:
                control_configs[robot_name][joint+'_controller'] = {
                    'type': jt_configs.get('type', 'position_controllers/JointGroupPositionController'),
                    'joint': joint,
                    'pid': pid.copy()
                }
        else:
            control_configs[robot_name]['joints_controller'] = {
                'type': jt_configs.get('type', 'position_controllers/JointGroupPositionController'),
                'publish_rate': 50,
                'joints': joint_names
            }
            control_configs[robot_name]['gazebo_ros_control/pid_gains'] = {}
            for joint in joint_names:
                control_configs[robot_name]['gazebo_ros_control/pid_gains'][joint] = pid.copy()
        os.makedirs(os.path.join(model_dir, 'config'), exist_ok=True)
        with open(os.path.join(model_dir, 'config', robot_name+'_controll.yaml'), 'w') as control_configs_file:
            yaml.dump_all([control_configs], control_configs_file, sort_keys=False)

