from .conversions import *
import FreeCAD
from xml.etree import ElementTree as ET


def pose_to_xml(pose, fmt='sdf'):
    '''Converts a pose/freecad placement/ to xml element
    with tag "pose" for sdf and "origin" for urdf'''
    xyz = pose.Base
    rpy = pose.Rotation.toEuler()

    if fmt == 'urdf':
        args = {'xyz': ' '.join([flt2str(i) for i in xyz]),
                'rpy': ' '.join([flt2str(deg2rad(j)) for j in rpy])}
        return ET.Element('origin', args)

    pose_elem = ET.Element('pose')
    pose_elem.text = ' '.join([flt2str(i) for i in xyz]
                              + [flt2str(deg2rad(j)) for j in rpy])

    return pose_elem

def pose_xyz(pose):
    '''Returns the xyz/Base portion of a pose as string'''
    xyz = pose.Base
    return ' '.join([flt2str(i) for i in xyz])


class SpatialEntity(object):
    '''A base class for sdf/urdf elements containing name, pose and urdf_pose'''
    def __init__(self, **kwargs):
        self.name = kwargs.get('name', '')
        self.pose = kwargs.get('pose', FreeCAD.Placement())
        self.urdf_pose = kwargs.get('urdf_pose', FreeCAD.Placement())

        self.formats = ['sdf', 'urdf']

    def to_xml(self, fmt='sdf'):
        '''Call this to check if a format is supported or not'''
        if not fmt in self.formats:
            raise Exception('Invalid export format')


class Model(SpatialEntity):
    '''A class representing a model/robot'''
    def __init__(self, **kwargs):
        super(Model, self).__init__(**kwargs)
        self.static = kwargs.get('static', False)
        self.self_collide = kwargs.get('self_collide', False)
        self.sdf_version = kwargs.get('sdf_version', 1.5)

        self.links = []
        self.joints = []

        if 'link' in kwargs:
            self.links.append(kwargs.get('link', Link()))
        self.links.extend(kwargs.get('links', []))

        if 'joint' in kwargs:
            self.joints.append(kwargs.get('joint', Joint()))
        self.joints.extend(kwargs.get('joints', []))

    def to_xml(self, fmt='sdf'):
        '''returns xml element of a model/robot'''
        super(Model, self).to_xml(fmt)
        tag = 'robot' if fmt=='urdf' else 'model'
        model = ET.Element(tag, name=self.name)

        if fmt == 'sdf':
            static = ET.SubElement(model, 'static')
            static.text = str(self.static).lower()
            self_collide = ET.SubElement(model, 'self_collide')
            self_collide.text = str(self.self_collide).lower()
        else:
            model.set('static', str(self.static).lower())

        for link in self.links:
            model.append(link.to_xml(fmt))

        for joint in self.joints:
            model.append(joint.to_xml(fmt))

        if fmt == 'sdf':
            sdf = ET.Element('sdf', version=str(self.sdf_version))
            sdf.append(model)
            return sdf

        return model


class Inertia(object):
    '''A clss representing an inertia element'''
    def __init__(self, **kwargs):
        self.ixx = kwargs.get('ixx', 0)
        self.ixy = kwargs.get('ixy', 0)
        self.ixz = kwargs.get('ixz', 0)
        self.iyy = kwargs.get('iyy', 0)
        self.iyz = kwargs.get('iyz', 0)
        self.izz = kwargs.get('izz', 0)
        if 'inertia' in kwargs:
            self.ixx, self.ixy, self.ixz = kwargs.get('inertia', [0]*6)[:3]
            self.iyy, self.iyz, self.izz = kwargs.get('inertia', [0]*6)[3:]
        self.coords = 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'

    def to_xml(self, fmt='sdf'):
        '''returns inetria xml element'''
        inertia = ET.Element('inertia')
        for coord in self.coords:
            if fmt == 'sdf':
                elem = ET.SubElement(inertia, coord)
                elem.text = flt2str(getattr(self, coord, 0))
            else:
                inertia.set(coord, flt2str(getattr(self, coord, 0)))
        return inertia


class Inertial(SpatialEntity):
    '''A class representing an inertial element'''
    def __init__(self, **kwargs):
        super(Inertial, self).__init__()
        self.mass = kwargs.get('mass', 0)
        self.inertia = kwargs.get('inertia', Inertia())

    def to_xml(self, fmt='sdf'):
        '''returns inertial xml element'''
        super(Inertial, self).to_xml(fmt)
        inertial = ET.Element('inertial')
        pose = self.pose if fmt=='sdf' else self.urdf_pose
        inertial.append(pose_to_xml(pose, fmt=fmt))
        mass = ET.SubElement(inertial, 'mass')
        if fmt == 'sdf':
            mass.text = flt2str(self.mass)
        else:
            mass.set('vaue', flt2str(self.mass))
        inertial.append(self.inertia.to_xml(fmt=fmt))

        return inertial


class Geom(SpatialEntity):
    '''A base class for collision and visual classes'''
    def __init__(self, **kwargs):
        super(Geom, self).__init__(**kwargs)
        self.mesh = kwargs.get('mesh', '')
        self.type = kwargs.get('type', 'visual')

    def to_xml(self, fmt='sdf'):
        '''returns visual or collision xml element'''
        super(Geom, self).to_xml(fmt)
        elem = ET.Element(self.type, name=self.name)
        pose = self.pose if fmt=='sdf' else self.urdf_pose
        elem.append(pose_to_xml(pose, fmt=fmt))
        geom = ET.SubElement(elem, 'geometry')
        mesh = ET.SubElement(geom, 'mesh')
        if fmt=='urdf':
            mesh.set('filename', 'package://' + self.mesh)
        else:
            uri = ET.SubElement(mesh, 'uri')
            uri.text = 'model://' + self.mesh

        return elem


class Visual(Geom):
    '''A class representing a visual element'''
    def __init__(self, **kwargs):
        super(Visual, self).__init__(type='visual', **kwargs)


class Collision(Geom):
    '''A class representing a collision element'''
    def __init__(self, **kwargs):
        super(Collision, self).__init__(type='collision', **kwargs)


class Link(SpatialEntity):
    '''A class representing a link element'''
    def __init__(self, **kwargs):
        super(Link, self).__init__(**kwargs)
        self.inertial = Inertial()
        self.visuals = []
        self.collisions = []

        if 'visual' in kwargs:
            self.visuals.append(kwargs.get('visual', Visual()))
        self.visuals.extend(kwargs.get('visuals', []))

        if 'collision' in kwargs:
            self.visuals.append(kwargs.get('collision', Collision()))
        self.collisions.extend(kwargs.get('collisions', []))

    def to_xml(self, fmt='sdf'):
        '''returns link xml element'''
        super(Link, self).to_xml(fmt)
        link = ET.Element('link', name=self.name)

        if fmt == 'sdf':
            link.append(pose_to_xml(self.pose, fmt=fmt))

        link.append(self.inertial.to_xml(fmt=fmt))

        for visual in self.visuals:
            link.append(visual.to_xml(fmt=fmt))
        for collision in self.collisions:
            link.append(collision.to_xml(fmt=fmt))

        return link


class Axis(SpatialEntity):
    '''A class representing an axis element'''
    def __init__(self, **kwargs):
        super(Axis, self).__init__(**kwargs)
        self.lower_limit = kwargs.get('lower_limit', 0)
        self.upper_limit = kwargs.get('upper_limit', 0)
        self.effort_limit = kwargs.get('effort_limit', 0)
        self.velocity_limit = kwargs.get('velocity_limit', 0)
        self.use_parent_frame = kwargs.get('use_parent_frame', False)

    def to_xml(self, fmt='sdf'):
        '''returns an axis xml element for sdf
        or an array of axis and limit xml elements for urdf'''
        super(Axis, self).to_xml(fmt)

        axis = ET.Element('axis')
        if fmt=='sdf':
            xyz = ET.SubElement(axis, 'xyz')
            xyz.text = pose_xyz(self.pose)
            limit = ET.SubElement(axis, 'limit')
            lower = ET.SubElement(limit, 'lower')
            lower.text = flt2str(deg2rad(self.lower_limit))
            upper = ET.SubElement(limit, 'upper')
            upper.text = flt2str(deg2rad(self.upper_limit))
            effort = ET.SubElement(limit, 'effort')
            effort.text = flt2str(self.effort_limit)
            velocity = ET.SubElement(limit, 'velocity')
            velocity.text = flt2str(self.velocity_limit)
            use_parent_frame = ET.SubElement(axis, 'use_parent_model_frame')
            use_parent_frame.text = str(self.use_parent_frame).lower()
        else:
            axis.set('xyz', pose_xyz(self.urdf_pose))
            axis.set('use_parent_model_frame', str(self.use_parent_frame).lower())
            limit = ET.Element('limit')
            limit.set('lower', flt2str(deg2rad(self.lower_limit)))
            limit.set('upper', flt2str(deg2rad(self.upper_limit)))
            limit.set('effort', flt2str(deg2rad(self.effort_limit)))
            limit.set('velocity', flt2str(deg2rad(self.velocity_limit)))

            return [axis, limit]
        return axis


class Joint(SpatialEntity):
    '''A class representing a joint element'''
    def __init__(self, **kwargs):
        super(Joint, self).__init__(**kwargs)
        self.parent = kwargs.get('parent', '')
        self.child = kwargs.get('child', '')
        self.type = kwargs.get('type', '')
        self.axis = kwargs.get('axis', Axis())

    def to_xml(self, fmt='sdf'):
        '''returns a joint xml element'''
        super(Joint, self).to_xml(fmt)
        joint = ET.Element('joint', {'name': self.name, 'type': self.type})
        pose = self.pose if fmt=='sdf' else self.urdf_pose
        joint.append(pose_to_xml(pose, fmt=fmt))

        parent = ET.SubElement(joint, 'parent')
        child = ET.SubElement(joint, 'child')
        if fmt == 'sdf':
            parent.text = self.parent
            child.text = self.child
            joint.append(self.axis.to_xml(fmt=fmt))
        else:
            parent.set('link', self.parent)
            child.set('Link', self.child)
            joint.extend(self.axis.to_xml(fmt=fmt))

        return joint

