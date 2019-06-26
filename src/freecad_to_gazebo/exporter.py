import FreeCAD, Mesh, os, numpy as np
import collada


def export(doc, exportList, filename, scale=1, quality=1, offset=np.zeros(3)):
    '''FreeCAD collada exporter
    scale - scaling factor for the mesh
    quality - mesh tessellation quality
    offset - offset of the origin of the resulting mesh'''

    colmesh = collada.Collada()
    colmesh.assetInfo.upaxis = collada.asset.UP_AXIS.Z_UP
    objind = 0
    scenenodes = []

    for obj in exportList:
        bHandled = False
        if obj.isDerivedFrom("Part::Feature"):
            bHandled = True
            m = obj.Shape.tessellate(quality)
            vindex = []
            nindex = []
            findex = []
            # vertex indices
            for v in m[0]:
                vindex.extend([a*scale+b for a, b in zip(v, offset)])
            # normals
            for f in obj.Shape.Faces:
                n = f.normalAt(0,0)
                for i in range(len(f.tessellate(quality)[1])):
                    nindex.extend([n.x,n.y,n.z])
            # face indices
            for i in range(len(m[1])):
                f = m[1][i]
                findex.extend([f[0],i,f[1],i,f[2],i])
        elif obj.isDerivedFrom("Mesh::Feature"):
            bHandled = True
            print("exporting mesh ",obj.Name, obj.Mesh)
            m = obj.Mesh
            vindex = []
            nindex = []
            findex = []
            # vertex indices
            for v in m.Topology[0]:
                vindex.extend([a*scale+b for a, b in zip(v, offset)])
            # normals
            for f in m.Facets:
                n = f.Normal
                nindex.extend([n.x,n.y,n.z])
            # face indices
            for i in range(len(m.Topology[1])):
                f = m.Topology[1][i]
                findex.extend([f[0],i,f[1],i,f[2],i])

        if bHandled:
            vert_src = collada.source.FloatSource("cubeverts-array"+str(objind),
                                                  np.array(vindex),
                                                  ('X', 'Y', 'Z'))
            normal_src = collada.source.FloatSource("cubenormals-array"+str(objind),
                                                    np.array(nindex),
                                                    ('X', 'Y', 'Z'))
            geom = collada.geometry.Geometry(colmesh,
                                             "geometry"+str(objind),
                                             obj.Label,
                                             [vert_src, normal_src])

            input_list = collada.source.InputList()
            input_list.addInput(0, 'VERTEX', "#cubeverts-array"+str(objind))
            input_list.addInput(1, 'NORMAL', "#cubenormals-array"+str(objind))
            triset = geom.createTriangleSet(np.array(findex),
                                            input_list,
                                            "materialref")
            geom.primitives.append(triset)
            colmesh.geometries.append(geom)

            geomnode = collada.scene.GeometryNode(geom)
            node = collada.scene.Node("node"+str(objind), children=[geomnode])

            #TODO: Add materials handling
            scenenodes.append(node)

        objind += 1

    scene = collada.scene.Scene("scene", scenenodes)
    colmesh.scenes.append(scene)
    colmesh.scene = scene

    colmesh.write(filename)
    print("file %s successfully created\n" % filename)

