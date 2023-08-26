import csv
import bpy
import bmesh

class MeshObject:
    def __init__(self):
        self.obj = None

def loadMesh(cellPath):
    vertices = []
    normals = []
    faces = []

    with open(cellPath) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if (row[0] == 'v'):
                vertex = (float(row[1]), float(row[2]), float(row[3]))
                vertices.append(vertex)
            if (row[0] == 'n'):
                normal = (float(row[1]), float(row[2]), float(row[3]))
                normals.append(normal)
            if (row[0] == 'f'):
                face = (int(row[1]), int(row[2]), int(row[3]))
                faces.append(face)

    return (vertices, normals, faces)

def createObject(cellPath):
    (vertices, normals, faces) = loadMesh(cellPath)
    
    mesh = bpy.data.meshes.new("mesh")  # add a new mesh
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    
    obj = bpy.data.objects.new("Cells", mesh)
    bpy.data.collections['CellCollection'].objects.link(obj)
    
    #print(bpy.context.collection.objects.data)

    # Get a BMesh representation
    bm = bmesh.new()
    bm.from_mesh(mesh)
    
    for f in bm.faces:
        f.smooth = True
        
    bm.to_mesh(mesh)
    bm.free()
    
    ############ Set the material
    # Get material
    mat = bpy.data.materials.get("Cell")
    if mat is None:
        # create material
        print("Couldn't find material")

    # Assign it to object
    if obj.data.materials:
        # assign to 1st material slot
        obj.data.materials[0] = mat
    else:
        # no slots
        obj.data.materials.append(mat)
    
    ############ Add subdivision modifier
    mod = obj.modifiers.new('Subsurf', "SUBSURF")
    mod.levels = 2
    
    ############ Save it in a mesh object
    meshObject = MeshObject()
    meshObject.obj = obj
    
    return meshObject

def makeKeyframe(meshObject, frame):
    ############ Animation
    meshObject.obj.keyframe_insert(data_path="location", frame=frame)

myCol = bpy.data.collections.new('CellCollection')
bpy.context.scene.collection.children.link(myCol)

cell_1 = createObject("D:/Data/Frameworks/ProjectiveDynamics/Binary/Release/Frames/cell_0_0.csv")
cell_2 = createObject("D:/Data/Frameworks/ProjectiveDynamics/Binary/Release/Frames/cell_1_0.csv")

cells = [cell_1, cell_2]

for frame in range(0, 680):
    for c in range(0, 2):
        filePath = "D:/Data/Frameworks/ProjectiveDynamics/Binary/Release/Frames/cell_" + str(c) + "_" + str(frame) + ".csv";

        (vertices, normals, faces) = loadMesh(filePath)
        
        # Replace vertices
        
        nm = bmesh.new()
        nm.to_mesh(cells[c].obj.data)
        nm.free()

        cells[c].obj.data.from_pydata(vertices, [], faces)
        
        # Make keyframe
        print(str(frame) + " " + str(c))
        print(len(vertices))
        #makeKeyframe(cell_1, frame)
        #makeKeyframe(cell_2, frame)
        
    bpy.context.scene.render.filepath = "D:/Data/Frameworks/ProjectiveDynamics/Binary/Release/Frames/out" + str(frame) + ".png"
    bpy.ops.render.render(write_still=True)
