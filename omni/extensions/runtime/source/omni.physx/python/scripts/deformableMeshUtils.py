# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import numpy
import typing
from omni.physx.scripts import deformableUtils, physicsUtils
from pxr import Usd, UsdGeom, Sdf, Gf, Tf, Vt, UsdPhysics, PhysxSchema

def loadTetFile(path):
    points = []
    indices = []
    
    kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")
    full_path = os.path.normpath(kit_folder + path)
    
    lines = []
    with open(full_path) as reader:
        lines = reader.readlines()
    
    flt_grp = '([-+]?\d*\.\d+|\d+)'
    v_pat = re.compile(r'^\s*[vV]\s*'+flt_grp+'\s*'+flt_grp+'\s*'+flt_grp+'\s*$')
    t_pat = re.compile(r'^\s*[tT]\s*(\d*)\s*(\d*)\s*(\d*)\s*(\d*)\s*$')

    for line in lines:
        m = v_pat.match(line)
        if m:
            point = [float(x) for x in m.groups()]
            points.append(Gf.Vec3f(point[0], point[1], point[2])*500)
        m = t_pat.match(line)
        if m:
            tet = [int(x) for x in m.groups()]
            indices.extend([tet[0], tet[1], tet[2], tet[3]])
            
    return (points, indices)

def calculateTetraVolume(a, b, c, d):
    a, b, c = a-d, b-d, c-d
    volume = (-1.0/6.0)*Gf.Dot(Gf.Cross(a, b), c)
    return volume

def fixupTetraMeshVolumes(points, indices):
    fixed_indices = []
    
    for t in range(0, len(indices)//4):
        t0, t1, t2, t3 = indices[t*4+0], indices[t*4+1], indices[t*4+2], indices[t*4+3]
        volume = calculateTetraVolume(points[t0], points[t1], points[t2], points[t3])
        if volume <= 0.0:
            fixed_indices.extend([t1, t0, t2, t3])
        else:
            fixed_indices.extend([t0, t1, t2, t3])
            
    return fixed_indices

def verifyTetraMesh(points, indices):
    
    #print("verifyTetraMesh:")
    #print("num points: " + str(len(points)))
    #print("num indices: " + str(len(indices)))
    
    if len(indices) % 4 != 0:
        carb.log_warn("verifyTetraMesh: len(indices) not multiple of 4")
        return
        
    for i in indices:
        if i >= len(points):
           carb.log_warn("verifyTetraMesh: invalid index " + str(i) + ". only " + str(len(points)) + " vertices" ) 
           return
    
    for t in range(0, len(indices)//4):
        t0, t1, t2, t3 = indices[t*4+0], indices[t*4+1], indices[t*4+2], indices[t*4+3]
        volume = calculateTetraVolume(points[t0], points[t1], points[t2], points[t3])
        if volume <= 0.0:
            carb.log_warn("verifyTetraMesh: tetra " + str(t) + " has no or negative volume " + str(volume))
            return
            
def cubeTetrahedra():
    tetra = []
    tetra.append([(0, 0, 0), (1, 0, 0), (1, 1, 0), (1, 0, 1)])
    tetra.append([(0, 0, 0), (1, 0, 1), (1, 1, 0), (0, 1, 1)])
    tetra.append([(0, 0, 0), (0, 0, 1), (1, 0, 1), (0, 1, 1)])
    tetra.append([(1, 0, 1), (1, 1, 1), (1, 1, 0), (0, 1, 1)])
    tetra.append([(0, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1)])
    return tetra


def createTetraVoxels(voxel_dim, occupancy_filter_func):
    dimx, dimy, dimz = voxel_dim, voxel_dim, voxel_dim
    
    grid = numpy.zeros((dimx, dimy, dimz), dtype='bool')
        
    #write voxel grid cell occupancy
    num_voxels = 0
    for (x, y, z), _ in numpy.ndenumerate(grid):
        if occupancy_filter_func(x, y, z, dimx, dimy, dimz):
            grid[x][y][z] = True
            num_voxels = num_voxels + 1
    
    #create vertex grid to compact list map
    grid_to_indices = numpy.full((dimx+1, dimy+1, dimz+1), -1, dtype='int32')
    
    index = 0
    for (x, y, z), _ in numpy.ndenumerate(grid_to_indices):
        #check adjacent cells
        (x_b, x_e) = (max(x-1, 0), min(x+1, dimx))
        (y_b, y_e) = (max(y-1, 0), min(y+1, dimy))
        (z_b, z_e) = (max(z-1, 0), min(z+1, dimz))
        neighbors = grid[x_b:x_e, y_b:y_e, z_b:z_e]
        if numpy.any(neighbors):
            grid_to_indices[x][y][z] = index
            index = index + 1
    
    #write points
    points = [0]*index
    for (x, y, z), index in numpy.ndenumerate(grid_to_indices):
        if index > -1:
            points[index] = Gf.Vec3f(x, y, z)
                    
    #write tetra indices
    cube_tetra = cubeTetrahedra()
    indices = [0]*num_voxels*len(cube_tetra)*4
    index = 0
    for (x, y, z), occupied in numpy.ndenumerate(grid):
        if occupied:
            mx, my, mz = x % 2, y % 2, z % 2
            flip = (mx+my+mz)%2
            for src_tet in cube_tetra:
                tet = [-1]*4
                if flip:
                    #flip tetrahedron if cube got mirrored an odd times
                    tet[0], tet[1], tet[2], tet[3] = src_tet[1], src_tet[0], src_tet[2], src_tet[3]
                else:
                    tet = src_tet
                    
                for (cx, cy, cz) in tet:
                    #mirror every other cube across all dimensions
                    wx = mx + (1-2*mx)*cx;
                    wy = my + (1-2*my)*cy;
                    wz = mz + (1-2*mz)*cz;
                    indices[index] = int(grid_to_indices[x + wx][y + wy][z + wz])
                    index = index + 1
    
    return points, indices

def voxel_sphere_test(x, y, z, dimx, dimy, dimz):
    c = Gf.Vec3f(dimx/2.0, dimy/2.0, dimz/2.0)
    r = dimx/2.0
    v = Gf.Vec3f(x + 0.5, y + 0.5, z + 0.5)
    return (v - c).GetLength() < r

def voxel_pass_all_test(x, y, z, dimx, dimy, dimz):
    return True

def createTetraVoxelBox(voxel_dim):
    points, indices = createTetraVoxels(voxel_dim, voxel_pass_all_test)
    voxel_dim_inv = 1.0 / voxel_dim
    for i in range(len(points)):
        points[i] = (points[i] * voxel_dim_inv) - Gf.Vec3f(0.5, 0.5, 0.5)
    return points, indices

def createTetraVoxelSphere(voxel_dim):
    points, indices = createTetraVoxels(voxel_dim, voxel_sphere_test)
    voxel_dim_inv = 1.0 / voxel_dim
    for i in range(len(points)):
        points[i] = (points[i] * voxel_dim_inv) - Gf.Vec3f(0.5, 0.5, 0.5)
    return points, indices

def addTriangle(points, indices, p0, p1, p2):
    o = len(points)
    indices.extend([o+0, o+1, o+2])
    points.extend([p0, p1, p2])     

def addTetra(points, indices, p0, p1, p2, p3):
    o = len(points)
    indices.extend([o+0, o+1, o+2, o+3])
    points.extend([p0, p1, p2, p3])

def convertTetraToTriangleSoup(points_in, indices_in):
    points = []
    indices = []
    for t in range(0, len(indices_in)//4):
        v0, v1, v2, v3 = indices_in[t*4+0], indices_in[t*4+1], indices_in[t*4+2], indices_in[t*4+3] 
        p0, p1, p2, p3 = points_in[v0], points_in[v1], points_in[v2], points_in[v3] 
        addTriangle(points, indices, p0, p1, p2)
        addTriangle(points, indices, p1, p3, p2)
        addTriangle(points, indices, p0, p3, p1)
        addTriangle(points, indices, p0, p2, p3)
        
    return (points, indices)
    
def explodeTriangleMesh(points_in, indices_in, factor):
    points = []
    indices = []
    for t in range(0, len(indices_in)//3):
        v0, v1, v2 = indices_in[t*3+0], indices_in[t*3+1], indices_in[t*3+2]
        p0, p1, p2 = points_in[v0], points_in[v1], points_in[v2]
        c = (p0 + p1 + p2)*(1.0/3.0)
        pr0, pr1, pr2 = p0-c, p1-c, p2-c
        c = c * factor
        p0, p1, p2 = c+pr0, c+pr1, c+pr2
        addTriangle(points, indices, p0, p1, p2)
    return (points, indices)

def explodeTetraMesh(points_in, indices_in, factor):
    points = []
    indices = []
    for t in range(0, len(indices_in)//4):
        v0, v1, v2, v3 = indices_in[t*4+0], indices_in[t*4+1], indices_in[t*4+2], indices_in[t*4+3]
        p0, p1, p2, p3 = points_in[v0], points_in[v1], points_in[v2], points_in[v3]
        c = (p0 + p1 + p2 + p3)*(1.0/4.0)
        pr0, pr1, pr2, pr3 = p0-c, p1-c, p2-c, p3-c
        c = c * factor
        p0, p1, p2, p3 = c+pr0, c+pr1, c+pr2, c+pr3
        addTetra(points, indices, p0, p1, p2, p3)
    return (points, indices)

def createTriangleMeshCube(dim: int):
    points, indices = createTetraVoxelBox(dim)
    tri_points, tri_indices = deformableUtils.extractTriangleSurfaceFromTetra(points, indices)
    return tri_points, tri_indices
