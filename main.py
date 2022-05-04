import numpy as np
from skimage import measure
import open3d as o3d



### Segment 1: load TSDF data from `.sdf` file
filepath = '/data/graspnet_dataset/models/004/textured.sdf'

with open(filepath, 'r') as my_file:
    # dimension of each axis
    nx, ny, nz = [int(i) for i in my_file.readline().split()]
    # origin
    ox, oy, oz = [float(i) for i in my_file.readline().split()]
    dims = np.array([nx, ny, nz])
    origin = np.array([ox, oy, oz])

    # resolution
    resolution = float(my_file.readline())
    sdf_data = np.zeros(dims)

    # loop through file, getting each value
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                sdf_data[i][j][k] = float(my_file.readline())



### Segment 2: generate point cloud from TSDF

def get_o3d_point_cloud_from_tsdf(tsdf_vol, vol_origin, voxel_size):
    """Extract a point cloud from the TSDF voxel volume.
    """
    verts = measure.marching_cubes_lewiner(tsdf_vol, level=0)[0]
    vert_points = verts * voxel_size + vol_origin

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vert_points)

    return pcd


pcd_from_tsdf = get_o3d_point_cloud_from_tsdf(tsdf_vol=sdf_data,
                                              vol_origin=origin,
                                              voxel_size=resolution)

print('visualize pcd generated from tsdf')
o3d.visualization.draw_geometries([pcd_from_tsdf])

print('visualize pcd generated from tsdf and raw pcd from file')
raw_pcd = o3d.io.read_point_cloud('/data/graspnet_dataset/models/004/nontextured.ply')
o3d.visualization.draw_geometries([pcd_from_tsdf, raw_pcd])




### Segment 3: generate TriangleMesh from TSDF

def get_o3d_mesh_from_tsdf(tsdf_vol, vol_origin, voxel_size):
    """Extract a triangle mesh from the TSDF voxel volume.
    """
    verts, faces, norms, _ = measure.marching_cubes_lewiner(tsdf_vol, level=0)
    vert_points = verts * voxel_size + vol_origin

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vert_points)
    print(faces.shape)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.vertex_normals = o3d.utility.Vector3dVector(norms)

    return mesh


mesh_from_tsdf = get_o3d_mesh_from_tsdf(tsdf_vol=sdf_data,
                                        vol_origin=origin,
                                        voxel_size=resolution)

print('visualize mesh generated from tsdf')
o3d.visualization.draw_geometries([mesh_from_tsdf])

print('visualize mesh generated from tsdf and raw mesh from file')
raw_mesh = o3d.io.read_triangle_mesh('/data/graspnet_dataset/models/004/textured.obj')
o3d.visualization.draw_geometries([mesh_from_tsdf, raw_mesh])