import numpy as np
import open3d as o3d
from PIL import Image

def depth_to_point_cloud(depth_map):
    fx = 525.0  # focal length x
    fy = 525.0  # focal length y
    cx = 319.5  # optical center x
    cy = 239.5  # optical center y

    factor = 5000  # for the 16-bit PNG files
    # OR: factor = 1 # for the 32-bit float images in the ROS bag files
    points = []
    for v in range(0,480):
        for u in range(0,640):
            Z = depth_map[v, u] / factor
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            points.append([X, Y, Z])
    return np.array(points)

lines = []
with open('C:/Users/ye/Desktop/TEST/rgbd_dataset_freiburg1_xyz/depth.txt', 'r') as file:
    for line in file:
        lines.append(line.split('/')[1].strip())
for name in lines:

    depth_path = "C:/Users/ye/Desktop/TEST/rgbd_dataset_freiburg1_xyz/depth/"+name
    depth_map = Image.open(depth_path)  # 替换成自己的路径
    depth_map = np.array(depth_map)

    fx = 517.3
    fy = 516.5
    cx = 318.6
    cy = 255.3

    points = depth_to_point_cloud(depth_map)
    # print(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud('C:/Users/ye/Desktop/TEST/rgbd_dataset_freiburg1_xyz/o3d/'+name[:-4]+'.ply', pcd)
    # o3d.io.write_point_cloud("./output.pcd", pcd)