
from cmath import nan
import rosbag
import numpy as np
import os
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

def rosbag_to_txt():
    """直接从rosbag包读取点云保存为txt文件
    """
    bag_file = './test.bag'
    bag = rosbag.Bag(bag_file, "r")
    info = bag.get_type_and_topic_info()
    print(info)

    bag_data = bag.read_messages('/fusion_all_points')
    for topic, msg, t in bag_data:
        lidar = pc2.read_points(msg)
        points = np.array(list(lidar))
        f = open("testpoint.txt", "w")
        cnt = 0
        for p1 in points:
            s = str(p1[0]) + ',' + str(p1[1]) + ',' + str(p1[2])
            f.write(s + '\n')
            cnt += 1
        print(cnt)
        f.close()
        #break

def csv_to_bin():
    # 将 csv格式转为bin格式
    # 反射强度需要数据归一化，这里简化直接置为0
    import pandas as pd
    import numpy as np
    import open3d as o3d
    points = pd.read_csv('/home/t/t/csv/50.csv')
    points = np.array(points)
    points[:, 3] = 0
    pcd_vector = o3d.geometry.PointCloud()
    # 加载点坐标
    pcd_vector.points = o3d.utility.Vector3dVector(points[:, :3])
    o3d.visualization.draw_geometries([pcd_vector])
    points.astype(np.float32).tofile('test_16.bin')
    print(points)

def txt_to_bin():
    from ctypes import pointer
    import open3d as o3d
    import numpy as np

    txt_path = './testpoint.txt'
    # 通过numpy读取txt点云
    pcd = np.genfromtxt(txt_path, delimiter=",")

    pcd_vector = o3d.geometry.PointCloud()
    # 加载点坐标
    pcd_vector.points = o3d.utility.Vector3dVector(pcd[:, :3])
    o3d.visualization.draw_geometries([pcd_vector])
    point = np.asanyarray(pcd_vector.points)
    size_point = len(point)
    point = np.column_stack((point, np.arange(1, size_point + 1)))
    point[:, 3] = 0
    point.astype(np.float32).tofile('test.bin')
    #print(point)
    np.save("test.npy", point)

def txt_to_pcd1():
    from ctypes import pointer
    import open3d as o3d
    import numpy as np

    txt_path = './testpoint.txt'
    # 通过numpy读取txt点云
    pcd = np.genfromtxt(txt_path, delimiter=",")

    pcd_vector = o3d.geometry.PointCloud()
    # 加载点坐标
    pcd_vector.points = o3d.utility.Vector3dVector(pcd[:, :3])
    #o3d.visualization.draw_geometries([pcd_vector]) #是否可视化
    point = np.asanyarray(pcd_vector.points)
    size_point = len(point)
    point = np.column_stack((point, np.arange(1, size_point + 1)))
    point[:, 3] = 0

    print(point.shape)
    savefilename = "./testpoint_txt2pcd1.pcd"
    if not os.path.exists(savefilename):
        f = open(savefilename, 'w')
        f.close()
    with open(savefilename, 'w') as file_to_write:
        file_to_write.writelines("# .PCD v0.7 - Point Cloud Data file format\n")
        file_to_write.writelines("VERSION 0.7\n")
        file_to_write.writelines("FIELDS x y z\n")
        file_to_write.writelines("SIZE 4 4 4\n")
        file_to_write.writelines("TYPE F F F\n")
        file_to_write.writelines("COUNT 1 1 1\n")
        file_to_write.writelines("WIDTH " + str(point.shape[0]) + "\n")
        file_to_write.writelines("HEIGHT 1\n")
        file_to_write.writelines("VIEWPOINT 0 0 0 1 0 0 0\n")
        file_to_write.writelines("POINTS " + str(point.shape[0]) + "\n")
        file_to_write.writelines("DATA ascii\n")
        #file_to_write.writelines("data ascii\n")
        for line in point:
            #file_to_write.writelines(str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + "\n")
            file_to_write.writelines(str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + " " + str(line[3])+ "\n")

def txt_to_pcd2():
    """已通过测试"""
    import math
    filename="./testpoint.txt"  #我的txt文件中没有点云的回波强度。只有点的x,y,z坐标
    xlist = []
    ylist = []
    zlist = []
    with open(filename, 'r') as file_to_read:
        while True:
            lines = file_to_read.readline()
            if not lines:
                break
                pass
            #x, y, z = [math.fabs(float(i)) for i in lines.split(',')]
            x, y, z = [float(i) for i in lines.split(',')]
            xlist.append(x)
            ylist.append(y)
            zlist.append(z)

    savefilename = "./testpoint.pcd"
    if not os.path.exists(savefilename):
        f = open(savefilename, 'w')
        f.close()
    with open(savefilename, 'w') as file_to_write:
        file_to_write.writelines("# .PCD v0.7 - Point Cloud Data file format\n")
        file_to_write.writelines("VERSION 0.7\n")
        file_to_write.writelines("FIELDS x y z\n")
        file_to_write.writelines("SIZE 4 4 4\n")
        file_to_write.writelines("TYPE F F F\n")
        file_to_write.writelines("COUNT 1 1 1\n")
        file_to_write.writelines("WIDTH " + str(len(xlist)) + "\n")
        file_to_write.writelines("HEIGHT 1\n")
        file_to_write.writelines("VIEWPOINT 0 0 0 1 0 0 0\n")
        file_to_write.writelines("POINTS " + str(len(xlist)) + "\n")
        file_to_write.writelines("DATA ascii\n")
        #file_to_write.writelines("data ascii\n")
        for i in range(len(xlist)):
            file_to_write.writelines(str(xlist[i]) + " " + str(ylist[i]) + " " + str(zlist[i]) + "\n")

def bin_to_txt():
    #bin_file="./kitti_0.bin"
    #txt_file="./kitti_0_bin2txt.txt"

    bin_file="./testpoint-pcd2bin.bin"
    txt_file="./testpoint-pcd2bin-bin2txt.txt"

    b = np.fromfile(bin_file, dtype=np.float64)
    print(b.shape)
    a = b.reshape(-1, 4) # 点云格式X,Y,Z,i
    #a = b.reshape(-1, 3) # 点云格式X,Y,Z

    fp = open(txt_file, 'w+')
    for x in a:
        fp.write('{},{},{},{}\n'.format(x[0], x[1], x[2], x[3]))
        #fp.write('{},{},{}\n'.format(x[0], x[1], x[2]))
    # print(a.shape)
    fp.close()

def pcd_to_txt():
    import open3d as o3d
    import numpy as np
    pcd_file = './testpoint.pcd'
    txt_file = './testpoint_pcd2txt.txt'
    pcd = o3d.io.read_point_cloud( pcd_file)  # Open3d读取到的点云通常存储到PointCloud类中,这个类中我们常用的属性就是points和colors
    points = np.asarray(pcd.points)
    lidar = []
    for linestr in points:
        if len(linestr) == 3:  # only x,y,z
            linestr_convert = list(map(float, linestr))
            linestr_convert.append(0)
            lidar.append(linestr_convert)
        if len(linestr) == 4:  # x,y,z,i
            linestr_convert = list(map(float, linestr))
            lidar.append(linestr_convert)
    data=np.array(lidar)
    print(data.shape)

    fp = open(txt_file, 'w+')
    for x in data:
        fp.write('{},{},{},{}\n'.format(x[0], x[1], x[2], x[3]))
        #fp.write('{},{},{}\n'.format(x[0], x[1], x[2]))
    # print(a.shape)
    fp.close()

    #data.tofile(txt_file)

def read_pcd(filepath):
    lidar = []
    pcd = o3d.io.read_point_cloud(filepath, format='pcd')
    points = np.array(pcd.points)
    for linestr in points:
        if len(linestr) == 3:  # only x,y,z
            print("x,y,z")
            linestr_convert = list(map(float, linestr))
            linestr_convert.append(0)
            lidar.append(linestr_convert)
        if len(linestr) == 4:  # x,y,z,i
            print("x,y,z，i")
            linestr_convert = list(map(float, linestr))
            lidar.append(linestr_convert)
    return np.array(lidar)

def pcd_to_bin():
    pcd_file="./testpoint.pcd"
    bin_file = "./testpoint-pcd2bin.bin"
    pl = read_pcd(pcd_file)
    #pl = pl.reshape(-1, 3).astype(np.float32)  # x,y,z,i
    pl.tofile(bin_file)

def bin_to_pcd():
    bin_file="./testpoint-pcd2bin.bin"
    pcd_file="./testpoint-pcd2bin-bin2pcd.pcd"
    #bin_file="./kitti_0.bin"
    #pcd_file="./kitti_0_bin2pcd.pcd"

    b = np.fromfile(bin_file, dtype=np.float64)
    print(b.shape)
    a = b.reshape(-1, 4) # 点云格式X,Y,Z,i

    if not os.path.exists(pcd_file):
        f = open(pcd_file, 'w')
        f.close()
    with open(pcd_file, 'w') as file_to_write:
        file_to_write.writelines("# .PCD v0.7 - Point Cloud Data file format\n")
        file_to_write.writelines("VERSION 0.7\n")
        file_to_write.writelines("FIELDS x y z\n")
        file_to_write.writelines("SIZE 4 4 4\n")
        file_to_write.writelines("TYPE F F F\n")
        file_to_write.writelines("COUNT 1 1 1\n")
        file_to_write.writelines("WIDTH " + str(a.shape[0]) + "\n")
        file_to_write.writelines("HEIGHT 1\n")
        file_to_write.writelines("VIEWPOINT 0 0 0 1 0 0 0\n")
        file_to_write.writelines("POINTS " + str(a.shape[0]) + "\n")
        file_to_write.writelines("DATA ascii\n")
        #file_to_write.writelines("data ascii\n")
        for line in a:
            #print(line[0])
            #file_to_write.writelines(str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + " " + str(line[3])+ "\n")
            file_to_write.writelines(str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + "\n")

if __name__ == '__main__':
    #rosbag_to_txt() #已通过测试
    #txt_to_bin() #已通过测试
    #txt_to_pcd2() #已通过测试
    #pcd_to_bin() #已通过测试
    #bin_to_txt() #已通过测试
    #pcd_to_txt() #已通过测试
    #bin_to_pcd() #已通过测试
    txt_to_pcd1() #已通过测试