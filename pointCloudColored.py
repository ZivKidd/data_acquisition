# coding:utf-8
import os
import cv2
import numpy as np
import yaml
import roslib
import message_filters
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import os
import sys
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import datetime
import struct
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

np.set_printoptions(suppress=True)

params = np.load('params_backpack.npz')
cameras = ['250', '251', '252', '253']
camera_matrix_arrays = []
distorts = []
lidarTcameras = []
for c in cameras:
    camera_matrix_arrays.append(params['camera_matrix_array_' + c])
    distorts.append(params['distort_' + c])
    lidarTcameras.append(params['lTc' + c])


class PointCloudColored:
    def __init__(self):
        self.bridge = CvBridge()
        self.pc_pub = rospy.Publisher('/pandar_points_colored', PointCloud2, queue_size=10)
        image_sub0 = message_filters.Subscriber('/image19279250', Image)
        image_sub1 = message_filters.Subscriber('/image19279251', Image)
        image_sub2 = message_filters.Subscriber('/image19279252', Image)
        image_sub3 = message_filters.Subscriber('/image19279253', Image)
        pc_sub = message_filters.Subscriber('/pandar_points', PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub0, image_sub1, image_sub2, image_sub3, pc_sub], 10,
                                                         0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, image0, image1, image2, image3, pc):
        points = point_cloud2.read_points(pc)
        points = np.array(list(points))

        cv_image0 = self.bridge.imgmsg_to_cv2(image0, "bgr8")
        cv_image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
        cv_image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
        cv_image3 = self.bridge.imgmsg_to_cv2(image3, "bgr8")

        pcColored0 = self.colored(cv_image0, points, 0)
        pcColored1 = self.colored(cv_image1, points, 1)
        pcColored2 = self.colored(cv_image2, points, 2)
        pcColored3 = self.colored(cv_image3, points, 3)

        pcColored = np.concatenate([pcColored0, pcColored1, pcColored2, pcColored3], axis=0)

        result_points = []
        for i in range(len(pcColored)):
            rgb = struct.unpack('I', struct.pack('BBBB', pcColored[i, 5], pcColored[i, 4], pcColored[i, 3], 255))[0]
            pt = [pcColored[i, 0], pcColored[i, 1], pcColored[i, 2], rgb]
            result_points.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1),
                  ]

        pc2 = point_cloud2.create_cloud(pc.header, fields, result_points)

        print pc.header
        #
        # msg = PointCloud2()
        # msg.header = pc.header
        #
        # msg.height = 1
        # msg.width = len(pcColored2)
        # msg.fields = [
        #     PointField('x', 0, PointField.FLOAT32, 1),
        #     PointField('y', 4, PointField.FLOAT32, 1),
        #     PointField('z', 8, PointField.FLOAT32, 1),
        #     PointField('rgba', 12, PointField.UINT32, 1)]
        #
        # msg.is_bigendian = False
        # msg.point_step = 24
        # msg.row_step = msg.point_step * points.shape[0]
        # # msg.is_dense = int(np.isfinite(points).all())
        # msg.is_dense = False
        # msg.data = result_points

        self.pc_pub.publish(pc2)

        # print()

    def colored(self, image, pc, camera_index):
        pcColored = 0
        camera_matrix_array = camera_matrix_arrays[camera_index]
        distort = distorts[camera_index]
        lidarTcamera = lidarTcameras[camera_index]

        points = pc[:, :3]
        inten = pc[:, 3:4]

        color = cv2.undistort(image,
                              camera_matrix_array,
                              distort)

        # cv2.imwrite(os.path.join(undistort_path, name) + '.png', color)
        # points=np.asarray(f['pcds'][time1])[:,:3]
        # points = np.loadtxt(pcds[i], delimiter=' ')[:,:3]
        points1 = np.ones([points.shape[0], 4])
        points1[:, :3] = points
        points1 = points1.dot(np.linalg.inv(lidarTcamera).T)
        points2 = points1.copy()[:, :3]
        points1[:, 0] = points1[:, 0] / points1[:, 2]
        points1[:, 1] = points1[:, 1] / points1[:, 2]
        points1[:, 2] = np.ones(points1[:, 2].shape)
        points1 = points1[:, :3]
        # points1[:,3]=np.ones(points1[:,3].shape)
        points1 = points1.dot(camera_matrix_array.T)

        # p_count=0
        # depth = np.zeros([600, 960]).astype(np.uint16)

        # Eliminate pixels outside view frustum
        valid_pix = np.logical_and(points1[:, 0] > 0,
                                   np.logical_and(points1[:, 0] < 959,
                                                  np.logical_and(points1[:, 1] > 0, points1[:, 1] < 599)))

        x = np.rint(points1[:, 0][valid_pix]).astype(np.int)
        y = np.rint(points1[:, 1][valid_pix]).astype(np.int)
        z = points2[valid_pix][:, 2]

        points_valid = points[valid_pix]
        inten = inten[valid_pix]

        points_rgb = np.ones([x.shape[0], 6])
        points_rgb[:, :3] = points_valid
        bgr = color[y, x]
        points_rgb[:, 3] = bgr[:, 2]
        points_rgb[:, 4] = bgr[:, 1]
        points_rgb[:, 5] = bgr[:, 0]

        valid_pix = np.where(z > 0)
        points_rgb = points_rgb[valid_pix]
        inten = inten[valid_pix]
        xyzrgbi = np.concatenate([points_rgb, inten], axis=1)
        # pointrgbs.extend(xyzrgbi.tolist())

        return xyzrgbi


rospy.init_node('pointCloudColored', anonymous=True)
pointCloudColored = PointCloudColored()
rospy.spin()

# def callback_pointcloud(data):
#     assert isinstance(data, PointCloud2)
#     # gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
#     # timestr = "%.6f" % msg.header.stamp.to_sec()
#     # laser_data_name = timestr + ".txt"
#     # laser_data_path = os.path.join(pcd, laser_data_name)
#     lidar = point_cloud2.read_points(data)
#     points = np.array(list(lidar))
#     # points=points[:,:3]
#     # np.savetxt(laser_data_path,points)
#     # print laser_data_path
#     # pcd.create_dataset(timestr, data=points)
#     # print 'pcd'+timestr
#     # print
#     # topic + ',' + timestr
#     print()
#
#
# rospy.init_node('pointCloudColored', anonymous=True)
# rospy.Subscriber('/pandar_points', PointCloud2, callback_pointcloud)
# rospy.spin()
#
#
#
# # print('读入lio mapping导出的轨迹')
# # odo = np.loadtxt(odo_path, skiprows=1, delimiter=',', dtype=np.str)
# # odo_list = []
# #
# # for i in range(odo.shape[0]):
# #     odo_list.append(float(odo[i, 2])/1e9)
# # print('有效时间：',odo_list[-1])
# # print(f.keys())
# # pcds_color=f.create_group('pcds_color')
# # print(f.keys())
# # for i in tqdm.tqdm(range(len(pcds))):
# for i, time in enumerate(tqdm.tqdm(f['pcds'].keys())):
#     # if(i<3500):
#     #     continue
#     # print(time)
#     # if (float(time)>odo_list[-1]):
#     #     print(i,'跳过')
#     #     continue
#     pointrgbs=[]
#     for k,c in enumerate(cameras):
#         for j, time1 in enumerate(f['image'+c].keys()):
#             if(abs(float(time)-float(time1))<0.05):
#
#                 lidarTcamera=lidarTcameras[k]
#                 camera_matrix_array=camera_matrix_arrays[k]
#                 distort=distorts[k]
#                 # name = os.path.split(pcds[i])[1][:-4]
#                 # if(i<150):
#                 #     continue
#                 # corner_lidar = getCornerFromLidar(pcds[i], 40, 130)
#                 # corner_camera = getCornerFromCamera(images[i], camera_matrix_array_250, distort_250)
#                 # print(time,time1)
#                 # print(images[i])
#                 points = np.asarray(f['pcds'][time])[:, :3]
#                 inten=np.asarray(f['pcds'][time])[:, 3:4]
#
#                 color = np.asarray(f['image'+c][time1])
#                 color = cv2.undistort(color,
#                                       camera_matrix_array,
#                                       distort)
#
#                 # cv2.imwrite(os.path.join(undistort_path, name) + '.png', color)
#                 # points=np.asarray(f['pcds'][time1])[:,:3]
#                 # points = np.loadtxt(pcds[i], delimiter=' ')[:,:3]
#                 points1 = np.ones([points.shape[0], 4])
#                 points1[:, :3] = points
#                 points1 = points1.dot(np.linalg.inv(lidarTcamera).T)
#                 points2 = points1.copy()[:, :3]
#                 points1[:, 0] = points1[:, 0] / points1[:, 2]
#                 points1[:, 1] = points1[:, 1] / points1[:, 2]
#                 points1[:, 2] = np.ones(points1[:, 2].shape)
#                 points1 = points1[:, :3]
#                 # points1[:,3]=np.ones(points1[:,3].shape)
#                 points1 = points1.dot(camera_matrix_array.T)
#
#                 # p_count=0
#                 # depth = np.zeros([600, 960]).astype(np.uint16)
#
#                 # Eliminate pixels outside view frustum
#                 valid_pix = np.logical_and(points1[:, 0] > 0,
#                                            np.logical_and(points1[:, 0] < 959,
#                                                           np.logical_and(points1[:, 1] > 0, points1[:, 1] < 599)))
#
#                 x = np.rint(points1[:, 0][valid_pix]).astype(np.int)
#                 y = np.rint(points1[:, 1][valid_pix]).astype(np.int)
#                 z = points2[valid_pix][:, 2]
#
#                 points_valid=points[valid_pix]
#                 inten=inten[valid_pix]
#
#                 points_rgb=np.ones([x.shape[0],6])
#                 points_rgb[:,:3]=points_valid
#                 bgr=color[y,x]
#                 points_rgb[:,3]=bgr[:,2]
#                 points_rgb[:,4]=bgr[:,1]
#                 points_rgb[:,5]=bgr[:,0]
#
#                 valid_pix=np.where(z>0)
#                 points_rgb=points_rgb[valid_pix]
#                 inten=inten[valid_pix]
#                 xyzrgbi=np.concatenate([points_rgb,inten],axis=1)
#                 pointrgbs.extend(xyzrgbi.tolist())
#     if(pointrgbs==[]):
#         continue
#     pointrgbs=np.asarray(pointrgbs)
#     # np.savetxt(os.path.join(point_color_path, time) + '.txt',pointrgbs)
#     f1.create_dataset(time,data=pointrgbs)
#
# f.close()
# f_xyzrgbi.close()


# z = points2[valid_pix][:, 2]
#
# z = z * 1000
# z[np.where(z > 65535)] = 0
# z[np.where(z < 0)] = 0
#
# depth[y, x] = z

# cv2.imwrite(os.path.join(depth_path, name) + '.png', depth)
#
# # depth_path = 'depth.png'
# # cv2.imwrite(depth_path, depth)
#
# rgb = o3d.io.read_image(os.path.join(undistort_path, name) + '.png')
# depth = o3d.io.read_image(os.path.join(depth_path, name) + '.png')
# print(os.path.join(undistort_path, name) + '.png')
# print(os.path.join(depth_path, name) + '.png')
# # intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 480, 517.306408, 516.469215, 318.643040, 255.313989)
# intrinsic = o3d.camera.PinholeCameraIntrinsic(960,
#                                               600,
#                                               camera_matrix_array[0, 0],
#                                               camera_matrix_array[1, 1],
#                                               camera_matrix_array[0, 2],
#                                               camera_matrix_array[1, 2])
# pc = o3d.geometry.PointCloud()
# pc.points=o3d.utility.Vector3dVector(points_rgb[:,:3])


# pc.colors=o3d.utility.Vector3dVector(points_rgb[:,3:]/255)
# image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, 1000.0, 50.0, False)
# pc = o3d.geometry.PointCloud.create_from_rgbd_image(image, intrinsic, np.eye(4))
# o3d.visualization.draw_geometries([pc], width=900, height=600, window_name='planes')

# o3d.io.write_point_cloud(os.path.join(point_color_path, time) + '.pcd',
#                          pc,write_ascii=True)
#
# sys.exit()
