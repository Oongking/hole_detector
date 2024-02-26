import numpy as np
import open3d as o3d
import copy

# pcd = o3d.io.read_point_cloud('/home/oongking/Research_ws/src/hole_detector/data/room/method_test/test1.pcd')
# pcd = o3d.io.read_point_cloud('/home/oongking/Research_ws/src/hole_detector/data/room/mock_test/filter_3cm.pcd')
pcd = o3d.io.read_point_cloud('/home/oongking/Research_ws/src/hole_detector/data/room/mock_test/mock3cm_0.pcd')



def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])
  
def Ry(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-np.sin(theta), 0, np.cos(theta)]])
  
def Rz(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])


def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a = (vec1 / np.linalg.norm(vec1)).reshape(3)
    b = (vec2 / np.linalg.norm(vec2)).reshape(3)
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    angle = np.arccos(c)
    return rotation_matrix,angle

def fixbox(rot,trans,z_offset,x = 0.4,y = 0.6 ,z = 0.1) :
    # Before rotate to canon
    
    fix_box = np.array([
    [-x/2,-y/2,z_offset],
    [-x/2, y/2,z_offset],
    [x/2, y/2,z_offset],
    [x/2,-y/2,z_offset],

    [-x/2,-y/2,z+z_offset],
    [-x/2, y/2,z+z_offset],
    [x/2,-y/2,z+z_offset],
    [x/2, y/2,z+z_offset]
    ])

    fixbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(fix_box))
    fixbox.rotate(rot,(0,0,0))
    fixbox.translate(np.asarray(trans,dtype=np.float64),relative=True)
    fixbox.color = (0, 0, 0)

    return fixbox 

class sphere:
    def __init__(self, center, radius = 0.001 , color = ( 1, 1, 0)):
        self.pcd = o3d.geometry.TriangleMesh.create_sphere(radius)
        self.pcd.compute_vertex_normals()
        self.pcd.translate((center[0], center[1], center[2]), relative=False)
        self.pcd.paint_uniform_color(color)


class Hole_detector:
    def __init__(self):

        self.box = fixbox(np.eye(3),[0.05, -0.058, 0.88],0,x=0.2,y=0.2,z = 0.1)
        # self.box = fixbox(np.eye(3),[0.83, -0.50, 0.84],0,x=0.2,y=0.2,z = 0.15)

    def Cluster_hole(self,pcd, Clus_eps=0.005, Clus_min_points=5, Out_nb_neighbors = 20, Out_std_ratio = 1.0, point_upper_limit = 150, point_lower_limit = 60, obj_size = 0.05):
        pcds = []
        
        labels = np.array(pcd.cluster_dbscan(eps = Clus_eps, min_points = Clus_min_points, print_progress=False))
        
        max_label = labels.max()
        for i in range(0,max_label+1):
            pcdcen = pcd.select_by_index(np.argwhere(labels==i))
            # pcdcen, ind = pcdcen.remove_statistical_outlier(nb_neighbors = Out_nb_neighbors,
            #                                             std_ratio = Out_std_ratio)
            print("pcdcen : ",np.asarray(pcdcen.points).shape)
            # print(f"pcdcen.get_max_bound() : {pcdcen.get_max_bound()}")
            # print(f"pcdcen.get_min_bound() : {pcdcen.get_min_bound()}")
            # print(f"np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound()) : {np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound())}")
            
            
            if point_upper_limit >np.asarray(pcdcen.points).shape[0]>point_lower_limit and np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound()) < obj_size:
                # o3d.visualization.draw_geometries([pcdcen])
                
                pcdcen.estimate_normals()
                pcds.append(pcdcen)

        return pcds
    
    def circle_fiting(self,points):

        x = points[:,0].T 
        y = points[:,1].T
        xy = points[:,:2]


        # print(f"x : {x.shape}")
        # print(f"y : {y.shape}")
        # print(f"xy : {xy.shape}")
        # print(f"xy : {np.power(np.linalg.norm(xy,axis=1),2)}")
        B = np.power(np.linalg.norm(xy,axis=1),2).T
        A = np.array([x*2, y*2,np.ones(x.shape[0])]).T
        # print(f"A : {A}")
        # print(f"B : {B}")
        # print(f"np.linalg.lstsq(A,B) : {np.linalg.lstsq(A,B, rcond=None)[0]}")
        ans = np.linalg.lstsq(A,B, rcond=None)[0][:3]
        # print(f"ans : {ans}")
        # print(f"ans : {ans[2]+np.power(ans[0],2)+np.power(ans[1],2)}")
        r = np.sqrt(ans[2]+np.power(ans[0],2)+np.power(ans[1],2))
        # print(f"radius : {r}")


        return ans[:2],r

    def find_hole(self,pcd_original):
        # Preprocess pointcloud
        pcd = pcd_original.crop(self.box)
        # pcd = pcd.voxel_down_sample(0.001)
        # o3d.visualization.draw_geometries([pcd,Realcoor])

        pcdcen, ind = pcd.remove_statistical_outlier(nb_neighbors = 10,
                                                    std_ratio = 4.0)
        # pcdcen = pcd
        # o3d.visualization.draw_geometries([pcdcen,Realcoor])


        plane_model, inliers = pcdcen.segment_plane(distance_threshold=0.005,
                                         ransac_n=3,
                                         num_iterations=1000)
        
        # o3d.visualization.draw_geometries([pcdcen.select_by_index(inliers)])
        pcdcen = pcdcen.select_by_index(inliers)
        # find plane and project point to plane
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        xyz = np.asarray(pcdcen.points)
        # d : distance to plan  n : Normal vector of plane
        d = (a*xyz[:,0]+b*xyz[:,1]+c*xyz[:,2]+d)/np.linalg.norm([a,b,c])
        n = np.array([a, b, c])/np.linalg.norm([a,b,c])

        plane_normals = np.full([d.shape[0],3],n)
        shift = np.multiply(plane_normals,-d.reshape(-1,1))

        pcdcen.points = o3d.utility.Vector3dVector(np.add(xyz,shift, dtype=np.float64))
        # pcdcen = pcdcen.voxel_down_sample(0.001)
        # o3d.visualization.draw_geometries([pcdcen,Realcoor])

        pcdcen_old  = copy.deepcopy(pcdcen)
         # find boundarys in tensor
        pcdcen_old_t = o3d.t.geometry.PointCloud.from_legacy(pcdcen_old, o3d.core.float64)

        pcdcen_old_t.estimate_normals(max_nn=30, radius=1.4*0.003)
        boundarys_old, mask = pcdcen_old_t.compute_boundary_points(0.015, 30)
        print(f"Detect {boundarys_old.point.positions.shape[0]} bnoundary points from {pcdcen_old_t.point.positions.shape[0]} points.")

        boundarys_old_pcd = boundarys_old.to_legacy()



        pcdcen.estimate_normals()
        normal = np.asarray(pcdcen.normals)
        normal[normal[:,2]>0] *= -1

        pcdcen.normals = o3d.utility.Vector3dVector(np.array(normal, dtype=np.float64))

        radii = [0.004]
        rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcdcen, o3d.utility.DoubleVector(radii))
        o3d.visualization.draw_geometries([pcdcen, rec_mesh])
        o3d.visualization.draw_geometries([rec_mesh])
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(rec_mesh)
        filled = mesh_t.fill_holes(hole_size= 0.01)
        filled = filled.to_legacy()
        # o3d.visualization.draw_geometries([filled])
        filled = filled.subdivide_midpoint(number_of_iterations=2)
        o3d.visualization.draw_geometries([filled])
        pcdcen.points = filled.vertices
        # o3d.visualization.draw_geometries([pcdcen])
        # o3d.visualization.draw_geometries([pcdcen_old])
        pcdcen_old_vis = copy.deepcopy(pcdcen_old)
        pcdcen_old_vis.translate([0.2,0,0])
        # o3d.visualization.draw_geometries([pcdcen,pcdcen_old_vis])
        pcdcen = pcdcen.voxel_down_sample(0.0008)
        pcdcen_old_vis = copy.deepcopy(pcdcen_old)
        pcdcen_old_vis.translate([0.2,0,0])
        o3d.visualization.draw_geometries([pcdcen,pcdcen_old_vis])

       



        
        # find boundarys in tensor
        pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcdcen, o3d.core.float64)


        pcd_t.estimate_normals(max_nn=30, radius=1.4*0.003)
        boundarys, mask = pcd_t.compute_boundary_points(0.015, 30,angle_threshold = 70)
        print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd_t.point.positions.shape[0]} points.")

        boundarys_pcd = boundarys.to_legacy()

        # o3d.visualization.draw_geometries([boundarys_pcd])
        # o3d.visualization.draw_geometries([boundarys_old_pcd])
        boundarys_old_pcd_vis = copy.deepcopy(boundarys_old_pcd)
        boundarys_old_pcd_vis.translate([0.2,0,0])
        o3d.visualization.draw_geometries([boundarys_pcd,boundarys_old_pcd_vis])

        boundarys_pcd.paint_uniform_color([1,0,0])
        boundarys_old_pcd.paint_uniform_color([0,1,0])
        o3d.visualization.draw_geometries([boundarys_pcd,boundarys_old_pcd])



        holes_old = self.Cluster_hole(boundarys_old_pcd)

        holes = self.Cluster_hole(boundarys_pcd)


        centers_fit_old = []
        for i,hole in enumerate(holes_old):
            tfm = np.eye(4)
            tfm[:3,:3] = rotation_matrix_from_vectors([0,0,1],n)[0]
            tfm[:3,3] = hole.points[0]
            inv_tfm = np.linalg.inv(tfm)
            hole.transform(inv_tfm)
            [x,y],r = self.circle_fiting(np.array(hole.points))
            center = [x,y,0,1]
            center = np.matmul(tfm,center)
            center_pcd = sphere(center[:3],color=[1,1,0]).pcd
            hole.transform(tfm)
            centers_fit_old.append(center_pcd)

        

        centers = []
        center_point = []
        centers_fit = []
        for i,hole in enumerate(holes):
            tfm = np.eye(4)
            tfm[:3,:3] = rotation_matrix_from_vectors([0,0,1],n)[0]
            tfm[:3,3] = hole.points[0]
            inv_tfm = np.linalg.inv(tfm)
            hole.transform(inv_tfm)
            [x,y],r = self.circle_fiting(np.array(hole.points))
            center = [x,y,0,1]
            center = np.matmul(tfm,center)
            center_pcd = sphere(center[:3],color=[0,1,0]).pcd
            hole.transform(tfm)
            centers_fit.append(center_pcd)

            color = np.random.randint(10, size=3)
            if r > 0.015:
                center = hole.get_center()
            center_point.append(center)

            holes[i].paint_uniform_color([color[0]/10, color[1]/10, color[2]/10])


        o3d.visualization.draw_geometries(holes+centers_fit_old+centers_fit+[Realcoor])

        return center_point,n


hole_detector = Hole_detector()

box = fixbox(np.eye(3),[0.05, -0.06, 0.88],0,x=0.2,y=0.2,z = 0.1)

box2 = fixbox(np.eye(3),[0.83, -0.50, 0.84],0,x=0.2,y=0.2,z = 0.15)

Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.005,(0,0,0))

# o3d.visualization.draw_geometries([pcd,Realcoor,box,box2])

mergpcd = pcd

# o3d.visualization.draw_geometries([mergpcd,Realcoor])

center_point,plane_normal = hole_detector.find_hole(mergpcd)

# center_point = np.asarray(center_point)
# sort_i = np.argsort(center_point[:,2])
# center_point = center_point[sort_i]


# cam2gantry = np.array([[-2.47323185e-03, -9.99996942e-01,  9.53512728e-07,0],
#                         [ 9.54676575e-01, -2.36085994e-03,  2.97635790e-01,0],
#                         [-2.97634878e-01,  7.37032614e-04,  9.54679494e-01,0],
#                         [0,0,0,1]])
# gantry2cam = np.linalg.inv(cam2gantry)
# cam2laser = np.array([[ 0.99993235 ,-0.00167084 , 0.01151127 , 0.05619813 ],
#                         [-0.00167084,  0.95873473,  0.28429725, -0.1144363 ],
#                         [-0.01151127, -0.28429725,  0.95866708,  0.03326182],
#                         [ 0.        ,  0.         , 0.         , 1.        ]])
# laser2cam = np.linalg.inv(cam2laser)
# laser2eff = np.matmul(laser2cam,cam2gantry)

# gan2laser = np.matmul(gantry2cam,cam2laser)

# print(f"gan2laser : {gan2laser}")
# cen_point = []
# for cen in center_point:
#     cen_point.append(sphere([cen[0],cen[1],cen[2]],0.005).pcd)

# def linePlaneIntersection(plane, rayDir):
#     """
#     Calculate the 3D intersection between a plane and a ray, returning a 3D point.
#     """
#     pOrigin, pNormal = plane
#     # print(f"pOrigin : {pOrigin.shape}, {pOrigin}")
#     # print(f"pNormal : {pNormal.shape}, {pNormal}")
#     # print(f"rayDir : {rayDir.shape}")
#     d = np.dot(pOrigin, pNormal) / np.dot(rayDir, pNormal)
#     return rayDir * d

# print(f"np.matmul(gan2laser[:3,:3],[0,0,1]) : {np.matmul(gan2laser[:3,:3],[0,0,1])}")
# set_laser_coors = []
# set_laser_tfms = []
# for point in center_point:
#     laser_tf = np.eye(4)
#     #offset
#     laser_tf[:3,3] = [point[0],point[1],point[2]]
#     laser_tf[:3,:3] = gan2laser[:3,:3]

#     laser_plane_shift = np.eye(4)
#     laser_plane_shift[3,3] = gan2laser[3,3]

#     plane_ref_point = np.linalg.inv(laser_tf)
#     plane_ref_point = np.matmul(plane_ref_point,laser_plane_shift)

#     shift = linePlaneIntersection([plane_ref_point[:3,3],np.matmul(plane_ref_point[:3,:3],[0,0,1])],np.matmul(gan2laser[:3,:3],[0,0,1]))

#     print(f"shift : {shift}")
#     shift_tf = np.eye(4)
#     shift_tf[:3,3] = shift
#     laser_tf = np.matmul(laser_tf,shift_tf)
#     laser_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
#     laser_coor.transform(laser_tf)
#     set_laser_coors.append(laser_coor)
#     set_laser_tfms.append(laser_tf)


# o3d.visualization.draw_geometries(cen_point+set_laser_coors+[mergpcd,Realcoor])



# off_set = []
# for tfm in set_laser_tfms:
#     off_set.append(sphere([tfm[0,3],tfm[1,3],tfm[2,3]]).pcd)

# o3d.visualization.draw_geometries(cen_point+off_set+[mergpcd,Realcoor])


# coors = set_laser_coors
# tfms = set_laser_tfms

# cam_coors = []
# gan_coors = []
# for i,tfm in enumerate(tfms):
#     tfm_gan2cam = np.matmul(tfm,laser2cam)
#     cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
#     cam_coor.transform(tfm_gan2cam)
#     cam_coors.append(cam_coor)

#     tfm_go = np.matmul(tfm,laser2eff)
#     gan_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
#     gan_coor.transform(tfm_go)
#     gan_coors.append(gan_coor)


    
#     # control_cartesian_arm(tfm_go[:3,3],tfm_go[:3,:3])
#     # aim_pose_srv(tfm_go[0,3]*1000,-tfm_go[1,3]*1000)
# o3d.visualization.draw_geometries(cen_point+off_set+ coors+ [mergpcd,Realcoor])
# o3d.visualization.draw_geometries(cen_point+off_set+ cam_coors+ [mergpcd,Realcoor])
# o3d.visualization.draw_geometries(cen_point+off_set+ gan_coors+ [mergpcd,Realcoor])

# cam_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))

# gantry2cam[:3,3] = [0.1,0,0]
# cam_coor.transform(gantry2cam)

# laser_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
# laser_coor.transform(np.matmul(gantry2cam,cam2laser))

# o3d.visualization.draw_geometries([Realcoor,cam_coor,laser_coor])

# o3d.visualization.draw_geometries( cen_point+[laser_coor,cam_coor,mergpcd,Realcoor])


# vecX_laser = np.matmul(gan2laser[:3,:3],[1,0,0])
# vecX_laser[2] = 0
# rad_diff = rotation_matrix_from_vectors([0,-1,0],vecX_laser)[1]

# print(f"rad_diff : {rad_diff}")
# print(f"rad_diff : {rad_diff/np.pi*180}")

# shift = np.tan(rad_diff)*(-np.array(tfms[0])[1,3])
# print(f"shift : {shift}")

