import numpy as np
import open3d as o3d

path = f"/home/oongking/Research_ws/src/hole_detector/data/precision_test/sum_hole_alu.pcd"
pcd_original = o3d.io.read_point_cloud(path)


def Cluster(pcd, Clus_eps=0.01, Clus_min_points=3, Out_nb_neighbors = 20, Out_std_ratio = 1.0, point_upper_limit = 100, point_lower_limit = 1, obj_size = 0.05):
    pcds = []
    
    labels = np.array(pcd.cluster_dbscan(eps = Clus_eps, min_points = Clus_min_points, print_progress=False))
    
    max_label = labels.max()
    for i in range(0,max_label+1):
        pcdcen = pcd.select_by_index(np.argwhere(labels==i))
        # pcdcen, ind = pcdcen.remove_statistical_outlier(nb_neighbors = Out_nb_neighbors,
        #                                             std_ratio = Out_std_ratio)
        # print("pcdcen : ",np.asarray(pcdcen.points).shape)
        # print(f"pcdcen.get_max_bound() : {pcdcen.get_max_bound()}")
        # print(f"pcdcen.get_min_bound() : {pcdcen.get_min_bound()}")
        # print(f"np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound()) : {np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound())}")
        
        
        if point_upper_limit >np.asarray(pcdcen.points).shape[0]>point_lower_limit and np.linalg.norm(pcdcen.get_max_bound()-pcdcen.get_min_bound()) < obj_size:
            # o3d.visualization.draw_geometries([pcdcen])
            
            pcdcen.estimate_normals()
            pcds.append(pcdcen)

    return pcds

# holes = Cluster(boundarys_pcd) #paper
center_cluster = Cluster(pcd_original)
o3d.visualization.draw_geometries(center_cluster)

norms = []
cen_points = []
for cluster in center_cluster:
    points = np.asarray(cluster.points)
    mean = np.mean(points,axis=0)
    print(f"mean : {mean}")
    cen_points.append(mean)
    median = np.median(points)

    vector = np.subtract(points,mean)
    # print(f"vector : {vector}")
    
    vector_norm = np.linalg.norm(vector,axis=1)
    # print(f"vector_norm : {vector_norm}")
    norms = np.append(norms,vector_norm)

print(cen_points)


# norms = np.array(norms)
# print(f"norms : {norms}")

# norm_mean = np.mean(norms)
# print(f"norm_mean : {norm_mean}")

# norm_median = np.median(norms)
# print(f"norm_median : {norm_median}")

# norm_max = np.max(norms)
# print(f"norm_max : {norm_max}")

# norm_min = np.min(norms)
# print(f"norm_min : {norm_min}")

# np.argmax(norms)
# o3d.io.write_point_cloud(f"/home/oongking/Research_ws/src/hole_detector/data/precision_test/alu_cen.pcd", pcdcen)

# np.savetxt("/home/oongking/Research_ws/src/hole_detector/data/precision_test/black_cen.csv", norms, delimiter=",")

