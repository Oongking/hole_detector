import numpy as np
import open3d as o3d
path = f"/home/oongking/Research_ws/src/hole_detector/data/zivid_test/blackhole.pcd"
pcd_original = o3d.io.read_point_cloud(path)


def pick_points(pcd):
    # print("")
    # print(
    #     "1) Please pick at least three correspondences using [shift + left click]"
    # )
    # print("   Press [shift + right click] to undo point picking")
    # print("2) Afther picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


PointWS = pick_points(pcd_original)

pcd_original = pcd_original.select_by_index(PointWS, invert=True)


o3d.visualization.draw_geometries([pcd_original])
# o3d.io.write_point_cloud(path, pcd_original)

# PointWS = pick_points(pcd)
# selectpoint = []
# for i,x in enumerate(PointWS):
#     selectpoint.append(np.asarray(pcd_original.points)[x])

# vector = np.subtract(selectpoint[0],selectpoint[1])

# print(f"Hole Diameter : {np.linalg.norm(vector)} \n\n")
