import open3d as o3d
import numpy as np


def demo_crop_geometry():
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    pcd = o3d.io.read_point_cloud("point_cloud_full.pcd")
    o3d.visualization.draw_geometries_with_editing([pcd])
def edit(pcd):
        
        orignal = pcd
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(orignal)
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window('Window for Cropping')
        vis.add_geometry(pcd)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        vis.run()
        vis.destroy_window()
        # get cropped result to do somthing else
        cropped_geometry= vis.get_cropped_geometry()
        croped = np.asarray(cropped_geometry.points )
        print('please wait ...')
        indices = find_matching_indices(orignal, croped)


        return croped , indices

def find_matching_indices(array1, array2):
    matching_indices = []
    for i, value in enumerate(array1):
        if value in array2:
            matching_indices.append(i)
    return matching_indices

