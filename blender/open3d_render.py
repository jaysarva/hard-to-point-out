import numpy as np
import open3d as o3d
import math
def change_background_to_black(vis):
    '''
    change background of render to black
    '''
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    return False

def custom_draw_geometry_with_custom_fov(pcd, rot_x, rot_y, path):
    '''
    rot_x and rot_y should be [0, 360) in degrees
    path is saved png path
    '''
    rot_x /= (0.003 * (180/math.pi)) ## converts degrees into degrees/pixels
    rot_y /= (0.003 * (180/math.pi)) ## converts degrees into degrees/pixels
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False) ## depending on computer, might need to run wih visible=True ....
    vis.add_geometry(pcd)
    ctr = vis.get_view_control()
    #print("Field of view (before changing) %.2f" % ctr.get_field_of_view())
    #ctr.change_field_of_view(step=fov_step)
    #print("Field of view (after changing) %.2f" % ctr.get_field_of_view())
    ctr.rotate(rot_x, rot_y)
    #vis.run()
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(path)
    vis.destroy_window()


if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("point_cloud.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    custom_draw_geometry_with_custom_fov(pcd, 0, 0, "rot0.png")
    custom_draw_geometry_with_custom_fov(pcd, 180, 0, "rot1.png")
    custom_draw_geometry_with_custom_fov(pcd, 360, 0, "rot11.png")
    custom_draw_geometry_with_custom_fov(pcd, 0, 180, "rot2.png")
    custom_draw_geometry_with_custom_fov(pcd, 0, 360, "rot22.png")
    custom_draw_geometry_with_custom_fov(pcd, 180, 180, "rot3.png")
    
    '''
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False) #works for me with False, on some systems needs to be true
    vis.add_geometry(pcd)
    ctr = vis.get_view_control()

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image("point_cloud_img.png")
    vis.destroy_window()
    '''

