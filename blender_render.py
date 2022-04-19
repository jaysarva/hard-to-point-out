import bpy
import open3d as o3d
import numpy as np

def render_scene(pointcloud_data, camera_pos, camera_ori):
	'''
	pointcloud_data is the overall scene'
	'''
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
	distances = pcd.compute_nearest_neighbor_distance()
	avg_dist = np.mean(distances)
	radius = 3 * avg_dist
	bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))
	mesh = bpa_mesh.simplify_quadric_decimation(100000)
	mesh.remove_degenerate_triangles()
	mesh.remove_duplicated_triangles()
	mesh.remove_duplicated_vertices()
	mesh.remove_non_manifold_edges()


	new_object = bpy.data.objects.new("view", mesh)
	view_layer=bpy.context.view_layer
	view_layer.active_layer_collection.collection.objects.link(new_object)


	light_data = bpy.data.lights.new('light', type='POINT')
	light = bpy.data.objects.new('light', light_data)
	bpy.context.collections.objects.link(light)
	light.location = (0, 0, 20)
	light.data.energy=200.0

	cam_data = bpy.data.cameras.new('camera')
	cam = bpy.data.objects.new('camera', cam_data)
	bpy.context.collection.objects.link(cam)
	scene = bpy.context.scene
	scene.camera=cam
	cam.location = camera_pos
	cam.rotation_euler(camera_ori)
