import numpy as np
import cv2
import open3d as o3d

def showd(image_path="depth-map.png", fx=600.0, fy=600.0, near=0.1, far=10.0, fov_deg=60):
    # === Step 1: Load the 4-channel PNG image ===
    rgba = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)  # (H, W, 4)
    if rgba is None:
        raise ValueError("Failed to load image.")

    rgb = cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_BGR2RGB)

    # === STEP 2: Decode real physical depth in meters ===
    # Assumes depth encoded as (depth / far) * 255 in red channel
    depth_meters = (rgba[:, :, 0].astype(np.float32) / 255.0) * far

    # Convert to 16-bit depth in millimeters (Open3D expects this format)
    depth_mm = (depth_meters * 1000.0).astype(np.uint16)
    depth_o3d = o3d.geometry.Image(depth_mm)

    # === STEP 3: Reconstruct intrinsics ===
    height, width = depth_meters.shape
    aspect = width / height
    fov_rad = np.deg2rad(fov_deg)
    fy = height / (2 * np.tan(fov_rad / 2))
    fx = fy * aspect
    cx = width / 2
    cy = height / 2

    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    # === STEP 4: Build point cloud ===
    color_o3d = o3d.geometry.Image(rgb)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d,
        depth_scale=1000.0,
        depth_trunc=far,
        convert_rgb_to_intensity=False
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # Flip to match Open3D view
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])

    # === STEP 5: Visualize ===
    o3d.visualization.draw_geometries([pcd])


def raw_pixel_to_pcd(image_path="depth-map.png", far=10.0):
    # === Load image ===
    rgba = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)  # (H, W, 4)
    if rgba is None:
        raise ValueError("Failed to load image.")

    height, width = rgba.shape[:2]
    depth = (rgba[:, :, 0].astype(np.float32) / 255.0) * 1.5625# * far  # Z in meters

    # === Generate raw pixel-based point cloud ===
    points = []

    for y in range(height):
        for x in range(width):
            z = depth[y, x]
            if z <= 0.0 or z >= far:
                continue
            points.append([float(x)/width, float(y)/height, float(z)])

    points = np.array(points, dtype=np.float32)

    # === Convert to Open3D point cloud ===
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Flip Y/Z to match Open3D's view (optional)
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])

    # === Show it ===
    o3d.visualization.draw_geometries([pcd])