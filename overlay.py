#!/usr/bin/env python3
"""生成 point cloud → ERP image overlay，对比 init vs NID final T_lc"""
import json, sys, numpy as np, cv2
from pathlib import Path

def quat_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx*qx+qy*qy+qz*qz+qw*qw); qx,qy,qz,qw = qx/n,qy/n,qz/n,qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])

def load_ply_xyz(path):
    with open(path, 'rb') as f:
        header, is_bin, elems, props = [], False, 0, []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header.append(line)
            if line.startswith('format binary'): is_bin = True
            if line.startswith('element vertex'): elems = int(line.split()[-1])
            if line.startswith('property'): props.append(line.split()[-1])
            if line == 'end_header': break
        if is_bin:
            n_float = sum(1 for p in props if p not in ())
            dtype = np.float32
            data = np.frombuffer(f.read(elems * n_float * 4), dtype=dtype).reshape(elems, n_float)
        else:
            data = np.loadtxt(f, max_rows=elems)
    idx = {p:i for i,p in enumerate(props)}
    return data[:, [idx['x'], idx['y'], idx['z']]]

def project_erp(pts_cam, W, H):
    """相机坐标 → ERP (u, v)"""
    x, y, z = pts_cam[:,0], pts_cam[:,1], pts_cam[:,2]
    r = np.linalg.norm(pts_cam, axis=1)
    valid = r > 0.5
    lon = np.arctan2(x, z)   # [-pi, pi]
    lat = np.arcsin(np.clip(y/np.maximum(r,1e-9), -1, 1))  # [-pi/2, pi/2]
    u = (lon/np.pi + 1) * 0.5 * W
    v = (lat/(np.pi/2) + 1) * 0.5 * H
    return u, v, r, valid

def overlay(img, pts_lidar, T_lc, color_by_depth=True):
    R = quat_to_R(*T_lc[3:])
    t = np.array(T_lc[:3])
    # T_camera_lidar: 点从 lidar frame → camera frame
    # T_lidar_camera 是 camera → lidar，所以 T_cam_lidar = inv(T_lc)
    R_cl = R.T; t_cl = -R.T @ t
    pts_cam = (R_cl @ pts_lidar.T).T + t_cl
    H, W = img.shape[:2]
    u, v, r, valid = project_erp(pts_cam, W, H)
    u, v, r = u[valid], v[valid], r[valid]
    ui, vi = np.round(u).astype(int) % W, np.round(v).astype(int)
    mask = (vi >= 0) & (vi < H)
    ui, vi, r = ui[mask], vi[mask], r[mask]
    out = img.copy()
    # 按深度着色: 近红远蓝
    r_norm = np.clip((r-1)/9, 0, 1)
    colors = cv2.applyColorMap((r_norm*255).astype(np.uint8), cv2.COLORMAP_JET).reshape(-1, 3)
    for (uu, vv, c) in zip(ui, vi, colors):
        cv2.circle(out, (int(uu), int(vv)), 2, tuple(int(x) for x in c), -1)
    return out

def main():
    calib = json.load(open('/data/out/calib.json'))
    T_init = calib['results']['init_T_lidar_camera_auto']
    T_final = calib['results']['T_lidar_camera']
    print(f"init  t = {T_init[:3]}")
    print(f"final t = {T_final[:3]}")

    for stem in sorted(Path('/data/out').glob('scan_*_bag.ply')):
        base = stem.stem
        png = Path('/data/out') / f'{base}.png'
        if not png.exists():
            print(f"skip {base}: no image"); continue
        img = cv2.imread(str(png))
        pts = load_ply_xyz(str(stem))
        print(f"{base}: {len(pts)} points, image {img.shape}")

        out_init  = overlay(img, pts, T_init)
        out_final = overlay(img, pts, T_final)
        # 拼接: 上=init  下=final
        cv2.putText(out_init,  'INIT  (SuperGlue)', (30,60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,255), 3)
        cv2.putText(out_final, 'FINAL (NID)',       (30,60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,255), 3)
        combined = np.vstack([out_init, out_final])
        # 缩放到 1920 宽方便查看
        h, w = combined.shape[:2]
        scale = 1920 / w
        combined = cv2.resize(combined, (1920, int(h*scale)))
        outp = f'/data/out/overlay_{base}.jpg'
        cv2.imwrite(outp, combined, [cv2.IMWRITE_JPEG_QUALITY, 85])
        print(f"  → {outp}")

if __name__ == '__main__':
    main()
