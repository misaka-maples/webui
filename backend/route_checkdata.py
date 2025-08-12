# backend/api.py
from flask import Blueprint, request, jsonify, send_file,render_template
import os
import io
import h5py
import numpy as np
from PIL import Image
from pathlib import Path

bp = Blueprint('check_data', __name__)

@bp.route('/')
def checkdata_index():
    return render_template('check_data.html')

BASE_DIR = '/home/wfx/A-wfx-project/webui'  # 你也可以从配置里传进来

def validate_path(p: str):
    p = Path(p).resolve()
    if BASE_DIR:
        base = Path(BASE_DIR).resolve()
        if not str(p).startswith(str(base)):
            raise ValueError("path outside allowed base dir")
    return p

def find_datasets(h5: h5py.File):
    # ... 省略同之前代码，照搬即可
    joints = []
    rgbs = []
    def scan_group(g, prefix=''):
        for k in g:
            item = g[k]
            full = f"{prefix}{k}"
            if isinstance(item, h5py.Dataset):
                lname = k.lower()
                if any(x in lname for x in ('joint', 'qpos', 'joints', 'angle', 'action')):
                    joints.append(full)
                elif any(x in lname for x in ('rgb','image','frame','video')):
                    rgbs.append(full)
            elif isinstance(item, h5py.Group):
                scan_group(item, prefix=full + '/')
    scan_group(h5, '')
    if not rgbs:
        def collect_dsets(g, prefix=''):
            for k in g:
                item = g[k]
                full = f"{prefix}{k}"
                if isinstance(item, h5py.Dataset):
                    if item.ndim >= 3:
                        rgbs.append(full)
                elif isinstance(item, h5py.Group):
                    collect_dsets(item, prefix=full + '/')
        collect_dsets(h5, '')
    return {"joints": joints, "rgbs": rgbs}

def extract_frame_from_dataset(h5path: Path, dataset_name: str, idx: int):
    # ... 同之前代码
    with h5py.File(h5path, 'r') as hf:
        if dataset_name not in hf:
            raise KeyError("dataset not found")
        d = hf[dataset_name]
        arr = d[...]
        if arr.ndim == 4:
            if arr.shape[0] > idx:
                frame = arr[idx]
            elif arr.shape[1] > idx and arr.shape[0] == 3:
                frame = arr[:, idx, ...].transpose(1, 2, 0)
            else:
                frame = arr[idx]
        elif arr.ndim == 3:
            if arr.shape[0] > idx:
                frame = arr[idx]
            elif arr.shape[-1] == 3:
                frame = arr
            else:
                frame = arr
        else:
            raise ValueError("unsupported array ndim: " + str(arr.shape))

        frame = np.asarray(frame)
        if frame.ndim == 2:
            mode = 'L'
        elif frame.ndim == 3 and frame.shape[2] == 3:
            mode = 'RGB'
        elif frame.ndim == 3 and frame.shape[0] == 3:
            frame = np.transpose(frame, (1, 2, 0))
            mode = 'RGB'
        else:
            frame = frame.squeeze()
            if frame.ndim == 3 and frame.shape[2] == 3:
                mode = 'RGB'
            elif frame.ndim == 2:
                mode = 'L'
            else:
                raise ValueError("cannot interpret frame shape: " + str(frame.shape))

        if frame.dtype != np.uint8:
            if np.issubdtype(frame.dtype, np.floating):
                fmin, fmax = frame.min(), frame.max()
                if fmax > fmin:
                    frame = (255 * (frame - fmin) / (fmax - fmin)).astype(np.uint8)
                else:
                    frame = np.clip(frame, 0, 1).astype(np.uint8) * 255
            else:
                frame = np.clip(frame, 0, 255).astype(np.uint8)

        pil = Image.fromarray(frame, mode=mode)
        buf = io.BytesIO()
        pil.save(buf, format='JPEG', quality=85)
        buf.seek(0)
        return buf

@bp.route('/list_files', methods=['POST'])
def list_files():
    data = request.get_json() or {}
    folder = data.get('folder', '')
    try:
        p = validate_path(folder)
    except Exception as e:
        return jsonify({"error": str(e)}), 400

    if not p.exists() or not p.is_dir():
        return jsonify({"error": "folder not found"}), 404

    files = sorted([f.name for f in p.iterdir() if f.is_file() and f.suffix.lower() in ('.h5', '.hdf5')])
    return jsonify({"files": files})

@bp.route('/file_info', methods=['POST'])
def file_info():
    data = request.get_json() or {}
    folder = data.get('folder', '')
    filename = data.get('filename', '')
    try:
        p = validate_path(folder) / filename
        p = p.resolve()
    except Exception as e:
        return jsonify({"error": str(e)}), 400

    if not p.exists():
        return jsonify({"error": "file not found"}), 404

    info = {}
    try:
        with h5py.File(p, 'r') as hf:
            det = find_datasets(hf)
            info['detected_joints'] = det['joints']
            info['detected_rgbs'] = det['rgbs']
            info['datasets'] = {}
            for name in det['joints'] + det['rgbs']:
                d = hf[name]
                info['datasets'][name] = {
                    "shape": d.shape,
                    "dtype": str(d.dtype),
                    "ndim": d.ndim
                }
            rgb_counts = {}
            for name in det['rgbs']:
                d = hf[name]
                if d.ndim == 4:
                    rgb_counts[name] = int(d.shape[0])
                elif d.ndim == 3:
                    rgb_counts[name] = int(d.shape[0])
                else:
                    rgb_counts[name] = int(d.shape[0])
            info['rgb_counts'] = rgb_counts
    except Exception as e:
        return jsonify({"error": "failed to open hdf5: " + str(e)}), 500

    return jsonify(info)

@bp.route('/frame', methods=['GET'])
def frame():
    folder = request.args.get('folder', '')
    filename = request.args.get('filename', '')
    dataset = request.args.get('dataset', '')
    idx = int(request.args.get('idx', '0'))
    try:
        p = validate_path(folder) / filename
        p = p.resolve()
    except Exception as e:
        return jsonify({"error": str(e)}), 400

    if not p.exists():
        return jsonify({"error": "file not found"}), 404

    try:
        buf = extract_frame_from_dataset(p, dataset, idx)
    except Exception as e:
        return jsonify({"error": "frame error: " + str(e)}), 500

    return send_file(buf, mimetype='image/jpeg')

@bp.route('/get_frame')
def get_frame():
    folder = request.args.get('folder')
    filename = request.args.get('filename')
    dataset = request.args.get('dataset')
    idx = int(request.args.get('idx', 0))

    try:
        with h5py.File(f"{folder}/{filename}", 'r') as f:
            data = f[dataset][idx]  # 假设是 ndarray, 形状(H, W, C)，uint8格式RGB
            data_gbr = data[:, :, [2, 1, 0]]
            # 转成图像
            img = Image.fromarray(data_gbr)

            # 编码成JPEG二进制流
            buf = io.BytesIO()
            img.save(buf, format='JPEG')
            buf.seek(0)

            return send_file(buf, mimetype='image/jpeg')
    except Exception as e:
        print(f"Error: {e}")
@bp.route('/joints', methods=['POST'])
def joints():
    data = request.get_json() or {}
    folder = data.get('folder','')
    filename = data.get('filename','')
    dataset = data.get('dataset','')
    try:
        p = validate_path(folder) / filename
        p = p.resolve()
    except Exception as e:
        return jsonify({"error": str(e)}), 400

    if not p.exists():
        return jsonify({"error": "file not found"}), 404

    try:
        with h5py.File(p, 'r') as hf:
            if dataset not in hf:
                return jsonify({"error":"dataset not found"}), 404
            arr = hf[dataset][...]
            max_return = 2000
            data_out = arr.tolist() if arr.size <= max_return else arr[:max_return].tolist()
            return jsonify({"shape": arr.shape, "dtype": str(arr.dtype), "data": data_out})
    except Exception as e:
        return jsonify({"error": "failed to read joints: "+str(e)}), 500
