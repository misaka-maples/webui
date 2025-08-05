import os
import h5py
import numpy as np
camera_names = ['left_wrist','top','right_wrist']

class GENERATOR_HDF5:
    def __init__(self):
        pass
    def save_hdf5(self,data_dict,path_to_save_hdf5,episode_idx,compressed=True,arm_name='all'):
        os.makedirs(path_to_save_hdf5, exist_ok=True)
        self.dataset_path = os.path.join(path_to_save_hdf5, f'episode_{episode_idx}.hdf5')

        try:
            with h5py.File(self.dataset_path, 'w') as root:
                root.attrs['sim'] = True
                obs = root.create_group('observations')
                images_group = obs.create_group('images')
                gp = root.create_group('gp')

                # 创建每个相机的数据集并写入数据
                for cam_name in camera_names:
                    if f'/observations/images/{cam_name}' in data_dict:
                        try:
                            cam_data = np.array(data_dict[f'/observations/images/{cam_name}'])
                            print(f"Saving image for {cam_name}, shape: {cam_data.shape}")  # 打印图片数据的尺寸
                            images_group.create_dataset(
                                cam_name.split('/')[-1],
                                data=cam_data,
                                dtype='uint8',
                                compression="gzip", 
                                compression_opts= 4 if compressed else 0
                            )
                        except Exception as e:
                            print(f"Error saving image data for camera {cam_name}: {e}")

                # 写入 qpos 数据
                if '/observations/qpos' in data_dict:
                    if 'qpos' in obs:
                        print("Dataset 'qpos' already exists. Updating it.")
                        del obs['qpos']
                    if arm_name == 'all':
                        qpos_data = np.array(data_dict['/observations/qpos'])
                        print(f"Saving qpos, shape: {qpos_data.shape}")
                        obs.create_dataset(
                            'qpos',
                            data=qpos_data,
                            dtype='float32'
                        )
                    elif arm_name == 'left_arm':
                        qpos_data = np.array(data_dict['/observations/qpos'])[:,:8]
                        print(f"Saving qpos, shape: {qpos_data.shape}")
                        obs.create_dataset(
                            'qpos',
                            data=qpos_data,
                            dtype='float32'
                        )
                    elif arm_name == 'right_arm':
                        qpos_data = np.array(data_dict['/observations/qpos'])[:,8:]
                        print(f"Saving qpos, shape: {qpos_data.shape}")
                        obs.create_dataset(
                            'qpos',
                            data=qpos_data,
                            dtype='float32'
                        )
                # 写入 action 数据
                if '/action' in data_dict:
                    if 'action' in root:
                        print("Dataset 'action' already exists. Updating it.")
                        del root['action']
                    if arm_name == 'all':
                        
                        action_data = np.array(data_dict['/action'])
                        print(f"Saving action, shape: {action_data.shape}")
                        root.create_dataset(
                            'action',
                            data=action_data,
                            dtype='float32'
                        )
                    elif arm_name == 'left_arm':
                        action_data = np.array(data_dict['/action'])[:,:8]
                        print(f"Saving action, shape: {action_data.shape}")
                        root.create_dataset(
                            'action',
                            data=action_data,
                            dtype='float32'
                        )
                    elif arm_name == 'right_arm':
                        action_data = np.array(data_dict['/action'])[:,8:]
                        print(f"Saving action, shape: {action_data.shape}")
                        root.create_dataset(
                            'action',
                            data=action_data,
                            dtype='float32'
                        )
                # 保存 gpstate, gppos, gpforce 数据
                if '/gp/gppos' in data_dict:
                    if 'gppos' in gp:
                        print("Dataset 'gppos' already exists. Updating it.")
                        del gp['gppos']
                    try:
                        gppos_data = np.array([int(x, 16) for x in data_dict['/gp/gppos']], dtype='int32')
                        print(f"Saving gppos, length: {len(gppos_data)}")
                        gp.create_dataset(
                            'gppos',
                            data=gppos_data
                        )
                    except Exception as e:
                        print(f"Error saving gppos data: {e}")
                        return

                if '/gp/gpstate' in data_dict:
                    if 'gpstate' in gp:
                        print("Dataset 'gpstate' already exists. Updating it.")
                        del gp['gpstate']
                    try:
                        gpstate_data = np.array([int(x, 16) for x in data_dict['/gp/gpstate']], dtype='int32')
                        print(f"Saving gpstate, length: {len(gpstate_data)}")
                        gp.create_dataset(
                            'gpstate',
                            data=gpstate_data
                        )
                    except Exception as e:
                        print(f"Error saving gpstate data: {e}")
                        return

                if '/gp/gpforce' in data_dict:
                    if 'gpforce' in gp:
                        print("Dataset 'gpforce' already exists. Updating it.")
                        del gp['gpforce']
                    try:
                        gpforce_data = np.array([int(x, 16) for x in data_dict['/gp/gpforce']], dtype='int32')
                        print(f"Saving gpforce, length: {len(gpforce_data)}")
                        gp.create_dataset(
                            'gpforce',
                            data=gpforce_data
                        )
                    except Exception as e:
                        print(f"Error saving gpforce data: {e}")
                        return

                # 强制刷新文件，确保数据写入
                root.flush()

        except Exception as e:
            print(f"Error during saving hdf5 file: {e}")
        
        print(f"\033[92mData saved to {self.dataset_path}\033[0m")
