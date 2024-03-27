import numpy as np
import os

class bc:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def cnt_staticAdynamic(np_data : np.ndarray):
    dynamic_cnt = np.count_nonzero(np_data[:,3])
    static_cnt = np_data.shape[0] - dynamic_cnt
    num_dict = {'static': static_cnt, 'dynamic': dynamic_cnt}
    return num_dict

def check_file_exists(file_path):
    if os.path.exists(file_path) is False:
        print("No such file: ", file_path)
        print("We must have this file to evaluate.")
        print("Exit now... check your path.")
        exit(0)
    else:
        return file_path
    

def filterOutRange(points: np.array, labels: np.array, max_range=50, min_range=3):
    """
    points: [N, 4] [x, y, z, intensity]
    labels: [N, 1] instance & semantic labels
    """
    sem_label = labels & 0xFFFF
    points_xyz = points[:, :3]
    distance = np.linalg.norm(points_xyz, axis=1)
    valid_mask = distance < max_range

    # HARD CODE HERE: since ego vehicle pts is labeled as unlabeled/outlier we will set to dynamic pts also
    ego_pts_mask = (distance < min_range) & ((sem_label == 0) | (sem_label == 1))
    sem_label[ego_pts_mask] = 252

    return points[valid_mask], sem_label[valid_mask]
    