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