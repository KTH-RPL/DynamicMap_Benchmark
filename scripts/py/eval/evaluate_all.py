
'''
# Created: 2023-04-16 18:03
# @Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
# @Author: Kin ZHANG  (https://kin-zhang.github.io/)

# If you find this repo helpful, please cite the respective publication in DynamicBenchmark.
# This script is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
'''

import numpy as np
from tabulate import tabulate

from time import time
import sys, os, math
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)
from utils.pcdpy3 import load_pcd
from utils import cnt_staticAdynamic, check_file_exists, bc

# TODO: Change the parameters below to your own settings ====>>
Result_Folder = "/home/kin/data/Dynamic_Papers_assets/BenchmarkPaper"
algorithms = ["removert", "erasor", "octomap", "octomapg", "octomapfg"]
all_seqs = ["00", "05", "av2", "semindoor"]
# TODO: Change the parameters below to your own settings <<===

if __name__ == "__main__":
    st_time = time()
    for seq in all_seqs:
        gt_pcd_path = f"{Result_Folder}/{seq}/gt_cloud.pcd"
        gt_pc_ = load_pcd(check_file_exists(gt_pcd_path))
        num_gt = cnt_staticAdynamic(gt_pc_.np_data)
        printed_data = []
        for algo in algorithms:
            et_pcd_path = f"{Result_Folder}/{seq}/eval/{algo}_output_exportGT.pcd"
            et_pc_ = load_pcd(check_file_exists(et_pcd_path))

            assert et_pc_.np_data.shape[0] == gt_pc_.np_data.shape[0] , \
                "Error: The number of points in et_pc_ and gt_pc_ do not match.\
                \nThey must match for evaluation, if not Please run `export_eval_pcd`."
            
            num_et = cnt_staticAdynamic(et_pc_.np_data)

            correct_static = np.count_nonzero((et_pc_.np_data[:,3] == 0) * (gt_pc_.np_data[:,3] == 0))
            missing_static = num_gt['static'] - correct_static
            correct_dynamic = np.count_nonzero((et_pc_.np_data[:,3] == 1) * (gt_pc_.np_data[:,3] == 1))
            missing_dynamic = num_gt['dynamic'] - correct_dynamic

            SA = float(correct_static) / float(num_gt['static']) * 100
            DA = float(correct_dynamic) / float(num_gt['dynamic']) * 100
            AA = math.sqrt(SA * DA)
            HA = 2 * SA * DA / (SA + DA)
            printed_data.append([algo] + [num_et['static'], num_et['dynamic'], SA, DA,  AA, HA])
            # break
        print(f"Evaluation results in seq {bc.HEADER}{seq}{bc.ENDC}")
        print(tabulate(printed_data, headers=['Methods', '# static', '# dynamics', 'SA [%] ↑', 'DA [%] ↑', 'AA [%] ↑', 'HA [%] ↑'], tablefmt='orgtbl'))
    print(f"Time cost: {(time() - st_time):.2f}s")