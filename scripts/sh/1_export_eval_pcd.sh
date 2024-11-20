data_folder=/home/kin/DATA_HDD/Dynamic_Papers_assets/Benchmark_data
benchmarks_code_path=/home/kin/code_online/DynamicMap_Benchmark
# ====> export output for eval
min_dis=0.05 # since all voxel-based methods resolution is 0.1 / 2 = 0.05
for seq_num in 00 05 semindoor av
do
    for method_name in removert erasor octomap octomapf octomapfg
    do
        data_path=${data_folder}/${seq_num}
        # if build file not exist, build it
        if [ ! -f ${benchmarks_code_path}/scripts/build/export_eval_pcd ]; then
            cd ${benchmarks_code_path}/scripts && cmake -B build && cmake --build build && cd ${benchmarks_code_path}
        fi
        ${benchmarks_code_path}/scripts/build/export_eval_pcd ${data_path} ${method_name}_output.pcd ${min_dis}
        echo "exported ${method_name} seq ${seq_num} check ${method_name}_exportGT.pcd"
    done
done