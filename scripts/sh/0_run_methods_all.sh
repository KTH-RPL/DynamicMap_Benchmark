data_folder=/home/kin
benchmarks_path=/home/kin/DynamicMap_Benchmark/methods

# ====> build methods, you can also build them manually only once
for method in octomap removert erasor
do
    cd ${benchmarks_path}/${method}
    cmake -B build && cmake --build build
done

# ====> run methods
for seq_num in 00 05 semindoor av2
do
    data_path=${data_folder}/data/${seq_num}

    echo "Processing sequence ${seq_num} ... in octomap [origin, w G, w GF]"
    ${benchmarks_path}/octomap/build/octomap_run ${data_path} ${benchmarks_path}/octomap/assets/config_g.yaml -1
    ${benchmarks_path}/octomap/build/octomap_run ${data_path} ${benchmarks_path}/octomap/assets/config_fg.yaml -1
    ${benchmarks_path}/octomap/build/octomap_run ${data_path} ${benchmarks_path}/octomap/assets/config.yaml -1

    echo "Processing sequence ${seq_num} ... in remover and erasor"
    if [ "${seq_num}" \> "av2" ]; then
        ${benchmarks_path}/removert/build/removert_run ${data_path} ${benchmarks_path}/removert/config/params_av.yaml -1
        ${benchmarks_path}/ERASOR/build/erasor_run ${data_path} ${benchmarks_path}/ERASOR/config/av2.yaml -1
    else
        ${benchmarks_path}/removert/build/removert_run ${data_path} ${benchmarks_path}/removert/config/params_kitti.yaml -1
        ${benchmarks_path}/ERASOR/build/erasor_run ${data_path} ${benchmarks_path}/ERASOR/config/seq_${seq_num}.yaml -1
    fi
done