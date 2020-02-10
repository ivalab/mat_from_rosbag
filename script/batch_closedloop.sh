# export CLOSEDLOOP_DIR=/media/yipuzhao/1399F8643500EDCD/ClosedNav_v4
export CLOSEDLOOP_DIR=/media/yipuzhao/651A6DA035A51611/Exp_ClosedLoop/Simulation/pc
# export CLOSEDLOOP_DIR=/mnt/DATA/tmp/ClosedNav/demo/ADIS16448

# find ${CLOSEDLOOP_DIR} -type f -name '*.bag'
shopt -s globstar
for file in ${CLOSEDLOOP_DIR}/**/*.bag
do
  echo "$file"
  rosrun mat_from_rosbag mat_from_rosbag ${file} /visual/odom /odom_sparse /desired_path
done 

