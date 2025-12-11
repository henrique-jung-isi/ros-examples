set -e

shopt -s expand_aliases

source ~/.bash_aliases

ros-source
ws-source
ros2 run cpp_pubsub listener
