#!/usr/bin/env bash
#
# Common ROS 2 env for host + Jetson (Domain 2, CycloneDDS)

# 1) Clean old settings
unset ROS_DOMAIN_ID
unset RMW_IMPLEMENTATION
unset CYCLONEDDS_URI

# 2) Source ROS distro (auto-detect)
if [ -f /opt/ros/foxy/setup.bash ]; then
  # Host / containers with Foxy
  source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  # Jetson with Humble
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  # If you ever use Jazzy natively
  source /opt/ros/jazzy/setup.bash
fi

# 3) Your workspaces (optional, only if they exist)
[ -f "$HOME/workspace/install/setup.bash" ] && source "$HOME/workspace/install/setup.bash"
[ -f "$HOME/rqs_iai_ws/install/setup.bash" ] && source "$HOME/rqs_iai_ws/install/setup.bash"

# 4) Set CycloneDDS + domain
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=5

# Different config paths on host vs Jetson:
if [ -f /etc/cyclonedds/cyclonedds.xml ]; then
  # Host
  export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
elif [ -f /etc/cyclonedds.xml ]; then
  # Jetson
  export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
fi

echo "ROS 2 env:"
echo "  ROS_DISTRO        = ${ROS_DISTRO:-<unknown>}"
echo "  ROS_DOMAIN_ID     = ${ROS_DOMAIN_ID:-<unset>}"
echo "  RMW_IMPLEMENTATION= ${RMW_IMPLEMENTATION:-<unset>}"
echo "  CYCLONEDDS_URI    = ${CYCLONEDDS_URI:-<unset>}"
