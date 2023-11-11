#!/bin/bash

robot=arcdog

build_urdf() {

  xacro_dir="../robots/quadrupedal_robot"
  urdf_dir="../urdf/quadrupedal_robot/$robot"

  if [ -d $xacro_dir ]; then
    echo "==== Building: $robot | Simulator: $1 ===="

    if [ "$1" == "none" ]; then
      xacro $xacro_dir/$robot.xacro simulator:="$1" >$urdf_dir/"$robot".urdf
      check_urdf $urdf_dir/$robot.urdf
    else
      xacro $xacro_dir/$robot.xacro simulator:="$1" >$urdf_dir/"$robot"_"$1".urdf
      check_urdf $urdf_dir/"$robot"_"$1".urdf
    fi

  else
    echo "ERROR: The xacro directory $xacro_dir does not exist."
    exit 1
  fi
}

## Get current folder and make sure it is *scripts*
curr_folder=${PWD##*/}
if [ "$curr_folder" != "scripts" ]; then
  echo "ERROR: you need to run the script from the arclab_robot_description/scripts directory."
  echo "$curr_folder"
  exit 1
fi

rm -f "$PWD"/../urdf/quadrupedal_robot/$robot/*.urdf
mkdir -p "$PWD"/../urdf/quadrupedal_robot/$robot

build_urdf ocs2
build_urdf test

exit 0