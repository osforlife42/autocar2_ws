#! /bin/bash
model_name=`basename ${1} .xacro`
xacro ${1} > ${model_name}.urdf 