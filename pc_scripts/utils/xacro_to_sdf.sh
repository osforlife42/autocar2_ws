#! /bin/bash
model_name=`basename ${1} .xacro`
xacro ${1} > ${model_name}.urdf 

model_sdf_folder=${model_name}
gz sdf -p ${model_name}.urdf > ${model_sdf_folder}/model.sdf 