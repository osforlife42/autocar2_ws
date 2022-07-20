#! /bin/bash
# create a folder with the same basename as the model 
# (without .urdf extension where the model will be inserted to)
model_sdf_folder=`basename ${1} .urdf`
gz sdf -p ${1} > ${model_sdf_folder}/model.sdf 