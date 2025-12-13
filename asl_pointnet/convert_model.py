import torch

import os

device = "cpu"

from point_net import PointNet

model_path = './asl_pointnet'
model_name = 'point_net_1.pth'
model = torch.load(os.path.join(model_path, model_name),weights_only=False,map_location=device) 

input_image = torch.randn(1, 21, 3)   # batch_size=1, landmarks=21, coordinates=3 (x,y,z),  

onnx_name = 'asl_pointnet.onnx'
output_file_path = os.path.join(model_path, onnx_name)

torch.onnx.export(model,               # model to export
                  input_image,         # model input (or a tuple for multiple inputs)
                  output_file_path,    # where to save the model (can be a file or file-like object)
                  export_params=True,  # store the trained parameter weights inside the model file
                  opset_version=12,    # the ONNX version to export the model to
                  do_constant_folding=True,  # whether to execute constant folding for optimization
                  input_names = ['input'],   # the model's input names
                  output_names = ['output'] # the model's output names
                  #dynamic_axes={'input' : {0 : 'batch_size'},    # variable length axes
                  #              'output' : {0 : 'batch_size'}}
                  )
