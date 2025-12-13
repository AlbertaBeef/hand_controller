import sys
import os
import cv2
import numpy as np

sys.path.append('/usr/local/rpp/lib')
import pyrt as trt

log = trt.Logger(trt.Logger.INTERNAL_ERROR)

def volume(obj):
    vol = 1
    for elem in obj:
        vol *= elem
    return vol
    
#model_path = './models'
#model_name = 'palm_detection_lite_sim.onnx'
#model_file = os.path.join(model_path, model_name)
model_file = sys.argv[1]
#int8 = True
int8 = False

builder = trt.Builder(log)
config = builder.createBuilderConfig()
if int8:
    config.setFlag(trt.BuilderFlag.INT8)
    int8_calibrator = trt.Int8EntropyCalibrator()
    config.setInt8Calibrator(int8_calibrator)
else:
    config.setFlag(trt.BuilderFlag.BF16)

net = builder.createNetwork()

print('Create onnx parser.')
parser = trt.OnnxParser(net, log)

print("Parsing model : ",model_file)
with open(model_file, "rb") as model:
    if not parser.parse(model.read()):
        print("ERROR: Failed to parse the ONNX file.")
        for error in range(parser.num_errors):
            print(parser.get_error(error))

print(f"   net ({model_file})")
print("      name = ",net.name)
print("      num_inputs = ",net.num_inputs)
print("      num_layers = ",net.num_layers)
print("      num_outputs = ",net.num_outputs)
print("      IsInputProcDisabled() = ",net.IsInputProcDisabled())
print("      IsOutputProcDisabled() = ",net.IsOutputProcDisabled())

num_inputs = net.num_inputs
num_outputs = net.num_outputs
num_layers = net.num_layers

bindings = []
input_dimensions = []
output_dimensions = []
input_bindings = []
output_bindings = []



print('Initialize IO buffers')
for i in range(net.num_inputs):
    inputx = net.get_input(i)
    print(f"   net.get_input({i})")
    print("      name = ",inputx.name)
    print("      dimensions = ",inputx.dimensions)
    print("      dataType = ",inputx.dataType)
    print("      isNetworkInput() = ",inputx.isNetworkInput())
    print("      isNetworkOutput() = ",inputx.isNetworkOutput())
    input_dimension = inputx.dimensions
    input_size = volume(input_dimension) * 4
    print("      input_dimension = ",input_dimension)
    print("      input_size = ",input_size)
    input_binding = trt.DeviceAllocation(input_size)
    #
    input_dimensions.append(input_dimension)
    input_bindings.append(input_binding)
    bindings.append(int(input_binding))

for i in range(net.num_outputs):
    outputx = net.get_output(i)
    print(f"   net.get_output({i})")
    print("      name = ",outputx.name)
    print("      dimensions = ",outputx.dimensions)
    print("      dataType = ",outputx.dataType)
    print("      isNetworkInput() = ",outputx.isNetworkInput())
    print("      isNetworkOutput() = ",outputx.isNetworkOutput())
    output_dimension = outputx.dimensions
    output_size = volume(output_dimension) * 4
    print("      output_dimension = ",output_dimension)
    print("      output_size = ",output_size)
    output_binding = trt.DeviceAllocation(output_size)
    #
    output_dimensions.append(output_dimension)
    output_bindings.append(output_binding)
    bindings.append(int(output_binding))
    
print('Build IEngine')
engine = builder.build_EngineWithConfig(net, config)
if engine is None:
    #return
    exit()

print("Create execution context")
context = engine.createExecutionContext()

print('Prepare Input')
rng = np.random.default_rng()
print(f"   Creating random test data")
test_data = rng.random(input_dimension,dtype=np.float32)
print("      test_data.shape = ",test_data.shape)
print("      test_data.dtype = ",test_data.dtype)
input_binding = input_bindings[0]
input_binding.copy_from_numpy(test_data)

# Inference (warmup)
print("Execute (warmup)")
context.execute(1, bindings)

print("Done !")

