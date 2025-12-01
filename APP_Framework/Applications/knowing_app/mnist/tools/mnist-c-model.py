#!/usr/bin/env python3

#tflite_file_path = 'mnist-default-quan.tflite'
tflite_file_path = 'mnist.tflite'
model_file_path = 'model.h'

tflite_file = open(tflite_file_path, 'rb')
tflite_data = tflite_file.read()
tflite_file.close()
tflite_array = [ '0x%02x' % byte for byte in tflite_data ]

model_content = '''unsigned char mnist_model[] = {
  %s
};
unsigned int mnist_model_len = %d;
'''
# 12 bytes in a line, the same with xxd
bytes_of_line = 12
model_data = (',\n  ').join([ (', ').join(tflite_array[i:i+bytes_of_line]) for i in range(0, len(tflite_array), bytes_of_line) ])
model_file = open(model_file_path, 'w')
model_file.write(model_content % (model_data, len(tflite_array)))
model_file.close()

