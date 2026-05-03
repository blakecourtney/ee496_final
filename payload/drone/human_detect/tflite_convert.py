# Save as convert.py and run: python convert.py
import os

model_path = "drone_model_quant.tflite"
header_path = "drone_model_quant.h"

with open(model_path, "rb") as f:
    model_data = f.read()

with open(header_path, "w") as f:
    f.write("#ifndef DRONE_MODEL_QUANT_H\n")
    f.write("#define DRONE_MODEL_QUANT_H\n\n")
    f.write(f"// Model size: {len(model_data)} bytes\n")
    f.write("const unsigned char g_model_data[] = {\n  ")
    
    # Convert bytes to hex
    hex_array = [f"0x{b:02x}" for b in model_data]
    
    # Write in chunks of 12 for readability
    for i in range(0, len(hex_array), 12):
        f.write(", ".join(hex_array[i:i+12]) + ",\n  ")
        
    f.write("\n};\n\n")
    f.write(f"const unsigned int g_model_data_len = {len(model_data)};\n\n")
    f.write("#endif // DRONE_MODEL_QUANT_H\n")

print(f"Successfully converted to {header_path}")