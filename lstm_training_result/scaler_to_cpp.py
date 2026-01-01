import joblib
import numpy as np

SCALER_PATH = "feature_scaler.pkl"
OUTPUT_HEADER = "feature_scaler.h"

scaler = joblib.load(SCALER_PATH)

scale = scaler.scale_
min_ = scaler.min_
feature_dim = scale.shape[0]

def array_to_c(arr, name):
    values = ", ".join(f"{x:.8f}f" for x in arr)
    return f"static const float {name}[{len(arr)}] = {{ {values} }};\n"

with open(OUTPUT_HEADER, "w") as f:
    f.write("#pragma once\n\n")
    f.write(f"#define FEATURE_DIM {feature_dim}\n\n")
    f.write(array_to_c(scale, "SCALER_SCALE"))
    f.write(array_to_c(min_, "SCALER_MIN"))

print("Scaler exported to:", OUTPUT_HEADER)
