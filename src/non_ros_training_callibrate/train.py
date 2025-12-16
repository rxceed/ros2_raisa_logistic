import pandas as pd
import numpy as np
import m2cgen as m2c
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score

# 1. LOAD DATA
# ---------------------------------------------------------
filename = '../../reeman_pozyx_logs/log.csv'
input_cols = ['pozyx_x', 'pozyx_y']
output_cols = ['reeman_x', 'reeman_y']
print(f"Loading data from '{filename}'...")

try:
    df = pd.read_csv(filename)
    X = df[input_cols].values
    y = df[output_cols].values
except FileNotFoundError:
    print("CSV not found. Generating dummy data...")
    data = np.random.rand(1000, 2) * 100
    X = data
    # Create dummy targets
    y = np.zeros((1000, 2))
    y[:, 0] = X[:, 0] * 2     # Target x2
    y[:, 1] = X[:, 1] + 5     # Target y2

# 2. SPLIT DATA
# ---------------------------------------------------------
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 3. TRAIN TWO SEPARATE MODELS
# ---------------------------------------------------------
print("Training Model X (for x2 output)...")
model_x = RandomForestRegressor(n_estimators=10, max_depth=5, random_state=42)
model_x.fit(X_train, y_train[:, 0]) # Train on first output column

print("Training Model Y (for y2 output)...")
model_y = RandomForestRegressor(n_estimators=10, max_depth=5, random_state=42)
model_y.fit(X_train, y_train[:, 1]) # Train on second output column

# 4. EVALUATE
# ---------------------------------------------------------
pred_x = model_x.predict(X_test)
pred_y = model_y.predict(X_test)
print(f"Model X R2: {r2_score(y_test[:, 0], pred_x):.4f}")
print(f"Model Y R2: {r2_score(y_test[:, 1], pred_y):.4f}")

# 5. EXPORT AND STITCH C CODE
# ---------------------------------------------------------
print("Generating C code...")

# Generate code for Model X
c_code_x = m2c.export_to_c(model_x)
# Rename the default 'score' function to 'predict_x'
c_code_x = c_code_x.replace("double score(double * input)", "double predict_x(double * input)")

# Generate code for Model Y
c_code_y = m2c.export_to_c(model_y)
# Rename the default 'score' function to 'predict_y'
c_code_y = c_code_y.replace("double score(double * input)", "double predict_y(double * input)")

# Create the final header file content
header_content = f"""
#ifndef INFERENCE_MODEL_H
#define INFERENCE_MODEL_H
#include <math.h>

// --- MODEL FOR X OUTPUT ---
{c_code_x}

// --- MODEL FOR Y OUTPUT ---
{c_code_y}

// --- COMBINED WRAPPER ---
// input: array [x1, y1]
// output: array [x2, y2]
void predict_all(double * input, double * output) {{
    output[0] = predict_x(input);
    output[1] = predict_y(input);
}}

#endif // INFERENCE_MODEL_H
"""

with open("model.h", "w") as f:
    f.write(header_content)

print("Success! 'model.h' created with 'predict_all' function.")