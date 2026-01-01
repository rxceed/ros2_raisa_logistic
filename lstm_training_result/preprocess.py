import pandas as pd
import numpy as np
import joblib
from sklearn.preprocessing import MinMaxScaler

# =========================
# PATHS
# =========================
DATASET_PATH = "dataset.csv"
OUTPUT_DATASET_PATH = "dataset_preprocessed.csv"
SCALER_PATH = "feature_scaler.pkl"

# =========================
# LOAD DATASET
# =========================
df = pd.read_csv(DATASET_PATH)

print("Loaded dataset shape:", df.shape)
print("Columns:", df.columns.tolist())

# =========================
# ADD time_working LABEL
# =========================

time_working = []
current_time = 0

for _, row in df.iterrows():
    left_angle = row["left_leg_angle"]
    right_angle = row["right_leg_angle"]

    # Sitting condition
    sitting = (
        (left_angle < 10 or left_angle > 350) and
        (right_angle < 10 or right_angle > 350)
    )

    if sitting:
        current_time += 1   # each row = 1 second
    else:
        current_time = 0

    time_working.append(current_time)

df["time_working"] = time_working

print("Added time_working feature")


# =========================
# CONVERT ANGLES TO COSINE
# =========================
df["left_leg_cos"] = np.cos(np.deg2rad(df["left_leg_angle"]))
df["right_leg_cos"] = np.cos(np.deg2rad(df["right_leg_angle"]))

# Drop raw angle columns (recommended)
df = df.drop(columns=["left_leg_angle", "right_leg_angle", "timestamp"])

print("Converted leg angles to cosine")

# =========================
# ADJUST ROBOT ACTIONS
# =========================
# Rule: If action is 2 but y_absolute > 3, change action to 3
# =========================
# ADJUST ROBOT ACTIONS
# =========================
# 1. Change action 2 to 3 if y_absolute > 3
mask_y = (df["robot_action"] == 2) & (df["y_absolute"] > 3)
df.loc[mask_y, "robot_action"] = 3

# 2. Change all instances of action 6 to 5
df.loc[df["robot_action"] == 6, "robot_action"] = 5

print(f"Logic applied: {mask_y.sum()} actions changed (2->3).")
print(f"Logic applied: {(df['robot_action'] == 5).sum()} actions are now 5 (including original 6s).")

# =========================
# DEFINE FEATURES & LABEL
# =========================
LABEL_COL = "robot_action"

FEATURE_COLS = [
    "x_absolute",                # UWB X
    "y_absolute",                # UWB Y
    "left_leg_cos",
    "right_leg_cos",
    "time_working"
]

# Sanity check
missing = set(FEATURE_COLS + [LABEL_COL]) - set(df.columns)
if missing:
    raise ValueError(f"Missing required columns: {missing}")

# =========================
# FIT SCALER (DO NOT APPLY)
# =========================
scaler = MinMaxScaler()
scaler.fit(df[FEATURE_COLS])

joblib.dump(scaler, SCALER_PATH)

print("Scaler saved to:", SCALER_PATH)
print("Scaler n_features_in_:", scaler.n_features_in_)
print("Scaler feature order:", FEATURE_COLS)

# =========================
# SAVE UNSCALED DATASET
# =========================
df.to_csv(OUTPUT_DATASET_PATH, index=False)

print("===================================")
print("Preprocessing completed successfully")
print("Saved UNSCALED dataset to:", OUTPUT_DATASET_PATH)
print("Saved scaler to:", SCALER_PATH)
print("Final dataset shape:", df.shape)
print("Final columns:", df.columns.tolist())
print("===================================")
