import numpy as np
import pandas as pd
import joblib
import matplotlib.pyplot as plt
import tensorflow as tf

from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, LSTM, Dense, Bidirectional, LayerNormalization, Dropout, Reshape
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import EarlyStopping

from sklearn.metrics import confusion_matrix, classification_report
import seaborn as sns

# =========================
# CONFIG
# =========================
WINDOW_SIZE = 10
FUTURE_STEPS = 4  # Predict t+1 through t+4
BATCH_SIZE = 128
EPOCHS = 50
LEARNING_RATE = 3e-4

DATASET_PATH = "dataset_preprocessed.csv"
SCALER_PATH = "feature_scaler.pkl"
LABEL_COL = "robot_action"

tf.config.set_visible_devices([], "GPU") # Keep on CPU as per your setup

# =========================
# LOAD & SCALE DATA
# =========================
df = pd.read_csv(DATASET_PATH)
scaler = joblib.load(SCALER_PATH)

features = df[scaler.feature_names_in_]
labels = df[LABEL_COL].astype(int)

X_scaled = scaler.transform(features).astype(np.float32)
y = labels.values.astype(np.int32)

num_classes = int(y.max()) + 1

# =========================
# SLIDING WINDOW (Multi-Step)
# =========================
X_seq, y_seq = [], []

for i in range(len(X_scaled) - WINDOW_SIZE - FUTURE_STEPS + 1):
    X_seq.append(X_scaled[i : i + WINDOW_SIZE])
    y_seq.append(y[i + WINDOW_SIZE : i + WINDOW_SIZE + FUTURE_STEPS])

X_seq = np.array(X_seq, dtype=np.float32)
y_seq = np.array(y_seq, dtype=np.int32)

# =========================
# THE FIX: TRAIN / VAL SPLIT
# =========================
# We split at 80% mark chronologically
split_idx = int(0.8 * len(X_seq))

X_train, X_val = X_seq[:split_idx], X_seq[split_idx:]
y_train, y_val = y_seq[:split_idx], y_seq[split_idx:]

print(f"Train shape: {X_train.shape}, Val shape: {X_val.shape}")

# =========================
# MODEL ARCHITECTURE
# =========================
inputs = Input(shape=(WINDOW_SIZE, X_seq.shape[2]))

# Layer 1
x = Bidirectional(LSTM(256, activation="tanh",
            recurrent_activation="sigmoid", return_sequences=True, dropout=0.3, implementation=2))(inputs)
#x = LayerNormalization()(x)

# Layer 2
x = Bidirectional(LSTM(128, activation="tanh",
            recurrent_activation="sigmoid", return_sequences=False, dropout=0.3, implementation=2))(x)
#x = LayerNormalization()(x)

# Fully Connected
x = Dense(64, activation="relu")(x)
x = Dropout(0.3)(x)

# Output Head: Predict (Steps * Classes) then Reshape
x = Dense(FUTURE_STEPS * num_classes, name="logits_flat")(x)
outputs = Reshape((FUTURE_STEPS, num_classes), name="logits_sequence")(x)

model = Model(inputs, outputs)


model.compile(
    optimizer=Adam(learning_rate=LEARNING_RATE),
    loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
    metrics=["accuracy"]
)
model.summary()

# =========================
# CALLBACKS
# =========================
early_stopping = EarlyStopping(
    monitor='val_loss',
    patience=7,              # Number of epochs with no improvement after which training will be stopped
    verbose=1,
    mode='min',              # We want to minimize loss
    restore_best_weights=True # Revert to the best weights found during training
)

# =========================
# TRAIN & EVALUATE
# =========================
history = model.fit(
    X_train,
    y_train,
    validation_data=(X_val, y_val),
    epochs=EPOCHS,
    batch_size=BATCH_SIZE,
    callbacks=[early_stopping], # Add callback here
    verbose=1
)

model.save_weights("model.weights.h5")
print("Weights saved")

# Per-step Accuracy Calculation
val_logits = model.predict(X_val, verbose=0)
val_preds = np.argmax(val_logits, axis=-1)

step_accuracies = [np.mean(val_preds[:, i] == y_val[:, i]) for i in range(FUTURE_STEPS)]

# =========================
# VISUALIZATION
# =========================
plt.figure(figsize=(15, 5))

# Plot Loss
plt.subplot(1, 2, 1)
plt.plot(history.history['loss'], label='Train')
plt.plot(history.history['val_loss'], label='Val')
plt.title("Model Loss")
plt.legend()

# Plot Per-Step Accuracy
plt.subplot(1, 2, 2)
steps = [f"t+{i+1}" for i in range(FUTURE_STEPS)]
plt.bar(steps, step_accuracies, color='skyblue', edgecolor='navy')
plt.ylim(0, 1)
plt.title("Accuracy per Prediction Step")
for i, v in enumerate(step_accuracies):
    plt.text(i, v + 0.02, f"{v:.2%}", ha='center')

plt.tight_layout()
plt.show()

# =========================
# CONFUSION MATRIX
# =========================

def plot_cm(y_true, y_pred, step_name, ax):
    cm = confusion_matrix(y_true, y_pred)
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', ax=ax, cbar=False)
    ax.set_title(f"Confusion Matrix: {step_name}")
    ax.set_xlabel("Predicted Label")
    ax.set_ylabel("True Label")

fig, axes = plt.subplots(1, 2, figsize=(14, 6))

# Step t+1 (Index 0)
plot_cm(y_val[:, 0], val_preds[:, 0], "Immediate Next Action (t+1)", axes[0])

# Step t+4 (Index 3)
plot_cm(y_val[:, 3], val_preds[:, 3], "Far Future Action (t+4)", axes[1])

plt.tight_layout()
plt.show()

# Print detailed classification report for the final step (t+4)
print("\nClassification Report for t+4:")
print(classification_report(y_val[:, 3], val_preds[:, 3]))