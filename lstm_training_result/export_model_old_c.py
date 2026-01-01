import tensorflow as tf
from tensorflow.python.framework import graph_util
from tensorflow.python.framework import graph_io
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2

import joblib
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, LSTM, Dense, Bidirectional, Reshape
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2

# =========================
# CONFIG
# =========================
WINDOW_SIZE = 10
PRED_HORIZON = 4
NUM_CLASSES = 6
SCALER_PATH = "feature_scaler.pkl"
WEIGHTS_PATH = "model.weights.h5"
OUTPUT_PB = "model.pb"

tf.config.set_visible_devices([], "GPU")

# Load Scaler
scaler = joblib.load(SCALER_PATH)
FEATURE_DIM = scaler.n_features_in_

# =========================
# BUILD EXPORT-FRIENDLY MODEL
# =========================
def build_export_model(feature_dim):
    inputs = Input(shape=(WINDOW_SIZE, feature_dim), name="input_1")
    
    # CHANGE: implementation=1 is the key fix for the AssignVariableOp error.
    # It avoids the complex 'resource' management of implementation=2.
    x = Bidirectional(LSTM(256, return_sequences=True, implementation=1))(inputs)
    x = Bidirectional(LSTM(128, return_sequences=False, implementation=1))(x)
    
    x = Dense(64, activation="relu")(x)
    x = Dense(PRED_HORIZON * NUM_CLASSES, name="logits_flat")(x)
    outputs = Reshape((PRED_HORIZON, NUM_CLASSES), name="output_1")(x)
    
    return Model(inputs, outputs)

# 1. Initialize with implementation=1
model = build_export_model(FEATURE_DIM)

# 2. Load the weights from your trained implementation=2 model.
# (Weights are compatible between implementations)
print("Loading weights...")
model.load_weights(WEIGHTS_PATH)
model.trainable = False

# ... (Keep your model building and weight loading code the same as before) ...

def save_for_old_c_api(model, output_path):
    # 1. Convert to Concrete Function
    full_model = tf.function(lambda x: model(x, training=False))
    full_model = full_model.get_concrete_function(
        tf.TensorSpec(model.inputs[0].shape, model.inputs[0].dtype, name="input_1"))

    # 2. Freeze Variables to Constants
    frozen_func = convert_variables_to_constants_v2(full_model)
    graph_def = frozen_func.graph.as_graph_def()

    # 3. CRITICAL: Downgrade/Clear Versioning and Simplify
    # We strip the versioning so the older C API doesn't reject it immediately
    graph_def.versions.producer = 0 
    graph_def.versions.min_consumer = 0

    # 4. Remove 'While' loop metadata that causes the 'LoopControlInputs' error
    # This forces the graph to be interpreted as a flat set of operations
    for node in graph_def.node:
        if "while" in node.name:
            # Clear out attributes that the old C API doesn't understand
            if "_control_flow_contexts" in node.attr:
                del node.attr["_control_flow_contexts"]

    # 5. Save the file
    tf.io.write_graph(graph_def, ".", output_path, as_text=False)
    print(f"Exported version-stripped model to {output_path}")

# Run the export
save_for_old_c_api(model, "model.pb")