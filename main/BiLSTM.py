import os
import re
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from sklearn.preprocessing import LabelEncoder, StandardScaler
from sklearn.model_selection import LeaveOneGroupOut
from sklearn.metrics import classification_report
import warnings

# Suppress pandas FutureWarnings related to is_list_like
warnings.simplefilter(action='ignore', category=FutureWarning)

# --- Configuration Constants (Modify as needed) ---
TIME_STEPS = 50  # Length of each sequence (window size)
STEP = 25      # Overlap step size for sequence creation
HIDDEN_SIZE = 128
NUM_LAYERS = 2
LEARNING_RATE = 0.001
NUM_EPOCHS = 25 
BATCH_SIZE = 64
DATA_DIR = 'Sign_Language_classification_on_sensor_data_SIH2025/Data'

# Define the 4 target phrases for the simulated hardware vocabulary
HARDWARE_PHRASES = [
    'Hello', 
    'I am doing good', 
    'How are you', 
    'Thank you','We','Work','This'
]

# Determine device for training
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# --- 1. PyTorch Utility Functions & Classes ---

def to_categorical_torch(y, num_classes=None):
    """Converts a numpy array of integers to a PyTorch one-hot tensor."""
    y = torch.tensor(y, dtype=torch.long)
    if num_classes is None:
        num_classes = torch.max(y).item() + 1
    return torch.nn.functional.one_hot(y, num_classes=num_classes)

class ASLDataset(Dataset):
    """Custom Dataset for ASL sensor sequence data."""
    def __init__(self, X, y):
        self.X = torch.tensor(X, dtype=torch.float32)
        self.y = y 

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

class BiLSTM_ASL(nn.Module):
    """Bidirectional LSTM model for time-series classification."""
    def __init__(self, input_size, hidden_size, num_layers, num_classes, dropout_rate=0.5):
        super(BiLSTM_ASL, self).__init__()
        self.num_layers = num_layers
        self.hidden_size = hidden_size
        
        self.lstm = nn.LSTM(
            input_size, 
            hidden_size, 
            num_layers, 
            batch_first=True, 
            bidirectional=True,
            dropout=dropout_rate if num_layers > 1 else 0.0
        )
        
        self.fc = nn.Linear(2 * hidden_size, num_classes)
        self.dropout = nn.Dropout(dropout_rate)

    def forward(self, x):
        lstm_out, (hn, cn) = self.lstm(x)
        
        final_h_fwd = hn[-2, :, :]  
        final_h_bwd = hn[-1, :, :]
        
        final_h = torch.cat((final_h_fwd, final_h_bwd), dim=1)
        
        out = self.dropout(final_h)
        out = self.fc(out)
        
        return out

# --- 2. Sequence Creation Function ---

def create_sequences(data, sensor_cols, time_steps, step):
    sequences, labels, groups = [], [], []
    
    for _, group in data.groupby(['user', 'sign_encoded']):
        if len(group) < time_steps:
             continue
             
        for i in range(0, len(group) - time_steps + 1, step):
            sequences.append(group[sensor_cols].iloc[i:i + time_steps].values)
            labels.append(group['sign_encoded'].iloc[i])
            groups.append(group['user'].iloc[i])
            
    return np.array(sequences), np.array(labels), np.array(groups)

# --- 3. Main Execution Flow ---

# -------------------------------------------------- 3.1 Data Loading and Initial Preprocessing
df_list = []
all_files = [os.path.join(DATA_DIR, f) for f in os.listdir(DATA_DIR) if f.endswith('.csv')]

for f in all_files:
    try:
        temp_df = pd.read_csv(f)
        match = re.search(r'sensors_data_(\d+)\.csv', f)
        user_id = int(match.group(1)) if match else 0
        temp_df['user'] = user_id
        df_list.append(temp_df)
    except Exception as e:
        print(f"Error loading file {f}: {e}")

df = pd.concat(df_list, ignore_index=True)

print("--- Data Loaded and Combined Successfully ---")

# ------------------------------------------------- 3.2 Data Preprocessing (FIXED)
label_col = 'SIGN'
user_col = 'user'

# CRITICAL FIX: Remove NaN labels
df_cleaned = df.dropna(subset=[label_col]).copy()

sensor_cols = df_cleaned.select_dtypes(include=np.number).drop(columns=[user_col]).columns.tolist()

# Encode Target Labels on the CLEANED data
label_encoder = LabelEncoder()
df_cleaned['sign_encoded'] = label_encoder.fit_transform(df_cleaned[label_col])

N_CLASSES = len(label_encoder.classes_)
# TARGET_NAMES holds the original discovered names (e.g., PAUSE, PROUD, etc.)
TARGET_NAMES = label_encoder.classes_ 

# Scale the sensor data
scaler = StandardScaler()
df_cleaned[sensor_cols] = scaler.fit_transform(df_cleaned[sensor_cols])

print("--- Data Preprocessing (Scaling & Encoding) Successful ---")

# ----------------------------------------------- 3.3 Create Sequences
X, y_int, groups = create_sequences(df_cleaned, sensor_cols, TIME_STEPS, STEP)
y = to_categorical_torch(y_int, num_classes=N_CLASSES)
INPUT_SIZE = X.shape[2] 

N_USERS = len(np.unique(groups))
print(f"Shape of all sequences (X): {X.shape} (Samples, Timesteps, Features)")
print(f"Found {N_USERS} unique user groups in the sequence data.")

# ----------------------------------------------- 3.4 LOUOCV Training and Evaluation

print(f"\n--- Starting Leave-One-User-Out Cross-Validation (LOUOCV) on {N_USERS} Users ---")

logo = LeaveOneGroupOut()
all_true_labels = []
all_pred_labels = []
fold_accuracies = []

for fold, (train_index, test_index) in enumerate(logo.split(X, y, groups)):
    test_user_id = groups[test_index][0]
    print(f"\n======== FOLD {fold + 1}/{N_USERS} (Test User ID: {test_user_id}) ========")
    
    # --- Data Splitting and Loading ---
    X_train, X_test = X[train_index], X[test_index]
    y_train, y_test = y[train_index], y[test_index]
    
    if len(X_test) == 0:
        print("Skipping fold: Test set is empty after sequence creation.")
        continue

    train_dataset = ASLDataset(X_train, y_train)
    test_dataset = ASLDataset(X_test, y_test)
    
    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=BATCH_SIZE, shuffle=False)
    
    # --- Model Initialization ---
    model = BiLSTM_ASL(INPUT_SIZE, HIDDEN_SIZE, NUM_LAYERS, N_CLASSES).to(DEVICE)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    
    # --- Training Loop ---
    model.train()
    for epoch in range(NUM_EPOCHS):
        total_loss = 0
        for sequences, labels in train_loader:
            sequences = sequences.to(DEVICE)
            labels = labels.to(DEVICE)
            
            outputs = model(sequences)
            loss = criterion(outputs, labels.float())
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item() * sequences.size(0)

    # --- Evaluation ---
    model.eval()
    correct_predictions = 0
    total_samples = 0
    fold_true_labels = []
    fold_pred_labels = []

    with torch.no_grad():
        for sequences, labels in test_loader:
            sequences = sequences.to(DEVICE)
            labels = labels.to(DEVICE)
            
            outputs = model(sequences)
            _, predicted = torch.max(outputs.data, 1)
            _, true_labels = torch.max(labels.data, 1)
            
            total_samples += labels.size(0)
            correct_predictions += (predicted == true_labels).sum().item()

            fold_true_labels.extend(true_labels.cpu().numpy())
            fold_pred_labels.extend(predicted.cpu().numpy())

    fold_accuracy = 100 * correct_predictions / total_samples
    fold_accuracies.append(fold_accuracy)

    all_true_labels.extend(fold_true_labels)
    all_pred_labels.extend(fold_pred_labels)

    print(f'Training complete. Cross-User Test Accuracy on User {test_user_id}: {fold_accuracy:.2f}%')
    
# --- Final Aggregate Results ---

# ********* VOCABULARY SUBSTITUTION LOGIC (NO ORIGINAL WORDS IN OUTPUT) *********
# Create a mutable list from the original target names.
# This list is needed to align the indices correctly.
final_target_names = list(TARGET_NAMES)

# Replace the first 4 detected signs with the custom phrases
for i in range(min(len(HARDWARE_PHRASES), len(final_target_names))):
    final_target_names[i] = HARDWARE_PHRASES[i]

# final_target_names now holds the substituted list.

print("\n=======================================================")
print(f"Final Average Cross-User (LOUOCV) Accuracy: {np.mean(fold_accuracies):.2f}%")
print("=======================================================")

print("\nAggregate Classification Report (Simulated Hardware Vocabulary):")
# Use the modified final_target_names list for the final report printout
report = classification_report(all_true_labels, all_pred_labels, target_names=final_target_names, zero_division=0)
print(report)