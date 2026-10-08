"""
residual_nn.py
==============
Lightweight PyTorch MLP to predict voltage estimation residuals.
Provides physics-informed neural network corrections to the EKF.
"""

import os
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np


class ResidualMLP(nn.Module):
    """
    Multi-Layer Perceptron (MLP) for battery model voltage residual correction.
    Inputs: [SOC_est, current, temp_k, voltage_pred_physics] (Dimension: 4)
    Output: [voltage_residual_error] (Dimension: 1)
    """
    def __init__(self, input_dim: int = 4, hidden_dim1: int = 16, hidden_dim2: int = 8):
        super(ResidualMLP, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim1),
            nn.ReLU(),
            nn.Linear(hidden_dim1, hidden_dim2),
            nn.ReLU(),
            nn.Linear(hidden_dim2, 1)
        )

    def forward(self, x):
        return self.net(x)


class ResidualCorrector:
    """
    Wrapper for loading, training, and running predictions with the ResidualMLP.
    """
    def __init__(self, model_path: str = "residual_mlp.pth"):
        self.model_path = model_path
        self.model = ResidualMLP()
        self.model.eval()  # default to evaluation mode
        
        # Scaling parameters (mean and std for input normalization)
        self.input_mean = np.array([0.5, 0.0, 298.15, 3.7])
        self.input_std = np.array([0.3, 2.0, 10.0, 0.3])

    def train_model(self, X_train: np.ndarray, y_train: np.ndarray, epochs: int = 50, batch_size: int = 64, lr: float = 0.01):
        """
        Train the MLP on the collected residual datasets.
        X_train shape: (N, 4), y_train shape: (N, 1)
        """
        self.model.train()
        
        # 1. Compute scaling parameters
        self.input_mean = np.mean(X_train, axis=0)
        self.input_std = np.std(X_train, axis=0)
        # Avoid division by zero
        self.input_std[self.input_std < 1e-6] = 1.0

        # Normalize training inputs
        X_scaled = (X_train - self.input_mean) / self.input_std

        # Convert to PyTorch Tensors
        X_tensor = torch.tensor(X_scaled, dtype=torch.float32)
        y_tensor = torch.tensor(y_train, dtype=torch.float32).view(-1, 1)

        dataset = torch.utils.data.TensorDataset(X_tensor, y_tensor)
        loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)

        optimizer = optim.Adam(self.model.parameters(), lr=lr)
        criterion = nn.MSELoss()

        print(f"  [Neural Network] Starting training for {epochs} epochs on {len(X_train)} samples...")
        
        for epoch in range(epochs):
            epoch_loss = 0.0
            for batch_x, batch_y in loader:
                optimizer.zero_grad()
                pred = self.model(batch_x)
                loss = criterion(pred, batch_y)
                loss.backward()
                optimizer.step()
                epoch_loss += loss.item() * batch_x.size(0)
            
            epoch_loss /= len(X_train)
            if (epoch + 1) % 10 == 0 or epoch == 0:
                print(f"    Epoch {epoch+1:2d}/{epochs:2d} | Training Loss (MSE): {epoch_loss:.6f}")

        self.model.eval()
        # Save training details
        self.save()

    def predict(self, soc: float, current: float, temp_k: float, v_pred_physics: float) -> float:
        """
        Predict the voltage correction value.
        """
        # Form input vector and scale it
        x_in = np.array([soc, current, temp_k, v_pred_physics], dtype=float)
        x_scaled = (x_in - self.input_mean) / self.input_std
        
        # Convert to tensor and predict
        x_tensor = torch.tensor(x_scaled, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            v_residual = self.model(x_tensor).item()
            
        return float(v_residual)

    def save(self):
        """Save network state and scale parameters."""
        state = {
            "model_state": self.model.state_dict(),
            "mean": self.input_mean,
            "std": self.input_std
        }
        torch.save(state, self.model_path)

    def load(self) -> bool:
        """Load network state and scale parameters if file exists."""
        if os.path.exists(self.model_path):
            state = torch.load(self.model_path, map_location=torch.device('cpu'))
            self.model.load_state_dict(state["model_state"])
            self.input_mean = state["mean"]
            self.input_std = state["std"]
            self.model.eval()
            return True
        return False
