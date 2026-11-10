#!/bin/bash

set -e

GZ_MODEL_DIR="$HOME/.gz/models"
mkdir -p "$GZ_MODEL_DIR"

download_model() {
    local model_name="$1"
    local model_url="https://fuel.gazebosim.org/1.0/OpenRobotics/models/$model_name"
    local dest_dir="$GZ_MODEL_DIR/$model_name"

    if [ -d "$dest_dir" ]; then
        echo "[✔] Model '$model_name' already installed at $dest_dir"
        return
    fi

    echo "[↓] Downloading $model_name..."
    wget -q --show-progress -O "/tmp/$model_name.tar.gz" "$model_url/download"

    echo "[➤] Extracting to $dest_dir..."
    mkdir -p "$dest_dir"
    tar -xzf /tmp/$model_name.tar.gz -C $dest_dir --strip-components=1

    rm "/tmp/$model_name.tar.gz"
    echo "[✔] Installed '$model_name' at $dest_dir"
}

echo "[🌍] Installing Gazebo default models into $GZ_MODEL_DIR"

download_model "Ground Plane"
download_model "Sun"

echo "[✅] All models installed."

# Optional: Add to ~/.bashrc if not already there
if ! grep -q "GZ_SIM_RESOURCE_PATH" ~/.bashrc; then
    echo "export GZ_SIM_RESOURCE_PATH=\$HOME/.gz/models" >> ~/.bashrc
    echo "[⚙] Added GZ_SIM_RESOURCE_PATH to ~/.bashrc"
    echo "Please run: source ~/.bashrc"
else
    echo "[ℹ] GZ_SIM_RESOURCE_PATH already set in ~/.bashrc"
fi
