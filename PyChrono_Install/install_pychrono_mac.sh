#!/bin/bash
# Edison Melendez
# Date: 7/18/25


set -e
echo "==== PyChrono Installer for macOS (M1/M2/M3) ==== name date"

# === Step 0: Auto-detect Miniconda install path ===
if [ -x "/opt/miniconda3/bin/conda" ]; then
    MINICONDA_PATH="/opt/miniconda3"
elif [ -x "$HOME/miniconda3/bin/conda" ]; then
    MINICONDA_PATH="$HOME/miniconda3"
else
    echo " Miniconda not found in /opt/miniconda3 or $HOME/miniconda3"
    echo "Please install Miniconda from: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

export PATH="$MINICONDA_PATH/bin:$PATH"

# === Step 1: Check conda ===
if ! command -v conda &> /dev/null; then
    echo "Conda not found in PATH. Try restarting your terminal or check PATH settings."
    exit 1
fi

# === Step 1.5: Ensure conda-forge channel is added ===
echo "➡️  Adding conda-forge channel (if not already added)..."
conda config --add channels http://conda.anaconda.org/conda-forge

# === Step 2: Check or create 'chrono' environment ===
if ! conda info --envs | grep -q "^chrono"; then
    echo "⚠️  Conda environment 'chrono' not found."
    read -p "Do you want to create it now with Python 3.10? [Y/n]: " CREATE_ENV
    if [[ "$CREATE_ENV" =~ ^[Nn]$ ]]; then
        echo " Aborting setup. Please create the environment manually: conda create -n chrono python=3.10"
        exit 1
    else
        echo "➡️  Creating conda environment 'chrono' with Python 3.10..."
        conda create -y -n chrono python=3.10
    fi
fi

# === Step 3: Activate environment ===
echo "➡️  Activating conda environment 'chrono'"
source "$MINICONDA_PATH/etc/profile.d/conda.sh"
conda activate chrono

# === Step 4: Install required packages ===
echo "➡️  Installing required packages from conda-forge..."
conda install -y -c conda-forge numpy=1.24.0
conda install -y -c conda-forge matplotlib
conda install -y -c conda-forge irrlicht=1.8.5
conda install -y -c conda-forge pytz
conda install -y -c conda-forge scipy

# === Step 5: Check for PyChrono tarball ===
TARBALL="$HOME/Downloads/pychrono-8.0.0-py310_2471.tar.bz2"
if [ ! -f "$TARBALL" ]; then
    echo " PyChrono tarball not found at: $TARBALL"
    echo "Please download it manually from conda-forge and place it in your Downloads folder."
    exit 1
fi

# === Step 6: Install PyChrono ===
echo "➡️  Installing PyChrono from tarball..."
conda install "$TARBALL"

# === Step 7: Set PYTHONPATH ===
export PYTHONPATH="$MINICONDA_PATH/envs/chrono/share/chrono/python"
echo " PYTHONPATH set to: $PYTHONPATH"

echo " PyChrono installation complete."
