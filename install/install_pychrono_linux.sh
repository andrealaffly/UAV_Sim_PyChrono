#!/bin/bash
# Created: 7/20/2025 by Edison Melendez
# Modified: 8/5/2025 by Ryan Dunk

echo "==== Installing PyChrono on Linux ===="

# Step 0: Check and install Miniconda if needed
if ! command -v conda &> /dev/null; then
  echo "[INFO] Conda not found. Installing Miniconda..."

  MINICONDA_INSTALLER=Miniconda3-latest-Linux-x86_64.sh
  wget https://repo.anaconda.com/miniconda/$MINICONDA_INSTALLER -O /tmp/$MINICONDA_INSTALLER

  bash /tmp/$MINICONDA_INSTALLER -b -p $HOME/miniconda3
  rm /tmp/$MINICONDA_INSTALLER

  # Initialize conda
  eval "$($HOME/miniconda3/bin/conda shell.bash hook)"
  conda init

  echo "[INFO] Miniconda installed successfully. Please restart your shell and re-run this script."
  exit 0
else
  echo "[INFO] Conda already installed."
  source "$(conda info --base)/etc/profile.d/conda.sh"
fi

# Step 1: Accept Terms of Service
echo "[INFO] Accepting Terms of Service for default Conda channels..."
conda tos accept --channel https://repo.anaconda.com/pkgs/main || true
conda tos accept --channel https://repo.anaconda.com/pkgs/r || true

# Step 2: Configure conda-forge channel
echo "[INFO] Adding conda-forge channel..."
conda config --add channels conda-forge
conda config --set channel_priority strict

# Step 3: Create environment if it does not exist
if conda env list | grep -q "^chrono\s"; then
  echo "[INFO] Conda environment 'chrono' already exists."
else
  echo "[INFO] Creating conda environment 'chrono' with Python 3.10..."
  conda create -y -n chrono python=3.10
fi

# Step 4: Activate environment
echo "[INFO] Activating environment..."
conda activate chrono

# Step 5: Install required dependencies
echo "[INFO] Installing required packages..."
conda install -c conda-forge numpy=1.24.0 matplotlib irrlicht=1.8.5 pytz scipy

# Step 6: Install PyChrono from tarball
VERSION="8.0.0"
PYTHON_TAG="py310_0"
FILENAME="pychrono-${VERSION}-${PYTHON_TAG}.tar.bz2"
DOWNLOAD_URL="https://anaconda.org/projectchrono/pychrono/${VERSION}/download/linux-64/${FILENAME}"
DOWNLOAD_DIR="$HOME/Downloads"
TARBALL="$DOWNLOAD_DIR/$FILENAME"

echo "[INFO] Checking for PyChrono tarball at: $TARBALL"
if [ ! -f "$TARBALL" ]; then
  echo "[INFO] Downloading PyChrono $VERSION tarball..."
  wget -O "$TARBALL" "$DOWNLOAD_URL"
else
  echo "[INFO] PyChrono tarball already present."
fi

echo "[INFO] Installing PyChrono from tarball..."
conda install -y "$TARBALL"

# Step 7: Validate installation
echo "[INFO] Verifying installation..."
python -c "import pychrono.irrlicht; import numpy; print('PyChrono installed successfully.')"

echo "[DONE] PyChrono installation complete."

