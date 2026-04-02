#!/bin/bash

echo "===== Installing PyChrono 9.0.1 on Linux (Python 3.12) ====="

# Step 0: Check and install Miniconda if needed
if ! command -v conda &> /dev/null; then
  echo "Conda not found. Installing Miniconda..."

  MINICONDA_INSTALLER=Miniconda3-latest-Linux-x86_64.sh
  wget https://repo.anaconda.com/miniconda/$MINICONDA_INSTALLER -O /tmp/$MINICONDA_INSTALLER

  bash /tmp/$MINICONDA_INSTALLER -b -p $HOME/miniconda3
  rm /tmp/$MINICONDA_INSTALLER

  # Initialize conda
  eval "$($HOME/miniconda3/bin/conda shell.bash hook)"
  conda init
  echo "Miniconda installed successfully. Please restart your shell before running this script again."
  exit 0
else
  echo "Conda already installed."
  source "$(conda info --base)/etc/profile.d/conda.sh"
fi

# Step 1: Accept Terms of Service for required channels
echo "Accepting Terms of Service for conda default channels..."
conda tos accept --channel https://repo.anaconda.com/pkgs/main || true
conda tos accept --channel https://repo.anaconda.com/pkgs/r || true

# Step 2: Add conda-forge channel
echo "Adding conda-forge channel..."
conda config --add channels conda-forge
conda config --set channel_priority strict

# Step 3: Create conda environment
ENV_NAME="chrono_v9"
PY_VER="3.12"

if conda env list | grep -q "^${ENV_NAME}\s"; then
  echo "Environment '${ENV_NAME}' already exists."
else
  echo "Creating environment '${ENV_NAME}' with Python ${PY_VER}..."
  conda create -y -n $ENV_NAME python=$PY_VER
fi

# Step 4: Activate
echo "Activating environment..."
conda activate $ENV_NAME

# Step 5: Install dependencies
echo "Installing dependencies..."
conda install -y -c conda-forge numpy=2.3.5 matplotlib irrlicht=1.8.5 pytz scipy pyyaml
pip install ruamel.yaml deap pymoo

# Step 6: Install PyChrono 9.0.1
VERSION="9.0.1"
FILENAME="pychrono-9.0.1-py312hf1de3a3_6463.conda"
DOWNLOAD_URL="https://anaconda.org/projectchrono/pychrono/9.0.1/download/linux-64/$FILENAME"
DOWNLOAD_DIR="$HOME/Downloads"
TARBALL="$DOWNLOAD_DIR/$FILENAME"

echo "Checking for PyChrono tarball: $TARBALL"
if [ ! -f "$TARBALL" ]; then
  echo "Tarball not found. Downloading automatically..."
  wget -O "$TARBALL" "$DOWNLOAD_URL"
  if [ $? -ne 0 ]; then
    echo "ERROR: Download failed. Please download manually:"
    echo "  $DOWNLOAD_URL"
    exit 1
  fi
else
  echo "PyChrono tarball already exists."
fi

echo "Installing PyChrono from tarball..."
conda install -y "$TARBALL"

# Step 7: Set PYTHONPATH
PYTHONPATH=$(conda info --base)/envs/$ENV_NAME/share/chrono/python
export PYTHONPATH
echo "PYTHONPATH set: $PYTHONPATH"

# Step 8: Add PYTHONPATH to .bashrc if not present
BASHRC="$HOME/.bashrc"
LINE="export PYTHONPATH=$PYTHONPATH"

if grep -Fxq "$LINE" "$BASHRC"; then
  echo "PYTHONPATH already in .bashrc"
else
  echo "$LINE" >> "$BASHRC"
  echo "Added PYTHONPATH to .bashrc"
fi

# Step 9: Show demo path
DEMO_PATH=$(python -c "import pychrono, os; print(os.path.dirname(pychrono.__file__) + '/demos')")

echo "==== Installation Complete ===="
echo "To test a demo:"
echo "    conda activate $ENV_NAME"
echo "    mkdir ~/pychrono_demos && cd ~/pychrono_demos"
echo "    cp -r $DEMO_PATH/* ."
echo "    cd mbs && python demo_MBS_revolute.py"

