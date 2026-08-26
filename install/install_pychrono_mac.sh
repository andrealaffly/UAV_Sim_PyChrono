#!/bin/bash

set -e
echo "===== PyChrono Installer for macOS (M1/M2/M3) ===== name date"

# === Step 0: Auto-detect Miniconda install path ===
if [ -x "/opt/miniconda3/bin/conda" ]; then
    MINICONDA_PATH="/opt/miniconda3"
elif [ -x "$HOME/miniconda3/bin/conda" ]; then
    MINICONDA_PATH="$HOME/miniconda3"
else
    echo "❌ Miniconda not found in /opt/miniconda3 or $HOME/miniconda3"
    echo "   ➡️ Please install Miniconda from: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

export PATH="$MINICONDA_PATH/bin:$PATH"

# === Step 1: Check conda ===
if ! command -v conda &> /dev/null; then
    echo "❌ Conda not found in PATH. Try restarting your terminal or check PATH settings."
    exit 1
fi

# === Step 1.5: Ensure conda-forge channel is added ===
echo "➡️  Adding conda-forge channel (if not already added)..."
conda config --add channels conda-forge || true  # harmless if already added

# === Step 1.6: Accept Anaconda Terms of Service (TOS) ===
echo "➡️  Ensuring Anaconda Terms of Service are accepted for required channels..."
CHANNELS=(
    "https://repo.anaconda.com/pkgs/main"
    "https://repo.anaconda.com/pkgs/r"
)
for CHANNEL in "${CHANNELS[@]}"; do
    echo "   🔎 Accepting ToS for: $CHANNEL"
    conda tos accept --override-channels --channel "$CHANNEL" || {
        echo "❌ Failed to accept ToS for $CHANNEL"
        echo "   💡 Please manually run:"
        echo "      conda tos accept --override-channels --channel $CHANNEL"
        exit 1
    }
done

# === Step 2: Check or create environment ===
ENV_NAME="chrono_v9"
PYVER="3.12"

if ! conda info --envs | grep -q "^$ENV_NAME"; then
    echo "⚠️  Conda env '$ENV_NAME' not found."
    read -p "➡️  Create it now with Python $PYVER? [Y/n]: " CREATE_ENV
    if [[ "$CREATE_ENV" =~ ^[Nn]$ ]]; then
        echo "❌ Aborting."
        echo "Create environment manually:"
        echo "conda create -n $ENV_NAME python=$PYVER"
        exit 1
    fi
    echo "➡️  Creating '$ENV_NAME'..."
    conda create -y -n "$ENV_NAME" python="$PYVER"
fi

# === Step 3: Activate environment ===
echo "➡️  Activating '$ENV_NAME'..."
source "$MINICONDA_PATH/etc/profile.d/conda.sh"
conda activate "$ENV_NAME"

# === Step 4: Install dependencies ===
echo "➡️  Installing dependencies..."
conda install -y -c conda-forge numpy=2.3.5 matplotlib irrlicht=1.8.5 pytz scipy pyyaml
pip install ruamel.yaml deap pymoo

# === Step 5: Ensure PyChrono tarball is available ===
TARBALL="$HOME/Downloads/pychrono-9.0.1-py312h70deae4_6418.conda"
TARBALL_URL="https://anaconda.org/projectchrono/pychrono/9.0.1/download/osx-arm64/pychrono-9.0.1-py312h70deae4_6418.conda"

echo "➡️  Checking for PyChrono tarball…"

if [ ! -f "$TARBALL" ]; then
    echo "⚠️  Tarball not found: $TARBALL"
    read -p "➡️  Download automatically? [Y/n]: " DOWNLOAD_PROMPT
    if [[ "$DOWNLOAD_PROMPT" =~ ^[Nn]$ ]]; then
        echo "❌ Aborting. Please download manually:"
        echo "   $TARBALL_URL"
        exit 1
    fi

    echo "⬇️  Downloading PyChrono tarball…"
    curl -L -o "$TARBALL" "$TARBALL_URL" || {
        echo "❌ Download failed."
        exit 1
    }
    echo "✅ Download complete."
fi

# === Step 6: Install PyChrono ===
echo "➡️  Installing PyChrono from tarball..."
conda install -y "$TARBALL"


# === Step 7: Set PYTHONPATH ===
export PYTHONPATH="$MINICONDA_PATH/envs/$ENV_NAME/share/chrono/python"
echo "✅ PYTHONPATH set to: $PYTHONPATH"

# === Step 8: Prompt to delete tarball ===
read -p "🧹 Delete PyChrono tarball? [y/N]: " DELETE_TARBALL
if [[ "$DELETE_TARBALL" =~ ^[Yy]$ ]]; then
    rm -f "$TARBALL"
    echo "Deleted: $TARBALL"
else
    echo "Tarball kept at: $TARBALL"
fi

echo "=============================================="
echo " PyChrono 9.0.1 installation complete!"
echo " To activate environment:"
echo "     conda activate $ENV_NAME"
echo "=============================================="