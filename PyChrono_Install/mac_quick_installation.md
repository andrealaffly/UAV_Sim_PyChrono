# TEP-BY-STEP GUIDE (macOS)

## **1. Make the installer script executable**
```bash
chmod +x install_pychrono_mac.sh
```

## **2. Add Conda to PATH**
*(Only if not already in your path)*  
```bash
export PATH="/opt/miniconda3/bin:$PATH"
```

## **3. Check Conda is available**
```bash
which conda
```

## **4. Go to Downloads and run the installer**
```bash
cd ~/Downloads
./install_pychrono_mac.sh
```

---

# POST-INSTALL STEPS

## **5. Initialize Conda for zsh shell**
```bash
/opt/miniconda3/bin/conda init zsh
```

## **6. Restart the shell**
```bash
exec zsh
```

## **7. Activate the PyChrono Conda environment**
```bash
conda activate chrono
```

## **8. Set the PYTHONPATH environment variable**
```bash
export PYTHONPATH=$(conda info --base)/envs/chrono/share/chrono/python
```

## **9. Copy the demo files to your home directory**
```bash
mkdir -p ~/pychrono_demos
cp -r "$PYTHONPATH/pychrono/demos/"* ~/pychrono_demos/
```

## **10. Run a demo**
```bash
cd ~/pychrono_demos/mbs
python demo_MBS_revolute.py
```
