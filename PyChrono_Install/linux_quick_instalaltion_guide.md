# Installation Guide for Linux

## **Miniconda Installation Commands**

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
chmod +x ~/miniconda3/miniconda.sh  # Make the installer script executable
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh

source ~/miniconda3/bin/activate

conda init --all
```

## Run the command for downloading pychrono in download folder

```bash
wget -P $HOME/Downloads https://anaconda.org/projectchrono/pychrono/8.0.0/download/linux-64/pychrono-8.0.0-py310_0.tar.bz2

```


## **Make the PyChrono Installer Executable**

```bash
chmod +x install_pychrono_linux.sh
```

## **Activate the PyChrono Conda Environment**

```bash
conda activate chrono
```

## **Set the PYTHONPATH Environment Variable**

```bash
export PYTHONPATH=$(dirname $(python -c "import pychrono; print(pychrono.__file__)"))/..
```

## **Copy the Demo Files to Your Home Directory**

```bash
mkdir -p ~/pychrono_demos
cp -r "$PYTHONPATH/pychrono/demos/"* ~/pychrono_demos/
```

## **Run a Demo**

```bash
cd ~/pychrono_demos/mbs
python demo_MBS_revolute.py
```