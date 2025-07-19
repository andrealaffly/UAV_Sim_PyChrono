# PyChrono Installation Guide

This guide walks you through the installation of [PyChrono](https://projectchrono.org/) for this project.


## Install Anaconda or Miniconda distribution

check the [installation guide](https://www.anaconda.com/docs/getting-started/miniconda/install#macos-linux-installation)

## Step 1: Add the conda-forge channel
Open a terminal window. For Windows users, please use Powershell.

```bash
conda config --add channels http://conda.anaconda.org/conda-forge
```

## Step 2: Create a new environment for PyChrono

```bash
conda create -n chrono python=3.10
```
You can replace "chrono" with the name you want.

To enter the environment:
```bash
conda activate chrono
```
To leave the environment:
```bash
conda deactivate
```
In the following steps, please stay in the created environment so that all the dependencies needed will stay in this environment.


## Step 3: Install dependencies for PyChrono
```bash
conda install -c conda-forge numpy=1.24.0
conda install -c conda-forge matplotlib
conda install -c conda-forge irrlicht=1.8.5
conda install -c conda-forge pytz
conda install -c conda-forge scipy
```

## Step 4: Download Pychrono package and install
Visit the [package repo](https://anaconda.org/projectchrono/pychrono/files?page=3)

(Choose the file with label "release")
For MacOS users, please click to download osx-arm64/pychrono-8.0.0-py310_2471.tar.bz2

For Windows users, please download win-64/pychrono-8.0.0-py310_0.tar.bz2

For Linux users, please download linux-64/pychrono-8.0.0-py310_0.tar.bz2

After the download completes, navigate to the Downloads folder and install PyChrono
```bash
cd Downloads/
conda install pychrono-8.0.0-py310_2471.tar.bz2
```

for Mac users:
```bash
export PYTHONPATH=$HOME/miniconda3/envs/chrono/share/chrono/python
```

## Step 5: Copy the demos and test the installation
First, navigate to the folder you want to keep the demos files.
```bash
mkdir pychrono_demos
cd pychrono_demos/
cp -r $PYTHONPATH/pychrono/demos/* .
```
Now you can try to run the demos to check if the installation is correct.
For example,
```bash
cd mbs/
python demo_MBS_revolute.py
```
## Step 6: Try to run the code
Clone the repo and navigate to the project folder.
```bash
git clone https://github.com/andrealaffly/UAV_Sim_PyChrono.git
cd UAV_Sim_PyChrono
python main.py
```
