# Step-by-Step Guide: Install & Run PyChrono on Windows

---

## **PART 1: INSTALLATION (in cmd.exe)**

### **Step 1: Open Command Prompt**
- Press **Win + R**, type:  
  ```cmd
  cmd
  ```  
  and hit **Enter**.

### **Step 2: Navigate to the folder where your installer is saved**  
*(Replace the path if you saved it somewhere else)*  
```cmd
cd %USERPROFILE%\Downloads
```

### **Step 3: Run the installer script**  
```cmd
install_pychrono.bat
```

This will:  
- Add the **conda-forge** channel  
- Create an environment called **chrono**  
- Install required dependencies  
- Install **pychrono-8.0.0-py310_0.tar.bz2** (which you downloaded manually into **Downloads**)  

After this, PyChrono will be fully installed in your **chrono** Conda environment.

---

## **PART 2: RUN A DEMO (in Anaconda Prompt)**

### **Step 1: Open Anaconda Prompt**

### **Step 2: Activate the environment**  
```cmd
conda activate chrono
```

### **Step 3: Navigate to the demo folder**  
```cmd
cd %CONDA_PREFIX%\Lib\site-packages\pychrono\demos
```

### **Step 4: (Optional) See what demos are available**  
```cmd
dir *.py
```
Or explore subfolders like `/mbs`, `/vehicle`, etc.

### **Step 5: Run a demo!**  
```cmd
cd mbs
python demo_MBS_revolute.py
```