import re
import ast
import yaml
import shutil
import tokenize
from io import BytesIO
from pathlib import Path
# from ruamel.yaml import YAML

# ---------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------

# def update_yaml_value_preserve_comments(yaml_file, key_path, new_value):
#     """
#     Update a specific key in a YAML file while preserving comments and formatting.

#     Args:
#         yaml_file (str or Path): Path to the YAML file.
#         key_path (list or str): List of nested keys or dot-separated string (e.g. "controller.gains.kp").
#         new_value: The new value to assign to the key.
#     """
#     yaml_file = Path(yaml_file)
#     if not yaml_file.exists():
#         raise FileNotFoundError(f"YAML file not found: {yaml_file}")

#     yaml = YAML()
#     yaml.preserve_quotes = True  # keep string quotes if present

#     # Load YAML (preserves comments)
#     with open(yaml_file, "r", encoding="utf-8") as f:
#         data = yaml.load(f)

#     # Parse key path (support dot-separated or list)
#     if isinstance(key_path, str):
#         key_path = key_path.split(".")

#     # Traverse dict to the target key
#     d = data
#     for k in key_path[:-1]:
#         if k not in d:
#             raise KeyError(f"[ERROR] Key path not found in YAML: {'.'.join(key_path)}")
#         d = d[k]

#     # Update value
#     final_key = key_path[-1]
#     if final_key not in d:
#         raise KeyError(f"[ERROR] Key not found in YAML: {final_key}")

#     d[final_key] = new_value

#     # Write back (preserving comments)
#     with open(yaml_file, "w", encoding="utf-8") as f:
#         yaml.dump(data, f)

def get_yaml_value(file_path, key_path):
    """
    Extracts a value from a YAML file using a dotted key path (e.g., "uav.type.name").

    Args:
        file_path (str | Path): Path to the YAML file.
        key_path (str): Dotted path to the key (e.g., "uav.type.name").

    Returns:
        Any: The value found at the specified key path, or None if not found.
    """
    file_path = Path(file_path)

    if not file_path.exists():
        raise FileNotFoundError(f"[ERROR] YAML file not found: {file_path}")

    with file_path.open("r", encoding="utf-8") as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ValueError(f"[ERROR] Invalid YAML format in {file_path}: {e}")

    if data is None:
        print(f"[WARN] Empty YAML file: {file_path}")
        return None

    keys = key_path.split(".")
    current = data
    for key in keys:
        if isinstance(current, dict) and key in current:
            current = current[key]
        else:
            print(f"[WARN] Key path '{key_path}' not found in {file_path.name}")
            return None

    return current

def get_class_names_from_file(file_path):
    """
    Parses a Python file and returns a list of all top-level class names defined in it.

    Args:
        file_path (str | Path): Path to the Python file.

    Returns:
        list[str]: A list of class names found in the file.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"File not found: {file_path}")

    with file_path.open("r", encoding="utf-8") as f:
        source = f.read()

    try:
        tree = ast.parse(source)
    except SyntaxError as e:
        raise SyntaxError(f"Failed to parse {file_path}: {e}")

    class_names = [
        node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)
    ]
    return class_names

def rename_internal_references(base_dir, old_name, new_name, exts=(".py", ".yaml")):
    """
    Recursively rename all internal references

    Args:
        base_dir (str | Path): Base directory where the rename should happen.
        old_name (str): Original name to replace.
        new_name (str): New name to substitute.
        exts (tuple): File extensions to process for text replacement (default: .py, .yaml).
    """
    base_dir = Path(base_dir)
    if not base_dir.exists():
        print(f"[ERROR] Base directory '{base_dir}' not found.")
        return

    for file in base_dir.rglob("*"):
        if not file.is_file():
            continue

        # Replace internal text references
        if file.suffix in exts:
            text = file.read_text(encoding="utf-8")
            new_text = re.sub(rf"\b{old_name}\b", new_name, text)
            if new_text != text:
                file.write_text(new_text, encoding="utf-8")
 
def replace_in_file(file_path, old_str, new_str, use_regex=False):
    """
    Replace a string (or regex pattern) inside a specific file, preserving encoding.

    Args:
        file_path (str | Path): Path to the file to modify.
        old_str (str): Text (or regex) to replace.
        new_str (str): Replacement text.
        use_regex (bool): If True, treat old_str as a regular expression (default: False).
    """
    file_path = Path(file_path)
    if not file_path.exists():
        print(f"[ERROR] File '{file_path}' not found.")
        return

    if not file_path.is_file():
        print(f"[ERROR] '{file_path}' is not a file.")
        return

    # Read the file
    text = file_path.read_text(encoding="utf-8")

    # Replace content
    if use_regex:
        new_text = re.sub(old_str, new_str, text)
    else:
        new_text = text.replace(old_str, new_str)

    # Write changes if modified
    if new_text != text:
        file_path.write_text(new_text, encoding="utf-8")
    #     print(f"✅ Updated '{file_path}' — replaced '{old_str}' → '{new_str}'.")
    # else:
    #     print(f"ℹ️ No matches for '{old_str}' found in '{file_path}'.")
        
def rename_files(base_dir, old_name, new_name):
    """
    Recursively rename all files under base_dir that contain old_name in their names.

    Args:
        base_dir (str | Path): Base directory where renaming should happen.
        old_name (str): Substring to replace.
        new_name (str): New substring to substitute.
    """
    base_dir = Path(base_dir)
    if not base_dir.exists():
        print(f"[ERROR] Base directory '{base_dir}' not found.")
        return

    for file in base_dir.rglob("*"):
        if file.is_file() and old_name in file.name:
            new_file = file.with_name(file.name.replace(old_name, new_name))
            file.rename(new_file)
            # print(f"[FILE] {file.name} → {new_file.name}")
            
def rename_folders(base_dir, old_name, new_name):
    """
    Recursively rename all folders under base_dir that contain old_name in their names.

    Args:
        base_dir (str | Path): Base directory where renaming should happen.
        old_name (str): Substring to replace.
        new_name (str): New substring to substitute.
    """
    base_dir = Path(base_dir)
    if not base_dir.exists():
        print(f"[ERROR] Base directory '{base_dir}' not found.")
        return

    # Sort in reverse order to rename deep folders first (avoids path conflicts)
    for folder in sorted(base_dir.rglob("*"), reverse=True):
        if folder.is_dir() and old_name in folder.name:
            new_folder = folder.with_name(folder.name.replace(old_name, new_name))
            folder.rename(new_folder)
            # print(f"[FOLDER] {folder.name} → {new_folder.name}")
            
def rename_all(base_dir, old_name, new_name):
    """
    Recursively rename all files and folders under base_dir containing old_name.

    Args:
        base_dir (str | Path): Base directory where renaming should happen.
        old_name (str): Substring to replace.
        new_name (str): New substring to substitute.
    """
    base_dir = Path(base_dir)
    if not base_dir.exists():
        print(f"[ERROR] Base directory '{base_dir}' not found.")
        return

    # First rename folders (deepest first)
    for folder in sorted(base_dir.rglob("*"), reverse=True):
        if folder.is_dir() and old_name in folder.name:
            new_folder = folder.with_name(folder.name.replace(old_name, new_name))
            folder.rename(new_folder)
            print(f"[FOLDER] {folder.name} → {new_folder.name}")

    # Then rename files
    for file in base_dir.rglob("*"):
        if file.is_file() and old_name in file.name:
            new_file = file.with_name(file.name.replace(old_name, new_name))
            file.rename(new_file)
            # print(f"[FILE] {file.name} → {new_file.name}")
      
def delete_folder(folder_path, force=False):
    """
    Deletes a folder and all its contents (even if non-empty).

    Args:
        folder_path (str | Path): Path to the folder to delete.
        force (bool): If True, skip confirmation prompt.

    Example:
        delete_folder("build/output", force=True)
    """
    folder_path = Path(folder_path)

    # Check existence
    if not folder_path.exists():
        print(f"[WARNING] Folder '{folder_path}' does not exist.")
        return

    if not folder_path.is_dir():
        print(f"[ERROR] '{folder_path}' is not a folder.")
        return

    # Confirmation prompt
    if not force:
        confirm = input(f"[WARNING] Are you sure you want to permanently delete '{folder_path}'? (y/N): ").strip().lower()
        if confirm not in {"y", "yes"}:
            print("[INFO] Deletion cancelled.")
            return

    try:
        shutil.rmtree(folder_path)
        print(f"[INFO] Folder '{folder_path}' deleted successfully.")
    except Exception as e:
        print(f"[ERROR] Failed to delete '{folder_path}': {e}")
              
import re
import ast
import yaml
import shutil
import tokenize
from io import BytesIO
from pathlib import Path
# from ruamel.yaml import YAML

# ---------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------

# def update_yaml_value_preserve_comments(yaml_file, key_path, new_value):
#     """
#     Update a specific key in a YAML file while preserving comments and formatting.

#     Args:
#         yaml_file (str or Path): Path to the YAML file.
#         key_path (list or str): List of nested keys or dot-separated string (e.g. "controller.gains.kp").
#         new_value: The new value to assign to the key.
#     """
#     yaml_file = Path(yaml_file)
#     if not yaml_file.exists():
#         raise FileNotFoundError(f"YAML file not found: {yaml_file}")

#     yaml = YAML()
#     yaml.preserve_quotes = True  # keep string quotes if present

#     # Load YAML (preserves comments)
#     with open(yaml_file, "r", encoding="utf-8") as f:
#         data = yaml.load(f)

#     # Parse key path (support dot-separated or list)
#     if isinstance(key_path, str):
#         key_path = key_path.split(".")

#     # Traverse dict to the target key
#     d = data
#     for k in key_path[:-1]:
#         if k not in d:
#             raise KeyError(f"[ERROR] Key path not found in YAML: {'.'.join(key_path)}")
#         d = d[k]

#     # Update value
#     final_key = key_path[-1]
#     if final_key not in d:
#         raise KeyError(f"[ERROR] Key not found in YAML: {final_key}")

#     d[final_key] = new_value

#     # Write back (preserving comments)
#     with open(yaml_file, "w", encoding="utf-8") as f:
#         yaml.dump(data, f)

def get_yaml_value(file_path, key_path):
    """
    Extracts a value from a YAML file using a dotted key path (e.g., "uav.type.name").

    Args:
        file_path (str | Path): Path to the YAML file.
        key_path (str): Dotted path to the key (e.g., "uav.type.name").

    Returns:
        Any: The value found at the specified key path, or None if not found.
    """
    file_path = Path(file_path)

    if not file_path.exists():
        raise FileNotFoundError(f"[ERROR] YAML file not found: {file_path}")

    with file_path.open("r", encoding="utf-8") as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ValueError(f"[ERROR] Invalid YAML format in {file_path}: {e}")

    if data is None:
        print(f"[WARN] Empty YAML file: {file_path}")
        return None

    keys = key_path.split(".")
    current = data
    for key in keys:
        if isinstance(current, dict) and key in current:
            current = current[key]
        else:
            print(f"[WARN] Key path '{key_path}' not found in {file_path.name}")
            return None

    return current

def get_class_names_from_file(file_path):
    """
    Parses a Python file and returns a list of all top-level class names defined in it.

    Args:
        file_path (str | Path): Path to the Python file.

    Returns:
        list[str]: A list of class names found in the file.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"File not found: {file_path}")

    with file_path.open("r", encoding="utf-8") as f:
        source = f.read()

    try:
        tree = ast.parse(source)
    except SyntaxError as e:
        raise SyntaxError(f"Failed to parse {file_path}: {e}")

    class_names = [
        node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)
    ]
    return class_names

def rename_internal_references(base_dir, old_name, new_name, exts=(".py", ".yaml")):
    """
    Recursively rename all internal references

    Args:
        base_dir (str | Path): Base directory where the rename should happen.
        old_name (str): Original name to replace.
        new_name (str): New name to substitute.
        exts (tuple): File extensions to process for text replacement (default: .py, .yaml).
    """
    base_dir = Path(base_dir)
    if not base_dir.exists():
        print(f"[ERROR] Base directory '{base_dir}' not found.")
        return

    for file in base_dir.rglob("*"):
        if not file.is_file():
            continue

        # Replace internal text references
        if file.suffix in exts:
            text = file.read_text(encoding="utf-8")
            new_text = re.sub(rf"\b{old_name}\b", new_name, text)
            if new_text != text:
                file.write_text(new_text, encoding="utf-8")
 
def replace_in_file(file_path, old_str, new_str, use_regex=False):
    """
    Replace a string (or regex pattern) inside a specific file, preserving encoding.

    Args:
        file_path (str | Path): Path to the file to modify.
        old_str (str): Text (or regex) to replace.
        new_str (str): Replacement text.
        use_regex (bool): If True, treat old_str as a regular expression (default: False).
    """
    file_path = Path(file_path)
    if not file_path.exists():
        print(f"[ERROR] File '{file_path}' not found.")
        return

    if not file_path.is_file():
        print(f"[ERROR] '{file_path}' is not a file.")
        return

    # Read the file
    text = file_path.read_text(encoding="utf-8")

    # Replace content
    if use_regex:
        new_text = re.sub(old_str, new_str, text)
    else:
        new_text = text.replace(old_str, new_str)

    # Write changes if modified
    if new_text != text:
        file_path.write_text(new_text, encoding="utf-8")
    #     print(f"✅ Updated '{file_path}' — replaced '{old_str}' → '{new_str}'.")
    # else:
    #     print(f"ℹ️ No matches for '{old_str}' found in '{file_path}'.")
        
def rename_files(base_dir, old_name, new_name):
    """
    Recursively rename all files under base_dir that contain old_name in their names.

    Args:
        base_dir (str | Path): Base directory where renaming should happen.
        old_name (str): Substring to replace.
        new_name (str): New substring to substitute.
    """
    base_dir = Path(base_dir)
    if not base_dir.exists():
        print(f"[ERROR] Base directory '{base_dir}' not found.")
        return

    for file in base_dir.rglob("*"):
        if file.is_file() and old_name in file.name:
            new_file = file.with_name(file.name.replace(old_name, new_name))
            file.rename(new_file)
            # print(f"[FILE] {file.name} → {new_file.name}")
            
def rename_folders(base_dir, old_name, new_name):
    """
    Recursively rename all folders under base_dir that contain old_name in their names.

    Args:
        base_dir (str | Path): Base directory where renaming should happen.
        old_name (str): Substring to replace.
        new_name (str): New substring to substitute.
    """
    base_dir = Path(base_dir)
    if not base_dir.exists():
        print(f"[ERROR] Base directory '{base_dir}' not found.")
        return

    # Sort in reverse order to rename deep folders first (avoids path conflicts)
    for folder in sorted(base_dir.rglob("*"), reverse=True):
        if folder.is_dir() and old_name in folder.name:
            new_folder = folder.with_name(folder.name.replace(old_name, new_name))
            folder.rename(new_folder)
            # print(f"[FOLDER] {folder.name} → {new_folder.name}")
            
def rename_all(base_dir, old_name, new_name):
    """
    Recursively rename all files and folders under base_dir containing old_name.

    Args:
        base_dir (str | Path): Base directory where renaming should happen.
        old_name (str): Substring to replace.
        new_name (str): New substring to substitute.
    """
    base_dir = Path(base_dir)
    if not base_dir.exists():
        print(f"[ERROR] Base directory '{base_dir}' not found.")
        return

    # First rename folders (deepest first)
    for folder in sorted(base_dir.rglob("*"), reverse=True):
        if folder.is_dir() and old_name in folder.name:
            new_folder = folder.with_name(folder.name.replace(old_name, new_name))
            folder.rename(new_folder)
            print(f"[FOLDER] {folder.name} → {new_folder.name}")

    # Then rename files
    for file in base_dir.rglob("*"):
        if file.is_file() and old_name in file.name:
            new_file = file.with_name(file.name.replace(old_name, new_name))
            file.rename(new_file)
            # print(f"[FILE] {file.name} → {new_file.name}")
      
def delete_folder(folder_path, force=False):
    """
    Deletes a folder and all its contents (even if non-empty).

    Args:
        folder_path (str | Path): Path to the folder to delete.
        force (bool): If True, skip confirmation prompt.

    Example:
        delete_folder("build/output", force=True)
    """
    folder_path = Path(folder_path)

    # Check existence
    if not folder_path.exists():
        print(f"[WARNING] Folder '{folder_path}' does not exist.")
        return

    if not folder_path.is_dir():
        print(f"[ERROR] '{folder_path}' is not a folder.")
        return

    # Confirmation prompt
    if not force:
        confirm = input(f"[WARNING] Are you sure you want to permanently delete '{folder_path}'? (y/N): ").strip().lower()
        if confirm not in {"y", "yes"}:
            print("[INFO] Deletion cancelled.")
            return

    try:
        shutil.rmtree(folder_path)
        print(f"[INFO] Folder '{folder_path}' deleted successfully.")
    except Exception as e:
        print(f"[ERROR] Failed to delete '{folder_path}': {e}")
              