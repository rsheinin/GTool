import os
from shutil import copytree, rmtree, make_archive, move
from tempfile import gettempdir

if __name__ == "__main__":
    base_folder = os.path.dirname(__file__)
    temp_folder = gettempdir() + os.path.sep + "compile_folder"
    temp_folder = os.path.abspath("compile_folder")
    if os.path.exists(temp_folder):
        rmtree(temp_folder)
    # os.makedirs(temp_folder)
    os.makedirs(temp_folder, exist_ok=True)
    os.chdir(temp_folder)
    python_base = base_folder
    os.system(f"set PYTHONPATH={python_base}")

    ok = os.system(rf"pyinstaller --onefile {base_folder}\interactive_handeye.py --paths C:\Users\rsheinin\debug\spat_calib\lib\site-packages\cv2")
    if ok == 1:
        raise RuntimeError("could not run pyinstaller")
    move(rf"{temp_folder}\dist\interactive_handeye.exe", fr"{temp_folder}\interactive_handeye.exe")
    print(rf"zip folder created at {temp_folder}\interactive_handeye.exe")
