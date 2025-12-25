import tkinter as tk
import subprocess
import os

running_processes = []

def run_file(filename):
    process = subprocess.Popen(["xterm", "-e", "python", filename])
    running_processes.append(process)

def run_rviz():
    process = subprocess.Popen(
        ["xterm", "-e", "rviz", "-d", "AquaTrace/simple_teleop.rviz"]
    )
    running_processes.append(process)

def on_closing():
    for process in running_processes:
        if process.poll() is None:
            process.terminate()
    root.destroy()

root = tk.Tk()
root.title("AquaTrace Control Panel")

root.protocol("WM_DELETE_WINDOW", on_closing)

# ---------- RViz Button ----------
rviz_button = tk.Button(
    root,
    text="Launch RViz (Simple Teleop)",
    bg="#2c3e50",
    fg="white",
    height=2,
    command=run_rviz
)
rviz_button.pack(pady=10)

# ---------- Python Script Buttons ----------
script_dir = os.path.dirname(__file__)
python_files = [
    f for f in os.listdir(script_dir)
    if f.endswith(".py") and f != os.path.basename(__file__)
]

for file in sorted(python_files):
    button = tk.Button(
        root,
        text=file,
        command=lambda f=os.path.join(script_dir, file): run_file(f)
    )
    button.pack(pady=5)

root.mainloop()