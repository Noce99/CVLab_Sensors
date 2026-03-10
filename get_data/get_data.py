import subprocess
import psutil
import time
import os
import signal
import datetime
import re
from tabulate import tabulate


ID = 0

def set_up_data_folders():
    current_path = os.path.dirname(os.path.abspath(__file__))
    data_folder = os.path.join(current_path, "data")
    if not os.path.exists(data_folder):
        os.makedirs(data_folder)
    
    # Create a folder for the current date and time
    now = datetime.datetime.now()
    time_folder_name = now.strftime("%Y_%m_%d__%H_%M_%S")
    data_folder_path = os.path.join(data_folder, time_folder_name)
    os.makedirs(data_folder_path)

    logs_folder_path = os.path.join(data_folder_path, "logs")
    os.makedirs(logs_folder_path)

    return data_folder_path, logs_folder_path
    
def get_unique_id():
    global ID
    ID += 1
    return ID-1

def keep_alive(processes):
    for key in processes:
        proces = processes[key]
        if proces.status() == "zombie":
            proces.stop()
            proces.run()


def print_nice_recap_table(processes, refresh_time_s = 1):
    table_data = []
    for proces in processes.values():
        table_data.append([proces.name, proces.status(), proces.frame_left, proces.frame_right])
    table = tabulate(table_data, headers=["Process", "Status", "Frame Left", "Frame Right"], tablefmt="grid")
    lines = table.split('\n')

    # Move cursor up to overwrite
    print(datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S"))
    print(table)
    time.sleep(refresh_time_s)
    print('\033[F' * (len(lines) + 1), end='')
    for i in range(len(lines) + 1):
        print(" "*100)
    print('\033[F' * (len(lines) + 1), end='')

class DataProcess:
    command : str
    data_path : str

    process : subprocess.Popen
    pid : int
    frame_left = None
    frame_right = None
    frames_left_path = None
    frames_right_path = None

    def __init__(self, name: str, command: str, data_path: str, log_path: str, frames_left_path: str, frames_right_path: str):
        self.id = get_unique_id()
        self.name = name
        self.command = command + " " + data_path + " > " + f"{log_path}_{self.id}" + " 2>&1 "
        self.data_path = data_path
        self.log_path = log_path
        self.frames_left_path = frames_left_path
        self.frames_right_path = frames_right_path


    def run(self):
        self.process = subprocess.Popen(self.command, shell=True)
        self.pid = self.process.pid

    def status(self):
        if psutil.pid_exists(self.pid):
            return psutil.Process(self.pid).status()
        else:
            return None

    def stop(self):
        parent = psutil.Process(self.pid)
        children = parent.children()
        for child in children:
            os.kill(child.pid, signal.SIGKILL)
            os.kill(self.pid, signal.SIGKILL)

    def count_frame(self):
        if self.frames_left_path is not None:
            self.frame_left = len(os.listdir(self.frames_left_path))
        if self.frames_right_path is not None:
            self.frame_right = len(os.listdir(self.frames_right_path))
    
class PythonScript(DataProcess):
    def __init__(self, script_path: str, data_path: str, logs_folder_path: str, frames_left_path: str, frames_right_path: str, extra_arguments: str=""):
        command = "../venv_3_12/bin/python -u " + script_path + " " + extra_arguments
        log_path = os.path.join(logs_folder_path, re.sub("\\.", "_", script_path) + ".log")
        super().__init__(script_path, command, data_path, log_path, frames_left_path, frames_right_path)

class CppScript(DataProcess):
    def __init__(self, executable_name: str, data_path: str, logs_folder_path: str, frames_left_path: str, frames_right_path: str, extra_arguments: str=""):
        command = "./cpp/" + executable_name + " " + extra_arguments
        log_path = os.path.join(logs_folder_path, re.sub("\\.", "_", executable_name) + ".log")
        super().__init__(executable_name, command, data_path, log_path, frames_left_path, frames_right_path)


if __name__ == "__main__":
    data_folder_path, logs_folder_path = set_up_data_folders()

    # RGB Data Folder
    rgb_left_path = os.path.join(data_folder_path, "rgb_left")
    rgb_right_path = os.path.join(data_folder_path, "rgb_right")
    os.mkdir(rgb_left_path)
    os.mkdir(rgb_right_path)

    # Event Data Folder
    event_left_path = os.path.join(data_folder_path, "event_left")
    event_right_path = os.path.join(data_folder_path, "event_right")
    os.mkdir(event_left_path)
    os.mkdir(event_right_path)

    # Livox Data Folder
    livox_data_folder = os.path.join(data_folder_path, "lidar_livox")
    os.mkdir(livox_data_folder)

    # Hesai Data Folder
    hesai_data_folder = os.path.join(data_folder_path, "lidar_hesai")
    os.mkdir(hesai_data_folder)

    # Be Sure Event Cameras are Ready
    subprocess.run(["sudo", "./scripts/disconnect_and_reconnect_event_cameras.sh"], capture_output=True, text=True)

    processes = {
        "rgbs":       PythonScript(
                        script_path="rgb.py",
                        data_path=data_folder_path,
                        logs_folder_path=logs_folder_path,
                        frames_left_path=rgb_left_path,
                        frames_right_path=rgb_right_path,
                      ),
        "event_left": PythonScript(
                        script_path="event.py",
                        data_path=event_left_path,
                        logs_folder_path=logs_folder_path,
                        frames_left_path=event_left_path,
                        frames_right_path=None,
                      ),
        "event_right": PythonScript(
                        script_path="event.py",
                        extra_arguments="--right-camera",
                        data_path=event_right_path,
                        logs_folder_path=logs_folder_path,
                        frames_left_path=None,
                        frames_right_path=event_right_path,
                      ),
        "livox":      CppScript(
                        executable_name="save_point_cloud_livox",
                        data_path=livox_data_folder,
                        logs_folder_path=logs_folder_path,
                        frames_left_path=livox_data_folder,
                        frames_right_path=None,
                      ),
        "hesai":      CppScript(
                        executable_name="save_point_cloud_hesai",
                        data_path=hesai_data_folder,
                        logs_folder_path=logs_folder_path,
                        frames_left_path=hesai_data_folder,
                        frames_right_path=None,
                      ),
    }
    
    try:
        for proces in processes.values():
            proces.run()

        while True:
            print_nice_recap_table(processes)
            for process in processes.values():
                process.count_frame()
            


    except KeyboardInterrupt:
        for proces in processes.values():
            proces.stop()

    for proces in processes.values():
        proces.stop()
    
    ok_to_move = True
    for proces in processes.values():
        this_proces_is_ok = False
        if proces != "rgb.py":
            if proces.frame_left is not None and proces.frame_left > 0:
                this_proces_is_ok = True
            if proces.frame_right is not None and proces.frame_right > 0:
                this_proces_is_ok = True
        else:
            if proces.frame_left is not None and proces.frame_left > 0 and proces.frame_right is not None and proces.frame_right > 0:
                this_proces_is_ok = True
        if not this_proces_is_ok:
            ok_to_move = False
            break

    if ok_to_move:
        subprocess.run(["mv", data_folder_path, "../CVLab_live_imgs/"], capture_output=True, text=True)
    else:
        print("Not all processes succeded.")

    