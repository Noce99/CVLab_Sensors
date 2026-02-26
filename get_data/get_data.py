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
        table_data.append([proces.name, proces.status()])
    table = tabulate(table_data, headers=["Process", "Status"], tablefmt="grid")
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

    def __init__(self, name: str, command: str, data_path: str, log_path: str):
        self.id = get_unique_id()
        self.name = name
        self.command = command + " " + data_path + " > " + f"{log_path}_{self.id}" + " 2>&1 "
        self.data_path = data_path
        self.log_path = log_path


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
    
class PythonScript(DataProcess):
    def __init__(self, script_path: str, data_path: str, logs_folder_path: str):
        command = "../venv_3_12/bin/python -u " + script_path
        log_path = os.path.join(logs_folder_path, re.sub("\\.", "_", script_path) + ".log")
        super().__init__(script_path, command, data_path, log_path)


if __name__ == "__main__":
    data_folder_path, logs_folder_path = set_up_data_folders()

    processes = {
        "test_1":   PythonScript(
                        script_path="rgb.py",
                        data_path=data_folder_path,
                        logs_folder_path=logs_folder_path
                    ),
        # "test_2":   PythonScript(
        #                 script_path="rgb.py",
        #                 data_path=data_folder_path,
        #                 logs_folder_path=logs_folder_path
        #             ),
        # "test_3":   PythonScript(
        #                 script_path="rgb.py",
        #                 data_path=data_folder_path,
        #                 logs_folder_path=logs_folder_path
        #             ),
    }
    
    try:
        for proces in processes.values():
            proces.run()

        while True:
            print_nice_recap_table(processes)
            # keep_alive(processes)


    except KeyboardInterrupt:
        for proces in processes.values():
            proces.stop()

    for proces in processes.values():
        proces.stop()

    