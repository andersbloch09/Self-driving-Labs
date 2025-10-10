from inspect import stack
from urllib import response
import requests
import time
import os
import threading
import ast
import json
from contextlib import ExitStack

class OT2Client:
    def __init__(self, ip: str):
        self.base_url = f"http://{ip}:31950"
        self.headers = {"opentrons-version": "2"}

    def extract_labware_names(self, file_path):
        with open(file_path, "r") as f:
            code = f.read()

        tree = ast.parse(code)
        labware = []

        for node in ast.walk(tree):
            # Look for function calls
            if isinstance(node, ast.Call):
            # Check if it's a method call: something.load_labware(...)
                if isinstance(node.func, ast.Attribute) and node.func.attr == "load_labware":
                # First argument is usually the labware name
                    if node.args and isinstance(node.args[0], ast.Constant):
                        labware.append(node.args[0].value)
        return labware
    

    def check_labware_in_custom_folder(self, labware_name, custom_folder):
        """Check if labware exists in a local folder of JSON labware files."""
        if not os.path.isdir(custom_folder):
            return False
        for root, _, files in os.walk(custom_folder):
            for file in files:
                if file.endswith(".json"):
                    try:
                        path = os.path.join(root, file)
                        with open(path, "r") as f:
                            data = json.load(f)

                        # Labware load name is usually stored in parameters.loadName
                        load_name = data.get("parameters", {}).get("loadName", "")
                        display_name = data.get("metadata", {}).get("displayName", "")
                        if labware_name == load_name or labware_name == display_name:
                            return True
                    except Exception:
                        continue
        return False

    def verify_labware(self, protocol_path, custom_labware_folder):
        labware_names = self.extract_labware_names(protocol_path)
        
        custom_labware = []

        for name in labware_names:
            print(f" - {name}: \n", end="")
            if self.check_labware_in_custom_folder(name, custom_labware_folder):
                print("🧩 Found in custom labware folder")
                custom_labware.append(os.path.join(custom_labware_folder, name + ".json"))
            else:
                print("ℹ️  Not found in custom labware folder, if labware is not in the Opentrons database, protocol may fail.")

        return custom_labware

    def upload_protocol(self, protocol_file, custom_labware):
        print(f"Uploading protocol {protocol_file} with {len(custom_labware)} custom labware files...")
        with ExitStack() as stack:
        # Open protocol file
            f_protocol = stack.enter_context(open(protocol_file, "rb"))

            # Open all labware files
            labware_file_objects = [stack.enter_context(open(f, "rb")) for f in custom_labware]

            # Build the files list for requests
            files = [("files", f_protocol)] + [("files", f) for f in labware_file_objects]

            # Print the names of all files being uploaded
            print("uploading files:", files)

            # Upload to robot
            resp = requests.post(f"{self.base_url}/protocols", headers=self.headers, files=files)
            resp.raise_for_status()
            protocol_info = resp.json()["data"]

        protocol_id = protocol_info["id"]
        # Print results
        return protocol_id

    def create_run(self, protocol_id: str) -> str:
        """Create a run for the given protocol. Returns run_id."""
        resp = requests.post(
            f"{self.base_url}/runs",
            headers=self.headers,
            json={"data": {"protocolId": protocol_id}}
        )
        resp.raise_for_status()
        run_id = resp.json()["data"]["id"]
        print(f"Created run ID: {run_id}")
        return run_id

    def start_run(self, run_id: str):
        """Start (play) the run."""
        resp = requests.post(
            f"{self.base_url}/runs/{run_id}/actions",
            headers=self.headers,
            json={"data": {"actionType": "play"}}
        )
        resp.raise_for_status()
        print(f"Run {run_id} started.")

    def stop_run(self, run_id: str):
        """Stop a running run."""
        resp = requests.post(
            f"{self.base_url}/runs/{run_id}/actions",
            headers=self.headers,
            json={"data": {"actionType": "stop"}}
        )
        resp.raise_for_status()
        print(f"Run {run_id} stopped.")

    def get_run_status(self, run_id: str) -> str:
        """Get the current status of a run."""
        resp = requests.get(f"{self.base_url}/runs/{run_id}", headers=self.headers)
        resp.raise_for_status()
        return resp.json()["data"]["status"]

    def run_protocol(self, protocol_path: str, poll_interval: float = 5.0, custom_labware_folder: str = None):
        """Upload, create, start, and monitor a protocol until completion."""
        custom_labware = self.verify_labware(protocol_path, custom_labware_folder)
        protocol_id = self.upload_protocol(protocol_path, custom_labware)
        run_id = self.create_run(protocol_id)
        self.start_run(run_id)

        print("Monitoring run...")
        while True:
            status = self.get_run_status(run_id)
            print(f"Run status: {status}")
            if status in ("succeeded", "failed", "stopped"):
                break
            time.sleep(poll_interval)
        print(f"Run finished with status: {status}")
        return status, run_id
    
    def get_protocols(self):
        """List all uploaded protocols."""
        resp = requests.get(f"{self.base_url}/protocols", headers=self.headers)
        resp.raise_for_status()
        return resp.json()["data"]
    

class OT2ClientWithStop(OT2Client):
    def run_protocol(self, protocol_path: str, poll_interval: float = 5.0, custom_labware_folder: str = None):
        """
        Upload, create, start, and monitor a protocol.
        Allows stopping the run by pressing Enter.
        Returns (final_status, run_id)
        """
        # Upload and create run
        print("searching for custom labware in published protocol file...")
        custom_labware = self.verify_labware(protocol_path, custom_labware_folder)
        print("found custom labware:", custom_labware)

        print("uploading protocol...")
        protocol_id = self.upload_protocol(protocol_path, custom_labware)
        print("uploaded protocol with ID:", protocol_id)

        print("creating run...")
        run_id = self.create_run(protocol_id)
        print("created run with ID:", run_id)

        print("starting run...")
        self.start_run(run_id)

        stop_flag = {"stop": False}

        # Thread that waits for user input to stop
        def stop_listener():
            input("Press Enter to stop the run at any time...\n")
            stop_flag["stop"] = True

        threading.Thread(target=stop_listener, daemon=True).start()

        print("Monitoring run...")
        while True:
            status = self.get_run_status(run_id)
            print(f"Run status: {status}")

            if stop_flag["stop"]:
                print("Stop requested! Stopping run...")
                self.stop_run(run_id)
                status = self.get_run_status(run_id)  # confirm stopped
                break

            if status in ("succeeded", "failed", "stopped"):
                break

            time.sleep(poll_interval)

        print(f"Run finished with status: {status}")
        return status, run_id


class OT2ClientROS(OT2Client):
    def __init__(self, ip: str):
        super().__init__(ip)
        self.stop_flag = {"stop": False}
        self.current_run_id = None
    
    def request_stop(self):
        """Method to request stopping the current run from external sources like ROS topics"""
        self.stop_flag["stop"] = True
    
    def run_protocol(self, protocol_path: str, poll_interval: float = 5.0, custom_labware_folder: str = None):
        """
        Upload, create, start, and monitor a protocol.
        Can be stopped by calling request_stop() method.
        Returns (final_status, run_id)
        """
        # Reset stop flag for new run
        self.stop_flag = {"stop": False}
        self.current_run_id = None
        
        # Upload and create run
        print("searching for custom labware in published protocol file...")
        custom_labware = self.verify_labware(protocol_path, custom_labware_folder)
        print("found custom labware:", custom_labware)

        print("uploading protocol...")
        protocol_id = self.upload_protocol(protocol_path, custom_labware)
        print("uploaded protocol with ID:", protocol_id)

        print("creating run...")
        run_id = self.create_run(protocol_id)
        self.current_run_id = run_id
        print("created run with ID:", run_id)

        print("starting run...")
        self.start_run(run_id)

        print("Monitoring run...")
        while True:
            status = self.get_run_status(run_id)
            print(f"Run status: {status}")

            if self.stop_flag["stop"]:
                print("Stop requested! Stopping run...")
                self.stop_run(run_id)
                status = self.get_run_status(run_id)  # confirm stopped
                break

            if status in ("succeeded", "failed", "stopped"):
                break

            time.sleep(poll_interval)

        print(f"Run finished with status: {status}")
        self.current_run_id = None
        return status, run_id
