#!/usr/bin/env python3

import cmd
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


    def verify_labware(self, node, protocol_path, custom_labware_folder):
        labware_names = self.extract_labware_names(protocol_path)
        
        node.get_logger().info(f"Checking labware in uploaded protocol...")

        custom_labware = []

        for name in labware_names:
            if self.check_labware_in_custom_folder(name, custom_labware_folder):
                node.get_logger().info(f"Found {name} in custom labware folder")
                custom_labware.append(os.path.join(custom_labware_folder, name + ".json"))
            else:
                node.get_logger().warn(f"{name} not found in custom labware folder, if labware is not in the Opentrons database, protocol may fail.")

        return custom_labware


    def upload_protocol(self, node, protocol_file, custom_labware):
        node.get_logger().info(f"Uploading protocol {os.path.basename(protocol_file)} with {len(custom_labware)} custom labware files...")
        with ExitStack() as stack:
        # Open protocol file
            f_protocol = stack.enter_context(open(protocol_file, "rb"))

            # Open all labware files
            labware_file_objects = [stack.enter_context(open(f, "rb")) for f in custom_labware]

            # Build the files list for requests
            files = [("files", f_protocol)] + [("files", f) for f in labware_file_objects]

            # Print the names of all files being uploaded
            #print("uploading files:", files)

            # Upload to robot
            resp = requests.post(f"{self.base_url}/protocols", headers=self.headers, files=files)
            resp.raise_for_status()
            protocol_info = resp.json()["data"]

        protocol_id = protocol_info["id"]
        # Print results
        return protocol_id


    def create_run(self, node, protocol_id: str) -> str:
        """Create a run for the given protocol. Returns run_id."""
        resp = requests.post(
            f"{self.base_url}/runs",
            headers=self.headers,
            json={"data": {"protocolId": protocol_id}}
        )
        resp.raise_for_status()
        run_id = resp.json()["data"]["id"]
        node.get_logger().info(f"Created run ID: {run_id}")
        return run_id


    def start_run(self, node, run_id: str):
        """Start (play) the run."""
        resp = requests.post(
            f"{self.base_url}/runs/{run_id}/actions",
            headers=self.headers,
            json={"data": {"actionType": "play"}}
        )
        resp.raise_for_status()
        node.get_logger().info(f"Run {run_id} started.")


    def stop_run(self, node, run_id: str):
        """Stop a running run."""
        resp = requests.post(
            f"{self.base_url}/runs/{run_id}/actions",
            headers=self.headers,
            json={"data": {"actionType": "stop"}}
        )
        resp.raise_for_status()
        node.get_logger().info(f"Run {run_id} stopped.")


    def get_run_status(self, run_id: str) -> str:
        """Get the current status of a run."""
        resp = requests.get(f"{self.base_url}/runs/{run_id}", headers=self.headers)
        resp.raise_for_status()
        return resp.json()["data"]["status"]
    
    def get_commands(self, run_id: str):
        """Get the list of commands for a run."""
        resp = requests.get(f"{self.base_url}/runs/{run_id}/commands", headers=self.headers)
        resp.raise_for_status()

        data = resp.json()["data"]

        current = resp.json()["links"]["current"]["meta"]["commandId"]

        commands = []

        for cmd in data:
            commands.append(cmd["id"])
        return commands, current
    


    def run_protocol(self, node, protocol_path: str, custom_labware_folder: str = None):
        """Upload, create, start, and monitor a protocol until completion."""
        custom_labware = self.verify_labware(node, protocol_path, custom_labware_folder)
        protocol_id = self.upload_protocol(node, protocol_path, custom_labware)
        run_id = self.create_run(node, protocol_id)
        self.start_run(node, run_id)
        status = self.get_run_status(run_id)
        #print(status)

        return status, run_id
    
    
    def get_protocols(self):
        """List all uploaded protocols."""
        resp = requests.get(f"{self.base_url}/protocols", headers=self.headers)
        resp.raise_for_status()
        return resp.json()["data"]
    
