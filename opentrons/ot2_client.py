import requests
import time
import os
import threading

class OT2Client:
    def __init__(self, ip: str):
        self.base_url = f"http://{ip}:31950"
        self.headers = {"opentrons-version": "2"}

    def upload_protocol(self, protocol_path: str) -> str:
        """Upload a Python protocol file to the OT-2. Returns protocol_id."""
        if not os.path.isfile(protocol_path):
            raise FileNotFoundError(f"Protocol file not found: {protocol_path}")

        with open(protocol_path, "rb") as f:
            files = {"files": f}  # must be "files"
            resp = requests.post(f"{self.base_url}/protocols", headers=self.headers, files=files)
            resp.raise_for_status()
            protocol_info = resp.json()["data"]
            print(f"Uploaded protocol: {protocol_info['metadata']['protocolName']}")
            return protocol_info["id"]

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

    def run_protocol(self, protocol_path: str, poll_interval: float = 5.0):
        """Upload, create, start, and monitor a protocol until completion."""
        protocol_id = self.upload_protocol(protocol_path)
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

class OT2ClientWithStop(OT2Client):
    def run_protocol(self, protocol_path: str, poll_interval: float = 5.0):
        """
        Upload, create, start, and monitor a protocol.
        Allows stopping the run by pressing Enter.
        Returns (final_status, run_id)
        """
        # Upload and create run
        protocol_id = self.upload_protocol(protocol_path)
        run_id = self.create_run(protocol_id)
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
