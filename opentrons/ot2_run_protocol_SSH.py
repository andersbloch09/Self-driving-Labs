import paramiko
import os

# --- Connection info ---
OT2_HOST = "169.254.122.228"  
OT2_USER = "root"
REMOTE_PATH = "/data/user_storage" 
PRIVATE_KEY_PATH = "/home/emil/.ssh/id_rsa"  # adjust if different

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

key = paramiko.RSAKey.from_private_key_file(PRIVATE_KEY_PATH)

try:
    ssh.connect(
        OT2_HOST,
        username=OT2_USER,
        pkey=key
    )
    stdin, stdout, stderr = ssh.exec_command("echo Hello from OT-2")
    print(stdout.read().decode())
finally:
    ssh.close()