from ot2_client import OT2ClientWithStop

OT2_IP = "169.254.122.228"
protocol_file = "/home/emil/Self-driving-Labs/opentrons/protocols/clanker_protocol_test.py"


client = OT2ClientWithStop(OT2_IP)
status, run_id = client.run_protocol(protocol_file)
print(f"Protocol finished with status: {status}, run ID: {run_id}")