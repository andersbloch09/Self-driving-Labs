#!/usr/bin/env python3

"""
Interactive Protocol Runner
Prompts user for protocol parameters and runs the action caller
"""

import os
import sys
import subprocess
import json

def get_default_protocol_path():
    pkg_dir = os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control')
    return os.path.join(pkg_dir, 'protocols', 'full_protocol.py')

def get_default_labware_folder():
    pkg_dir = os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control')
    return os.path.join(pkg_dir, 'custom_labware')

def prompt_with_default(prompt_text, default_value, value_type=str):
    """Prompt user for input with a default value"""
    while True:
        user_input = input(f"{prompt_text} [{default_value}]: ").strip()
        
        if not user_input:
            return default_value
        
        if value_type == str:
            return user_input
        
        try:
            return value_type(user_input)
        except ValueError:
            print(f"Invalid input. Please enter a valid {value_type.__name__}.")

def list_available_protocols():
    """List available protocols in the protocols directory"""
    pkg_dir = os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control')
    protocols_dir = os.path.join(pkg_dir, 'protocols')
    
    if not os.path.isdir(protocols_dir):
        return []
    
    protocols = [f for f in os.listdir(protocols_dir) if f.endswith('.py')]
    return protocols

def main():
    print("="*70)
    print("Interactive OT-2 Protocol Runner")
    print("="*70)
    print()
    
    # List available protocols
    protocols = list_available_protocols()
    if protocols:
        print("Available protocols:")
        for i, protocol in enumerate(protocols, 1):
            print(f"  {i}. {protocol}")
        print()
    
    # Protocol path
    default_protocol = get_default_protocol_path()
    protocol_input = prompt_with_default(
        "Enter protocol path, filename, or number from above",
        default_protocol,
        str
    )
    
    # Check if user entered a number (selecting from the list)
    pkg_dir = os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control')
    if protocol_input.isdigit() and protocols:
        index = int(protocol_input) - 1
        if 0 <= index < len(protocols):
            protocol_path = os.path.join(pkg_dir, 'protocols', protocols[index])
        else:
            print(f"Invalid protocol number. Using default.")
            protocol_path = default_protocol
    elif os.path.isabs(protocol_input) or protocol_input.startswith('~'):
        # Absolute path or home directory path
        protocol_path = protocol_input
    else:
        # Assume it's a filename in the protocols directory
        protocol_path = os.path.join(pkg_dir, 'protocols', protocol_input)
    
    # Verify protocol exists
    if not os.path.isfile(protocol_path):
        print(f"Warning: Protocol file not found at {protocol_path}")
        proceed = input("Continue anyway? (y/n): ").strip().lower()
        if proceed != 'y':
            print("Aborted.")
            return 1
    
    # Custom labware folder
    default_labware = get_default_labware_folder()
    labware_folder = prompt_with_default(
        "Enter custom labware folder path",
        default_labware,
        str
    )
    
    print()
    print("-"*70)
    print("Protocol Parameters")
    print("-"*70)
    
    # Protocol parameters
    sample_count = prompt_with_default(
        "Number of samples",
        1,
        int
    )
    
    heating_time = prompt_with_default(
        "Heating time (seconds)",
        60,
        int
    )
    
    large_tips_used = prompt_with_default(
        "Large tips already used",
        0,
        int
    )
    
    max_concentration = prompt_with_default(
        "Maximum concentration",
        5.0,
        float
    )
    
    min_concentration = prompt_with_default(
        "Minimum concentration",
        1.0,
        float
    )
    
    # Summary
    print()
    print("="*70)
    print("Configuration Summary")
    print("="*70)
    print(f"Protocol: {protocol_path}")
    print(f"Labware folder: {labware_folder}")
    print(f"Parameters:")
    params = {
        "sample_count": sample_count,
        "heating_time": heating_time,
        "large_tips_used": large_tips_used,
        "max_concentration": max_concentration,
        "min_concentration": min_concentration,
    }
    print(json.dumps(params, indent=2))
    print("="*70)
    print()
    
    # Confirm
    proceed = input("Run protocol with these settings? (y/n): ").strip().lower()
    if proceed != 'y':
        print("Aborted.")
        return 0
    
    # Build command
    action_caller_path = os.path.join(
        os.path.expanduser('~'), 
        'sdl_ws', 
        'src', 
        'ot2_control', 
        'ot2_control', 
        'action_caller.py'
    )
    
    cmd = [
        'python3',
        action_caller_path,
        '--protocol', protocol_path,
        '--labware', labware_folder,
        '--sample-count', str(sample_count),
        '--heating-time', str(heating_time),
        '--large-tips-used', str(large_tips_used),
        '--max-concentration', str(max_concentration),
        '--min-concentration', str(min_concentration),
    ]
    
    print()
    print("Running action caller...")
    print()
    
    # Run the action caller
    try:
        result = subprocess.run(cmd, check=True)
        return result.returncode
    except subprocess.CalledProcessError as e:
        print(f"Error running action caller: {e}")
        return e.returncode
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130

if __name__ == '__main__':
    sys.exit(main())
