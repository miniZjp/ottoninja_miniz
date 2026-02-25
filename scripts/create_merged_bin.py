#!/usr/bin/env python3
"""
Script to create merged binary file from ESP32 build artifacts.
This makes flashing easier by combining all binary files into one.
"""

import os
import sys
import subprocess
from pathlib import Path

def main():
    # Get the project root directory (parent of scripts folder)
    script_dir = Path(__file__).parent
    project_dir = script_dir.parent
    build_dir = project_dir / "build"
    
    if not build_dir.exists():
        print(f"Error: Build directory not found: {build_dir}")
        print("Please build the project first using 'idf.py build'")
        sys.exit(1)
    
    # Read flash arguments from flash_args file
    flash_args_file = build_dir / "flash_args"
    if not flash_args_file.exists():
        print(f"Error: flash_args not found: {flash_args_file}")
        sys.exit(1)
    
    with open(flash_args_file, 'r') as f:
        lines = f.readlines()
    
    # Parse flash_args
    flash_settings = lines[0].strip()
    flash_files = []
    for line in lines[1:]:
        line = line.strip()
        if line:
            parts = line.split(maxsplit=1)
            if len(parts) == 2:
                offset, filepath = parts
                flash_files.append((offset, filepath))
    
    # Extract flash parameters from first line
    flash_params = {}
    for param in flash_settings.split():
        if param.startswith('--'):
            key_value = param.split(maxsplit=1)
            if len(key_value) == 2:
                key = key_value[0].strip('-')
                flash_params[key] = key_value[1]
    
    flash_mode = flash_params.get('flash_mode', 'dio')
    flash_freq = flash_params.get('flash_freq', '80m')
    flash_size = flash_params.get('flash_size', '16MB')
    
    # Try to find esptool.py
    esptool_cmd = "esptool.py"
    
    # Check if esptool.py is available
    try:
        subprocess.run([sys.executable, "-m", "esptool", "version"], 
                      capture_output=True, check=True)
        # Use python -m esptool instead
        use_module = True
        print("Using esptool module")
    except:
        use_module = False
        print("Using esptool.py command")
    
    # Output merged binary filename
    output_file = build_dir / "ninjaottoAI.bin"
    
    # Build esptool merge_bin command
    if use_module:
        cmd = [sys.executable, "-m", "esptool"]
    else:
        cmd = [esptool_cmd]
    
    cmd.extend([
        "--chip", "esp32s3",
        "merge-bin",
        "-o", str(output_file),
        "--flash-mode", flash_mode,
        "--flash-freq", flash_freq,
        "--flash-size", flash_size
    ])
    
    # Add flash files with their offsets
    for offset, filepath in flash_files:
        abs_file_path = build_dir / filepath
        if abs_file_path.exists():
            cmd.extend([offset, str(abs_file_path)])
        else:
            print(f"Warning: File not found: {abs_file_path}")
    
    # Execute esptool command
    print("=" * 80)
    print("Creating merged binary file...")
    print("=" * 80)
    print(f"Flash mode: {flash_mode}")
    print(f"Flash freq: {flash_freq}")
    print(f"Flash size: {flash_size}")
    print()
    print(f"Files to merge:")
    for offset, filepath in flash_files:
        print(f"  {offset}: {filepath}")
    print()
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True, cwd=build_dir)
        print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        
        print()
        print("=" * 80)
        print(f"✅ SUCCESS! Merged binary created:")
        print(f"   {output_file}")
        print()
        if output_file.exists():
            print(f"File size: {output_file.stat().st_size / 1024 / 1024:.2f} MB")
        print()
        print("To flash the merged binary:")
        print(f"   esptool.py --chip esp32s3 --port COMX write_flash 0x0 build\\ninjaottoAI.bin")
        print()
        print("Or using idf.py:")
        print(f"   idf.py -p COMX write_flash 0x0 build\\ninjaottoAI.bin")
        print("=" * 80)
        
    except subprocess.CalledProcessError as e:
        print(f"Error executing esptool: {e}")
        if e.stdout:
            print(e.stdout)
        if e.stderr:
            print(e.stderr, file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print(f"Error: esptool.py not found")
        print("Please install esptool: pip install esptool")
        sys.exit(1)

if __name__ == "__main__":
    main()
