import subprocess
import re
import time

def kill_macos_hijackers():
    subprocess.run("killall -9 PTPCamera", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    subprocess.run("killall -9 'Image Capture Extension'", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(1)

def scan_all_missions():
    kill_macos_hijackers()
    
    print("[*] Fetching folders...")
    folders = subprocess.run("mtp-folders", shell=True, capture_output=True, text=True).stdout
    
    # 1. Find the map_preview folder ID
    preview_id = None
    for line in folders.split('\n'):
        if "map_preview" in line:
            preview_id = line.split()[0]
            break
            
    # 2. Find ALL UUID folders
    uuid_matches = re.findall(r'^\s*(\d+)\s+([A-F0-9]{8}-[A-F0-9]{4}-[A-F0-9]{4}-[A-F0-9]{4}-[A-F0-9]{12})\b', folders, re.MULTILINE | re.IGNORECASE)
    
    if not uuid_matches:
        print("[-] Could not find any mission UUID folders.")
        return
        
    print(f"[+] Found {len(uuid_matches)} UUID folders.")
    print("[*] Fetching file index (this takes a few seconds)...")
    files = subprocess.run("mtp-files", shell=True, capture_output=True, text=True).stdout
    file_blocks = files.split("File ID: ")
    
    for dummy_folder_id, dummy_uuid in uuid_matches:
        dummy_uuid = dummy_uuid.upper()
        print(f"\n---------------- MISSION: {dummy_uuid} ----------------")
        
        # Extract files sitting inside this folder
        found_files = False
        for block in file_blocks:
            if f"Parent ID: {dummy_folder_id}" in block:
                filename_match = re.search(r'Filename:\s*(.+)', block)
                size_match = re.search(r'File size\s*(\d+)', block)
                if filename_match and size_match:
                    print(f"  [Folder] -> {filename_match.group(1).strip()} ({size_match.group(1)} bytes)")
                    found_files = True
        
        # Check map preview for this UUID
        if preview_id:
            for block in file_blocks:
                if f"Parent ID: {preview_id}" in block and dummy_uuid in block:
                    filename_match = re.search(r'Filename:\s*(.+)', block)
                    size_match = re.search(r'File size\s*(\d+)', block)
                    if filename_match and size_match:
                        print(f"  [Preview Image] -> {filename_match.group(1).strip()} ({size_match.group(1)} bytes)")
                        found_files = True
                        
        if not found_files:
            print("  (Folder is empty - Likely a ghost folder from a failed script run)")
            
    print("\n--------------------------------------------------------------\n")

if __name__ == "__main__":
    scan_all_missions()