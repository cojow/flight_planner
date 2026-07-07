import subprocess
import re
import time
import sys

def kill_macos_hijackers():
    print("[*] Temporarily pausing macOS USB camera background services...")
    subprocess.run("killall -9 PTPCamera", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    subprocess.run("killall -9 'Image Capture Extension'", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(1)

def scan_and_clean_ghosts():
    kill_macos_hijackers()
    
    print("[*] Scanning controller folder index...")
    folders_run = subprocess.run("mtp-folders", shell=True, capture_output=True, text=True)
    if "LIBMTP PANIC" in folders_run.stdout or "Access denied" in folders_run.stderr:
        print("[-] MTP Connection Busy/Panic. Unplug the controller, wait 5 seconds, replug, and retry.")
        return

    folders_output = folders_run.stdout
    
    # Match standard UUID format folders (Group 1: Folder ID, Group 2: UUID Name)
    uuid_matches = re.findall(r'^\s*(\d+)\s+([A-F0-9]{8}-[A-F0-9]{4}-[A-F0-9]{4}-[A-F0-9]{4}-[A-F0-9]{12})\b', folders_output, re.MULTILINE | re.IGNORECASE)
    
    if not uuid_matches:
        print("[+] No mission UUID folders found on the controller.")
        return

    print(f"[*] Found {len(uuid_matches)} total UUID folders. Scanning internal contents...")
    files_output = subprocess.run("mtp-files", shell=True, capture_output=True, text=True).stdout
    file_blocks = files_output.split("File ID: ")

    empty_ghosts = []
    populated_missions = []

    for folder_id, uuid_name in uuid_matches:
        # Check if any file block explicitly reports this folder_id as its Parent ID
        has_files = any(f"Parent ID: {folder_id}" in block for block in file_blocks)
        
        if has_files:
            populated_missions.append((folder_id, uuid_name.upper()))
        else:
            empty_ghosts.append((folder_id, uuid_name.upper()))

    # --- REPORT SUMMARY ---
    print("\n================== SCAN RESULTS ==================")
    print(f"[+] Active/Populated Missions Found ({len(populated_missions)}):")
    for fid, uname in populated_missions:
        print(f"    - ID: {fid:<6} | UUID: {uname} (SAFE - Kept)")
        
    print(f"\n[-] Empty Ghost Folders Found ({len(empty_ghosts)}):")
    if not empty_ghosts:
        print("    No empty folders to clean up! Your controller is already tidy.")
        print("==================================================\n")
        return

    for fid, uname in empty_ghosts:
        print(f"    - ID: {fid:<6} | UUID: {uname}")
    print("==================================================\n")

    # --- CONFIRMATION & DELETION ---
    choice = input(f"Are you sure you want to permanently delete these {len(empty_ghosts)} empty folders? (y/N): ").strip().lower()
    
    if choice != 'y':
        print("[*] Cleanup aborted. No changes were made to the controller.")
        return

    print("\n[*] Deleting ghost folders...")
    deleted_count = 0
    for fid, uname in empty_ghosts:
        print(f"    Removing folder ID {fid} ({uname})...", end=" ")
        # In libmtp tools, mtp-delfile -n <id> deletes the specified object ID
        del_cmd = f"mtp-delfile -n {fid}"
        del_run = subprocess.run(del_cmd, shell=True, capture_output=True, text=True)
        
        if del_run.returncode == 0 or "Deleting file" in del_run.stdout:
            print("DONE")
            deleted_count += 1
        else:
            # Fallback syntax if -n flag isn't supported in your local build of libmtp
            del_fallback = subprocess.run(f"mtp-delfile {fid}", shell=True, capture_output=True, text=True)
            if del_fallback.returncode == 0 or "Deleting file" in del_fallback.stdout:
                print("DONE")
                deleted_count += 1
            else:
                print(f"FAILED ({del_run.stderr.strip() or 'Unknown MTP lock'})")
        time.sleep(0.3) # Short pause to prevent overwhelming the device buffer

    print(f"\n[+] Cleanup complete! Successfully removed {deleted_count} empty ghost folders.")

if __name__ == "__main__":
    scan_and_clean_ghosts()