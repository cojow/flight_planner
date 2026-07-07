import os
import sys
import uuid
import subprocess
import time
import re

def kill_macos_hijackers():
    """Kills the Apple background processes that lock the USB MTP port."""
    print("===============================================================")
    print("[!] IMPORTANT: Ensure macOS 'Preview', 'Photos', and ")
    print("    'Android File Transfer' apps are fully CLOSED before running.")
    print("===============================================================\n")
    print("[*] Clearing macOS background USB locks...")
    subprocess.run("killall -9 PTPCamera", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    subprocess.run("killall -9 'Image Capture Extension'", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(2) 

def get_dynamic_waypoint_id():
    """Dumps the folder tree and uses Regex to extract the waypoint ID."""
    print("[*] Finding the dynamic waypoint folder ID for this session...")
    try:
        result = subprocess.run("mtp-folders", shell=True, capture_output=True, text=True)
        full_output = result.stdout + "\n" + result.stderr
    except Exception as e:
        print(f"[-] Failed to run mtp-folders: {e}")
        return None

    if "LIBMTP PANIC" in full_output or "Access denied" in full_output:
        print("[-] LIBMTP PANIC DETECTED: The USB port is locked by the macOS kernel.")
        print("    Fix: Unplug the RC 2, wait 5 seconds, plug it back in, and try again.")
        return None

    match = re.search(r'^\s*(\d+)\s+waypoint\b', full_output, re.MULTILINE | re.IGNORECASE)
    
    if match:
        return match.group(1)
    else:
        print("[-] Regex failed to find 'waypoint' in the MTP dump.")
        return None

def transfer_mission(local_kmz_path):
    if not os.path.exists(local_kmz_path):
        print(f"[-] Error: Could not find local file '{local_kmz_path}'")
        return

    kill_macos_hijackers()
    
    waypoint_id = get_dynamic_waypoint_id()
    if not waypoint_id:
        return
        
    print(f"[+] Found waypoint folder (Current ID: {waypoint_id})")
    time.sleep(1.5)

    mission_uuid = str(uuid.uuid4()).upper()
    print(f"[*] Generated Mission UUID: {mission_uuid}")

    # 3. Create the folder
    print(f"[*] Creating folder on RC 2...")
    create_cmd = f'mtp-newfolder "{mission_uuid}" {waypoint_id} 0'
    create_result = subprocess.run(create_cmd, shell=True, capture_output=True, text=True)
    
    match = re.search(r'ID:\s*(\d+)', create_result.stdout)
    if match:
        new_folder_id = match.group(1)
        print(f"[+] Folder created successfully (New ID: {new_folder_id})")
    else:
        print(f"[-] Failed to create folder. Controller output:\n{create_result.stdout}\n{create_result.stderr}")
        return
            
   # 1. Strip macOS metadata that causes Android MTP to choke
    print("[*] Stripping macOS extended attributes...")
    subprocess.run(f'xattr -c "{local_kmz_path}"', shell=True)
    
    # 2. Brute-force the FUSE cooldown. 
    # Android 11+ is notoriously slow at registering new folders in /Android/data/
    print("[*] Initiating 6-second cooldown for Android's internal database...")
    time.sleep(6)
    
    # 3. Force the hardware refresh
    print("[*] Forcing Android MTP database refresh...")
    subprocess.run("mtp-folders", shell=True, capture_output=True, text=True)
    
    # Give the MTP daemon one final breath before the file push
    time.sleep(2)

    # 4. Push the KMZ file (Removed the trailing '0' here)
    remote_filename = f"{mission_uuid}.kmz"
    print(f"[*] Pushing {os.path.basename(local_kmz_path)} to RC 2...")
    
    push_cmd = f'mtp-sendfile "{local_kmz_path}" "{remote_filename}" {new_folder_id}'
    push_result = subprocess.run(push_cmd, shell=True, capture_output=True, text=True)
    
    if push_result.returncode == 0 or "Sent file" in push_result.stdout:
        print("\n[+] TRANSFER COMPLETE! ")
        print(f"[*] The mission is now located inside: /waypoint/{mission_uuid}/")
        print("[*] Boot up DJI Fly to check your mission.")
    else:
        print(f"[-] Transfer failed. Output:\n{push_result.stdout}\n{push_result.stderr}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python bridge.py <path_to_your_mission.kmz>")
        sys.exit(1)
    
    transfer_mission(sys.argv[1])