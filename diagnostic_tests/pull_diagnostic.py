import os
import sys
import subprocess
import time

def kill_macos_hijackers():
    print("===============================================================")
    print("[!] IMPORTANT: Ensure 'Preview' and 'Photos' are CLOSED.")
    print("===============================================================\n")
    print("[*] Clearing macOS background USB locks...")
    subprocess.run("killall -9 PTPCamera", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    subprocess.run("killall -9 'Image Capture Extension'", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(2) 

def run_diagnostic():
    kill_macos_hijackers()

    # The UUID from your screenshot (The working mission)
    target_uuid = "73171D20-24B0-4230-95CA-D60862024A92"
    # The UUID we successfully pushed in the last run
    pushed_uuid = "9C525F72-1EF1-4FB1-BC14-8EE9D28DA3DC"

    print("[*] Fetching the master file index from RC 2 (this may take 10-20 seconds)...")
    files_result = subprocess.run("mtp-files", shell=True, capture_output=True, text=True)

    if "LIBMTP PANIC" in files_result.stdout or "Access denied" in files_result.stderr:
        print("[-] LIBMTP PANIC: Please unplug, wait 5s, plug back in, and retry.")
        return

    # Save the massive dump locally so we can manually review it if needed
    with open("mtp_files_dump.txt", "w") as f:
        f.write(files_result.stdout)
    print("[+] Saved complete file index to mtp_files_dump.txt")

    # Parse the output to extract File IDs, Sizes, and Parent Directory IDs
    lines = files_result.stdout.split('\n')
    
    current_file_id = None
    current_filename = None
    current_size = None
    
    target_details = {}
    pushed_details = {}

    print("[*] Scanning index for our target UUIDs...")
    for line in lines:
        line = line.strip()
        
        # Safe string parsing
        if line.startswith("File ID:"):
            current_file_id = line.split(":", 1)[1].strip()
        elif line.startswith("Filename:"):
            current_filename = line.split(":", 1)[1].strip()
        elif line.startswith("File size"):
            # libmtp outputs "File size 44 bytes" (no colon)
            current_size = line.replace("File size", "").replace("bytes", "").strip()
        elif line.startswith("Parent ID:"):
            current_parent = line.split(":", 1)[1].strip()

            # When we hit the parent ID, we have a complete block of file metadata
            if current_filename and current_filename.endswith(".kmz"):
                if target_uuid in current_filename:
                    target_details = {"id": current_file_id, "name": current_filename, "size": current_size, "parent": current_parent}
                elif pushed_uuid in current_filename:
                    pushed_details = {"id": current_file_id, "name": current_filename, "size": current_size, "parent": current_parent}

    print("\n---------------- DIAGNOSTIC RESULTS ----------------")
    if target_details:
        print(f"[+] FOUND Working Mission ({target_uuid}):")
        print(f"    File ID:   {target_details['id']}")
        print(f"    Parent ID: {target_details['parent']}")
        print(f"    Size:      {target_details['size']} bytes")
    else:
        print(f"[-] Could not find working mission {target_uuid} in the file index.")

    print("")

    if pushed_details:
        print(f"[+] FOUND Pushed Mission ({pushed_uuid}):")
        print(f"    File ID:   {pushed_details['id']}")
        print(f"    Parent ID: {pushed_details['parent']}")
        print(f"    Size:      {pushed_details['size']} bytes")
    else:
        print(f"[-] Could not find the mission we pushed earlier ({pushed_uuid}).")
    print("----------------------------------------------------\n")

    # If we found the working mission, pull it to the Mac
    if target_details:
        local_filename = f"PULLED_{target_uuid}.kmz"
        print(f"[*] Pulling the working mission from the RC 2 to your Mac...")
        
        pull_cmd = f"mtp-getfile {target_details['id']} {local_filename}"
        pull_result = subprocess.run(pull_cmd, shell=True, capture_output=True, text=True)

        if pull_result.returncode == 0 or "Getting file" in pull_result.stdout:
            print(f"\n[+] Successfully pulled {local_filename}!")
            print("[*] Next step: Rename the file extension to .zip, unzip it, and review the XML files inside.")
        else:
            print(f"[-] Failed to pull file. Output:\n{pull_result.stdout}\n{pull_result.stderr}")

if __name__ == "__main__":
    run_diagnostic()