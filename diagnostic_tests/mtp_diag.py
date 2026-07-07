import subprocess
import time
import os
import re

def run_diagnostic():
    print("=== DJI RC 2 MTP SILENT FAILURE DIAGNOSTIC ===")
    
    # 1. Kill Mac hijackers
    subprocess.run("killall -9 PTPCamera", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    subprocess.run("killall -9 'Image Capture Extension'", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(1)

    # 2. Find a target nest
    print("[*] Finding a dummy mission folder...")
    folders = subprocess.run("mtp-folders", shell=True, capture_output=True, text=True).stdout
    matches = re.findall(r'^\s*(\d+)\s+([A-F0-9]{8}-[A-F0-9]{4}-[A-F0-9]{4}-[A-F0-9]{4}-[A-F0-9]{12})\b', folders, re.MULTILINE | re.IGNORECASE)
    
    if not matches:
        print("[-] No dummy folders found. Aborting.")
        return
        
    target_folder_id = matches[0][0]
    target_uuid = matches[0][1]
    print(f"[+] Targeting Folder ID: {target_folder_id} (UUID: {target_uuid})")

    # 3. Create a local test file
    test_filename = "mtp_test_payload.txt"
    with open(test_filename, "w") as f:
        f.write("DJI_MTP_WRITE_TEST_SUCCESSFUL")
    
    local_size = os.path.getsize(test_filename)
    print(f"[*] Created local test payload ({local_size} bytes).")

    # 4. Push the payload
    print("[*] Pushing payload to controller via MTP...")
    push_cmd = f'mtp-sendfile "{test_filename}" "test_payload.txt" {target_folder_id}'
    push_run = subprocess.run(push_cmd, shell=True, capture_output=True, text=True)
    
    if push_run.returncode != 0:
        print("[-] MTP strictly refused the transfer. Output:")
        print(push_run.stderr)
        return
        
    print("[+] MTP reported successful transfer. Initiating FUSE cooldown (3s)...")
    time.sleep(3)

    # 5. Verify it actually exists on the controller
    print("[*] Asking controller for file index...")
    files = subprocess.run("mtp-files", shell=True, capture_output=True, text=True).stdout
    
    found_on_device = False
    device_file_id = None
    
    file_blocks = files.split("File ID: ")
    for block in file_blocks:
        if f"Parent ID: {target_folder_id}" in block and "test_payload.txt" in block:
            found_on_device = True
            device_file_id = block.split()[0].strip()
            size_match = re.search(r'File size\s*(\d+)', block)
            device_size = size_match.group(1) if size_match else "Unknown"
            print(f"[+] CRITICAL SUCCESS: File physically exists on controller! (Size: {device_size} bytes)")
            break
            
    if not found_on_device:
        print("[-] CRITICAL FAILURE: mtp-sendfile lied. The file is NOT on the controller's hard drive.")
        print("    Conclusion: Android FUSE is dropping identical-name overwrites.")
    
    # Cleanup
    if device_file_id:
        subprocess.run(f"mtp-delfile -n {device_file_id}", shell=True, stderr=subprocess.DEVNULL)
    if os.path.exists(test_filename):
        os.remove(test_filename)

if __name__ == "__main__":
    run_diagnostic()