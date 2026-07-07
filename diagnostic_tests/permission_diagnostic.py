import subprocess

def diagnostic():
    # 1. Ask mtp-folders for deep details on the waypoint folder (375)
    print("[*] Inspecting permissions of Parent ID 375...")
    # Using 'mtp-folders -v' (verbose) if available, otherwise just raw dump
    result = subprocess.run(["mtp-folders"], capture_output=True, text=True)
    
    # 2. Specifically look for the subfolder we created (12031)
    # This checks if the controller even considers 12031 a valid storage child
    print("[*] Searching for child folder 12031 in file system...")
    if "12031" in result.stdout:
        print("[+] SUCCESS: Controller still sees folder 12031 as a valid object.")
    else:
        print("[-] FAILURE: Controller has 'forgotten' folder 12031 (The bouncer deleted it).")
        
    # 3. Check if we can list files inside the parent
    print("[*] Attempting to list files in Parent ID 375 (to verify Read capability)...")
    # mtp-files often lists everything; we filter for the parent ID
    files = subprocess.run(["mtp-files"], capture_output=True, text=True)
    
    # Check if any files exist in 375
    parent_files = [line for line in files.stdout.split('\n') if "Parent ID: 375" in line]
    print(f"[*] Found {len(parent_files)} existing files in the waypoint directory.")

if __name__ == "__main__":
    diagnostic()