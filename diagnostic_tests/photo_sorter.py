import os
import shutil
from datetime import datetime, timedelta
from PIL import Image

def get_exif_datetime(filepath):
    """Extracts the exact time the photo was taken from EXIF data."""
    try:
        with Image.open(filepath) as img:
            exif = img._getexif()
            if not exif:
                return None
            
            # 36867 is the EXIF tag for DateTimeOriginal
            for tag, value in exif.items():
                if tag == 36867: 
                    return datetime.strptime(value, "%Y:%m:%d %H:%M:%S")
    except Exception as e:
        print(f"Could not read EXIF for {os.path.basename(filepath)}: {e}")
    return None

def group_images_by_time(source_folder, output_folder, target_date_str, gap_minutes=5):
    """
    Filters images by date and groups them into an output folder based on time gaps.
    """
    target_date = datetime.strptime(target_date_str, "%Y-%m-%d").date()
    valid_extensions = {'.jpg', '.jpeg', '.png', '.tif', '.tiff'}
    
    # Ensure the main output directory exists
    os.makedirs(output_folder, exist_ok=True)
    
    # 1. Collect and filter images
    image_data = []
    for filename in os.listdir(source_folder):
        ext = os.path.splitext(filename)[1].lower()
        if ext in valid_extensions:
            filepath = os.path.join(source_folder, filename)
            taken_time = get_exif_datetime(filepath)
            
            if taken_time and taken_time.date() == target_date:
                image_data.append({'path': filepath, 'name': filename, 'time': taken_time})
                
    if not image_data:
        print(f"No images found for date: {target_date_str} in {source_folder}")
        return

    # 2. Sort images chronologically
    image_data.sort(key=lambda x: x['time'])

    # 3. Group images based on the time gap
    groups = []
    current_group = [image_data[0]]
    gap_threshold = timedelta(minutes=gap_minutes)

    for i in range(1, len(image_data)):
        time_diff = image_data[i]['time'] - image_data[i-1]['time']
        
        if time_diff <= gap_threshold:
            current_group.append(image_data[i])
        else:
            groups.append(current_group)
            current_group = [image_data[i]]
            
    # Add the last group
    if current_group:
        groups.append(current_group)

    # 4. Create folders in the OUTPUT directory and copy files
    print(f"Found {len(groups)} distinct groups for {target_date_str}.")
    
    for i, group in enumerate(groups):
        # --- THIS IS THE UPDATED SECTION ---
        # Create a folder name based on the date and time of the first photo in the group
        group_start_datetime = group[0]['time'].strftime("%Y-%m-%d_%H-%M-%S")
        folder_name = f"Group_{i+1}_{group_start_datetime}"
        # -----------------------------------
        
        # Route the new folder to the specific output directory
        folder_path = os.path.join(output_folder, folder_name)
        
        os.makedirs(folder_path, exist_ok=True)
        
        for img in group:
            target_path = os.path.join(folder_path, img['name'])
            # Using copy2 preserves original metadata during the copy
            shutil.copy2(img['path'], target_path) 
            
        print(f"Copied {len(group)} images to {folder_path}")

# --- Execution ---
if __name__ == "__main__":
    SOURCE_DIR = "/Volumes/DJI_Drone/DCIM/DJI_001"
    OUTPUT_DIR = "/Users/connor/Desktop/Sorted_Flights"  # The new destination folder
    TARGET_DATE = "2026-06-30"  # Format: YYYY-MM-DD
    GAP_MINUTES = 5 # Start a new group if photos are more than 5 mins apart
    
    group_images_by_time(SOURCE_DIR, OUTPUT_DIR, TARGET_DATE, GAP_MINUTES)