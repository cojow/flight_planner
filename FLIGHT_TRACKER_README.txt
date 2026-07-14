FLIGHT LOG TRACKER - HOW TO USE
================================

What this is
-------------
A simple app separate from the main Flight Planner. It reads every .kmz
flight file in a folder and plots them all on a single map. 
When connected to a shared folder (from Box, Google Drive, etc.),
every user can see the combined history of flights that have
actually been flown. It does NOT edit or create missions - it's read-only
(other than saving files you upload into the folder).


ONE-TIME SETUP (For Box)
---------------
1. Install Box Drive (box.com/drive) if you don't already have it, and sign
   in with your Box account.
2. Get added as a collaborator on the shared folder or create your own.
3. Once shared, that folder will show up as a normal folder on your
   computer through Box Drive - usually somewhere like:
     Mac:     /Users/<you>/Library/CloudStorage/Box-Box/Flight Logs
     Windows: C:\Users\<you>\Box\Flight Logs
   The exact path depends on how Box Drive is set up on your machine - just
   find the folder in Finder/Explorer and note its full path.
4. IMPORTANT: Box Drive can "stream" files instead of keeping them fully
   downloaded, which can make the tracker slow to read a folder the first
   time. Right-click the shared folder in Box Drive and choose "Available
   Offline" (or the equivalent) so its contents are always kept synced
   locally. This isn't required, but it makes things noticeably snappier.


RUNNING THE TRACKER
--------------------
From the flight_planner project folder, with the same Python environment
used for the main app, run:

    streamlit run flight_tracker.py

This opens a new browser tab, separate from the main Flight Planner app.
Both apps can run at the same time (they'll just need different ports if
you launch them together - Streamlit will offer you one automatically).


THE INTERFACE
--------------
  - The map: All flight plans within the folder are visible and displayed on the map.
  Hover over the line to see information. 
  - Top-right: the ⚙️ Settings button opens a small panel with the shared
    folder path, a Browse button, and the file uploader. Below it is a
    "Show FAA restrictions" checkbox.
  - Bottom-right: the flight count and a "📋 Flight List" button. This lists the 
  flight date, name, type (Fly or Pilot), Height, Pitch, overlap, and photo count. 


FIRST-TIME CONFIGURATION
--------------------------
The first time you run it, open the ⚙️ Settings panel (top-right) and point
it at your shared folder:

  - Paste the full path to your local Box-synced "Flight Logs" folder into
    the "Shared flight folder" box, OR
  - Click "Browse" to pick it visually.

By default only .kmz files directly inside that folder are read. If your
team organizes flights into subfolders (by date, location, etc.), check
"Include subdirectories" to also scan every folder inside it.

The tracker remembers these choices (saved in a small local file called
.tracker_config.json) so you won't need to set them again next time you
run it on this machine. Each person sets this once, pointing at their own
Box Drive's copy of the same shared folder.


UPLOADING A COMPLETED FLIGHT
------------------------------
Open ⚙️ Settings and drag your .kmz file(s) onto the "Upload completed
flight(s)" box (or click it to browse), then let Box Drive sync in the
background. Anyone else pointed at the same shared folder will see it
appear next time they load or refresh the tracker.

If you upload a file with the exact same name as one that's already there,
it's saved alongside it with a timestamp added to the name rather than
overwriting the existing one - so you'll never accidentally lose someone
else's upload.


READING THE MAP
-----------------
Each flight shows up as one colored line tracing its path, with small
wedges spaced periodically along it showing roughly which way the camera
was facing at that point. Hover your mouse over a line to see that
flight's details:
  - Name and platform (Fly or Pilot)
  - Date flown
  - Number of photos
  - Height, gimbal pitch, and overlap

This intentionally leaves out the clutter the main app's Viewer shows
(individual waypoints, per-photo image footprints, elevation/distance
labels) - just enough to see where you've flown and glance at the specs.

Check "Show FAA restrictions" (top-right) to overlay the same FAA UAS
Facility Map (LAANC ceiling) grid the main app's Creator/Editor/Viewer
show - green means no restriction, red means a 0 ft LAANC ceiling.


THE FLIGHT LIST
-----------------
Click "📋 Flight List" (bottom-right) to slide up a panel covering the
bottom half of the map, with a sortable table of every flight. Use the search
box at the top of that panel to filter by typing part of a flight's name
or its date (e.g. "Testy" or "2026-07") - either one works in the same
box. Press Enter to apply the search. Click the ✕ to close the panel.


A NOTE ON HEIGHT / PITCH / OVERLAP
-------------------------------------
These are read straight from the filename the main app saves missions
with (e.g. "Mission_Fly_H100A90OL75.kmz" = 100 ft, 90 degree pitch, 75%
overlap). If a file was renamed to something that doesn't follow that
pattern, the tracker falls back to reading height and pitch directly out
of the file - overlap can't be recovered that way, so it'll show "n/a" for
files renamed outside the app's convention.


TROUBLESHOOTING
-----------------
"No flights found yet" - double check the folder path is correct and that
Box Drive has actually finished syncing the .kmz files down to your
machine (see the "Available Offline" tip above).

A flight is missing from the map / shows under "couldn't be read" - the
.kmz is either corrupted, empty, or isn't a mission file the main app
generated. The error message next to it in that section explains why.

Folder path looks wrong after Browse - the native folder picker returns
whatever you selected; if Box Drive shows a slightly different path than
you expected, that's usually just how your OS mounts the Box Drive volume.
