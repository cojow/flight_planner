# Flight_planner
An open-source flight planning software for DJI drones. Specifically Mavic 3M model currently.

## Running the Planner
Download the repository

Install the dependencies
```
pip install -r requirements.txt
```

Run the following in the terminal 
```
streamlit run app.py
```

The app will open in your web browser. Draw your mission path, set paramaters, then click "save and generate KMZ"

Currenlty there is a Creator where you make the missions, and Editor where you can edit your made images, and a Viewer where you can see the paramaters for mission.

## Current Editable parameters

### Global Config
- File name
- Safe Take off Altitude (ft)
- Take off Speed

### Waypoint Settings
- Realative Altitude (ft)
- Gimble Pitch
- Side (Aircraft takes pictures to the left or right of the path)
-Auto calcuatates cm/px-

### Hardware & Payload
- Drone Type (Only Mavic 3M and 3E are currenlty availible)
- Sensor mode (RBG, MS, or both)

### Trigger & Speed
- Type of photo interval (By distance or time)
- Interval length
- Overlap (How much of one picture will be in the next one. This and the interval are connected. Also affected by the altitude, but doesn't edit it)
- Flight Speed (mph)