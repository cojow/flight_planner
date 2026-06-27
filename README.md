# Flight_planner
An open-source flight planning software for DJI drones. The Mavic 3M model is the only confirmed model to work currently.

## Running the Planner
Download this repository.

Install the dependencies with the following code.
```
pip install -r requirements.txt
```

Launch the app by running the following in the terminal. 
```
streamlit run app.py
```

The app will open in your web browser using a local host.

## Using the Flight Planner
There are three parts to the planner:
1. **The Creator**: Use this tool to create your mission flights. 
2. **The Editor**: Edit previously made missions here. 
3. **The Viewer**: View one or more missions at a time.

## How To Plan a Flight

#### Selecting a Location
Once the app is running, the map will be centered at BYU in Provo Utah. From here you can scroll out, click and drag the map to your desired location. You may also use the search bar to jump to a specific address or latitude and longitude. 

On the leftside of the map, select the "Draw a Polyline" icon at the top of the shapes. Use the tool to draw your flight plan. Clicking will create a point that will become a waypoint. Once you have your route finished, click on the last point to finish the line. The last two icons will allow you to edit the line, or deleat the line to start over. The flight line must be continuous. The app will give you the option to make more, but doing so will mess with the flight plan. 

#### Setting Parameters 
Before or after the line is created

IMPORTANT: 

Every waypoint will be assigned an elevation relative to the first one.

#### Global Config
* File name (Dynamically appends the altitude, gimbal pitch, and overlap to the final saved filename as _HxxAxxOLxx)
* Safe Take off Altitude (ft)
* Take off Speed (mph)
* Save Destination (Choose between the root missions directory, creating a new directory, or outputting to a custom absolute file path)

#### Waypoint Settings
* Relative Altitude (ft)
* Elevation Source (Select between Open-Elevation, USGS 3DEP API, or Local GeoTIFF files for terrain-following calculations)
* Gimbal Pitch
* Side (Aircraft takes pictures to the left or right side of the path)
* Auto calculates estimated Ground Sample Distance (GSD) in cm/px

#### Hardware & Payload
* Drone Type (Currently optimized for the Mavic 3M)
* Sensor mode (RGB, Multispectral, or both)

#### Trigger & Speed
* Start Photos at Waypoint Index (Delays the camera trigger until a designated waypoint is reached. 0 defaults to the first waypoint.)
* Type of photo interval (By distance or time)
* Interval length (ft)
* Overlap (How much of one picture will be in the next one. This and the interval are connected. Also affected by the altitude, but doesn't edit it)
* Flight Speed (mph) (Includes auto-calculation for time-based intervals and visual warnings if the speed exceeds the sensor's minimum photo interval. 4 mph is recommended if following the drone.)

#### Map & Visual Features
* FAA Airspace Restrictions (Toggles a live ArcGIS overlay showing LAANC grid ceilings)
* Image Footprints (Projects the camera's field of view onto the map based on altitude, pitch, and drone heading)
* GeoTIFF Boundaries (Displays the bounding box of local surface files to ensure the flight path is contained within the data)
* Dynamic distance and elevation difference markers rendered between waypoints
* Coordinate data editor table for fine-tuning exact latitude and longitude values

#### Viewer Capabilities
* Inspect individual or multiple combined DJI KMZ flight plans
* View aggregated mission statistics including total distance and total estimated photos
* Automatically parses and displays mission metadata directly from the KMZ files