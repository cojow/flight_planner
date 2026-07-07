# Flight_planner
An open-source flight planning software for DJI drones. 
Plans can be made for both DJI Pilot 2 (Enterprise) and DJI Fly (Commercial) apps.

## Running the Planner Steps
1. Download this repository.

2. Install the dependencies with the following code.
```
pip install -r requirements.txt
```

3. Launch the app by running the following in the terminal. 
The app will open in your web browser using a local host.
```
streamlit run app.py
```

## Using the Flight Planner
The mission planner is split into three creation tabs: the *Creator*, the *Editor*, and the *Viewer*. 

The *Photo Sorter* and *DJI Fly Transfer* are tabs designed specifically for getting around the restrictions and annoyances that come with the more affordable commercial drone that use the  DJI Fly app. These are explained in a later section. 


### **The Creator**

#### Selecting a Location
Once the app is running, the map will be centered at BYU in Provo, Utah. 
From here, you can scroll out, click, and drag the map to your desired location. 
You may also use the search bar to jump to a specific address or latitude and longitude. 

On the leftside of the map, select the "Draw a Polyline" icon at the top of the shapes.
Use the tool to draw your flight plan.
Clicking will create a waypoint. 
Once you have your route finished, click on the last waypoint to finish the line. 
The last two icons on the left will allow you to edit the line or delete the line to start over. 
The flight line must be continuous. 
The app will give you the option to make more, but doing so will mess with the flight plan. 

(In the future, the other shapes may be integrated to allow area-cover flights.)

#### Setting Parameters 
Before or after the line is created, the parameters need to be set. 
These are divided into 5 categories: Hardware, Global Config, waypoint settings, triggers and speed, and visuals. 
Each is explained in the following sections.

##### Hardware & Payload
* **Drone Platform**: DJI Fly or DJI Pilot 2. ❗**THIS OPTION IS VERY IMPORTANT**❗, as it will determine how your flight plan is created.
DJI Pilot 2 works for higher-end enterprise drones/controllers, such as the Mavic Enterprise series.
This option allows for very complicated flight plans with hundreds of photos. 
DJI Fly is the stripped-back version used in the commercial series which doesn't easily support flight plans. 
It is limited to 99 photos per flight plan due to each photo having to be a waypoint. 
*Know which program your drone takes before planning your flight*
* Sensor mode (RGB, Multispectral, or both for 3M - DJI Pilot 2 only)

##### Global Config
* **File name**: Assign a memorable name. It will also dynamically append the altitude, gimbal pitch, and overlap to the final saved filename as "_HxxAxxOLxx", as well as whether the mission is designed for DJI Pilot 2 or DJI Fly.
* **Safe Take-off Altitude (ft) and Take off Speed (mph)**: Most often, the defaults are adequate for these. 


##### Waypoint Settings
* **Relative Altitude (ft)**: This is based on the drone's take-off point, starting at zero.
❗Important❗: Every waypoint will be assigned an elevation relative to the first one, which will be set to this value. 
Thus, the drone should take off as close as possible to the first waypoint.
* **Elevation Source**: Select between Open-Elevation (high error, Global application), USGS 3DEP (Low error, USA only), or Local GeoTIFF files for terrain-following calculations. 
This is very important if your flight plan is not over a flat surface. 
* **Gimbal Pitch**: Angle at which the pictures will be taken. 
* **Side**: The aircraft takes pictures to the left or right side of the path.

##### Trigger & Speed
* **Start Photos at Waypoint Index** :Delays the camera trigger until a designated waypoint is reached. 
0 defaults to the first waypoint.
* **Type of photo interval**: By distance or time. Unless you have a specific reason to choose one or the other, use distance. 
* **Interval length (ft/sec)**: Space/time between photoes.
* **Overlap**: How much of one picture will be in the next one. 
This and the interval are connected and will changed dynamically based on each other. 
This is also affected by the altitude, but doesn't edit it.
* **Flight Speed (mph)**: Includes auto-calculation for time-based intervals and visual warnings if the speed exceeds the sensor's minimum photo interval. 
4 mph is recommended when following the drone.

##### Map & Visual Features
* **FAA Airspace Restrictions**: Toggles a live ArcGIS overlay showing LAANC grid ceilings.
Use this to know if you need to submit an LAANC report before flying. 
In app submitting is not available, you must use another app to do so.


##### Main Page Features
* **Save Destination**: Located above the map. Use the drop down box to choose an existing mission or the default root "missions/" directory. You can create a new folder by pressing the "+" button, or browse to a custom folder location with the file icon. 
* **Jump to Address/Latlong**: Located at the bottom of the map. Allows you to center the screen on a specific address or latitude and longitude instead of scrolling to it on the map. 


Once your parameters are set and the flight path is drawn, the total path distance and estimated number of photos to be taken will appear above the map.
Save the flight plan by pressing the "save and generate KMZ" button, also found on this line. 

### **The Viewer**

This tab allows the user to view previously made flight plans. 
Features include:

* **Mission select**: Inspect individual or multiple combined flight plans in the viewer.
 These are chosen from either the root "missions" folder or other custom folders. 
* **Flight Path Characteristics**: Arrows displaying the direction of the drone's flight path, the direction of the drone during flight, the distance between waypoints, and the elevation differences between waypoints. 
* **Image Footprints**: Projects the camera's field of view onto the map based on altitude, pitch, and drone heading. 
Allows one to get an estimate of the area covered by the drone photos. 
This may be turned on or off with a toggle to the left.
* **Mission Metadata**: View aggregated mission statistics, including total distance and total estimated photos. 
FAA restrictions may also be turned on or off with a toggle to the left.


### **The Editor**  

This tab will allow you to edit previously made flight plans. 
Many of the same parameters in the creator may be edited here, with exceptions noted below. 
Many parts of the viewer can also be seen. 
(CURRENTLY DOESN'T WORK WITH DJI FLY MISSIONS)

* **Flight Path**: May be edited using the coordinate data editor table to fine-tune exact latitude and longitude values. 
You can not click and drag the waypoints
* **File name**: WIP



## Using a Enterprise Series Drone (DJI Pilot 2 app)
*(This app comes with the enterprise-tier controllers, such as RC Plus and the RC Pro Smart, and only works with Enterprise series drones)*

This is a relatively simple process. 
First, export your missions from their saved location and place them onto SD card. 
Second, plug this SD card into your controller, navigate to your flight plans, and hit the import button in the top right corner. 
From here navigate through your SD card storage to your flight plans, select all the ones you want, and upload them. 
They are now ready to be used to fly the drone.

## Using a Commercial Series Drone (DJI Fly)
*(This method is currently tested for DJI RC 2 controllers. The phone app is not yet tested.)*

As mentioned before, DJI makes it very difficult to upload pre-made flight plans into their commercial drones by heavily locking down their controllers and removing a native import button. They also dump all mission images into one general folder, making it difficult to distinguish between different flight plans. These issues are circumvented by the *Photo Sorter* and *DJI Fly Transfer* tabs. 


### **Photo Sorter**
This tab allows you to automatically group mission photos into folders based on the times that they were taken. It uses the following inputs:
* **Source Directory**: The location where the drone's image folder is located. 
* **Output Directory**: Where you want the separated mission folders to be saved. The folders are saved with the date and time of the first image in the folder. 
* **Target Date**: What date the sorter will look for when grouping missions. Only photos taken on this date will be sorted. 
* **Time Gap (minutes)**: How long there needs to be between photos for them to be considered as part of different groups/missions. 

### **DJI Fly Transfer**
This tab allows you to transfer DJI Fly missions to a RC 2 controller directly from your computer. It is split into the following sections:

#### 1. Source Missions
Allows you to choose the folder where your missions are located from the root "mission" folder, a created subfolder, or another folder on your computer. Choosing a folder will tell you how many missions are DJI_Fly missions and are available to be transferred. 

#### 2. Controller Nests
Connect your powered on controller directly to your computer via a USB cable and press the scan button. After a few seconds it will tell you how many missions you currently have on the controller, and section 3 will appear. 

If section 3 is not showing, make sure the controller is powered on, and that the preview app (Mac), Android File Transfer tools, and MTP tools are closed and not running. These interfere with the connections.  

#### 3. Assign and Transfer


#### The Manual Thumbnail Refresh Checklist
The controller does not automatically update the thumbnails next to the flight missions. Since 

## Acknowledgements
- Luigi Pirelli for providing the photo footprint code base. 
- Mavenbridge for inspiring the DJI Fly Transfer Code (No MavenBridge code was used in development).