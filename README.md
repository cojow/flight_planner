# Flight_planner
An open-source flight planning software for DJI drones. 
Plans can be made for both DJI Pilot 2 (Mavic) and DJI Fly (Mini) apps. (DJI Fly is under development)

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
The planner is split into three tabs: The Creator, the Editor, and the Viewer. 
Each have their own applications that are useful as you plan your missions. 

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
* **Drone Platform**: DJI Pilot 2 or DJI Fly. ❗**THIS OPTION IS VERY IMPORTANT**❗, as it will determine how your flight plan is created.
DJI Pilot 2 works for higher-end drones/controllers, such as the Mavic series.
 This option allows for very complicated flight plans with hundreds of photos. 
DJI Fly is the stripped-back version used in the Mini series which doesn't easily support flight plans. 
It is limited to 99 photos per flight plan due to each photo having to be a waypoint. 
*Know which program your drone takes before planning your flight*
* Sensor mode (RGB, Multispectral, or both for 3M only)

##### Global Config
* **File name**: Assign a memorable name. IT will also dynamically append the altitude, gimbal pitch, and overlap to the final saved filename as "_HxxAxxOLxx")
* **Safe Take-off Altitude (ft) and Take off Speed (mph)**: Most often the defaults are adequate for these. 


##### Waypoint Settings
* **Relative Altitude (ft)**: This is based on the drone's take-off point.
❗Important❗: Every waypoint will be assigned an elevation relative to the first one, which will be set to this value. 
Thus, the drone should take off as close as possible to the first waypoint.
* **Elevation Source**: Select between Open-Elevation (high error, Global), USGS 3DEP (Low error, USA only), or Local GeoTIFF files for terrain-following calculations. 
This is very important if your flight plan is not over a flat surface. 
* **Gimbal Pitch**: Angle that the pictures will be taken. 
* **Side**: The aircraft takes pictures to the left or right side of the path

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
Use this to know if you need to submit an LAANC report before flying. (PROCESS TO SUBMIT THIS IS A WIP)

Once your parameters are set and the flight path draw, the total path distance, estimated number of photos to be taken, and the flight speed will appear above the map, along with the * **Save Destination** parameter. 
Choose between the root "missions" directory, creating a new directory within the "missions" root, or outputting to a custom absolute file path. 

After choosing the save destination, save the flight plan by pressing the "save and generate KMZ" button. 

### **The Viewer**

This tab allows the user to view previously made flight plans. 
Features include the following:

* **Mission select**: Inspect individual or multiple combined flight plans in the viewer.
 These are chosen from either the root "missions" folder or other custom folders. 
* **Flight Path Characteristics**: Arrows displaying the direction of the drone flight, the distance between waypoints, and the elevation differences between waypoints. 
* **Image Footprints**: Projects the camera's field of view onto the map based on altitude, pitch, and drone heading. 
Allows one to get an estimate of the area covered by the drone photos. 
This may be turned on or off. 
* **Mission Metadata**: View aggregated mission statistics, including total distance and total estimated photos. 
FAA restrictions may also be turned on or off. 


### **The Editor**  

This tab will allow you to edit previously made flight plans. 
Many of the same parameters in the creator may be edited here, with exceptions noted below. 
May parts of the viewer can also be seen. 

* **Flight Path**: May be edited using the coordinate data editor table to fine-tune exact latitude and longitude values. 
You can not click and drag the waypoints
* **File name**: WIP


## Uploading to UAV
This process is different if using a drone that has DJI Pilot 2 or DJI Fly.

### DJI Pilot 2 
*(This app comes with the enterprise-tier controllers, such as RC Plus and the RC Pro Smart)*

This is a relatively simple process. 
First, export your missions from their saved location and place them onto SD card. 
Second, plug this SD card into your controller, navigate to your flight plans, and hit the import button in the top right corner. 
From here navigate through your SD card storage to your flight plans, select all the ones you want, and upload them. 
They are now ready to be used to fly the UAV.

### DJI Fly
*(This method is currently tested for DJI RC 2 controllers. The phone app is not yet tested.)*

As mentioned before, DJI makes it very difficult to upload pre-made flight plans into their cheaper drones that use this software. 
But it is not impossible. 

The architecture of the file is a stripped back version of the DJI Pilot 2 code, and can not be simply imploted into the controller since it is a highly locked down version of a android phone. 
The current workaround is to use an application name MavenBridge ---> [Link to download](https://www.mavenpilot.com/mavenbridge/)

MavenBridge is a free, closed source application that is able to push a flight plan to RC controllers. 
This is done by overwriting a dummy flight plan that was created on the controller. 
Thus, before using MavenBridge, you will need to have flight plans already created on your controller for the app to override. 

To transfer the files, connect your controller to your computer and open MavenBridge. 
On the left, you will see the controller missions. 
These are saved internally in the controller as a long string of numbers and letters that DO NOT coordinate with what they are named on the controller. 
Keeping straight which flight plans are which is the hardest part of this method. On the top right column you can select the folder icon and navigate to where your missions are saved. 
Now, select the flight plan to be overwritten on the left and the file to overwrite with on the right. 
Then press the arrow pointing left at the top of the right column. 

To access your uploaded flight plan, first connect your controller to your drone. 
Then press the blue "GO" button. 
After that, the left middle side of the screen will show the waypoint mission button (arrow with a S shapped line behind it). 
Pressing this will open up a small window on the bottom of the screen. Press the white square button and this will take you into your flight history. 
Here you can find your flight program under the original name of the dummy flight you overwrote. 

## Acknowledgements
- Luigi Pirelli for providing the photo footprint code base. 
- Mavenbridge