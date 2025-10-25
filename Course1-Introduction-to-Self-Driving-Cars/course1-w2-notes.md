# Course 1 - W2 - MODULE 2 : Self-Driving Hardware and Software Architectures

## Overview 
- System architectures for self-driving vehicles are extremely diverse, as no standardized solution has yet emerged. 
- This module describes both the hardware and software architectures commonly used and some of the tradeoffs in terms of **cost**, **reliability**, **performance** and **complexity** that constrain autonomous vehicle design.
  
**Course Objectives :**
- Design an omnidirectional **multi-sensor system** for an autonomous vehicle
- Describe the basic architecture of a typical self-driving **software system**
- Break down the **mapping requirements** for self-driving cars based on their intended uses.

## Autonomous Vehicle Hardware, Software and Environment Representation

### Lesson 1: Sensors and Computing Hardware
*[OWN] In this lesson, we will see:*
- *sensor types and characteristics,*
- *self-driving computing hardware available today*

**Sensors** : device that measures or detects a property of the environment, or changes to a/another property overtime.

<img src="./resources/w2/self-driving-cars-sensors.png" width="600" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

- Categorization : 
  - **exteroceptive** : extero=surroundings => they record the property of the environment
  - **proprioceptive** : proprio=internal => they record the property of the `ego-vehicule`

Sensors for perception : 
- `Exteroceptive`
  - **Cameras** : essential for correctly perceiving.
    - *Passive* & light-collecting sensors that are used for capturing rich detailed information about a scene
    - Comparison metrics:
      - `Resolution` (quality of the image) : number of pixels that create the image (l x w)
      - `Field of view (FOV)` : the horizontal and vertical angular extent that is *visible* to the camera (depending lens selection and zoom)
      - `Dynamic Range` : the difference btw the **darkest** and **lightest** tones in an image
        - High Dynamic range (HDR) is essential for self-driving vehicles to navigate in variable lighting conditions, particularly at night.
    - Trade-off btw resolution and FOV ?
      - Wilder `field of view` allows a larger viewing region in the environment but fewer pixels that absorb light from one particular object
      - FOV increases, the resolution needs to be increases as well, to still be able to perceive with the same quality
      - other properties that affect perception : 
        - focal length, depth of field and frame rate
        - further explanation **on course 3 : visual perception**
    - Cameras types :
      - Exteroceptive STEREO Camera: the combination of two cameras with overlapping FOVs and aligned image planes.
      - enables depth estimation from image data (synchronized image pairs)
      - Pixel values from image can be matched to the other image producing a **disparity map** of the scene then be used to estimate depth at each pixel
  - **LiDAR (Light Detection And Ranging)** : involves shooting light beams into the environment and measuring the reflet return
    - By measuring the amount of returned light and time of flight of the beam. Both an intensity and range to the reflecting object can be estimated.
    - LIDAR : includes a spinning element with multiple stacked light sources and outputs a detailed 3D scene geometry from LIDAR point cloud
    - Active sensor w/ it's own light sources therefore not affected by the environments lighting.
    - So LIDAR do not face the same challenges as cameras when operating in poor or variable lighting conditions. 
    - Comparison metrics:
      - **Number of beams** (of sources) : most common sizes 8, 16, 32, 64 
      - **Points per second** : the faster the point collection, the more detailed the 3D point cloud can be.
      - **Rotation Rate** : The higher this rate, the faster the 3D point clouds are updated 
      - **Detection Range** : dictated by the power output of the light source
      - **FOV**
    - Upcoming : Solid state LIDAR ! 
      - High-Resolution Solid-state LIDAR : without a rotational component from typical LIDAR
        - Extremely low-cost and reliable
  - **RADAR (Radio Detection And Ranging)** : old than Lidars and robustly detects **large** objects in the environment
    - They are particularly useful in adverse weather (fog and rain) as they are mostly unaffected by precipitation (poor visibility)
    - Comparison metrics:
      - **Detection Range** 
      - **FOV** 
      - **Position and speed measurement accuracy**
    - Configurations : 
      - Short range : Wide FOV (WFOV)
      - Long range : Narrow FOV (NFOV)
  - **ULTRASONIC/SONARS(Sound Navigation and Ranging)** : measures range using sound waves
    - Short-range all weather distance measurement  
    - applications : 
      - good for low-cost parking scenarios, where ego-vehicle needs to make movement very close to other cars
    -  Unaffected by lighting, precipitation
    - Comparison metrics:
      - Range : The maximum Range they can measure
      - FOV :
      - Cost
  
- `Proprioceptive` : 
  - **GNSS (Global Navigation Satellite Systems)** (examples include): 
    - GPS (Global Positioning System) with variable Range  : 5 to 10m
    - Galileo : more accurated than GPS
    - GNSS receivers are used to measure:
      - ego vehicle position,
      - velocity and
      - (sometimes) heading
    - the accuracy depends a lot on the actual positioning methods and the corrections used: RTK, PPP, DGPS
  
  - **IMU (Inertial Measurement Units)** :   
    - angular rotation rate
    - acceleration 
    - combined measurements can be used to estimate the 3D orientation of the vehicle
      - heading (determined from combined information from IMU, GPS) : the most important of vehicle control
    
  - **WHEEL ODOMETRY** : 
    - Tracks wheel rates of rotation and orientation
    - Uses these to calculate overall speed and orientation of car
      - Speed accuracy
      - position drift (heading rate)
      - tracks the mileage on the vehicle

**Computing Hardware** : the most crucial part is the **computing brain** and the main decision making unit of the car
- Takes in all sensor data
- computes actions
- Companies tend to go for custom hardware, but already existing advanced systems that do self driving car processing exist
  - Eg : NVIDIA Drive Px/AGX, Intel & Mobileye EyeQ
- computer brain needs both serial and parallel compute modules 
  - Image LiDAR processing: to do segmentation, Object detection, Mapping
    - GPUs : Graphic Processing Unit 
    - FPGAs : Field Programmable Gate Array
    - ASICs: Application Specific Integrated Chip
- Hardware synchronization : 
  - To synchronize different modules and provide **common clock**
  - sensor measurements must be timestamped with consitent times for `Sensor Fusion` to function correctly
  - GPS relies on extremely accurate timing and act as an appropriate reference clock when available

*[OWN]*
***Summary:*** 
  - *Sensors (proprioceptive and exteroceptive - camera, LIDAR, RADAR, SONAR/ultrasonics, GNSS, IMU, wheel odometry) and,*
  - *self-driving computing hardware.*

### Lesson 1 Supplementary Reading: Sensors and Computing Hardware

[The ME597 - Autonomous Mobile Robotics Course at the University of Waterloo](http://wavelab.uwaterloo.ca/sharedata/ME597/ME597_Lecture_Slides/ME597-4-Measurement.pdf)

### Lesson 2: Hardware Configuration Design
*[OWN]*
Recall commonly available sensors from the previous lesson:
- These are the camera for appearance input.
- The stereo camera for depth information,
- lidar for all weather 3D input,
- radar for object detection,
- ultrasonic for short-range 3D input and
- GNSS/IMU data and wheel odometry for ego state estimation.

Sensor coverage requirement for different scenarios : 
- Highway driving
- Urban driving
  - Overall coverage, blind spots

**Assumptions** : we define the deceleration driving which will drive the detection ranges need for the sensors
- **Aggressive deceleration**  = `5 m/s^2` 
  - when braking hard in case of emergency
- **Comfortable deceleration** = `2 m/s^2`
  - This is the norm, unless otherwise stated 
-  ## Stopping distance: **$\frac{v^2 }{2a}$**
  - *[OWN] To make sense of this formula, imagine a vel-time graph and current vel = $v$ m/s. Area under the curve for this graph is the distance. If we assume constant deceleration, we have a right-triangle, with height = $v$, and base is the time required to come to a stop. This can be determined from the definition of acceleration, which is `delta_v / delta_t`. If you plug in the expression for `delta_t` from this formula into the area of the triangle, you can get this formula.*
  - *[OWN] You could also factor in reaction time and road friction but we're keeping things simple for now.*

    where : d - the distance
            v - the vehicle velocity
            a - the rate of of deceleration

**Where to place sensors ?**
- Need sensors to support maneuvers within ODD 
  - ODD our system can produce decision for, or,
  - We shall be able to provide all of the decision with suffient input to accommodate our ODD
- Broadly, we have two driving environments
  - Highway driving
  - Urban driving

|--| Highway | Urban/Residential|
|--|--|--|
|Traffic Speed |High|Low - Medium|
|Traffic Volume |High|Medium - High|
|# of lanes|More |2-4 typically|
|Other features|Fewer, gradual curves; merges|Many turns and intersections|

`Highway Analysis` : 
- Broadly, 3 kinds of maneuvers : 
  1. Emergency Stop : 
     - Longitudinal Coverage : If there is a blockage ahead, we want to stop in time 
      - Applying the stopping distance eq : `v  = 120 kmph => d = 110 m` (aggressive deceleration)
      - Most self-driving systems aim for sensing ranges of a `150 to 200 meters` in front of the vehicle as a result.
     - Lateral Coverage : To avoid collision in our lane, or to change lanes in general,
       - Be able to sense at least adjacent lanes (3.7 meters wide in North America), since we may change lanes to avoid a **hard stop**
     
  2. Maintain Speed : *[OWN] To maintain speed during vehicle following, we need to sense the vehicle in our own lane. Both their relative position and the speed are important to maintain a safe following distance.* 
     - relative speeds are typically less than 30kmph
     - Longitudinal Coverage :
       - `2s` is the reaction time in Nominal conditions for human drivers: 2s (it can be accessible in aggressive deceleration of vehicle in front and the our ego-vehicle behind)
       - At 120kph ==> 65m of relative distance is needed, and typical systems usually maintain 100m to meet this requirement.
       - Both vehicles are moving, so don't need to look as far as emergency-stop case

     - Lateral Coverage : Maintain speed with merge 
       - Usually current lane (ego-vehicle)
       - Adjacent lanes would be preferred for merging vehicle detection 
       - A wide 160 to 180 degree FOV is required to track adjacent lanes and a range of 40 to 60m is needed to find space btw vehicles
  
  3. Lane Change
      - We want to move safely to an adjacent lane (left or rigth). To perform lane change, we require the following sensing.
      - Longitudinal coverage : Need to look forward to maintain a safe distance from the leading vehicle.
        - But also needs to look behind to look what others vehicles are doing
      - Lateral Coverage : Need wider sensing
        - We need to look not just in the adjacent lanes, but probably further
        - What if ? Other vehicle attemps to maneuver lane at the same time as we do?
          -  We'll need to coordinate our lane changes room maneuvers so we don't crash
        - The sensing requirements for lane change are roughly equivalent to those in the maintain speed scenario : front, behind and side of ego-vehicle

- Overall Sensor Coverage Requirements for the Highway Driving Scenario
<img src="./resources/w2/overallcoverage-sensors.png" width="540" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

`Urban Aanalysis`
- Broadly, 6 kinds of maneuveres: 
  1. Emergency Stop
  2. Maintain Speed
  3. Lane Change
    - The sensor coverage analysis for these first three maneuvers is the same as the highway scenario, but since we're not moving as quickly we don't need the same extent for our long-range sensing.

  4. Overtaking *(a parked car)*
    - Longitudinal converage : 
      - need to sense the parked car as well as look for oncoming traffic.
      - So, we need both sensors, wide short-range sensors to detect the parked car and narrow long-range sensors to identify if oncoming traffic is approaching.
    - Lateral coverage : 
      - Always need to observe beyond just the adjacent lanes to check for other merging vehicles.

  5. Turning, crossing at intersections
     - Intersections : 
       - Observe behoyond intersection for approaching vehicles, pedestrian crossings, clear exit lanes.
       - Requires near omnidirectional sensing for arbitrary intersection angles

  6. Passing roundabouts(rondpoints)
    - Longitudinal coverage :
      -  Due to the shape of the roundabout, need a wider FOV
    - Lateral coverage : 
      - Vehicles are slower than usual, limited range requirement, but wider FOV.

- Overall Coverage urban
  
<img src="resources/w2/overallcoverage-sensors-urban.png" width="540" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

The final choice of configuration also depends on : 
- requiremnt of operating conditions and the driving scenarios we might be in and the maneuvers we may want to take
- should contain both longitudinal and lateral coverage
- sensor redundacy due to failure and budgets
- budget

### Lesson 2 Supplementary Reading: Hardware Configuration Design

- [K.J. Bussemaker's master's thesis: Sensing requirements for an automated vehicle for highway and rural environments](https://repository.tudelft.nl/islandora/object/uuid:2ae44ea2-e5e9-455c-8481-8284f8494e4e)


### Lesson 3: Software Architecture

<img src="./resources/w2/sw-stack.png" width="540" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

Software stack for self-driving cars :

**Environment Perception**
- Localization : localizing the ego-vehicle in space
  - inputs : 
    - GPS location
    - IMU measurements
    - Wheel odometry
  - outputs : 
    -  accurate vehicle position
  - for greater/better accuracy a Lidar and camera data can be incorporated

- Classifying and locating the important elements of the environment: This is divided into two segments:
  - Dynamic Object Detection (DOD)
    - inputs: 
      - GPS/IMU/Odometry
      - Lidar
      - cameras
    - outputs :
      - 3D Bounding Boxes : encode the class or type of object, position, orientation, and size of the object
    - Once dynamic objects are detected, they are tracked using Dynamic Object Tracking (DOT). 
      - inputs : Bounding Boxes
      - outputs : Object is tracked (position and history path in the environment)
    - The history of the path is used along with the roadmaps to predict the future path of all dynamic objects, Object Motion Prediction (OMP).
      - inputs : Objects tracks (from DOT), current environment (roadmap)
      - outputs: Predicted path of the identified dynamic objects.
      - ML/DL predictive model (???)
  - Static Object Detection (SOD) : 
    - inputs: HD Road Map (LIDAR + Cameras)
    - Outputs : Static objects in the environment (current lane, location of regulatory elements like signs and traffic lights)


**Environment Mapping** : creates several different types of representation of the current environment around the autonomous car. There are three types of maps that we briefly discuss:

Occupancy Grid Map : map of all static objects in the environment surrounding the vehicle. 
- inputs: 
  - objects tracks
  - LIDAR data (used to construct the occupancy grid map)
    - after a filtering are applied in the input data to make it usable by the occupancy grid, e.g. drivable surface points and dynamic object points are removed.
- outputs : 
  - Occupancy Grid Map : set of cells, with each cell having a probability representing occupancy state (references : postural monitoring project)

Localization Map : used by the localization module in order to improve ego* state estimation. Sensors data are compared to the localization map while driving to determine the motion of car relative to the localiztion map. This motion is combined with other proprioceptive sensor information to accurately localize the ego vehicle.
- inputs: 
  - LIDAR, camera data is used to create this map.
- outputs : 
  - Localization Map

Detailed Road Map : provides the map of road segments representing the driving env. It captures signs and lane markings so that this map can be used by the motion planning module. It is created from prerecorded data and as well as incoming information from the current static environment gathered by the perception stack.
- inputs: 
  - Prior Road Map
  - Vehicle position
  - Segmented image
  - Static objects
- outputs :
  - Detailed Road Map  
- This modules interacts constantly with the perception module to improve the performance of both modules
  - eg : the perception provides the static env need to update the detailed road map => prediction module => accurate dynamic object predictions

**Motion Planning** : challenging task and hard to solve in a single integrate processs. Needs to be decomposed into several layers of abstraction

- Mission Planner (Top level) : performs long-term planning and defines a mission over entire horizon of the driving task
   - inputs: 
     - Current Goal
     - Detailed Road Map
     - Vehicle Position
   - outputs(graphs) : sequences of road segments that connect the origin <=> destination and passes to the next layer (Behavior Planner)
     - complete mission

- Behavior Planner : solves shorts term planning problems
  - estabilishes a set of safe actions/maneuvers to be executed while travelling along the mission path
    - Eg : whether the vehicle Shall merge into an adjacent lane given the desired speed and predicted behaviors of nealy vehicles
  - inputs: 
    - Detailed Road Map
    - Missin Path
    - Dynamics objects
    - Occupancy Grid
  - outputs : 
    - Maneuver decision
    - behaviors Contraints 
  
- Local Planner : performs immediate or reactive planning, and is responsible for defining a specific path and velocity profile to drive.
  - inputs:
    - Occupancy Grid    
    - behaviors Contraints 
    - Vehicle operating limits
    - Dynamic objects in the env
  - outputs : 
    - Planned trajectory which is a combined desired path and velocity profile for a short period of time into the future. 

**Controller**: takes a trajectory plan turns it into a set of precise actuation commands for vehicle to apply 
- Velocity Controller (Longitudinal) : 
  - inputs : 
    - planned trajectory 
    - vehicle position 
  - outputs :
    - regulated the Throttles, gears, braking system to achieve a correct velocity
    - Error of tracking  performance of local plan and adjust the current actuation cmds to minimize errors going forward 

- Steering Controller (Lateral)
  - inputs :
    - planned trajectory 
    - vehicle position 
  - outputs:
    - Steering Angle
    - Error of tracking performance of local plan and adjust angles

**System Supervisor**: continuously moritoring of all aspect of the ego-vehicle gives the appropriate warning in the event of the subsystem failure 

- HW supervisor : 
  - monitors all hw components to check for any fault : sensors failure, missing measurements, degraded information.
  - analyse hw output for any outputs which do not match the domain which the self-driving car was programmed to perform under.
    - For example, if one of the camera sensors is blocked by a paper bag or if snow is falling and corrupting the LIDAR point cloud data.

- SW supervisor : 
  - responsible for SW stack validation; all pipelines (env perception, mapping, planning, controller) are running at the right frequencies and providing complete outputs.
  - analyzes output inconsistency results of all modules


### Lesson 3 Supplementary Reading: Software Architecture

- [Software architecture from the Team VictorTango - DARPA Urban Challenge Technical Paper](https://www.romela.org/wp-content/uploads/2015/05/Odin-Team-VictorTango%e2%80%99s-Entry-in-the-DARPA-Urban-Challenge.pdf)
- [OWN: another link for the same resource] (https://www.semanticscholar.org/paper/DARPA-Urban-Challenge-Technical-Paper-Reinholtz-Alberi/c10acd8c64790f7d040ea6f01d7b26b1d9a442db?p2df)

### Lesson 4: Environment Representation

Environment Map Types : 
1. Localization of vehicle in the environment

    - Localization point cloud or feature map
    - data comes from Lidar or camera features
    - usage : 
      - In combination of GPS/IMU/Wheel Odometry by the localization module to estimate the vehicle location
  
2. Collision avoidance with static objects
     - Occupancy grid map
     - Uses Lidar points to build map for static/stationary objects
     - used to plan safe collision-free paths for self-driving cars

3. Path planning
   - Detailed road map
   - contains all regulatory elements, attributes and lane markings

#### **Localization Map**

<img src="./resources/w2/localization-map.png" width="540" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

- Collects continuous sets if LIDAR
- The different btw LIDAR maps is used to calculate the movement of the autonomous vehicle
- The movement of the vehicle is based on the evolution of the LIDAR points
  
#### **Occupancy grid map**

<img src="./resources/w2/grid-map-final.png" width="400" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

The gray rectangle is the grip map

- Discretized fine grain grid map
  - Can be 2D or 3D
- uses LIDAR points
- Occupancy by static object
  - Tree, buildings, curbs and so on
- Curbs and other non drivable surfaces
  - Dynamic objects are removed (by removing all lidar points that are found within the bounding boxes od DOD in the **perception** module)

- each grid cell is represented by a probability value (100%: occupied or 0% : free)
  
<img src="./resources/w2/grid-map.png" width="400" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

- only the relevant writer points from statics objects remain

<img src="./resources/w2/grid-map-covered-occupied-free.png" width="400" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

- the filtering process is not perfect 

#### **Detailed Roadmap**

- Used to plan a safe and efficient path to be taken by the self-driving car

<img src="./resources/w2/detailed-map.png" width="400" style="border:0px solid #FFFFFF; padding:1px; margin:1px">

- 3 Methods of creation : 
  - `Fully Online`
    - uses statics objects of the perception stack to accuretely label and localize all relevant static objects to create the map such as : lane boundaries, traffic regulation(signs and lights), regulations attributes on the lanes (right turns marking or crosswalks)
    - rarely used due the complexity
  - `Fully Offline`
    - created from collectiong data of given road several times
    - specialized vehicles with high accuracy sensors are driven along roadways regularly to construct offline maps
    - after the collection is complete the information is then labelled with the use of automatic labelling from static object perception and human annotation and correction

  - `Created Offline and Update Online`
    - Created Offline and Update Online with new relevant information
    - creating a highly accurate roadmap which can be updated while driving 
    - better than offline and online methods 

### Lesson 4 Supplementary Reading: Environment Representation

- ["Lanelets: Efficient map representation for autonomous driving paper by P. Bender, J. Ziegler and C. Stiller"](https://ieeexplore.ieee.org/abstract/document/6856487) 


### The Future of Autonomous Vehicles
- OK
# References

- [Self-driving papers](https://www.semanticscholar.org/paper/DARPA-Urban-Challenge-Technical-Paper-Reinholtz-Alberi/c10acd8c64790f7d040ea6f01d7b26b1d9a442db?p2df#related-papers)

