from dronekit import connect, VehicleMode, LocationLocal, LocationGlobalRelative, LocationGlobal, mavutil
import time
import math


def get_bearing(location1, location2):
    """
    Calculates the bearing between two locations.
    """
    lat1, lon1 = math.radians(location1.lat), math.radians(location1.lon)
    lat2, lon2 = math.radians(location2.lat), math.radians(location2.lon)
    d_lon = lon2 - lon1

    x = math.sin(d_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360

    return bearing

def get_distance_metres(location1, location2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        # Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)



def change_mode(vehicle, mode_name):
    while vehicle.mode != VehicleMode(mode_name):
        vehicle.mode = VehicleMode(mode_name)
        time.sleep(1)

def condition_yaw(vehicle, heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW command to point vehicle to a specified heading (in degrees).
    
    :param heading:    Target heading in degrees
    :param relative:   If True, heading is relative to current heading
    """
    if relative:
        is_relative = 1 # yaw relative to direction of travel
    else:
        is_relative = 0 # yaw is an absolute angle

    # Create the CONDITION_YAW command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,           # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,              # confirmation
        heading,        # param 1, yaw in degrees
        0,              # param 2, yaw speed (not used)
        1,              # param 3, direction -1 ccw, 1 cw
        is_relative,    # param 4, relative or absolute
        0, 0, 0)        # param 5 ~ 7 not used

    # Send command to vehicle
    vehicle.send_mavlink(msg)


def drone_run():
    # Connect to UDP endpoint.
    vehicle = connect('127.0.0.1:14550', wait_ready=True)

    # Print vehicle mode info
    print("Mode: %s" % vehicle.mode.name)


   

    # Take off to 100 meters altitude
    arm_and_takeoff(vehicle, 100)

    target_location = LocationGlobal(50.443326, 30.448078, 50) # 100 meters altitude


    change_mode(vehicle, "ALT_HOLD")

    counter = 0

    # Move towards target location
    while True:
        current_location = vehicle.location.global_frame
        bearing = get_bearing(current_location, target_location)
        distance = get_distance_metres(current_location, target_location)
        print("Distance to target: ", distance)

        if distance < 10 or counter > 10:  # Stop if close to target
            print("Reached target location")
            break

        counter += 1

        # need this not to lose altitude while moving
        vehicle.channels.overrides['3'] = 1500

        # Simplified control logic (requires tuning)
        # Adjust these values based on your testing
        if bearing < 180:
            vehicle.channels.overrides['2'] = 1600  # Forward (pitch)
        else:
            vehicle.channels.overrides['1'] = 1550  # Right (roll)

        time.sleep(1)

    # Clear channel overrides
    vehicle.channels.overrides = {}


    change_mode(vehicle, "GUIDED")

    time.sleep(5)

    print('turning yaw 350')

    condition_yaw(vehicle, 350, relative=True)

    time.sleep(10)


    # Land
    print("Landing...")
    change_mode(vehicle, "LAND")

    # close connection
    vehicle.close()

drone_run()