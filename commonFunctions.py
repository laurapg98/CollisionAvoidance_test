from math import asin,cos,pi,sin
from dronekit import *
import json


def deg2rad(angle):
    return angle*pi/180

def download_mission(cmds):
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    missionlist=[]
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist


def save_mission(aFileName, cmds):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist = download_mission(cmds)
    output='QGC WPL 110\n'
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        file_.write(output)


def rad2deg(angle):
    return angle*180/pi

def truncate(n, decimals=0):
    multiplier = 10 ** decimals
    return int(n * multiplier) / multiplier


def pointRadialDistance(lat1, lon1, bearing, distance):
    """
    Return final coordinates (lat2,lon2) [in degrees] given initial coordinates
    (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]
    """
    rEarth = 6371.01 # Earth's average radius in km
    epsilon = 0.000001 # threshold for floating-point equality

    rlat1 = deg2rad(lat1)
    rlon1 = deg2rad(lon1)
    degreeBearing = ((360-bearing)%360)
    rbearing = deg2rad(degreeBearing)
    rdistance = (distance)  / rEarth # normalize linear distance to radian angle

    rlat = asin(sin(rlat1) * cos(rdistance) + cos(rlat1) * sin(rdistance) * cos(rbearing) )

    if cos(rlat) == 0 or abs(cos(rlat)) < epsilon: # Endpoint a pole
        rlon=rlon1
    else:
        rlon = ( (rlon1 - asin( sin(rbearing)* sin(rdistance) / cos(rlat) ) + pi ) % (2*pi) ) - pi

    lat = rad2deg(rlat)
    lon = rad2deg(rlon)
    return LocationGlobal(lat, lon,0)


def edit_mission_values(mission_values, distance, spaceDistance, width, spaceBtwLines, height, latFlight, lonFlight, headingFlight):
    if distance is not None:
        mission_values['distance'] = distance
    if spaceDistance is not None:
        mission_values['spaceDistance'] = spaceDistance
    if width is not None:
        mission_values['width'] = width
    if spaceBtwLines is not None:
        mission_values['spaceLines'] = spaceBtwLines
    if height is not None:
        mission_values['height'] = height
    if latFlight is not None:
        mission_values['latFlight'] = latFlight
    if lonFlight is not None:
        mission_values['lonFlight'] = lonFlight
    if headingFlight is not None:
        mission_values['headingFlight'] = headingFlight

    return mission_values

def edit_mission_json(mission_values):
    try:
        with open('./scripts/mission_parameters.json', 'r+') as f:
            data = []
            data = json.load(f)
            data['mission_info'] = mission_values
            f.seek(0)
            json.dump(data, f, ensure_ascii=False)
            f.truncate()
            f.close()
    except:
        with open( './scripts/mission_parameters.json', 'w') as f:
            data_json = {
                "mission_info": {},
            }
            data_json['mission_info'] = mission_values
            f.seek(0)
            json.dump(data_json, f, ensure_ascii=False)
            f.truncate()
            f.close()



    




