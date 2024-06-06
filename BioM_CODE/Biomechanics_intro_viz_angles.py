# -*- coding: utf-8 -*-
"""
@author: Damion
"""


import kineticstoolkit.lab as ktk
import kineticstoolkit as ktk
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import scipy as stats


#%% Reading files + print markers

'''
it's ideal to print the markers to know what you're working with if the data 
was not collected by you.
'''

filename = "ADD_file.c3d"
markers = ktk.read_c3d(filename)["Points"]


p = ktk.Player(markers)

p.up = "z"
p.anterior = "-y"


p.target = (0.0, 0.0, 1.0)
p.azimuth = 3.1416 / 4  # pi/4 = 45 deg
p.elevation = 3.1416 / 6  # pi/6 = 30 deg
p.zoom = 0.75

markers.data.keys()

interconnections = dict()

#to find each marker
print("Marker Labels:")
for marker_label in markers.data.keys():
    print(marker_label)


#%% vizualization

'''
Interconnections can be simpler. However, my system only allowed me to use two 
links a the time.
'''

#                                              lower LimB


interconnections["LLowerLimb"] = {
    "Color": (0, 0.5, 1), 
    "Links": [  
        ["*LTOE","*LMANK", "*LHEE", "*LANK", "*LTOE"],
        ["*LANK", "*LTIB", "*LKNE", "*LTHI","*LASI","*LMKNE","*LMKNE", "*LMANK"],
        

    ],
}

interconnections["RLowerLimb"] = {
    "Color": (0, 0.5, 1), 
    "Links": [  
        ["*RTOE", "*RMANK", "*RHEE", "*RANK", "*RTOE"],
        ["*RANK", "*RTIB", "*RKNE", "*RTHI", "*RASI", "*RMKNE", "*RMKNE", "*RMANK"],
        
    ],
}


#                                               TrunkPelvis


interconnections["TrunkPelvis1"] = {
    "Color": (0.5, 1, 0.5),
    "Links": [
        ["*LPSI", "*LASI", "*RASI", "*RPSI", "*LPSI"],
        ["*RSHO", "*CLAV", "*LSHO", "*C7", "*RSHO"], 

    ],
}



interconnections["TrunkPelvis2"] = {
    "Color": (0.5, 1, 0.5),
    "Links": [
        ["*C7","*T10", "*LPSI","*T10", "*RPSI"], 
        ["*CLAV", "*STRN", "*RASI", "*STRN", "*LASI"],
    ],
}

interconnections["TrunkPelvis3"] = {
    "Color": (0.5, 1, 0.5),
    "Links": [
        ["*RSHO","*RASI"], 
        ["*LSHO","*LASI"], 
        
    ],
}

#                             RightLeft TrunkPelvis LowerLimb Connection

interconnections["RLTrunkPelvisLLimB"] = {
    "Color": (0.5, 1, 0.5),
    "Links": [
        ["*LKNE", "*LPSI"],
        ["*RKNE", "*RPSI"], 

    ],
}

#                               UpperLimb

interconnections["LUpperLimb"] = {
    "Color": (0, 0.5, 1),
    "Links": [
        ["*LSHO", "*LELB", "*LMELB", "*LSHO"],
        ["*LFRM", "*LELB", "*LMELB", "*LFRM", ],
    ],
}

interconnections["LUpperLimb2"] = {
    "Color": (0, 0.5, 1),
    "Links": [
        ["*LFRM","*LWRA", "*LFIN", "*LWRB", "*LFRM"],
        ["*LWRA", "*LWRB" ],
    ],
}


interconnections["LUpperLimbRot"] = {
    "Color": (0, 0.5, 1),
    "Links": [
        ["*LSHO","*LUPA","*LFRM", "*LFIN"],
        ["*LWRA", "*LWRB" ],
    ],
}


#                                 Right Upper Limb

interconnections["RUpperLimb"] = {
    "Color": (0, 0.5, 1),
    "Links": [
        ["*RSHO", "*RELB", "*RMELB", "*RSHO"],
        ["*RFRM", "*RELB", "*RMELB", "*RFRM"],
    ],
}

interconnections["RUpperLimb2"] = {
    "Color": (0, 0.5, 1),
    "Links": [
        ["*RFRM", "*RWRA", "*RFIN", "*RWRB", "*RFRM"],
        ["*RWRA", "*RWRB"],
    ],
}

interconnections["RUpperLimbRot"] = {
    "Color": (0, 0.5, 1),
    "Links": [
        ["*RSHO", "*RUPA", "*RFRM", "*RFIN"],
        ["*RWRA", "*RWRB"],
    ],
}



#                                 Head


interconnections["Head"] = {
    "Color": (1, 0.5, 1),
    "Links": [
        ["*C7", "*LFHD", "*RFHD", "*C7", "*LBHD", "*RBHD", "*C7"],
        ["*LBHD", "*LFHD", "*RFHD", "*RBHD", "*LBHD"],
    ],
}


p = ktk.Player(
    markers,
    interconnections=interconnections,
    up="z",
    anterior="-y",
    target=(0, 0.5, 1),
    azimuth=0.1,
    zoom=1.5, 
)

p.point_size = 8
p.default_point_color = 'r'

p.interconnection_width = 3.5


#%% Arm Coodrinate system Step A

#                                                                   Arm    R 

originArmR = markers.data["RSHO"]

yArmR = markers.data["RSHO"] - 0.5 * (
    markers.data["RELB"] + markers.data["RMELB"]
)

yzArmR = markers.data["RELB"] - markers.data["RMELB"]


framesArmR = ktk.TimeSeries(time=markers.time)

framesArmR.data["ArmR"] = ktk.geometry.create_frames(origin=originArmR, y=yArmR, yz=yzArmR)

p.set_contents(markers.merge(framesArmR))


#                                                                   Arm     L


originArmL = markers.data["LSHO"]

yArmL = markers.data["LSHO"] - 0.5 * (
    markers.data["LELB"] + markers.data["LMELB"]
)

yzArmL = markers.data["LELB"] - markers.data["LMELB"]


framesArmL = ktk.TimeSeries(time=markers.time)

framesArmL.data["ArmL"] = ktk.geometry.create_frames(origin=originArmL, y=yArmL, yz=yzArmL)

p.set_contents(markers.merge(framesArmL))

#%% FArm Coodrinate system Step B

#                                                      Farm =      ForeArm  R


originFarmR = markers.data["RWRB"]

yFarmR = (
    0.5
    * (markers.data["RELB"] + markers.data["RMELB"])
    - markers.data["RWRB"]
)

yzFarmR = markers.data["RWRA"] - markers.data["RWRB"]

framesFarmR = ktk.TimeSeries(time=markers.time)

framesFarmR.data["ForeArmR"] = ktk.geometry.create_frames(origin=originFarmR, y=yFarmR, yz=yzFarmR)

p.set_contents(markers.merge(framesFarmR))


#                                                      Farm =      ForeArm  L

originFarmL = markers.data["LWRB"]  

yFarmL = (
    0.5
    * (markers.data["LELB"] + markers.data["LMELB"])
    - markers.data["LWRB"]
)

yzFarmL = markers.data["LWRA"] - markers.data["LWRB"]  

framesFarmL = ktk.TimeSeries(time=markers.time)

framesFarmL.data["ForeArmL"] = ktk.geometry.create_frames(origin=originFarmL, y=yFarmL, yz=yzFarmL)

p.set_contents(markers.merge(framesFarmL))



#%% extracting angles Forearm


arm_to_forearmR = ktk.geometry.get_local_coordinates(
    framesFarmR.data["ForeArmR"], framesArmR.data["ArmR"]
)

arm_to_forearmR

arm_to_forearmL = ktk.geometry.get_local_coordinates(
    framesFarmL.data["ForeArmL"], framesArmL.data["ArmL"]
)

arm_to_forearmL

euler_anglesR = ktk.geometry.get_angles(arm_to_forearmR, "ZXY", degrees=True)

euler_anglesR


euler_anglesL = ktk.geometry.get_angles(arm_to_forearmL, "ZXY", degrees=True)

euler_anglesL

# Creating TimeSeries objects for right and left angles
anglesR = ktk.TimeSeries(time=markers.time)
anglesR.data["Elbow flexion"] = euler_anglesR[:, 0]
anglesR.data["Forearm pronation"] = euler_anglesR[:, 2]
anglesR = anglesR.add_data_info("Elbow flexion", "Unit", "deg")
anglesR = anglesR.add_data_info("Forearm pronation", "Unit", "deg")

anglesL = ktk.TimeSeries(time=markers.time)
anglesL.data["Elbow flexion"] = euler_anglesL[:, 0]
anglesL.data["Forearm pronation"] = euler_anglesL[:, 2]
anglesL = anglesL.add_data_info("Elbow flexion", "Unit", "deg")
anglesL = anglesL.add_data_info("Forearm pronation", "Unit", "deg")

# Plotting both sets of angles on the same axis
plt.figure(figsize=(10, 6))
plt.plot(anglesR.time, anglesR.data["Elbow flexion"], label="Right Elbow flexion")
plt.plot(anglesR.time, anglesR.data["Forearm pronation"], label="Right Forearm pronation")
plt.plot(anglesL.time, anglesL.data["Elbow flexion"], label="Left Elbow flexion")
plt.plot(anglesL.time, anglesL.data["Forearm pronation"], label="Left Forearm pronation")
plt.xlabel("Time")
plt.ylabel("Angle (degrees)")
plt.title("Elbow Flexion and Forearm Pronation")
plt.legend()
plt.grid(True)
plt.show()


#%% hip coordinate system

# Hip Coordinate System
originHipR = markers.data["RASI"]
yHipR = markers.data["RASI"] - markers.data["RPSI"]
yzHipR = markers.data["RKNE"] - markers.data["RPSI"]

framesHipR = ktk.TimeSeries(time=markers.time)
framesHipR.data["HipR"] = ktk.geometry.create_frames(origin=originHipR, y=yHipR, yz=yzHipR)

p.set_contents(markers.merge(framesHipR))

originHipL = markers.data["LASI"]
yHipL = markers.data["LASI"] - markers.data["LPSI"]
yzHipL = markers.data["LKNE"] - markers.data["LPSI"]

framesHipL = ktk.TimeSeries(time=markers.time)
framesHipL.data["HipL"] = ktk.geometry.create_frames(origin=originHipL, y=yHipL, yz=yzHipL)

p.set_contents(markers.merge(framesHipL))

#pelvis
originPelvis = 0.5 * (markers.data["LASI"] + markers.data["RASI"])

# Y-axis: vector from midpoint of LASI and RASI to midpoint of LPSI and RPSI
midLASI_RASI = 0.5 * (markers.data["LASI"] + markers.data["RASI"])
midLPSI_RPSI = 0.5 * (markers.data["LPSI"] + markers.data["RPSI"])
yPelvis = midLPSI_RPSI - midLASI_RASI
yzPelvis = markers.data["LASI"] - markers.data["RASI"]

framesPelvis = ktk.TimeSeries(time=markers.time)
framesPelvis.data["Pelvis"] = ktk.geometry.create_frames(origin=originPelvis, y=yPelvis, yz=yzPelvis)

markers_with_pelvis = markers.merge(framesPelvis)
p.set_contents(markers_with_pelvis)

#%% Knee coordinate system

# Knee Coordinate System

originKneeR = markers.data["RKNE"]
yKneeR = markers.data["RKNE"] - markers.data["RANK"]
yzKneeR = markers.data["RKNE"] - markers.data["RHEE"]

framesKneeR = ktk.TimeSeries(time=markers.time)
framesKneeR.data["KneeR"] = ktk.geometry.create_frames(origin=originKneeR, y=yKneeR, yz=yzKneeR)

p.set_contents(markers.merge(framesKneeR))


originKneeL = markers.data["LKNE"]
yKneeL = markers.data["LKNE"] - markers.data["LANK"]
yzKneeL = markers.data["LKNE"] - markers.data["LHEE"]

framesKneeL = ktk.TimeSeries(time=markers.time)
framesKneeL.data["KneeL"] = ktk.geometry.create_frames(origin=originKneeL, y=yKneeL, yz=yzKneeL)

p.set_contents(markers.merge(framesKneeL))


#%% Ankle Coordinate System

originAnkleR = markers.data["RANK"]
yAnkleR = markers.data["RANK"] - markers.data["RTOE"]
yzAnkleR = markers.data["RHEE"] - markers.data["RTOE"]

framesAnkleR = ktk.TimeSeries(time=markers.time)
framesAnkleR.data["AnkleR"] = ktk.geometry.create_frames(origin=originAnkleR, y=yAnkleR, yz=yzAnkleR)

p.set_contents(markers.merge(framesAnkleR))


originAnkleL = markers.data["LANK"]
yAnkleL = markers.data["LANK"] - markers.data["LTOE"]
yzAnkleL = markers.data["LHEE"] - markers.data["LTOE"]

framesAnkleL = ktk.TimeSeries(time=markers.time)
framesAnkleL.data["AnkleL"] = ktk.geometry.create_frames(origin=originAnkleL, y=yAnkleL, yz=yzAnkleL)

p.set_contents(markers.merge(framesAnkleL))


#%% Shoulder Coordinate System

# Right Shoulder Coordinate System
originShoulderR = markers.data["RSHO"]
yShoulderR = markers.data["RSHO"] - markers.data["CLAV"]
yzShoulderR = markers.data["RSHO"] - markers.data["RUPA"]  

framesShoulderR = ktk.TimeSeries(time=markers.time)
framesShoulderR.data["ShoulderR"] = ktk.geometry.create_frames(origin=originShoulderR, y=yShoulderR, yz=yzShoulderR)

p.set_contents(markers.merge(framesShoulderR))


# Left Shoulder Coordinate System
originShoulderL = markers.data["LSHO"]
yShoulderL = markers.data["LSHO"] - markers.data["CLAV"]
yzShoulderL = markers.data["LSHO"] - markers.data["LUPA"]  

framesShoulderL = ktk.TimeSeries(time=markers.time)
framesShoulderL.data["ShoulderL"] = ktk.geometry.create_frames(origin=originShoulderL, y=yShoulderL, yz=yzShoulderL)

p.set_contents(markers.merge(framesShoulderL))


#%%  Calculating Joint Angles


# Hip to Knee
hip_to_kneeR = ktk.geometry.get_local_coordinates(framesKneeR.data["KneeR"], framesHipR.data["HipR"])
hip_to_kneeL = ktk.geometry.get_local_coordinates(framesKneeL.data["KneeL"], framesHipL.data["HipL"])

euler_anglesHipR = ktk.geometry.get_angles(hip_to_kneeR, "ZXY", degrees=True)
euler_anglesHipL = ktk.geometry.get_angles(hip_to_kneeL, "ZXY", degrees=True)

# Knee to Ankle
knee_to_ankleR = ktk.geometry.get_local_coordinates(framesAnkleR.data["AnkleR"], framesKneeR.data["KneeR"])
knee_to_ankleL = ktk.geometry.get_local_coordinates(framesAnkleL.data["AnkleL"], framesKneeL.data["KneeL"])

euler_anglesKneeR = ktk.geometry.get_angles(knee_to_ankleR, "ZXY", degrees=True)
euler_anglesKneeL = ktk.geometry.get_angles(knee_to_ankleL, "ZXY", degrees=True)


#%% Creating TimeSeries objects for lower limb and hip angles


anglesLowerLimbR = ktk.TimeSeries(time=markers.time)
anglesLowerLimbR.data["Hip flexion"] = euler_anglesHipR[:, 0]
anglesLowerLimbR.data["Hip abduction"] = euler_anglesHipR[:, 1]
anglesLowerLimbR.data["Hip rotation"] = euler_anglesHipR[:, 2]
anglesLowerLimbR.data["Knee flexion"] = euler_anglesKneeR[:, 0]
anglesLowerLimbR.data["Ankle dorsiflexion"] = euler_anglesKneeR[:, 1]
anglesLowerLimbR.data["Ankle eversion"] = euler_anglesKneeR[:, 2]

anglesLowerLimbL = ktk.TimeSeries(time=markers.time)
anglesLowerLimbL.data["Hip flexion"] = euler_anglesHipL[:, 0]
anglesLowerLimbL.data["Hip abduction"] = euler_anglesHipL[:, 1]
anglesLowerLimbL.data["Hip rotation"] = euler_anglesHipL[:, 2]
anglesLowerLimbL.data["Knee flexion"] = euler_anglesKneeL[:, 0]
anglesLowerLimbL.data["Ankle dorsiflexion"] = euler_anglesKneeL[:, 1]
anglesLowerLimbL.data["Ankle eversion"] = euler_anglesKneeL[:, 2]


#%% Combine and Export Results

angles_df = pd.DataFrame({
    "Time": anglesR.time,
    "Right Elbow Flexion": anglesR.data["Elbow flexion"],
    "Right Forearm Pronation": anglesR.data["Forearm pronation"],
    "Left Elbow Flexion": anglesL.data["Elbow flexion"],
    "Left Forearm Pronation": anglesL.data["Forearm pronation"],
    "Right Hip Flexion": anglesLowerLimbR.data["Hip flexion"],
    "Right Hip Abduction": anglesLowerLimbR.data["Hip abduction"],
    "Right Hip Rotation": anglesLowerLimbR.data["Hip rotation"],
    "Right Knee Flexion": anglesLowerLimbR.data["Knee flexion"],
    "Right Ankle Dorsiflexion": anglesLowerLimbR.data["Ankle dorsiflexion"],
    "Right Ankle Eversion": anglesLowerLimbR.data["Ankle eversion"],
    "Left Hip Flexion": anglesLowerLimbL.data["Hip flexion"],
    "Left Hip Abduction": anglesLowerLimbL.data["Hip abduction"],
    "Left Hip Rotation": anglesLowerLimbL.data["Hip rotation"],
    "Left Knee Flexion": anglesLowerLimbL.data["Knee flexion"],
    "Left Ankle Dorsiflexion": anglesLowerLimbL.data["Ankle dorsiflexion"],
    "Left Ankle Eversion": anglesLowerLimbL.data["Ankle eversion"],
    "Right Pelvis Tilt": anglesLowerLimbR.data["Pelvis tilt"],
    "Right Pelvis Obliquity": anglesLowerLimbR.data["Pelvis obliquity"],
    "Right Pelvis Rotation": anglesLowerLimbR.data["Pelvis rotation"],
    "Left Pelvis Tilt": anglesLowerLimbL.data["Pelvis tilt"],
    "Left Pelvis Obliquity": anglesLowerLimbL.data["Pelvis obliquity"],
    "Left Pelvis Rotation": anglesLowerLimbL.data["Pelvis rotation"],
})

angles_df.to_csv("joint_angles_3.csv", index=False) #to save data
