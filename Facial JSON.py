import bpy
import json
import os.path
from os import path
import math
import mathutils

####################################################################################
####################################################################################
####################################################################################
# Setup variabes, change these to adapt the script to a rig.
#      Chages to the face rig needed for script to work:
#           Move Chine to be parented to lowerjaw instead of lowerFaceRig (in edit mode)
#           In edit mode scale all face bones in x and y to zero and zero out roll.
####################################################################################
####################################################################################
####################################################################################

FileIdentifier = "IsaacFace"
DestArmName = "Phillip"
#Go Into Edit mode and get delta x between earlobes
#we know the pixel distance between ears in open pse so knowing it for the character
#biveus the converation between pixels and blender units.
EarLobeToEarLobedistanceInBlenderUnits = .17463
StartFrameNumber = 0
mouth_KeyFrame_Every_Nth_Frame = 3
eyes_KeyFrame_Every_Nth_Frame = 10
#number of frames to transfer, make really t
NumberOfFramesToTransfer = 600
keyFrame = True

####################################################################################
####################################################################################
####################################################################################
# Furntions for converting and manipulation of points.
####################################################################################
####################################################################################
####################################################################################
def GetPoint (Array, index):
    baseIndex = index*3
    x = float(Array[baseIndex])
    y = float(Array[baseIndex + 1])
    return [x,y]

    #this equation rotates the a point around a center, needed to take head tilt out of facial capture.
def rotatePoint(point, center, angle):
    #got equation from https://www.gamefromscratch.com/post/2012/11/24/GameDev-math-recipes-Rotating-one-point-around-another-point.aspx
    angle = math.radians(angle ) #Convert to radians
    rotatedX = math.cos(angle) * (point[0] - center[0]) - math.sin(angle) * (point[1]-center[1]) + center[0];
    rotatedY = math.sin(angle) * (point[0] - center[0]) + math.cos(angle) * (point[1] - center[1]) + center[1];
    return [rotatedX,rotatedY]

def AverageTwoPoints(pt1, pt2):
    avgX = (pt1[0] + pt2[0])/2
    avgY = (pt1[1] + pt2[1])/2
    return [avgX, avgY]

def DifferenceBetweenPoint(pt1, pt2):
    diffX = (pt1[0] - pt2[0])
    diffY = (pt1[1] - pt2[1])
    return [diffX, diffY]

def InverseXandY(pt):
    x = pt[0]*-1
    y = pt[1]*-1
    return [x, y]

def InverseX(pt):
    x = pt[0]*-1
    y = pt[1]
    return [x, y]

def multiply(pt, val):
    x = pt[0]*val
    y = pt[1]*val
    return [x, y]

####################################################################################
####################################################################################
####################################################################################
# This class get's data from open pose data provided and has a bunch of methods that 
#get bone angles and position used to postion a bone or get another bones rotation.
####################################################################################
####################################################################################
####################################################################################

class PersonJSONData:
  def __init__(self, pose, face, EarLobeToEarLobedistanceInBlenderUnits):
    self.poseStart = pose
    self.faceStart = face
    self.poseCurrent = pose
    self.faceCurrent = face
    #distance is distance between ears
    self.distancBetweenEars = abs(GetPoint(pose, 18)[0] - GetPoint(pose, 18)[1])
    self.StartNosePosition = GetPoint(pose, 0)
    self.StartFaceNosePosition = GetPoint(face, 30)
    self.StartRightEyePosition = GetPoint(pose, 15)
    self.StartLeftEyePosition = GetPoint(pose, 16)
    #self.StartRightEyePosition = GetPoint(face, 68)
    #self.StartLeftEyePosition = GetPoint(face, 69)
    self.StartHeadTilt = self.getHeadTiltSideToSide()
    self.EarLobeToEarLobedistanceInBlenderUnits = EarLobeToEarLobedistanceInBlenderUnits

  def SetCurrentPose(self, pose, face):
    self.poseCurrent = pose
    self.faceCurrent = face
    
    #given a point return it correct for head tilt at the current point in time.
  def GetCurrentPointHeadTiltCorrected(self, Point):
      angle = self.getHeadTiltSideToSide()
      #point 8 is the nose we rotate around it by the tilt of the head
      PointCorrrected = rotatePoint(GetPoint(self.faceCurrent, 8), Point,(angle*-1))
      return PointCorrrected

    #this method returns a start point corrected for the head tilt at start, good for returning a point 
    #In Space at start of capture corrected for head tilt at start of capture.
  def GetStartPointHeadTiltCorrected(self, Point):
      angle = self.StartHeadTilt
      #point 8 is the nose we rotate around it by the tilt of the head
      PointCorrrected = rotatePoint(GetPoint(self.faceStart, 8), Point,(angle*-1))
      return PointCorrrected

  def GetDistanceBetweenTwoCurrentPoints(self, Pnt1, Pnt2):
      x2 = Pnt2[0]
      x1 = Pnt1[0]
      y2 = Pnt2[1]
      y1 = Pnt1[1]
      dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
      return dist
  
  def ConvertPixelPointToBlenderUnits(self, pnt):
      pntx = pnt[0] * (self.EarLobeToEarLobedistanceInBlenderUnits/self.distancBetweenEars)
      pnty = pnt[1] * (self.EarLobeToEarLobedistanceInBlenderUnits/self.distancBetweenEars)
      return [pntx,pnty]
  
  def getHeadTiltSideToSide(self):
    #We get the tils by taking the angle difference between the left and right years
    leftEar = GetPoint(self.poseCurrent, 18)
    rightEar = GetPoint(self.poseCurrent, 17)
    deltaY = leftEar[1] - rightEar[1]
    deltaX = leftEar[0] - rightEar[0]
    tilt = math.atan(deltaY/deltaX)
    return math.degrees(tilt*-1)
      
  def getHeadTiltUpDown(self):
    #get the angle by using the nose vertical translation and approximating the distance
    #between the nose and the head pivot using the ear to ear distance from the start.
    CurrentNosePosition = GetPoint(self.poseCurrent, 0)
    deltaY = CurrentNosePosition[1] - self.StartNosePosition[1]
    tilt = math.atan(deltaY/self.distancBetweenEars)
    return math.degrees(tilt)
      
  def getHeadTiltLeftRight(self):
    #get the angle by using the nose horizontal translation and approximating the distance
    #between the nose and the head pivot using the ear to ear distance from the start.
    CurrentNosePosition = GetPoint(self.poseCurrent, 0)
    deltaX = CurrentNosePosition[0] - self.StartNosePosition[0]
    tilt = math.atan(deltaX/self.distancBetweenEars)
    return math.degrees(tilt)
      
  def getJawTiltUpDown(self):
    #To get the jaw tilt up down we rotate the point around the nose by the head tilt angle
    #Then we 
    NoseToJawStartDist = self.GetDistanceBetweenTwoCurrentPoints(GetPoint(self.faceStart, 8),GetPoint(self.faceStart, 30))
    NoseToJawCurrentDist = self.GetDistanceBetweenTwoCurrentPoints(GetPoint(self.faceCurrent, 8),GetPoint(self.faceCurrent, 30))
    JawDelta = NoseToJawStartDist - NoseToJawCurrentDist
    #approximate the distance from the tip of your jaw to it's pivot by taking the distance between the ears and subtracting an eigth
    PivottoEndOfJaw = self.distancBetweenEars - .125*self.distancBetweenEars
    Jawtilt = math.atan(JawDelta/PivottoEndOfJaw)
    return math.degrees(Jawtilt)

    #This takes two points and averages them in the start frame and the current frame.  It then
  def getFacePosition(self, ptnum1, ptnum2, RelativeTo):
    ptAvgStart = AverageTwoPoints(GetPoint(self.faceStart, ptnum1),GetPoint(self.faceStart, ptnum2))
    ptAvgCurrent = AverageTwoPoints(GetPoint(self.faceCurrent, ptnum1),GetPoint(self.faceCurrent, ptnum2))
    
    if RelativeTo == "lefteye":
        ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.poseCurrent, 16))
        #ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.faceCurrent, 69))
        ptDiffStart = DifferenceBetweenPoint(ptAvgStart,self.StartLeftEyePosition)
    elif RelativeTo == "righteye":
        ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.poseCurrent, 15))
        #ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.faceCurrent, 68))
        ptDiffStart = DifferenceBetweenPoint(ptAvgStart,self.StartRightEyePosition)
    else:
        ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.faceCurrent, 30))
        ptDiffStart = DifferenceBetweenPoint(ptAvgStart,self.StartFaceNosePosition)
    
    #StartPos = self.GetStartPointHeadTiltCorrected(ptDiffStart)
    #CurrentPos = self.GetCurrentPointHeadTiltCorrected(ptDiffCurrent)
    #DeltaPixels = DifferenceBetweenPoint(GetPoint(self.faceStart, ptnum1),GetPoint(self.faceCurrent, ptnum1))
    #DeltaPixels = DifferenceBetweenPoint(ptAvgStart, ptAvgCurrent)
    DeltaPixels = DifferenceBetweenPoint(ptDiffStart, ptDiffCurrent)
    #DeltaPixels = DifferenceBetweenPoint(StartPos,CurrentPos)
    return DeltaPixels

  def getLowerLipCenterPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(57,66, "nose"))
    return multiply(output, correction)

  def getLowerLipCenterLeftPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(56,65, "nose"))
    return multiply(output, correction)

  def getLowerLipCenterRightPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(58,67, "nose"))
    return multiply(output, correction)

  def getUpperLipCenterPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(51,62, "nose"))
    return multiply(output, correction)

  def getUpperLipCenterLeftPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(52,63, "nose"))
    return multiply(output, correction)

  def getUpperLipCenterRightPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(50,61, "nose"))
    return multiply(output, correction)


  def getUpperLipOuterLeftPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(53,53, "nose"))
    return multiply(output, correction)

  def getUpperLipOuterRightPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(49,49, "nose"))
    return multiply(output, correction)

  def getLowerLipOuterLeftPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(55,55, "nose"))
    return multiply(output, correction)

  def getLowerLipOuterRightPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(59,59, "nose"))
    return multiply(output, correction)


  def getLipLeftPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(54,64, "nose"))
    return multiply(output, correction)

  def getLipRightPosition(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(48,60, "nose"))
    return multiply(output, correction)



  def getEyeBrowLeftOuter(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(25,26, "lefteye"))
    return multiply(output, correction)

  def getEyeBrowLeftCenter(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(24,24, "lefteye"))
    return multiply(output, correction)

  def getEyeBrowLeftInner(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(22,23, "lefteye"))
    return multiply(output, correction)



  def getEyeBrowRightOuter(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(17,18, "righteye"))
    return multiply(output, correction)

  def getEyeBrowRightCenter(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(19,19, "righteye"))
    return multiply(output, correction)

  def getEyeBrowRightInner(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(20,21, "righteye"))
    return multiply(output, correction)


  def getEyelidRight(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(37,38, "righteye"))
    return multiply(output, correction)


  def getEyelidLeft(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(43,44, "lefteye"))
    return multiply(output, correction)


  def getEyelidLowerRight(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(40,41, "righteye"))
    return multiply(output, correction)


  def getEyelidLowerLeft(self, correction):
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(46,47, "lefteye"))
    return multiply(output, correction)

####################################################################################
####################################################################################
####################################################################################
# Code for iteration through frames and applying positions and angles to rig
####################################################################################
####################################################################################
####################################################################################

keepGoing = True
filenum = 0 
FirstTime = True

print('')
print('Start of Everything')
print('')
DestArm  = bpy.data.objects[DestArmName]
scene = bpy.context.scene
head_bone =  DestArm.pose.bones["head"]
jaw_bone =  DestArm.pose.bones["lowerJaw"]
lower_face =  DestArm.pose.bones["lowerFaceRig"]
lower_lip_center =  DestArm.pose.bones["LipLowerMiddle"]
lower_lip_center_l =  DestArm.pose.bones["lLipLowerInner"]
lower_lip_center_r =  DestArm.pose.bones["rLipLowerInner"]
upper_lip_center =  DestArm.pose.bones["LipUpperMiddle"]
upper_lip_center_l =  DestArm.pose.bones["lLipUpperInner"]
upper_lip_center_r =  DestArm.pose.bones["rLipUpperInner"]

upper_lip_outer_r =  DestArm.pose.bones["rLipUpperOuter"]
lower_lip_outer_r =  DestArm.pose.bones["rLipLowerOuter"]
upper_lip_outer_l =  DestArm.pose.bones["lLipUpperOuter"]
lower_lip_outer_l =  DestArm.pose.bones["lLipLowerOuter"]

corner_lip_r =  DestArm.pose.bones["rLipCorner"]
Corner_lip_l =  DestArm.pose.bones["lLipCorner"]


eyebrow_outer_r =  DestArm.pose.bones["rBrowOuter"]
eyebrow_center_r =  DestArm.pose.bones["rBrowMid"]
eyebrow_inner_r =  DestArm.pose.bones["rBrowInner"]

eyebrow_outer_l =  DestArm.pose.bones["lBrowOuter"]
eyebrow_center_l =  DestArm.pose.bones["lBrowMid"]
eyebrow_inner_l =  DestArm.pose.bones["lBrowInner"]

eyelid_l =  DestArm.pose.bones["Eyelid.l"]
eyelid_r =  DestArm.pose.bones["Eyelid.r"]

eyelid_lower_l =  DestArm.pose.bones["EyelidLower.l"]
eyelid_lower_r =  DestArm.pose.bones["EyelidLower.r"]

CurrentFrame = 0

while keepGoing:
    numberString = str(filenum).zfill(12)
    filenum = filenum + 1
    mypath = bpy.path.abspath("//" + FileIdentifier + "_" + numberString + "_keypoints.json")
    
    if path.exists(mypath):
        
        print("We are On File: " + str(filenum))
        
        with open(mypath) as json_file:
            face1_dict = json.load(json_file)
            
        #face1 = '{"version":1.3,"people":[{"person_id":[-1],"pose_keypoints_2d":[880.441,483.283,0.877448,859.957,809.893,0.519232,509.674,777.567,0.352327,0,0,0,0,0,0,1227.86,786.323,0.347931,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,798.059,415.538,0.896947,951.13,406.708,0.918684,698.036,468.529,0.858658,1045.25,447.943,0.872412,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"face_keypoints_2d":[699.986,426.614,0.845591,706.545,470.884,0.915689,716.382,513.514,0.889852,726.22,557.783,0.855176,740.976,598.773,0.825968,768.85,631.565,0.810219,801.642,661.078,0.833424,839.353,682.393,0.858697,886.901,687.312,0.848232,931.171,677.474,0.870025,967.242,649.601,0.884166,996.755,616.809,0.898078,1018.07,577.458,0.867331,1026.27,536.468,0.857222,1031.19,492.199,0.847157,1036.11,447.929,0.896024,1037.75,405.299,0.8281,722.941,397.101,0.868592,745.895,375.787,0.908432,773.768,367.589,0.938036,808.2,367.589,0.91255,837.713,379.066,0.884488,895.099,370.868,0.846583,922.973,356.111,0.888079,955.765,352.832,0.948471,985.278,356.111,0.902596,1008.23,377.426,0.933312,867.226,411.858,0.878828,868.866,438.092,0.921134,870.505,465.965,0.893925,870.505,492.199,0.882079,837.713,521.712,0.897178,854.109,524.991,0.945999,875.424,529.91,0.932919,893.46,523.351,0.929055,913.135,516.793,0.900395,763.931,426.614,0.929163,781.967,416.777,0.934847,804.921,415.137,0.898184,826.236,428.254,0.905151,804.921,433.173,0.941087,781.967,433.173,0.943364,918.054,420.056,0.959887,934.45,405.299,0.959568,957.405,402.02,0.963585,977.08,410.218,0.9434,960.684,420.056,0.952746,937.729,421.696,0.940418,819.678,579.098,0.9055,842.632,570.9,0.90693,863.947,564.341,0.946204,880.343,567.621,0.923158,898.379,562.702,0.917559,919.694,565.981,0.944104,945.927,569.26,0.928293,921.333,590.575,0.944567,901.658,600.413,0.933296,883.622,602.052,0.932713,865.587,602.052,0.951636,844.272,595.494,0.898185,829.515,579.098,0.921253,863.947,579.098,0.947624,881.983,579.098,0.934409,900.018,577.458,0.946441,936.09,572.539,0.90652,900.018,577.458,0.937046,881.983,579.098,0.932065,863.947,579.098,0.967957,793.444,421.696,0.82772,945.927,408.579,0.919685],"hand_left_keypoints_2d":[],"hand_right_keypoints_2d":[],"pose_keypoints_3d":[],"face_keypoints_3d":[],"hand_left_keypoints_3d":[],"hand_right_keypoints_3d":[]}]}'

        #face1_dict = json.loads(face1)
        #print(face1_dict.keys())

        people = face1_dict['people']
        #print(people[0].keys())
        ##dict_keys(['person_id', 'pose_keypoints_2d', 'face_keypoints_2d', 'hand_left_keypoints_2d', 'hand_right_keypoints_2d', 'pose_keypoints_3d', 'face_keypoints_3d', 'hand_left_keypoints_3d', 'hand_right_keypoints_3d'])

        #print(json.dumps(people[0]))
        myPerson = people[0]

        pose = myPerson['pose_keypoints_2d']
        face = myPerson['face_keypoints_2d']
        
        if FirstTime:
            p1 = PersonJSONData(pose, face, EarLobeToEarLobedistanceInBlenderUnits)
            FirstTime = False
        else:
            p1.SetCurrentPose(pose, face)
        
        #print(GetPoint(pose,0))
        head_angle = mathutils.Euler((math.radians(p1.getHeadTiltUpDown()),math.radians(p1.getHeadTiltLeftRight()),math.radians(p1.getHeadTiltSideToSide())), 'XYZ')
        
        head_bone.rotation_mode = 'XYZ'
        head_bone.rotation_euler = head_angle
        
        jaw_bone.rotation_mode = 'XYZ'
        jaw_bone.rotation_euler.z = math.radians(p1.getJawTiltUpDown())
        
        lower_face.rotation_mode = 'XYZ'
        lower_face.rotation_euler.x = math.radians(p1.getJawTiltUpDown())
        
        Offset = p1.getLowerLipCenterPosition(1)
        lower_lip_center.location.x = Offset[0]*-1
        lower_lip_center.location.z = Offset[1]*-1
        
        Offset = p1.getLowerLipCenterLeftPosition(1)
        lower_lip_center_l.location.x = Offset[0]
        lower_lip_center_l.location.z = Offset[1]
        
        Offset = p1.getLowerLipCenterRightPosition(1)
        lower_lip_center_r.location.x = Offset[0]
        lower_lip_center_r.location.z = Offset[1]
        
        Offset = p1.getUpperLipCenterPosition(1)
        upper_lip_center.location.x = Offset[0]
        upper_lip_center.location.z = Offset[1]
        
        Offset = p1.getUpperLipCenterLeftPosition(1)
        upper_lip_center_l.location.x = Offset[0]
        upper_lip_center_l.location.z = Offset[1]
        
        Offset = p1.getUpperLipCenterRightPosition(1)
        upper_lip_center_r.location.x = Offset[0]
        upper_lip_center_r.location.z = Offset[1]
        
        
        
        Offset = p1.getUpperLipOuterRightPosition(1)
        upper_lip_outer_r.location.x = Offset[0]
        upper_lip_outer_r.location.z = Offset[1]
        
        Offset = p1.getLowerLipOuterRightPosition(1)
        lower_lip_outer_r.location.x = Offset[0]
        lower_lip_outer_r.location.z = Offset[1]
        
        Offset = p1.getUpperLipOuterLeftPosition(1)
        upper_lip_outer_l.location.x = Offset[0]
        upper_lip_outer_l.location.z = Offset[1]
        
        Offset = p1.getLowerLipOuterLeftPosition(1)
        lower_lip_outer_l.location.x = Offset[0]
        lower_lip_outer_l.location.z = Offset[1]
        
        
        
        Offset = p1.getLipRightPosition(1)
        corner_lip_r.location.x = Offset[0]
        corner_lip_r.location.z = Offset[1]
        
        Offset = p1.getLipLeftPosition(1)
        Corner_lip_l.location.x = Offset[0]
        Corner_lip_l.location.z = Offset[1]
                
        
        eyecorrection = 2
        Offset = p1.getEyeBrowLeftOuter(1)
        eyebrow_outer_r.location.x = Offset[0]
        eyebrow_outer_r.location.z = Offset[1]
        
        Offset = p1.getEyeBrowLeftCenter(1)
        eyebrow_center_r.location.x = Offset[0]
        eyebrow_center_r.location.z = Offset[1]
        
        Offset = p1.getEyeBrowLeftInner(1)
        eyebrow_inner_r.location.x = Offset[0]
        eyebrow_inner_r.location.z = Offset[1]
                
        
        Offset = p1.getEyeBrowRightOuter(1)
        eyebrow_outer_l.location.x = Offset[0]
        eyebrow_outer_l.location.z = Offset[1]
        
        Offset = p1.getEyeBrowRightCenter(1)
        eyebrow_center_l.location.x = Offset[0]
        eyebrow_center_l.location.z = Offset[1]
        
        Offset = p1.getEyeBrowRightInner(1)
        eyebrow_inner_l.location.x = Offset[0]
        eyebrow_inner_l.location.z = Offset[1]
        
        Offset = p1.getEyelidLeft(10)
        eyelid_l.location.z = Offset[1]
        
        Offset = p1.getEyelidRight(10)
        eyelid_r.location.z = Offset[1]
        
        Offset = p1.getEyelidLowerLeft(1)
        eyelid_lower_l.location.z = Offset[1]
        
        Offset = p1.getEyelidLowerRight(1)
        eyelid_lower_r.location.z = Offset[1]
        
        #print(p1.getHeadTiltSideToSide())
        #print(p1.getHeadTiltUpDown())
        #print(p1.getHeadTiltLeftRight())
        
        #if current frome is an exact multiple of Every Nth Frame
        if (CurrentFrame % mouth_KeyFrame_Every_Nth_Frame == 0) & (keyFrame):
            head_bone.keyframe_insert(data_path='rotation_euler',frame= StartFrameNumber + CurrentFrame)
            jaw_bone.keyframe_insert(data_path='rotation_euler',frame= StartFrameNumber + CurrentFrame)
            lower_face.keyframe_insert(data_path='rotation_euler',frame= StartFrameNumber + CurrentFrame)
            lower_lip_center.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            lower_lip_center_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            lower_lip_center_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            upper_lip_center.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            upper_lip_center_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            upper_lip_center_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            
            upper_lip_outer_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            lower_lip_outer_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            upper_lip_outer_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            lower_lip_outer_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            
            corner_lip_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            Corner_lip_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)

        #if current frome is an exact multiple of Every Nth Frame
        if (CurrentFrame % eyes_KeyFrame_Every_Nth_Frame == 0) & (keyFrame):
            eyebrow_outer_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            eyebrow_center_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            eyebrow_inner_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
       
            eyebrow_outer_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            eyebrow_center_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            eyebrow_inner_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
       
            eyelid_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            eyelid_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
       
            eyelid_lower_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
            eyelid_lower_l.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)


        CurrentFrame = CurrentFrame + 1
        bpy.context.scene.frame_set(StartFrameNumber + CurrentFrame)
        if CurrentFrame > NumberOfFramesToTransfer:
            keepGoing = False
    else:
        keepGoing = False
