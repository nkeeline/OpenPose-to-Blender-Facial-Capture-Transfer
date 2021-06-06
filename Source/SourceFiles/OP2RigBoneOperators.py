import bpy
import json
import os.path
from os import path
import math
import mathutils




####################################################################################
####################################################################################
####################################################################################
# Functions for converting and manipulation of points.
####################################################################################
####################################################################################
####################################################################################

def combineRReliability(pt1, pt2):
    rel1 = pt1[2]
    rel2 = pt2[2]
    if rel1 < rel2:
        relOut = rel1
    else:
        relOut = rel2
    return relOut

def GetPoint (Array, index):
    baseIndex = index*3
    x = float(Array[baseIndex])
    y = float(Array[baseIndex + 1])
    reliability = float(Array[baseIndex + 2])
    return [x,y,reliability]

    #this equation rotates the a point around a center, needed to take head tilt out of facial capture.
def rotatePoint(point, center, angle):
    #got equation from https://www.gamefromscratch.com/post/2012/11/24/GameDev-math-recipes-Rotating-one-point-around-another-point.aspx
    angle = math.radians(angle ) #Convert to radians
    rotatedX = math.cos(angle) * (point[0] - center[0]) - math.sin(angle) * (point[1]-center[1]) + center[0];
    rotatedY = math.sin(angle) * (point[0] - center[0]) + math.cos(angle) * (point[1] - center[1]) + center[1];
    return [rotatedX,rotatedY,point[2]]

def AverageTwoPoints(pt1, pt2):
    avgX = (pt1[0] + pt2[0])/2
    avgY = (pt1[1] + pt2[1])/2
    return [avgX, avgY,combineRReliability(pt1,pt2)]

def DifferenceBetweenPoint(pt1, pt2):
    diffX = (pt1[0] - pt2[0])
    diffY = (pt1[1] - pt2[1])
    return [diffX, diffY,combineRReliability(pt1,pt2)]

def InverseXandY(pt):
    x = pt[0]*-1
    y = pt[1]*-1
    return [x, y, pt[2]]

def InverseX(pt):
    x = pt[0]*-1
    y = pt[1]
    return [x, y, pt[2]]

def multiply(pt, val):
    x = pt[0]*val
    y = pt[1]*val
    return [x, y, pt[2]]


####################################################################################
####################################################################################
####################################################################################
# This class get's data from open pose data provided and has a bunch of methods that 
#get bone angles and position used to postion a bone or get another bones rotation.
####################################################################################
####################################################################################
####################################################################################

class PersonJSONData:
  def __init__(self, pose, face, EarLobeToEarLobedistanceInBlenderUnits, TieEyelidsTogether):
    self.poseStart = pose
    self.faceStart = face
    self.poseCurrent = pose
    self.faceCurrent = face
    self.TieEyelidsTogether = TieEyelidsTogether
    #distance is distance between ears
    self.distancBetweenEars = abs(GetPoint(pose, 18)[0] - GetPoint(pose, 18)[1])
    self.StartNosePosition = GetPoint(pose, 0)
    self.StartFaceNosePosition = GetPoint(face, 30)
    self.StartFaceChinPosition = GetPoint(face, 8)
    self.StartRightEyePosition = GetPoint(pose, 15)
    self.StartLeftEyePosition = GetPoint(pose, 16)
    #self.StartRightEyePosition = GetPoint(face, 68)
    #self.StartLeftEyePosition = GetPoint(face, 69)
    self.LastGoodHeadTilt = 0
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
      reliability = pnt[2]
      return [pntx,pnty,reliability]
  
  def getHeadTiltSideToSide(self):
    #We get the tils by taking the angle difference between the left and right years
    leftEar = GetPoint(self.poseCurrent, 18)
    rightEar = GetPoint(self.poseCurrent, 17)
    deltaY = leftEar[1] - rightEar[1]
    deltaX = leftEar[0] - rightEar[0]
    if leftEar[2] < .1 or rightEar[2] < .1:
        tiltOut = self.LastGoodHeadTilt
    else:
        tilt = math.atan(deltaY/deltaX)
        tiltOut = math.degrees(tilt*-1)
        self.LastGoodHeadTilt = tiltOut
    return tiltOut
      
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
    
    if RelativeTo == "LEFTEYE":
        ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.poseCurrent, 16))
        #ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.faceCurrent, 69))
        ptDiffStart = DifferenceBetweenPoint(ptAvgStart,self.StartLeftEyePosition)
    elif RelativeTo == "RIGHTEYE":
        ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.poseCurrent, 15))
        #ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.faceCurrent, 68))
        ptDiffStart = DifferenceBetweenPoint(ptAvgStart,self.StartRightEyePosition)
    elif RelativeTo == "CHIN":
        ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.faceCurrent, 8))
        ptDiffStart = DifferenceBetweenPoint(ptAvgStart,self.StartFaceChinPosition)
    elif RelativeTo == "upperlip":
        ptDiffCurrent = DifferenceBetweenPoint(ptAvgCurrent,GetPoint(self.faceCurrent, 51))
        ptDiffStart = DifferenceBetweenPoint(ptAvgStart,self.StartFaceChinPosition)
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

  def getJawPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(8,8, thisref))
    return multiply(output, correction)

  def getLowerLipCenterPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(57,66, thisref))
    return multiply(output, correction)

  def getLowerLipCenterLeftPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(65,65, thisref))
    return multiply(output, correction)

  def getLowerLipCenterRightPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(67,67, thisref))
    return multiply(output, correction)

  def getUpperLipCenterPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(62,62, thisref))
    return multiply(output, correction)

  def getUpperLipCenterLeftPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(52,63, thisref))
    return multiply(output, correction)

  def getUpperLipCenterRightPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(50,61, thisref))
    return multiply(output, correction)


  def getUpperLipOuterLeftPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(63,63, thisref))
    return multiply(output, correction)

  def getUpperLipOuterRightPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(61,61, thisref))
    return multiply(output, correction)

  def getLowerLipOuterLeftPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(65,65, thisref))
    return multiply(output, correction)

  def getLowerLipOuterRightPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(67,67, thisref))
    return multiply(output, correction)


  def getLipLeftPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(54,64, thisref))
    return multiply(output, correction)

  def getLipRightPosition(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "NOSE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(48,60, thisref))
    return multiply(output, correction)



  def getEyeBrowLeftOuter(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "LEFTEYE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(25,26, thisref))
    return multiply(output, correction)

  def getEyeBrowLeftCenter(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "LEFTEYE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(24,24, thisref))
    return multiply(output, correction)

  def getEyeBrowLeftInner(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "LEFTEYE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(22,23, thisref))
    return multiply(output, correction)



  def getEyeBrowRightOuter(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "RIGHTEYE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(17,18, thisref))
    return multiply(output, correction)

  def getEyeBrowRightCenter(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "RIGHTEYE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(19,19, thisref))
    return multiply(output, correction)

  def getEyeBrowRightInner(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "RIGHTEYE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(20,21, thisref))
    return multiply(output, correction)


  def getEyelidRight(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "RIGHTEYE"
    else:
        thisref = ref
    if self.TieEyelidsTogether:
        eyeR = self.getFacePosition(37,38, "RIGHTEYE")
        eyeL = self.getFacePosition(43,44, "LEFTEYE")
        output = self.ConvertPixelPointToBlenderUnits(AverageTwoPoints(eyeR,eyeL))
    else:
        output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(37,38, thisref))
    return multiply(output, correction)


  def getEyelidLeft(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "LEFTEYE"
    else:
        thisref = ref
    if self.TieEyelidsTogether:
        eyeR = self.getFacePosition(37,38, "RIGHTEYE")
        eyeL = self.getFacePosition(43,44, "LEFTEYE")
        output = self.ConvertPixelPointToBlenderUnits(AverageTwoPoints(eyeR,eyeL))
    else:
        output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(43,44, thisref))
    return multiply(output, correction)


  def getEyelidLowerRight(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "RIGHTEYE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(40,41, thisref))
    return multiply(output, correction)


  def getEyelidLowerLeft(self, correction, ref):
    if ref == 'DEFAULT':
        thisref = "LEFTEYE"
    else:
        thisref = ref
    output = self.ConvertPixelPointToBlenderUnits(self.getFacePosition(46,47, thisref))
    return multiply(output, correction)


####################################################################################
####################################################################################
####################################################################################
#Rig bone mappping data.   This is the data on what map to each bone.  Each Bone
# can be saved to a file, so this object just contains the bones mapping info only
####################################################################################
####################################################################################
####################################################################################

class BoneJSONData:
  def __init__(SourceBoneLocationName, DestinationBoneName, BoneModificationType, ApplyToX, ApplyToY, DirectionQuatToApplyXandY):
    #This is the source from JSON like head_bone that designates where this bone is getting it's information from in the JSON file
    self.SourceBoneLocationName = SourceBoneLocationName  
    #This is the name for the rig bone name.
    self.DestinationBoneName = DestinationBoneName
    #keyframe type is either 'LOC' for location or 'ROT' for rotation
    self.BoneModificationType = BoneModificationType
    #boolean true/false saying do you want to apply X (horizontal) data from JSON)
    self.ApplyToX = ApplyToX
    #boolean true/false saying do you want to apply Y (vertiacl) data from JSON)
    self.ApplyToY = ApplyToY
    #This quaternion will rotat the bone then translate it in the rotated direction then rotate it back at the end.
    self.DirectionQuatToApplyXandY = DirectionQuatToApplyXandY
    
  def GetJSONDataAsString(self): 
    # a Python object (dict):
    DataDictionary = {
      "SourceBoneLocationName": self.SourceBoneLocationName,
      "DestinationBoneName": self.DestinationBoneName,
      "BoneModificationType": self.BoneModificationType,
      "ApplyToX": self.ApplyToX,
      "ApplyToY": self.ApplyToY,
      "DirectionQuatToApplyXandY": self.DirectionQuatToApplyXandY,
    }
    # convert into JSON:
    return json.dumps(DataDictionary)
        
  def SetJSONDataFromSTring(self,JSONString): 
    # parse JSONString:
    y = json.loads(JSONString)

    self.SourceBoneLocationName = y["SourceBoneLocationName"]  
    self.DestinationBoneName = y["DestinationBoneName"]
    self.BoneModificationType = y["BoneModificationType"]
    self.ApplyToX = y["ApplyToX"]
    self.ApplyToY = y["ApplyToY"]
    self.DirectionQuatToApplyXandY = y["DirectionQuatToApplyXandY"]

####################################################################################
####################################################################################
####################################################################################
# This Class is a Bone class for every Bone in the Armature we want to modify
# with the JSON Data Object. Each bone contains an location it belongs to like
# face, body , hand etc and a bone modification type like rotation or 
####################################################################################
####################################################################################
####################################################################################

class BoneToMap:
  def __init__(Armature, SourceBoneLocationName, DestinationBoneName, BoneModificationType, BoneDataName, ApplyToX, ApplyToY, ApplyToZ):
    #This is the source from JSON like head_bone that designates where this bone is getting it's information from in the JSON file
    self.SourceBoneLocationName = SourceBoneLocationName  
    #This is the destination bone name on the rig to apply the modifications to.
    self.BoneName = BoneName
    self.Bone = Armature.pose.bones[BoneName]
    self.BoneDataName = BoneDataName
    #keyframe type is either 'LOC' for location or 'ROT' for rotation
    self.BoneModificationType = BoneModificationType
    self.ApplyToX = ApplyToX
    self.ApplyToY = ApplyToY
    self.ApplyToZ = ApplyToZ

  def SetBoneLocation(self, X, Y, Z): 
    if self.ApplyToX:
        self.Bone.location.x = X
    if self.ApplyToY:
        self.Bone.location.y = Y
    if self.ApplyToZ:
        self.Bone.location.z = Z
        
  def SetBoneRotationEuler(self, X, Y, Z): 
    self.Bone.rotation_mode = 'XYZ'
    self.Bone.rotation_euler = head_angle
            
  def KeyFrame(self):
    if BoneModificationType == 'LOC':
        self.Bone.keyframe_insert(data_path='location',frame= FinalFrameNumber)
    if BoneModificationType == 'ROT':
        self.Bone.keyframe_insert(data_path='rotation_euler',frame= FinalFrameNumber)
     
# ------------------------------------------------------------------------
#    operators
# ------------------------------------------------------------------------

####################################################################################
####################################################################################
####################################################################################
# Code for iteration through frames and applying positions and angles to rig
####################################################################################
####################################################################################
####################################################################################

class ReadInApplyToRigOperator(bpy.types.Operator):
    bl_idname = "wm.applyjsonfiles"
    bl_label = "Read in OpenPose JSON Files and Apply to Character"

#    def get_roll_correction(self, bone, whichone):
#        #we start by seeting which roll correction we're getting
#        if whichone == 0:
#            we_should_do_it = bone.RollCorrection
#        else:
#            we_should_do_it = bone.RollCorrection2
#        if we_should_do_it:
#            #tempcode for now
#            out = 0
#        else:
#            out = 0
#        return out

    def get_correction_Angle(self, bone_axis, correction):
        if bone_axis == 'NEGX' or bone_axis == 'NEGY' or bone_axis == 'NEGZ':
            correction = correction*(-1)
        if bone_axis == 'PLUSX' or bone_axis == 'NEGX':
            cAngle = mathutils.Euler((math.radians(correction), math.radians(0), math.radians(0)), 'XYZ').to_quaternion()
        if bone_axis == 'PLUSY' or bone_axis == 'NEGY':
            cAngle = mathutils.Euler((math.radians(0), math.radians(correction), math.radians(0)), 'XYZ').to_quaternion()
        if bone_axis == 'PLUSZ' or bone_axis == 'NEGZ':
            cAngle = mathutils.Euler((math.radians(0), math.radians(0), math.radians(correction)), 'XYZ').to_quaternion()
        return CAngle
                
    def get_parent_correction(self, bone):
        return 0
    
    #def get_horizontal(self, bone):
        
        return horizontal 
    def execute(self, context):
        scene = context.scene
        op2rig = scene.openpose_2_rig_settings
        bone_list = context.scene.bone_mapping_list
        
        keepGoing = True
        filenum = 0 
        FirstTime = True

        print('')
        print('Start of Everything')
        print('')
        DestArm  = bpy.data.objects[op2rig.rig_name]
        #Rigobj = RigModificationProcess(op2rig.rig_name, DestArm)
        
        #going to force Eyelid Keyframe if it's drastically different
        PreviousEyelidValue_r = 0 
        PreviousEyelidValue_l = 0  
        CurrentFrame = op2rig.start_frame_to_apply

        win = bpy.context.window_manager

        win.progress_begin(0, op2rig.number_of_frames_to_apply)

        while keepGoing:
            numberString = str(filenum).zfill(12)
            filenum = filenum + 1
            filename = op2rig.first_JSON_file_to_read_in
            if not filename.endswith("_keypoints.json"):
                print("Error, you did not select a OpenPoseJSON File")
                
            beforeNumber = filename.find("_keypoints.json") - 12
            mypath = bpy.path.abspath(filename[0:beforeNumber] + numberString + "_keypoints.json")
            print("Beginning File " + mypath)
            if path.exists(mypath):
                
                #print("We are On File: " + str(filenum))
                win.progress_update(filenum)
                
                with open(mypath) as json_file:
                    face1_dict = json.load(json_file)
                    
                #face1 = '{"version":1.3,"people":[{"person_id":[-1],"pose_keypoints_2d":[880.441,483.283,0.877448,859.957,809.893,0.519232,509.674,777.567,0.352327,0,0,0,0,0,0,1227.86,786.323,0.347931,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,798.059,415.538,0.896947,951.13,406.708,0.918684,698.036,468.529,0.858658,1045.25,447.943,0.872412,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"face_keypoints_2d":[699.986,426.614,0.845591,706.545,470.884,0.915689,716.382,513.514,0.889852,726.22,557.783,0.855176,740.976,598.773,0.825968,768.85,631.565,0.810219,801.642,661.078,0.833424,839.353,682.393,0.858697,886.901,687.312,0.848232,931.171,677.474,0.870025,967.242,649.601,0.884166,996.755,616.809,0.898078,1018.07,577.458,0.867331,1026.27,536.468,0.857222,1031.19,492.199,0.847157,1036.11,447.929,0.896024,1037.75,405.299,0.8281,722.941,397.101,0.868592,745.895,375.787,0.908432,773.768,367.589,0.938036,808.2,367.589,0.91255,837.713,379.066,0.884488,895.099,370.868,0.846583,922.973,356.111,0.888079,955.765,352.832,0.948471,985.278,356.111,0.902596,1008.23,377.426,0.933312,867.226,411.858,0.878828,868.866,438.092,0.921134,870.505,465.965,0.893925,870.505,492.199,0.882079,837.713,521.712,0.897178,854.109,524.991,0.945999,875.424,529.91,0.932919,893.46,523.351,0.929055,913.135,516.793,0.900395,763.931,426.614,0.929163,781.967,416.777,0.934847,804.921,415.137,0.898184,826.236,428.254,0.905151,804.921,433.173,0.941087,781.967,433.173,0.943364,918.054,420.056,0.959887,934.45,405.299,0.959568,957.405,402.02,0.963585,977.08,410.218,0.9434,960.684,420.056,0.952746,937.729,421.696,0.940418,819.678,579.098,0.9055,842.632,570.9,0.90693,863.947,564.341,0.946204,880.343,567.621,0.923158,898.379,562.702,0.917559,919.694,565.981,0.944104,945.927,569.26,0.928293,921.333,590.575,0.944567,901.658,600.413,0.933296,883.622,602.052,0.932713,865.587,602.052,0.951636,844.272,595.494,0.898185,829.515,579.098,0.921253,863.947,579.098,0.947624,881.983,579.098,0.934409,900.018,577.458,0.946441,936.09,572.539,0.90652,900.018,577.458,0.937046,881.983,579.098,0.932065,863.947,579.098,0.967957,793.444,421.696,0.82772,945.927,408.579,0.919685],"hand_left_keypoints_2d":[],"hand_right_keypoints_2d":[],"pose_keypoints_3d":[],"face_keypoints_3d":[],"hand_left_keypoints_3d":[],"hand_right_keypoints_3d":[]}]}'

                #face1_dict = json.loads(face1)
                #print(face1_dict.keys())

                people = face1_dict['people']
                #print(people[0].keys())
                ##dict_keys(['person_id', 'pose_keypoints_2d', 'face_keypoints_2d', 'hand_left_keypoints_2d', 'hand_right_keypoints_2d', 'pose_keypoints_3d', 'face_keypoints_3d', 'hand_left_keypoints_3d', 'hand_right_keypoints_3d'])

                #print(json.dumps(people[0]))
                peopleFound = False
                if len(people) == 1:
                    myPerson = people[0]
                    peopleFound = True
                elif len(people) > 1:
                    print("Warning: More than one person found in file... finding person with non Zero Face Points")
                    #finding first person with non zero facial data
                    i=0
                    for p in people:
                        tempface = p['face_keypoints_2d']
                        if tempface[0] != 0:
                            myPerson = people[i]
                            peopleFound = True
                            
                        i+=1
                if peopleFound:
                    pose = myPerson['pose_keypoints_2d']
                    face = myPerson['face_keypoints_2d']
                    if FirstTime:
                        p1 = PersonJSONData(pose, face, op2rig.ear_to_ear_conversion_distance, op2rig.tie_eyelids_together)
                        FirstTime = False
                    else:
                        p1.SetCurrentPose(pose, face)
                    
                    # CODE FOR SETTING BONE POSITIONS:
                    for bone_settings in bone_list:
                        bone = DestArm.pose.bones[bone_settings.DestinationBoneName]
                        bone.rotation_mode = 'XYZ'
                        bone.rotation_euler = mathutils.Euler((0.0, 0.0, 0.0), 'XYZ')
                        bone.rotation_mode = 'QUATERNION'
                        
                        #apply corrections START
    #                    if bone_settings.RollCorrection:
    #                        cAngle = get_correction_Angle(bone_settings.BoneRollCorrectionAxis, bone_settings.BoneRollCorrection)
    #                        bone.rotation = bone.rotation @ cAngle
    #                    if bone_settings.RollCorrection2:
    #                        cAngle = get_correction_Angle(bone_settings.BoneRollCorrectionAxis2, bone_settings.BoneRollCorrection2)
    #                        bone.rotation_quaternion = bone.rotation_quaternion @ cAngle
                        
                        #Apply offset in Rotation or 
                        g = bone_settings.BoneGain
                        x = 0
                        y = 0
                        z = 0
                        apply = False
                        eyelidfastmotiondetected_l =  False
                        eyelidfastmotiondetected_r =  False
                        eyelidtiedtoegetherfound = False
                        if bone_settings.SourceBoneType == 'FACE':
                            if bone_settings.BoneModificationType == 'ROT':
                                if bone_settings.SourceBoneLocationNameFace == 'head':
                                    #sideToSideHeadTile = p1.getHeadTiltSideToSide()
                                    horizontal = p1.getHeadTiltLeftRight()
                                    vertical = p1.getHeadTiltUpDown()
                                    twist = p1.getHeadTiltSideToSide()
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'Jaw':
                                    #sideToSideHeadTile = p1.getHeadTiltSideToSide()
                                    horizontal = 0
                                    vertical = p1.getJawTiltUpDown()
                                    apply = True
                                else:
                                    print('Bone: ' + bone_settings.SourceBoneLocationNameFace + ' not supported in rotation mode.')
                                    #InnerEyebrowTiltCorrection = -0.0004*sideToSideHeadTile
                                    #OuterEyebrowTiltCorrection = -0.000367*sideToSideHeadTile
                                if apply:
                                    if bone_settings.ApplyToX:
                                        if bone_settings.BoneHorizontalAxis == 'PLUSX': 
                                            x =  horizontal
                                        elif bone_settings.BoneHorizontalAxis == 'NEGX':
                                            x =  horizontal*-1
                                        elif bone_settings.BoneHorizontalAxis == 'PLUSY': 
                                            y =  horizontal
                                        elif bone_settings.BoneHorizontalAxis == 'NEGY':
                                            y =  horizontal*-1
                                        elif bone_settings.BoneHorizontalAxis == 'PLUSZ': 
                                            z =  horizontal
                                        elif bone_settings.BoneHorizontalAxis == 'NEGZ':
                                            z =  horizontal*-1
                                    if bone_settings.ApplyToY:
                                        if bone_settings.BoneVerticalAxis == 'PLUSX': 
                                            x =  vertical
                                        elif bone_settings.BoneVerticalAxis == 'NEGX':
                                            x =  vertical*-1
                                        elif bone_settings.BoneVerticalAxis == 'PLUSY': 
                                            y =  vertical
                                        elif bone_settings.BoneVerticalAxis == 'NEGY':
                                            y =  vertical*-1
                                        elif bone_settings.BoneVerticalAxis == 'PLUSZ': 
                                            z =  vertical
                                        elif bone_settings.BoneVerticalAxis == 'NEGZ':
                                            z =  vertical*-1
                                    if bone_settings.ApplyToZ:
                                        if bone_settings.BoneTwistAxis == 'PLUSX': 
                                            x =  twist
                                        elif bone_settings.BoneTwistAxis == 'NEGX':
                                            x =  twist*-1
                                        elif bone_settings.BoneTwistAxis == 'PLUSY': 
                                            y =  twist
                                        elif bone_settings.BoneTwistAxis == 'NEGY':
                                            y =  twist*-1
                                        elif bone_settings.BoneTwistAxis == 'PLUSZ': 
                                            z =  twist
                                        elif bone_settings.BoneTwistAxis == 'NEGZ':
                                            z =  twist*-1
                                    head_angle = mathutils.Euler((math.radians(x*g),math.radians(y+g),math.radians(z*g)), 'XYZ')
                                    bone.rotation_mode = 'XYZ'
                                    bone.rotation_euler = head_angle
                            if bone_settings.BoneModificationType == 'LOC':
                                ref = bone_settings.OpenPoseFaceMovementReference
                                if bone_settings.SourceBoneLocationNameFace == 'LowerLipCenter':
                                    Offset = p1.getLowerLipCenterPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'Jaw':
                                    Offset = p1.getJawPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'LowerLipCenterLeft':
                                    Offset = p1.getLowerLipCenterLeftPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'LowerLipCenterRight':
                                    Offset = p1.getLowerLipCenterRightPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'UpperLipCenter':
                                    Offset = p1.getUpperLipCenterPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'UpperLipCenterLeft':
                                    Offset = p1.getUpperLipCenterLeftPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'UpperLipCenterRight':
                                    Offset = p1.getUpperLipCenterRightPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'UpperLipOuterLeft':
                                    Offset = p1.getUpperLipOuterLeftPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'UpperLipOuterRight':
                                    Offset = p1.getUpperLipOuterRightPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'LowerLipOuterLeft':
                                    Offset = p1.getLowerLipOuterLeftPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'LowerLipOuterRight':
                                    Offset = p1.getLowerLipOuterRightPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'LipLeft':
                                    Offset = p1.getLipLeftPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'LipRight':
                                    Offset = p1.getLipRightPosition(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyeBrowLeftOuter':
                                    Offset = p1.getEyeBrowLeftOuter(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyeBrowLeftCenter':
                                    Offset = p1.getEyeBrowLeftCenter(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyeBrowLeftInner':
                                    Offset = p1.getEyeBrowLeftInner(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyeBrowRightOuter':
                                    Offset = p1.getEyeBrowRightOuter(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyeBrowRightCenter':
                                    Offset = p1.getEyeBrowRightCenter(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyeBrowRightInner':
                                    Offset = p1.getEyeBrowRightInner(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyelidRight':
                                    if not eyelidtiedtoegetherfound and op2rig.tie_eyelids_together:
                                        eyelidtiedtoegetherfound = True
                                        right = p1.getEyelidRight(g,ref)
                                        left = p1.getEyelidLeft(g,ref)
                                        Average = AverageTwoPoints(right, left)
                                        Offset = Average
                                        GlobalTiedTogetherValue = Average
                                    elif op2rig.tie_eyelids_together:
                                        Offset = GlobalTiedTogetherValue
                                    else:
                                        Offset = p1.getEyelidRight(g,ref)
                                    apply = True
                                    #have to get the flutter detect first time only
                                    if not eyelidfastmotiondetected_l:
                                        eyelid_r_current = Offset[1]
                                        eyelidfastmotiondetected_l = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyelidLeft':
                                    if not eyelidtiedtoegetherfound and op2rig.tie_eyelids_together:
                                        eyelidtiedtoegetherfound = True
                                        right = p1.getEyelidRight(g,ref)
                                        left = p1.getEyelidLeft(g,ref)
                                        Average = AverageTwoPoints(right, left)
                                        Offset = Average
                                        GlobalTiedTogetherValue = Average
                                    elif op2rig.tie_eyelids_together:
                                        Offset = GlobalTiedTogetherValue
                                    else:
                                        Offset = p1.getEyelidLeft(g,ref)
                                    apply = True
                                    #have to get the flutter detect first time only
                                    if not eyelidfastmotiondetected_r:
                                        eyelid_l_current = Offset[1]
                                        eyelidfastmotiondetected_r = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyelidLowerRight':
                                    Offset = p1.getEyelidLowerRight(g,ref)
                                    apply = True
                                elif bone_settings.SourceBoneLocationNameFace == 'EyelidLowerLeft':
                                    Offset = p1.getEyelidLowerLeft(g,ref)
                                    apply = True
                                else:
                                    print('Bone: ' + bone_settings.SourceBoneLocationNameFace + ' not supported in location mode.')
                                if apply:
                                    horizontal = Offset[0]
                                    vertical = Offset[1]
                                    if bone_settings.ApplyToX:
                                        if bone_settings.BoneHorizontalAxis == 'PLUSX': 
                                            x =  horizontal
                                        elif bone_settings.BoneHorizontalAxis == 'NEGX':
                                            x =  horizontal*-1
                                        elif bone_settings.BoneHorizontalAxis == 'PLUSY': 
                                            y =  horizontal
                                        elif bone_settings.BoneHorizontalAxis == 'NEGY':
                                            y =  horizontal*-1
                                        elif bone_settings.BoneHorizontalAxis == 'PLUSZ': 
                                            z =  horizontal
                                        elif bone_settings.BoneHorizontalAxis == 'NEGZ':
                                            z =  horizontal*-1
                                        if bone_settings.ApplyTranslationAlongMoreAxisHorizontal:
                                            if (bone_settings.BoneHorizontalAxis == 'PLUSX' or bone_settings.BoneHorizontalAxis == 'NEGX') and bone_settings.TranslationAlongMoreAxisHorizontal.x != 0:
                                                y = y + x*(bone_settings.TranslationAlongMoreAxisHorizontal.y/bone_settings.TranslationAlongMoreAxisHorizontal.x)
                                                z = z + x*(bone_settings.TranslationAlongMoreAxisHorizontal.z/bone_settings.TranslationAlongMoreAxisHorizontal.x)
                                            if (bone_settings.BoneHorizontalAxis == 'PLUSY' or bone_settings.BoneHorizontalAxis == 'NEGY') and bone_settings.TranslationAlongMoreAxisHorizontal.y != 0:
                                                x = x + y*(bone_settings.TranslationAlongMoreAxisHorizontal.x/bone_settings.TranslationAlongMoreAxisHorizontal.y)
                                                z = z + y*(bone_settings.TranslationAlongMoreAxisHorizontal.z/bone_settings.TranslationAlongMoreAxisHorizontal.y)
                                            if (bone_settings.BoneHorizontalAxis == 'PLUSZ' or bone_settings.BoneHorizontalAxis == 'NEGZ') and bone_settings.TranslationAlongMoreAxisHorizontal.z != 0:
                                                x = x + z*(bone_settings.TranslationAlongMoreAxisHorizontal.x/bone_settings.TranslationAlongMoreAxisHorizontal.z)
                                                y = y + z*(bone_settings.TranslationAlongMoreAxisHorizontal.y/bone_settings.TranslationAlongMoreAxisHorizontal.z)
                                    if bone_settings.ApplyToY:
                                        if bone_settings.BoneVerticalAxis == 'PLUSX': 
                                            x =  vertical
                                        elif bone_settings.BoneVerticalAxis == 'NEGX':
                                            x =  vertical*-1
                                        elif bone_settings.BoneVerticalAxis == 'PLUSY': 
                                            y =  vertical
                                        elif bone_settings.BoneVerticalAxis == 'NEGY':
                                            y =  vertical*-1
                                        elif bone_settings.BoneVerticalAxis == 'PLUSZ': 
                                            z =  vertical
                                        elif bone_settings.BoneVerticalAxis == 'NEGZ':
                                            z =  vertical*-1
                                        if bone_settings.ApplyTranslationAlongMoreAxisVertical:
                                            if (bone_settings.BoneVerticalAxis == 'PLUSX' or bone_settings.BoneVerticalAxis == 'NEGX') and bone_settings.TranslationAlongMoreAxisVertical.x != 0: 
                                                y = y + x*(bone_settings.TranslationAlongMoreAxisVertical.y/bone_settings.TranslationAlongMoreAxisVertical.x)
                                                z = z + x*(bone_settings.TranslationAlongMoreAxisVertical.z/bone_settings.TranslationAlongMoreAxisVertical.x)
                                            if (bone_settings.BoneVerticalAxis == 'PLUSY' or bone_settings.BoneVerticalAxis == 'NEGY') and bone_settings.TranslationAlongMoreAxisVertical.y != 0: 
                                                x = x + y*(bone_settings.TranslationAlongMoreAxisVertical.x/bone_settings.TranslationAlongMoreAxisVertical.y)
                                                z = z + y*(bone_settings.TranslationAlongMoreAxisVertical.z/bone_settings.TranslationAlongMoreAxisVertical.y)
                                            if (bone_settings.BoneVerticalAxis == 'PLUSZ' or bone_settings.BoneVerticalAxis == 'NEGZ') and bone_settings.TranslationAlongMoreAxisVertical.z != 0: 
                                                x = x + z*(bone_settings.TranslationAlongMoreAxisVertical.x/bone_settings.TranslationAlongMoreAxisVertical.z)
                                                y = y + z*(bone_settings.TranslationAlongMoreAxisVertical.y/bone_settings.TranslationAlongMoreAxisVertical.z)
                                    bone.location.x = x
                                    bone.location.y = y
                                    bone.location.z = z
                        
                        #apply corrections END (un-does roll corrections)
    #                    if bone_settings.RollCorrection:
    #                        cAngle = get_correction_Angle(bone_settings.BoneRollCorrectionAxis, bone_settings.BoneRollCorrection)
    #                        bone.rotation = bone.rotation @ cAngle.inverted()
    #                    if bone_settings.RollCorrection2:
    #                        cAngle = get_correction_Angle(bone_settings.BoneRollCorrectionAxis2, bone_settings.BoneRollCorrection2)
    #                        bone.rotation_quaternion = bone.rotation_quaternion @ cAngle.inverted()
                    
                    
                    FinalFrameNumber = op2rig.start_frame_to_apply + CurrentFrame
                    
                    #we fast motion detect each eyelid and once true, keyfram ALL eyelid bones for each side.
                    eyelidfastmotiondetected =  False
                                    
                    # CODE FOR Keyframing Bones:
                    for bone_settings in bone_list:
                        bone = DestArm.pose.bones[bone_settings.DestinationBoneName] 
                        if bone_settings.UseCustomKeyFrameNumber:
                            if (CurrentFrame % bone_settings.CustomBonKeyFrameNumber == 0):  
                                if bone_settings.BoneModificationType == 'ROT':
                                    bone.rotation_mode = 'XYZ'
                                    bone.keyframe_insert(data_path='rotation_euler',frame= FinalFrameNumber)
                                if bone_settings.BoneModificationType == 'LOC':
                                    bone.keyframe_insert(data_path='location',frame= FinalFrameNumber)                     
                        elif bone_settings.SourceBoneLocationNameFace == 'EyelidLeft':
                            if not eyelidfastmotiondetected:
                                #keyfram if the eyelid changes a LOT. That way we catch blinks.
                                AmountTheEyelidChanged = abs(eyelid_l_current - PreviousEyelidValue_l)
                                if (AmountTheEyelidChanged > op2rig.eyelid_noise_removal_distance):
                                    eyelidfastmotiondetected = True
                            if eyelidfastmotiondetected or (CurrentFrame % op2rig.eye_keyframe_every_n_frames == 0):
                                bone.keyframe_insert(data_path='location',frame= FinalFrameNumber)
                            PreviousEyelidValue_l = eyelid_l_current 
                        elif bone_settings.SourceBoneLocationNameFace == 'EyelidRight':
                            if not eyelidfastmotiondetected:
                                #keyfram if the eyelid changes a LOT. That way we catch blinks.
                                AmountTheEyelidChanged = abs(eyelid_r_current - PreviousEyelidValue_r)
                                if (AmountTheEyelidChanged > op2rig.eyelid_noise_removal_distance):
                                    eyelidfastmotiondetected = True
                            if eyelidfastmotiondetected or (CurrentFrame % op2rig.eye_keyframe_every_n_frames == 0):
                                bone.keyframe_insert(data_path='location',frame= FinalFrameNumber)
                            PreviousEyelidValue_r = eyelid_r_current 
                        elif (CurrentFrame % op2rig.mouth_keyframe_every_n_frames == 0):
                            if bone_settings.BoneModificationType == 'ROT':
                                bone.rotation_mode = 'XYZ'
                                bone.keyframe_insert(data_path='rotation_euler',frame= FinalFrameNumber)
                            if bone_settings.BoneModificationType == 'LOC':
                                bone.keyframe_insert(data_path='location',frame= FinalFrameNumber)
                    
                    CurrentFrame = CurrentFrame + 1
                    bpy.context.scene.frame_set(FinalFrameNumber)
                    if CurrentFrame > op2rig.number_of_frames_to_apply:
                        keepGoing = False               
                
            else:
                keepGoing = False
        
        win.progress_end()
        return {'FINISHED'}

            
            
            
# ------------------------------------------------------------------------
# register and unregister
# ------------------------------------------------------------------------
def register():
    bpy.utils.register_class(ReadInApplyToRigOperator)
    
def unregister():
    bpy.utils.unregister_class(ReadInApplyToRigOperator)