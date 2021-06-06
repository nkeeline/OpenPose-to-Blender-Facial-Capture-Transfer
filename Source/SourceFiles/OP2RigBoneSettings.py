import bpy
 
     
class BoneMappingListItem(bpy.types.PropertyGroup): 
      #"""Group of properties representing a bone mapping from OpenPose to a Rig""" 
      
    name : bpy.props.StringProperty()
    label : bpy.props.StringProperty()
    description : bpy.props.StringProperty()
    
    SourceBoneType: bpy.props.EnumProperty(
        name="Type Of Bone",
        description="Is this a face, body or hand bone",
        items=[ ('FACE', "Face", ""),
                ('BODY', "Body", ""),
                ('HAND', "Hand", "")
               ]
        )
        
    SourceBoneLocationNameFace: bpy.props.EnumProperty(
        name="Source Bone Name Face",
        description="This is the source from JSON like head_bone that designates where this bone is getting it's information from in the JSON file",
        items=[ ('head', "head", ""),
                ('Jaw', "Jaw", ""),
                ('LowerLipCenter', "LowerLipCenter", ""),
                ('LowerLipCenterLeft', "LowerLipCenterLeft", ""),
                ('LowerLipCenterRight', "LowerLipCenterRight", ""),
                ('UpperLipCenter', "UpperLipCenter", ""),
                ('UpperLipCenterLeft', "UpperLipCenterLeft", ""),
                ('UpperLipCenterRight', "UpperLipCenterRight", ""),
                ('UpperLipOuterLeft', "UpperLipOuterLeft", ""),
                ('UpperLipOuterRight', "UpperLipOuterRight", ""),
                ('LowerLipOuterLeft', "LowerLipOuterLeft", ""),
                ('LowerLipOuterRight', "LowerLipOuterRight", ""),
                ('LipLeft', "LipLeft", ""),
                ('LipRight', "LipRight", ""),
                ('EyeBrowLeftOuter', "EyeBrowLeftOuter", ""),
                ('EyeBrowLeftCenter', "EyeBrowLeftCenter", ""),
                ('EyeBrowLeftInner', "EyeBrowLeftInner", ""),
                ('EyeBrowRightOuter', "EyeBrowRightOuter", ""),
                ('EyeBrowRightCenter', "EyeBrowRightCenter", ""),
                ('EyeBrowRightInner', "EyeBrowRightInner", ""),
                ('EyelidRight', "EyelidRight", ""),
                ('EyelidLeft', "EyelidLeft", ""),
                ('EyelidLowerRight', "EyelidLowerRight", ""),
                ('EyelidLowerLeft', "EyelidLowerLeft", "")
               ]
        )
        
    OpenPoseFaceMovementReference: bpy.props.EnumProperty(
        name="Face Reference",
        description="This is the reference point on the face the script will calculate offset from.  So basically it's the anchor point on the face used to measure distnance from.",
        items=[ ('DEFAULT', "Default", ""),
                ('RIGHTEYE', "Right Eye", ""),
                ('LEFTEYE', "Left Eye", ""),
                ('NOSE', "Nose", ""),
                ('CHIN', "Chin", "")
               ]
        )
    SourceBoneLocationNameBody: bpy.props.EnumProperty(
        name="Source Bone Name Body",
        description="This is the source from JSON like head_bone that designates where this bone is getting it's information from in the JSON file",
        items=[ ('CHEST', "Chest", ""),
                ('BICEPRIGHT', "Bicep Right", ""),
                ('FOREARMRIGHT', "ForeArm Right", "")
               ]
        )
        
    SourceBoneLocationNameHand: bpy.props.EnumProperty(
        name="Source Bone Name Hand",
        description="This is the source from JSON like head_bone that designates where this bone is getting it's information from in the JSON file",
        items=[ ('INDEXTIP', "Index Tip", ""),
                ('INDEXCENTER', "Index Center", ""),
                ('INDEXBASE', "Index Base", ""),
                ('PALM', "Palm", "")
               ]
        )
        
    DestinationBoneName: bpy.props.StringProperty(
        name="Destination Bone Name",
        description="This is the name for the rig bone name.",
        default="",
        maxlen=1024
        )
      
    BoneModificationType: bpy.props.EnumProperty(
        name="Bone Modification Type",
        description="Translate the bone method either location or rotation",
        items=[ ('LOC', "Location", ""),
                ('ROT', "Rotation", ""),
               ]
        )
        
    BoneGain: bpy.props.FloatProperty(
        name="Bone Gain",
        description="The amount of gain to apply to the bone, values greater than one make it move more.",
        default = 1
        )
        
    ApplyToX: bpy.props.BoolProperty(
        name="Apply Horizontal",
        description="Applies the horizontal translation or rotatiation of the openpose points to the bone",
        default = True
        )
      
    BoneHorizontalAxis: bpy.props.EnumProperty(
        name="Horizontal Axis",
        description="Axis to Apply horizontal translation or rotation to.",
        items=[ ('PLUSX', "+X", ""),
                ('PLUSY', "+Y", ""),
                ('PLUSZ', "+Z", ""),
                ('NEGX', "-X", ""),
                ('NEGY', "-Y", ""),
                ('NEGZ', "-Z", "")
               ]
        )
    ApplyTranslationAlongMoreAxisHorizontal: bpy.props.BoolProperty(
        name="Translate along more Axis Horizontal",
        description="This will take the translation along the axis above selected and move the other two axis proportionally by an amount",
        default = False
        )
        
        
    TranslationAlongMoreAxisHorizontal: bpy.props.FloatVectorProperty(
        name="Move other axis Horizontally",
        description="Move the bone in the direction you want it to go and put it's x, y and z in here to have it move in this direction when openpose horizontal motion occurs",
        subtype = 'TRANSLATION',
        unit = 'LENGTH',
        default = (0.0, 0.0, 0.0), 
        size = 3
        )
        
    ApplyToY: bpy.props.BoolProperty(
        name="Apply Vertical",
        description="Applies the vertical translation or rotatiation of the openpose points to the bone",
        default = True
        )
      
    BoneVerticalAxis: bpy.props.EnumProperty(
        name="Vertical Axis",
        description="Axis to Apply vertical translation or rotation to.",
        items=[ ('PLUSX', "+X", ""),
                ('PLUSY', "+Y", ""),
                ('PLUSZ', "+Z", ""),
                ('NEGX', "-X", ""),
                ('NEGY', "-Y", ""),
                ('NEGZ', "-Z", "")
               ]
        )
    ApplyTranslationAlongMoreAxisVertical: bpy.props.BoolProperty(
        name="Translate along more Axis Vertical",
        description="This will take the translation along the axis above selected and move the other two axis proportionally by an amount",
        default = False
        )
        
        
    TranslationAlongMoreAxisVertical: bpy.props.FloatVectorProperty(
        name="Move other axis Vertically",
        description="Move the bone in the direction you want it to go and put it's x, y and z in here to have it move in this direction when openpose Vertical motion occurs",
        subtype = 'TRANSLATION',
        unit = 'LENGTH',
        default = (0.0, 0.0, 0.0), 
        size = 3
        )
        
    ApplyToZ: bpy.props.BoolProperty(
        name="Apply Twist",
        description="Applies the twist translation or rotatiation of the openpose points to the bone",
        default = False
        )
      
    BoneTwistAxis: bpy.props.EnumProperty(
        name="Twist Axis",
        description="Axis to Apply twist translation or rotation to.",
        items=[ ('PLUSX', "+X", ""),
                ('PLUSY', "+Y", ""),
                ('PLUSZ', "+Z", ""),
                ('NEGX', "-X", ""),
                ('NEGY', "-Y", ""),
                ('NEGZ', "-Z", "")
               ]
        )
        
#    ApplyRollCorrection: bpy.props.BoolProperty(
#        name="Apply Roll Correction",
#        description="Apply a Roll Correction to the Bone if we don't have an axis that is straight up and down to the openpose capture.",
#        default = False
#        )
#        
#        
#    RollCorrection: bpy.props.FloatProperty(
#        name="1st Roll Correction",
#        description="The Angle to roll the bone before applying a transform to it.",
#        subtype = 'ANGLE',
#        unit = 'ROTATION',
#        default = 0
#        )
#      
#    BoneRollCorrectionAxis: bpy.props.EnumProperty(
#        name="Roll 1st Correction Axis",
#        description="Axis to Apply vertical translation or rotation to.",
#        items=[ ('PLUSX', "X", ""),
#                ('PLUSY', "Y", ""),
#                ('PLUSZ', "Z", "")
#               ]
#        )
#        
#    ApplyRollCorrection2: bpy.props.BoolProperty(
#        name="Apply 2nd Roll Correction",
#        description="Apply a 2nd Roll Correction to the Bone if we don't have an axis that is straight up and down to the openpose capture.",
#        default = False
#        )
#        
#        
#    RollCorrection2: bpy.props.FloatProperty(
#        name="2nd Roll Correction",
#        description="The Angle to roll the bone before applying a transform to it.",
#        subtype = 'ANGLE',
#        unit = 'ROTATION',
#        default = 0
#        )
#      
#    BoneRollCorrectionAxis2: bpy.props.EnumProperty(
#        name="Roll Correction Axis",
#        description="Axis to Apply vertical translation or rotation to.",
#        items=[ ('PLUSX', "X", ""),
#                ('PLUSY', "Y", ""),
#                ('PLUSZ', "Z", "")
#               ]
#        )
        
    UseCustomKeyFrameNumber: bpy.props.BoolProperty(
        name="Use Custom Keyframe Number",
        description="Override the global keyframe number to remove noise or increase sensitivity.",
        default = False
        )
        
        
    CustomBonKeyFrameNumber: bpy.props.IntProperty(
        name="Keyframe Bone Every N Frames",
        description="Keyframe this bone every n Frames",
        default = 5,
        min = 1,
        max = 100
        )
        
#    RemoveParentBonesTranslationEffectCorrection: bpy.props.BoolProperty(
#        name="Removes the effects of a Parent Bone",
#        description="If the jaw Bone moves a lip bone, this correction moves the lower lip back together so we can then translate them with respect to the nose.",
#        default = False
#        )
#        
#    ParentBoneCorrectionName: bpy.props.StringProperty(
#        name="Name of Parent Bone to this one.",
#        description="This is the name for the parent bone that is effecting this one.",
#        default="",
#        maxlen=1024
#        )
#      
#    ParentCorrectionType: bpy.props.EnumProperty(
#        name="Parent Bone Type of Motion to Nullify",
#        description="The parent bones motion type that will be nulled out in the child bone",
#        items=[ ('LOC', "Location", ""),
#                ('ROT', "Rotation", ""),
#               ]
#        )
#      
#    ParentCorrectionVerticalAxis: bpy.props.EnumProperty(
#        name="Parent Axis to remove motion or Angle from child",
#        description="Axis to Apply vertical translation or rotation to.",
#        items=[ ('PLUSX', "X", ""),
#                ('PLUSY', "Y", ""),
#                ('PLUSZ', "Z", "")
#               ]
#        )
#        
#        
#    VerticalParentTranslationRemovalAmount: bpy.props.FloatProperty(
#        name="Parent Bones Translation Amount",
#        description="If the parent bone is moved by this amount, move the child back up by the below amount.",
#        subtype = 'DISTANCE',
#        default = 0
#        )
#        
#        
#    VerticalParentRotationAmount: bpy.props.FloatProperty(
#        name="Rotation in degrees of parent bone to correct out.",
#        description="If the parent bone is rotated along the above axis by this amount, translate the child the below amount vertically.",
#        subtype = 'ANGLE',
#        default = 0
#        )
#        
#        
#    VerticalTranslationRemovalAmount: bpy.props.FloatProperty(
#        name="This Bones translation to null out angle above.",
#        description="If the parent bone moves the above amount move it vertically back by this amount.",
#        subtype = 'DISTANCE',
#        default = 0
#        )
#      
#    ParentCorrectionHorizontalAxis: bpy.props.EnumProperty(
#        name="Parent Axis to remove motion or Angle from child",
#        description="Axis to Apply horizontal translation or rotation to.",
#        items=[ ('PLUSX', "X", ""),
#                ('PLUSY', "Y", ""),
#                ('PLUSZ', "Z", "")
#               ]
#        )
#        
#        
#    HorizontalParentTranslationRemovalAmount: bpy.props.FloatProperty(
#        name="Parent Bones Translation Amount",
#        description="If the parent bone is moved by this amount, move the child back up by the below amount.",
#        subtype = 'DISTANCE',
#        default = 0
#        )
#        
#        
#    HorizontalParentRotationAmount: bpy.props.FloatProperty(
#        name="Rotation in degrees of parent bone to correct out.",
#        description="If the parent bone is rotated along the above axis by this amount, translate the child the below amount Horizontally.",
#        subtype = 'ANGLE',
#        default = 0
#        )
#        
#        
#    HorizontalTranslationRemovalAmount: bpy.props.FloatProperty(
#        name="This Bones translation to null out angle above.",
#        description="If the parent bone moves the above amount move it horizontally back by this amount.",
#        subtype = 'DISTANCE',
#        default = 0
#        )
               
#    DirectionQuatToApplyXandY: bpy.props.FloatVectorProperty(
#        name="Rotation Correction",
#        description="This quaternion will rotat the bone then translate it in the rotated direction then rotate it back at the end.",
#        #default = Quaternion(1,0,0,0)
#        subtype = 'QUATERNION',
#        size = 4
#        )
        
# ------------------------------------------------------------------------
# register and unregister
# ------------------------------------------------------------------------
def register():
    bpy.utils.register_class(BoneMappingListItem)
    bpy.types.Scene.custom_index = bpy.props.IntProperty()
    bpy.types.Scene.bone_mapping_list = bpy.props.CollectionProperty(type = BoneMappingListItem) 
    
def unregister():
    bpy.utils.unregister_class(BoneMappingListItem)
    del bpy.types.Scene.bone_mapping_list
    del bpy.types.Scene.custom_index