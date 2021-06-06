import bpy

class OpenPoseToRigifySettings(bpy.types.PropertyGroup):

    facial_capture: bpy.props.BoolProperty(
        name="Facial Capture",
        description="As We read in the JSON files do we apply facial capture to the character",
        default = True
        )
        
    body_capture: bpy.props.BoolProperty(
        name="Apply Body Capture",
        description="As We read in the JSON files do we apply body capture to the character",
        default = True
        )
        
    tie_eyelids_together: bpy.props.BoolProperty(
        name="Tie Eyelids Together",
        description="Ties the characters Eyelids Together, NO WINKING",
        default = True
        )

    number_of_frames_to_apply: bpy.props.IntProperty(
        name = "Number of Samples",
        description="Number of Samples to read in and apply",
        default = 10000,
        min = 0,
        max = 10000
        )

    start_frame_to_apply: bpy.props.IntProperty(
        name = "Starting Frame",
        description="Frame to Apply Motion Capture To",
        default = 0,
        min = 0,
        max = 10000
        )


    mouth_keyframe_every_n_frames: bpy.props.IntProperty(
        name = "Mouth Keyframe Number",
        description="Frame to Apply a Keyframe to, 1 is every frame",
        default = 3,
        min = 1,
        max = 100
        )

    eye_keyframe_every_n_frames: bpy.props.IntProperty(
        name = "Eye Keyframe Number",
        description="Frame to Apply a Keyframe to, 1 is every frame",
        default = 15,
        min = 1,
        max = 100
        )
        
    ear_to_ear_conversion_distance: bpy.props.FloatProperty(
        name = "Ear to Ear Distance",
        description = "The distance in blender units between each ear of the character",
        default = .16,
        min = 0.001,
        max = 1000.0
        )
        
    eyelid_noise_removal_distance: bpy.props.FloatProperty(
        name = "Ignore Eyelid Flutter",
        description = "Blender Units to ignore eyelid jitter, higher values makes eyelids flutter less.",
        default = .01,
        min = 0.00001,
        precision = 6,
        max = 1000.0
        )
    rig_name: bpy.props.StringProperty(
        name="Rig Name",
        description="Rig Name to Apply Capture To",
        default="",
        maxlen=1024
        )
        
    first_JSON_file_to_read_in: bpy.props.StringProperty(
        name="First JSON Capture File",
        description="Choose the first JSON file to read in:",
        default="",
        maxlen=1024,
        subtype='FILE_PATH'
        )
        
    bone_mapping_file: bpy.props.StringProperty(
        name="Bone Mapping File to Read and Save",
        description="Select a File to Read In:",
        default="",
        maxlen=1024,
        subtype='FILE_PATH'
        )
       
    #list_index: bpy.props.IntProperty(name = "Index for my_list", default = 0)
#class ListIndex(bpy.types.IntProperty):
      #list_index: bpy.props.IntProperty(name = "Index for my_list", default = 0)
       # bpy.types.Scene.list_index = IntProperty(name = "Index for my_list", default = 0)   
#class MyBoneMapIndex(bpy.types.PropertyGroup):
    # use an annotation
    #bone_index : bpy.props.IntProperty(name = "Index for my_list", default = 0)   
#class MyBoneMapIndex(bpy.types.IntProperty):
    # use an annotation
    #bpy.props.IntProperty(name = "Index for my_list", default = 0)  


	
def register():
    bpy.utils.register_class(OpenPoseToRigifySettings)
    bpy.types.Scene.openpose_2_rig_settings = bpy.props.PointerProperty(type=OpenPoseToRigifySettings)


def unregister():
    bpy.utils.unregister_class(KeeMapSettings)
    del bpy.types.Scene.openpose_2_rig_settings	    