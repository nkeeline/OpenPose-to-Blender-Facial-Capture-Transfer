import bpy
            
class OpenPoseToRigToolsPanel(bpy.types.Panel):
    """Creates a Panel for the OpenPose to Rig Tools Window"""
    bl_label = "OP2Rig"
    bl_idname = "OPENPOSE_2_RIG_PT_MAINPANEL"
    bl_space_type = "VIEW_3D"   
    bl_region_type = "UI"    
    bl_category = 'OpenPose2Rig'
    bl_context = "posemode"   

    @classmethod
    def poll(self,context):
        return context.object is not None

    def draw(self, context):
        layout = self.layout
        layout.operator("wm.applyjsonfiles")
                
class OP2RigPanelOne(OpenPoseToRigToolsPanel, bpy.types.Panel):
    bl_idname = "OPENPOSE_2_RIG_PT_TRANSFERSETTINGS"
    bl_label = "Transfer Settings"

    def draw(self, context):
        layout = self.layout
        op2rig = bpy.context.scene.openpose_2_rig_settings 
            
        #layout.label(text="Transfer Settings")
        layout.prop(op2rig, "facial_capture")
        layout.prop(op2rig, "body_capture")
        layout.prop(op2rig, "tie_eyelids_together")
        layout.prop(op2rig, "number_of_frames_to_apply")
        layout.prop(op2rig, "start_frame_to_apply")
        layout.prop(op2rig, "mouth_keyframe_every_n_frames")
        layout.prop(op2rig, "eye_keyframe_every_n_frames")
        layout.prop(op2rig, "ear_to_ear_conversion_distance")
        layout.prop(op2rig, "eyelid_noise_removal_distance")
        layout.prop(op2rig, "rig_name")
        layout.prop(op2rig, "first_JSON_file_to_read_in")
        #layout.operator("wm.applyjsonfiles")
        
class OP2RigPanelTwo(OpenPoseToRigToolsPanel, bpy.types.Panel):
    bl_idname = "OPENPOSE_2_RIG_PT_BONEMAPPING"
    bl_label = "Bone Mapping"

    def draw(self, context):
        layout = self.layout    
        scene = context.scene	
        op2rig = bpy.context.scene.openpose_2_rig_settings 
        layout.prop(op2rig, "bone_mapping_file")
    
        row = layout.row()
        row.operator("wm.read_file")
        row.operator("wm.save_file")
        row = layout.row()
        row.template_list("MY_UL_List", "The_List", scene, "bone_mapping_list", scene,"custom_index")#, type='COMPACT')#, "index")
        row = layout.row() 
        row.operator('bone_mapping_list.new_item', text='NEW') 
        row.operator('bone_mapping_list.delete_item', text='REMOVE') 
        row.operator('bone_mapping_list.move_item', text='UP').direction = 'UP' 
        row.operator('bone_mapping_list.move_item', text='DOWN').direction = 'DOWN'
        row = layout.row()
        row.operator("wm.auto_populate_bone_values")
        
        if scene.custom_index >= 0 and scene.bone_mapping_list: 
            item = scene.bone_mapping_list[scene.custom_index] 
            layout = self.layout    
            row = layout.row() 
            row.label(text="Selected Bone Mapping Parameters")
            box = layout.box()
            box.prop(item, "name") 
            box.prop(item, "SourceBoneType")
            if item.SourceBoneType == 'FACE':
                box.prop(item, "SourceBoneLocationNameFace")
                box.prop(item, "OpenPoseFaceMovementReference")
            if item.SourceBoneType == 'BODY':
                layout.label(text="Body Not Supported ATM")
                #box.prop(item, "SourceBoneLocationNameBody")
            if item.SourceBoneType == 'HAND':
                layout.label(text="Hand Not Supported ATM")
                #box.prop(item, "SourceBoneLocationNameHand")
            box.prop(item, "BoneGain")
            box.prop(item, "DestinationBoneName")
            box.prop(item, "BoneModificationType")        
            row = layout.row() 
            row.prop(item, "ApplyToX")
            if item.ApplyToX:
                box = layout.box()
                box.prop(item, "BoneHorizontalAxis")
                box.prop(item, "ApplyTranslationAlongMoreAxisHorizontal")
                if item.ApplyTranslationAlongMoreAxisHorizontal and item.BoneModificationType == 'LOC':
                    box.prop(item, "TranslationAlongMoreAxisHorizontal")
            row = layout.row() 
            row.prop(item, "ApplyToY")
            if item.ApplyToY:
                box = layout.box()
                box.prop(item, "BoneVerticalAxis")
                box.prop(item, "ApplyTranslationAlongMoreAxisVertical")
                if item.ApplyTranslationAlongMoreAxisVertical and item.BoneModificationType == 'LOC':
                    box.prop(item, "TranslationAlongMoreAxisVertical")
            row = layout.row() 
            if item.BoneModificationType == 'ROT':
                row.prop(item, "ApplyToZ")
                if item.ApplyToZ:
                    box = layout.box()
                    box.prop(item, "BoneTwistAxis")
#            row = layout.row() 
#            row.prop(item, "ApplyRollCorrection")
#            if item.ApplyRollCorrection:
#                box = layout.box()
#                box.prop(item, "BoneRollCorrectionAxis")
#                box.prop(item, "RollCorrection")
#            row = layout.row() 
#            row.prop(item, "ApplyRollCorrection2")
#            if item.ApplyRollCorrection2:
#                box = layout.box()
#                box.prop(item, "BoneRollCorrectionAxis2")
#                box.prop(item, "RollCorrection2")
            row = layout.row() 
            row.prop(item, "UseCustomKeyFrameNumber")
            if item.UseCustomKeyFrameNumber:
                box = layout.box()
                box.prop(item, "CustomBonKeyFrameNumber")
#            row.prop(item, "RemoveParentBonesTranslationEffectCorrection")
#            if item.RemoveParentBonesTranslationEffectCorrection:
#                box = layout.box()
#                box.prop(item, "ParentBoneCorrectionName")
#                box.prop(item, "ParentCorrectionType")
#                box.prop(item, "ParentCorrectionVerticalAxis")
#                if item.ParentCorrectionType == 'LOC':
#                    box.prop(item, "VerticalParentTranslationRemovalAmount")
#                else:
#                    box.prop(item, "VerticalParentRotationAmount")
#                box.prop(item, "VerticalTranslationRemovalAmount")
#                box.prop(item, "ParentCorrectionHorizontalAxis")
#                if item.ParentCorrectionType == 'LOC':
#                    box.prop(item, "HorizontalParentTranslationRemovalAmount")
#                else:
#                    box.prop(item, "HorizontalParentRotationAmount")
#                box.prop(item, "HorizontalTranslationRemovalAmount")
            row = layout.row() 
 
def register():
    bpy.utils.register_class(OP2RigPanelOne)
    bpy.utils.register_class(OP2RigPanelTwo)
    bpy.utils.register_class(OpenPoseToRigToolsPanel)


def unregister():
    bpy.utils.unregister_class(OpenPoseToRigToolsPanel)
    bpy.utils.unregister_class(OP2RigPanelOne)
    bpy.utils.unregister_class(OP2RigPanelTwo)  