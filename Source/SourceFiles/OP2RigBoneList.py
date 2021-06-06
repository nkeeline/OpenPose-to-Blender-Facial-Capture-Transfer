import bpy
import json   

class LIST_OT_NewItem(bpy.types.Operator): 
    """Add a new item to the list.""" 
    bl_idname = "bone_mapping_list.new_item" 
    bl_label = "Add a new item" 

    def execute(self, context): 
        context.scene.bone_mapping_list.add() 
        return{'FINISHED'}

class LIST_OT_ReadInFile(bpy.types.Operator): 
    """Read in Bone Mapping File""" 
    bl_idname = "wm.read_file" 
    bl_label = "Read In Bone Mapping File" 

    def execute(self, context): 
        
        context.scene.custom_index - 0    
        bone_list = context.scene.bone_mapping_list
        bone_list.clear()
        
        op2rig = bpy.context.scene.openpose_2_rig_settings 
        filepath = bpy.path.abspath(op2rig.bone_mapping_file)
        file = open(filepath, 'r')

        data = json.load(file)
        
        op2rig.facial_capture = data['facial_capture']
        op2rig.body_capture = data['body_capture']
        op2rig.tie_eyelids_together = data['tie_eyelids_together']
        op2rig.number_of_frames_to_apply = data['number_of_frames_to_apply']
        op2rig.start_frame_to_apply = data['start_frame_to_apply']
        op2rig.mouth_keyframe_every_n_frames = data['mouth_keyframe_every_n_frames']
        op2rig.eye_keyframe_every_n_frames = data['eye_keyframe_every_n_frames']
        op2rig.ear_to_ear_conversion_distance = data['ear_to_ear_conversion_distance']
        op2rig.eyelid_noise_removal_distance = data['eyelid_noise_removal_distance']
        op2rig.rig_name = data['rig_name']
        op2rig.first_JSON_file_to_read_in = data['first_JSON_file_to_read_in']
        #op2rig.rig_type = data['rig_type']
        op2rig.bone_mapping_file = data['bone_mapping_file']
        i = 0
        for p in data['bones']:
            bone_list.add()
            bone = bone_list[i]
            
            bone.name = p['name']
            bone.SourceBoneType = p['SourceBoneType']
            bone.BoneGain = p['BoneGain']
            bone.SourceBoneLocationNameFace = p['SourceBoneLocationNameFace']
            bone.OpenPoseFaceMovementReference = p['OpenPoseFaceMovementReference']
            bone.SourceBoneLocationNameBody = p['SourceBoneLocationNameBody']
            bone.SourceBoneLocationNameHand = p['SourceBoneLocationNameHand']
            bone.DestinationBoneName = p['DestinationBoneName']
            bone.BoneModificationType = p['BoneModificationType']
            bone.ApplyToX = p['ApplyToX']
            bone.BoneHorizontalAxis = p['BoneHorizontalAxis']
            bone.ApplyTranslationAlongMoreAxisHorizontal = p['ApplyTranslationAlongMoreAxisHorizontal']
            bone.TranslationAlongMoreAxisHorizontal.x = p['TranslationAlongMoreAxisHorizontal_x']
            bone.TranslationAlongMoreAxisHorizontal.y = p['TranslationAlongMoreAxisHorizontal_y']
            bone.TranslationAlongMoreAxisHorizontal.z = p['TranslationAlongMoreAxisHorizontal_z']
            bone.ApplyToY = p['ApplyToY']
            bone.BoneVerticalAxis = p['BoneVerticalAxis']
            bone.ApplyTranslationAlongMoreAxisVertical = p['ApplyTranslationAlongMoreAxisVertical']
            bone.TranslationAlongMoreAxisVertical.x = p['TranslationAlongMoreAxisVertical_x']
            bone.TranslationAlongMoreAxisVertical.y = p['TranslationAlongMoreAxisVertical_y']
            bone.TranslationAlongMoreAxisVertical.z = p['TranslationAlongMoreAxisVertical_z']
            bone.ApplyToZ = p['ApplyToZ']
            bone.BoneTwistAxis = p['BoneTwistAxis']
#            bone.ApplyRollCorrection = p['ApplyRollCorrection']
#            bone.RollCorrection = p['RollCorrection']
#            bone.BoneRollCorrectionAxis = p['BoneRollCorrectionAxis']
#            bone.ApplyRollCorrection2 = p['ApplyRollCorrection2']
#            bone.RollCorrection2 = p['RollCorrection2']
#            bone.BoneRollCorrectionAxis2 = p['BoneRollCorrectionAxis2']
            bone.UseCustomKeyFrameNumber = p['UseCustomKeyFrameNumber']
            bone.CustomBonKeyFrameNumber = p['CustomBonKeyFrameNumber']
#            bone.RemoveParentBonesTranslationEffectCorrection = p['RemoveParentBonesTranslationEffectCorrection']
#            bone.ParentBoneCorrectionName = p['ParentBoneCorrectionName']
#            bone.ParentCorrectionType = p['ParentCorrectionType']
#            bone.ParentCorrectionVerticalAxis = p['ParentCorrectionVerticalAxis']
#            bone.VerticalParentTranslationRemovalAmount = p['VerticalParentTranslationRemovalAmount']
#            bone.VerticalParentRotationAmount = p['VerticalParentRotationAmount']
#            bone.VerticalTranslationRemovalAmount = p['VerticalTranslationRemovalAmount']
#            bone.ParentCorrectionHorizontalAxis = p['ParentCorrectionHorizontalAxis']
#            bone.HorizontalParentTranslationRemovalAmount = p['HorizontalParentTranslationRemovalAmount']
#            bone.HorizontalParentRotationAmount = p['HorizontalParentRotationAmount']
#            bone.HorizontalTranslationRemovalAmount = p['HorizontalTranslationRemovalAmount']
            i = i + 1
        file.close()
        
        return{'FINISHED'}


class LIST_OT_AutoReadInValues(bpy.types.Operator): 
    """Take In currently Selected Bone and Auto Populate Values""" 
    bl_idname = "wm.auto_populate_bone_values" 
    bl_label = "Update Bone Name from Selected Bone" 

    def execute(self, context): 
        op2rig = bpy.context.scene.openpose_2_rig_settings 
        bone_list = context.scene.bone_mapping_list
        index = context.scene.custom_index 
        if len(context.selected_objects) == 1:
            rigname = context.selected_objects[0].name
            op2rig.rig_name = rigname
        if len(context.selected_pose_bones) == 1:
            bonename = context.selected_pose_bones[0].name
            #bone_list[index]. = context.selected_pose_bones[0].position.x
            if bone_list[index].name == '':
                bone_list[index].name = bonename
            bone_list[index].DestinationBoneName = bonename
        return{'FINISHED'}
    

class LIST_OT_SaveToFile(bpy.types.Operator): 
    """Save Out Bone Mapping File""" 
    bl_idname = "wm.save_file" 
    bl_label = "Save Bone Mapping File" 

    def execute(self, context): 
        #context.scene.bone_mapping_list.clear() 
        op2rig = bpy.context.scene.openpose_2_rig_settings 
        filepath = bpy.path.abspath(op2rig.bone_mapping_file)
        file = open(filepath, 'w+')
        
        rootParams = {
        "facial_capture":op2rig.facial_capture,
        "body_capture":op2rig.body_capture,
        "tie_eyelids_together":op2rig.tie_eyelids_together,
        "number_of_frames_to_apply":op2rig.number_of_frames_to_apply,
        "start_frame_to_apply":op2rig.start_frame_to_apply,
        "mouth_keyframe_every_n_frames":op2rig.mouth_keyframe_every_n_frames,
        "eye_keyframe_every_n_frames":op2rig.eye_keyframe_every_n_frames,
        "ear_to_ear_conversion_distance":op2rig.ear_to_ear_conversion_distance,
        "eyelid_noise_removal_distance":op2rig.eyelid_noise_removal_distance,
        "rig_name":op2rig.rig_name,
        "first_JSON_file_to_read_in":op2rig.first_JSON_file_to_read_in,
        "bone_mapping_file":op2rig.bone_mapping_file,
        } 
        bone_list = context.scene.bone_mapping_list
        jsonbones = {}
        jsonbones['bones'] = []
        for bone in bone_list:
            jsonbones['bones'].append({
                'name': bone.name,
                'SourceBoneType': bone.SourceBoneType,
                'SourceBoneLocationNameFace': bone.SourceBoneLocationNameFace,
                'OpenPoseFaceMovementReference': bone.OpenPoseFaceMovementReference,
                'SourceBoneLocationNameBody': bone.SourceBoneLocationNameBody,
                'SourceBoneLocationNameHand': bone.SourceBoneLocationNameHand,
                'DestinationBoneName': bone.DestinationBoneName,
                'BoneModificationType': bone.BoneModificationType,
                'BoneGain': bone.BoneGain,
                'ApplyToX': bone.ApplyToX,
                'BoneHorizontalAxis': bone.BoneHorizontalAxis,
                'ApplyTranslationAlongMoreAxisHorizontal': bone.ApplyTranslationAlongMoreAxisHorizontal,
                'TranslationAlongMoreAxisHorizontal_x': bone.TranslationAlongMoreAxisHorizontal.x,
                'TranslationAlongMoreAxisHorizontal_y': bone.TranslationAlongMoreAxisHorizontal.y,
                'TranslationAlongMoreAxisHorizontal_z': bone.TranslationAlongMoreAxisHorizontal.z,
                'ApplyToY': bone.ApplyToY,
                'BoneVerticalAxis': bone.BoneVerticalAxis,
                'ApplyTranslationAlongMoreAxisVertical': bone.ApplyTranslationAlongMoreAxisVertical,
                'TranslationAlongMoreAxisVertical_x': bone.TranslationAlongMoreAxisVertical.x,
                'TranslationAlongMoreAxisVertical_y': bone.TranslationAlongMoreAxisVertical.y,
                'TranslationAlongMoreAxisVertical_z': bone.TranslationAlongMoreAxisVertical.z,
                'ApplyToZ': bone.ApplyToZ,
                'BoneTwistAxis': bone.BoneTwistAxis,
#                'ApplyRollCorrection': bone.ApplyRollCorrection,
#                'RollCorrection': bone.RollCorrection,
#                'BoneRollCorrectionAxis': bone.BoneRollCorrectionAxis,
#                'ApplyRollCorrection2': bone.ApplyRollCorrection2,
#                'RollCorrection2': bone.RollCorrection2,
#                'BoneRollCorrectionAxis2': bone.BoneRollCorrectionAxis2,
                'UseCustomKeyFrameNumber': bone.UseCustomKeyFrameNumber,
                'CustomBonKeyFrameNumber': bone.CustomBonKeyFrameNumber,
#                'RemoveParentBonesTranslationEffectCorrection': bone.RemoveParentBonesTranslationEffectCorrection,
#                'ParentBoneCorrectionName': bone.ParentBoneCorrectionName,
#                'ParentCorrectionType': bone.ParentCorrectionType,
#                'ParentCorrectionVerticalAxis': bone.ParentCorrectionVerticalAxis,
#                'VerticalParentTranslationRemovalAmount': bone.VerticalParentTranslationRemovalAmount,
#                'VerticalParentRotationAmount': bone.VerticalParentRotationAmount,
#                'VerticalTranslationRemovalAmount': bone.VerticalTranslationRemovalAmount,
#                'ParentCorrectionHorizontalAxis': bone.ParentCorrectionHorizontalAxis,
#                'HorizontalParentTranslationRemovalAmount': bone.HorizontalParentTranslationRemovalAmount,
#                'HorizontalParentRotationAmount': bone.HorizontalParentRotationAmount,
#                'HorizontalTranslationRemovalAmount': bone.HorizontalTranslationRemovalAmount
            })
        jsonbones.update(rootParams)
        print(jsonbones)
        json.dump(jsonbones, file)  
        file.close()
        return{'FINISHED'}

class LIST_OT_DeleteItem(bpy.types.Operator): 
    """Delete the selected item from the list.""" 
    bl_idname = "bone_mapping_list.delete_item" 
    bl_label = "Deletes an item" 
    
    @classmethod 
    def poll(cls, context): 
        return context.scene.bone_mapping_list 
    
    def execute(self, context): 
        bone_mapping_list = context.scene.bone_mapping_list
        index = context.scene.custom_index 
        bone_mapping_list.remove(index) 
        index = min(max(0, index - 1), len(bone_mapping_list) - 1) 
        return{'FINISHED'}

class LIST_OT_MoveItem(bpy.types.Operator): 
    """Move an item in the list.""" 
    bl_idname = "bone_mapping_list.move_item" 
    bl_label = "Move an item in the list" 
    direction: bpy.props.EnumProperty(items=(('UP', 'Up', ""), ('DOWN', 'Down', ""),)) 

    @classmethod 
    def poll(cls, context): 
        return context.scene.bone_mapping_list 
    
    def move_index(self): 
        """ Move index of an item render queue while clamping it. """ 
        scene = bpy.context.scene	
        index = scene.custom_index 
        list_length = len(bpy.context.scene.bone_mapping_list) - 1 # (index starts at 0) 
        new_index = index + (-1 if self.direction == 'UP' else 1) 
        index = max(0, min(new_index, list_length)) 
    
    def execute(self, context): 
        bone_mapping_list = context.scene.bone_mapping_list 
        scene = context.scene	
        index = scene.custom_index 
        neighbor = index + (-1 if self.direction == 'UP' else 1) 
        bone_mapping_list.move(neighbor, index) 
        self.move_index() 
        return{'FINISHED'}

        
        
class MY_UL_List(bpy.types.UIList): 
    """Demo UIList.""" 
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        # We could write some code to decide which icon to use here... 
        custom_icon = 'OBJECT_DATAMODE' 
#        ob = data
#        slot = item
#        ma = slot.material
#        # draw_item must handle the three layout types... Usually 'DEFAULT' and 'COMPACT' can share the same code.
#        if self.layout_type in {'DEFAULT', 'COMPACT'}:
#            # You should always start your row layout by a label (icon + text), or a non-embossed text field,
#            # this will also make the row easily selectable in the list! The later also enables ctrl-click rename.
#            # We use icon_value of label, as our given icon is an integer value, not an enum ID.
#            # Note "data" names should never be translated!
#            layout.label(text="", translate=False, icon_value=custom_icon)
#        # 'GRID' layout type should be as compact as possible (typically a single icon!).
#        elif self.layout_type in {'GRID'}:
#            layout.alignment = 'CENTER'
#            layout.label(text="", icon_value=icon)
        
        # Make sure your code supports all 3 layout types if 
        if self.layout_type in {'DEFAULT', 'COMPACT'}: 
            layout.label(text=item.name, icon = custom_icon) 
        elif self.layout_type in {'GRID'}: 
            layout.alignment = 'CENTER' 
            layout.label(text="", icon = custom_icon) 





# ------------------------------------------------------------------------
# register and unregister
# ------------------------------------------------------------------------
def register():
    bpy.utils.register_class(MY_UL_List)
    bpy.utils.register_class(LIST_OT_NewItem)
    bpy.utils.register_class(LIST_OT_DeleteItem)
    bpy.utils.register_class(LIST_OT_MoveItem)
    bpy.utils.register_class(LIST_OT_ReadInFile)
    bpy.utils.register_class(LIST_OT_SaveToFile)
    bpy.utils.register_class(LIST_OT_AutoReadInValues)
    
def unregister():
    bpy.utils.unregister_class(MY_UL_List)
    bpy.utils.unregister_class(LIST_OT_NewItem)
    bpy.utils.unregister_class(LIST_OT_DeleteItem)
    bpy.utils.unregister_class(LIST_OT_MoveItem)
    bpy.utils.unregister_class(LIST_OT_ReadInFile)
    bpy.utils.unregister_class(LIST_OT_SaveToFile)
    bpy.utils.unregister_class(LIST_OT_AutoReadInValues)