# OpenPose-to-Blender-Facial-Capture-Transfer
This blender Python Script maps an OpenPose Facial Capture to a blender facial Rig

[Example](https://www.youtube.com/watch?v=bzGW4TDXE-0)

---  
[Usage Tutorial Here](https://www.youtube.com/watch?v=EUR6vsE0k6E)

[and Here](https://www.youtube.com/watch?v=rosD1ckjg4E)

---  
[Blender Artists thread](https://blenderartists.org/t/openpose-ai-facial-motion-capture-to-blender-tutorial/1223147)

---
I recently uploaded OpenPoseToRig.py and it IS NOT FUNCTIONAL.  I will be making a tutorial and finishing this script soon.

# Script Modifications

Open the script in Blender and modify lines as follows to make the script work on your rig:

>In the Setup Variable section modify the lines per the comments:

>>The OpenPose name prefix for the open pose capture, so for example when I ran this command prompt to create an open pose capture:

>>>bin\OpenPoseDemo.exe --video examples\media\IsaacFace.mp4 --write_json output\ --face --write_video output\IsaacFace.avi

>>>the capture made files like: IsaacFace_000000000000_keypoints.json .....

>>>Place all of the JSON files from open pose in the same directory as the blend file you have saved.

>>Then in the script make the top line the following so the script can readin the json file:

        FileIdentifier = "IsaacFace"

>>In the blender file give the script the name of the armature you want to map the facial poses to:

        DestArmName = "Phillip"

>>In the following section measure the distance in blender units between your characters earlobes and put in for the EarLobeToEarLobedistanceInBlenderUnits variable below:

        #Go Into Edit mode and get delta x between earlobes
        #we know the pixel distance between ears in open pose so knowing it for the character
        #give us the conversion between pixels and blender units.
        EarLobeToEarLobedistanceInBlenderUnits = .1746

>>Next put the start frame you wish to start applying the motion character to the character on.

        StartFrameNumber = 0

>>You can change how often you wish to keyframe the mouth and eyes, higher numbers give smoother animations but don't capture each nuance, so this is a touch of an art.  There is two numbers here one for the mouth which openpose does well and moves fast, and the eyes are seperate since it is not as accurate since the points are so close together and tend to jitter more.

        mouth_KeyFrame_Every_Nth_Frame = 3
        eyes_KeyFrame_Every_Nth_Frame = 10

>>If you only wish to transfer the beginning of the captuer you can make this a small number, the script will automatically stop when this number is reached OR it runs out of JSON files to parse:

      
        NumberOfFramesToTransfer = 600

>>Keyframe should always be true, I left it up here in the script for my convenience in testing.

        keyFrame = True
  
---

>>Next way down in the script are lines like this:

       head_bone =  DestArm.pose.bones["head"]

>>if your head bone in your armature isn't name "head" change it here to your rigs name.
Delete any lines for bones your rig doesn't have.

>>Then in the script are lines like this:

        Offset = p1.getLowerLipOuterLeftPosition(1)
        lower_lip_outer_l.location.x = Offset[0]
        lower_lip_outer_l.location.z = Offset[1]
        
>>the json data object returns the lower lip position it wants to put the lip at.  It returns an [x,y] array where x is in blender units int he horzontal direction and y is in blender units in the z.  All bones in your rig must move up and down in the z direction and horzontally in the x direction.  To achieve this in edit mode for the rig, scale the bone 0 in the z and zero in the x and then zero out it's 'roll'.  At this point you can tilt the bone a little so the lower lip follows the surface of the teeth or something.
Farther down in the script are lines like this:

            lower_lip_center_r.keyframe_insert(data_path='location',frame= StartFrameNumber + CurrentFrame)
>This line keyframes the center lip bone, delete these lines for bones you don't have.
