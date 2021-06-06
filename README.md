# Released official Blender Addon with UI for use as a Openpose to rig facial capture.
Download the release from the releases section and unzip to a single zip file.  You need to make sure there isn't a zip file inside the one you download, only a folder.  Then go to Blender Preferences, hit the install button and select the zip file.  In pose mode you will now have a new ui show up with a tab called openpose2rig in blender.  You can then map openpose bones to your rig.   I'm sorry, normally I have a tutorial on how to use the addon, but we're using it production and the team is trained on it's use so no need at this point, just know that if you futz around you can map openpose facial outputs to your rigs facial bones and save out the mapping file for future use.

# UI Overview
# Transfer Settings
This section of the UI controls the aspects of the transfer from the source json files to the rig.

![Image of the Blender UI](https://github.com/nkeeline/OpenPose-to-Blender-Facial-Capture-Transfer/blob/master/Pictures/capturesettings.JPG)
If you check the Facial Capture box it will map the facial bones to your rig via the mapping settings below.  Apply Body Capture is a placeholder and doesn't do anything at this time.
Tie Eyelids together makes the eyelids move in unison, it will mean the character cannot wink, but tieing them together may look more natural in most situations as eyelids are tiny and hard to track so tieing them together improves noise rejection.
Number of samples is the 'max' number of files to transfer, if greater than the number of JSON files the transfer will stop at the number of files you have.  Lower this for testing etc.
Starting frame is the frame in blender you wish to start applying the files to your rig.
Mouth Keyframe number is the number of frames to keyframe, lower number such as 1 will keyframe every frame, but may be noisy, two high rejects noise but looses detail performance.
Eye Keyframe number is same as mouth but only for the eyelids which are twitchy because they are small and hard to track in openpose, so they are broken out here.
Ear to Ear distance is a global distance that scales every bone's motion to reasonable distances.  Changing this number can make a performance more subtle or exaggerated.  There may still be a bug in the code on some characters where the displacement is way off, so change this number until you get the right amount of motion in your character.
Ignore Eyelid Flutter is a threshold that attempts to keep the eyelids from moving too quickly.  Change this if you see a lot of noise in the eyelids on transfer.
# Bone Mapping
To map the json openpose 2d data to your rig create a mapping from source to bone using this ui.  You can browse to a file and click the save button to save your bone mapping out to an external json text file:

![Image of the Blender UI](https://github.com/nkeeline/OpenPose-to-Blender-Facial-Capture-Transfer/blob/master/Pictures/BoneMapping.JPG)

Below is the list of all the mappings you can pick from the source JSON onto your rig:

![Image of the Blender UI](https://github.com/nkeeline/OpenPose-to-Blender-Facial-Capture-Transfer/blob/master/Pictures/sourceBoneTypes.JPG)

# Master 'GO' Button
This is the button you press to map a series of files onto your rig:

![Image of the Blender UI](https://github.com/nkeeline/OpenPose-to-Blender-Facial-Capture-Transfer/blob/master/Pictures/Run.JPG)

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
