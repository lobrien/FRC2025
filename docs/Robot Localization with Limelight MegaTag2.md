# Robot Localization with Limelight MegaTag2



Foundation document: [Robot Localization with MegaTag2 | Limelight Documentation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2



Step 1: Access limelight via Web interface

Try http://10.38.81.11

If no answer, work through: [Limelight 3 Quick-Start | Limelight Documentation](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3#3-accessing-the-web-interface) (or whichever model Limelight we have)

From MegaTag2 page:

* [ ] Your limelight's robot-space pose has been configured in the webUI or via the API

  ​	I think this means that you have to specify the offset of the limelight from the origin of your robot. I think we can leave these as [0,0,0] until the limelight's mounting position on the robot is finalized. **Caution:** This is likely to take some trial and error to get accurate enough for aligning with reef. We might need to create at least 2 sides of a reef with accurate dimensions and place apriltags appropriately. (For instance, maybe IDs 9 & 10 or IDs 10 & 11)

  ![]

  The origin of the robot is [0,0,0]. The x,y values are 0 at the center of the robot. The z value might be 0 at the bottom of the robot's frame or on the floor, I do not know. Additionally, the offsets might be measured to the center of the limelight's case

  	- [ ] Modify `VisionConsts.LIMELIGHT_X_OFFSET`, `VisionConsts.LIMELIGHT_Y_OFFSET`, `VisionConsts.LIMELIGHT_Z_OFFSET` 
  	- [ ] 

  

  

## Notes:

According to [TeamUpdate12.pdf](https://firstfrc.blob.core.windows.net/frc2025/Manual/TeamUpdates/TeamUpdate12.pdf), United States matches will use the **Welded** field perimeter locations for the AprilTags. These are available in [CSV Reefscape](https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape-welded.csv) and [JSON Reefscape](https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape-welded.json)