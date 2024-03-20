package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  private DoubleArraySubscriber m_aprilTagPoseTopic;
  private IntegerPublisher m_priorityTagIdPub;

  private String m_limelightName;

  public Limelight() {
    this("limelight");
  }

  public Limelight(String cameraName) {
    m_limelightName = cameraName;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(m_limelightName);
    m_aprilTagPoseTopic =
        table
            .getDoubleArrayTopic("targetpose_robotspace")
            .subscribe(new double[] {0, 0, 0, 0, 0, 0});
    m_priorityTagIdPub = table.getIntegerTopic("priorityid").publish();


    NetworkTableInstance.getDefault().getTable(m_limelightName).putValue(
      "targetpose_robotspace", 
      NetworkTableValue.makeDoubleArray(new double[] {1., 2., 3., 0., 0., 0.})
    ); 

    NetworkTableInstance.getDefault().getTable(m_limelightName).putValue(
      "tid", 
      NetworkTableValue.makeInteger(1)
    ); 
  }

  public void SetTagIDToTrack(int tagID) {
    m_priorityTagIdPub.accept(tagID);
  }

  @Override
  public void periodic() {
    AprilTag currentTag = getCurrentAprilTag();
    SmartDashboard.putNumber("AprilTag/tagID", currentTag.ID);
    SmartDashboard.putNumber("AprilTag/pose/X", currentTag.pose.getX());
    SmartDashboard.putNumber("AprilTag/pose/Y", currentTag.pose.getY());
    SmartDashboard.putNumber("AprilTag/pose/Z", currentTag.pose.getZ());
  }

  /**
   * This function gets the april tag the camera is viewing.
   *
   * @return The AprilTag the camera is looking at plus a Pose3d - x, y, z, ROTx, ROTy, ROTz.
   */
  public AprilTag getCurrentAprilTag() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(m_limelightName);
    NetworkTableEntry tid = table.getEntry("tid");
    int aprilTagId = (int) tid.getInteger(-1);
    TimestampedDoubleArray poseArray =
        m_aprilTagPoseTopic.getAtomic(); // (x, y, z, rotx, roty, rotz)

    if (poseArray.value.length < 6 || aprilTagId < 0) return new AprilTag(-1, new Pose3d());

    Translation3d poseTranslation =
        new Translation3d(
            poseArray.value[0], // x
            poseArray.value[1], // y
            poseArray.value[2] // z
            );

    Rotation3d poseOrientation =
        new Rotation3d(
            Units.degreesToRadians(poseArray.value[3]), // roll = rotx
            Units.degreesToRadians(poseArray.value[4]), // pitch = roty
            Units.degreesToRadians(poseArray.value[5]) // yaw = rotz
            );

    Pose3d aprilTagPose = new Pose3d(poseTranslation, poseOrientation);
    // creating pose3d based off of our translation3d and rot3d and tid

    Pose3d aprilTagPoseInBotFrame =
        aprilTagPose.relativeTo(Constants.AprilTagID.LimelightToBotPose);

    System.err.println("Before: " + aprilTagPose);
    System.err.println("After: " + aprilTagPoseInBotFrame);

    AprilTag tag = new AprilTag(aprilTagId, aprilTagPoseInBotFrame);
    System.out.println(tag);

    return tag;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("BotToLimeLight/Transform3d/Translation/X", Constants.AprilTagID.LimelightToBotPose::getX, null);
    builder.addDoubleProperty("BotToLimeLight/Transform3d/Translation/Y", Constants.AprilTagID.LimelightToBotPose::getY, null);
    builder.addDoubleProperty("BotToLimeLight/Transform3d/Translation/Z", Constants.AprilTagID.LimelightToBotPose::getZ, null);
    builder.addDoubleProperty("BotToLimeLight/Transform3d/Rotation/X", () -> Constants.AprilTagID.LimelightToBotPose.getRotation().getX(), null);
    builder.addDoubleProperty("BotToLimeLight/Transform3d/Rotation/Y", () -> Constants.AprilTagID.LimelightToBotPose.getRotation().getY(), null);
    builder.addDoubleProperty("BotToLimeLight/Transform3d/Rotation/Z", () -> Constants.AprilTagID.LimelightToBotPose.getRotation().getZ(), null);

    builder.addDoubleProperty("AprilTag/tagID", () -> getCurrentAprilTag().ID, null);
    builder.addDoubleProperty("AprilTag/pose/X", () -> getCurrentAprilTag().pose.getX(), null);
    builder.addDoubleProperty("AprilTag/pose/Y", () -> getCurrentAprilTag().pose.getY(), null);
    builder.addDoubleProperty("AprilTag/pose/Z", () -> getCurrentAprilTag().pose.getZ(), null);
  }
}
