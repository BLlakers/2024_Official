package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private DoubleArraySubscriber m_aprilTagPoseTopic;
  private IntegerPublisher m_priorityTagIdPub;

  private String m_limelightName;
  private AprilTag m_currentAprilTag = new AprilTag(-1, new Pose3d());

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
  }

  public void SetTagIDToTrack(int tagID) {
    m_priorityTagIdPub.accept(tagID);
  }

  @Override
  public void periodic() {
    m_currentAprilTag = getCurrentAprilTag();
    SmartDashboard.putNumber("AprilTag/tagID", m_currentAprilTag.ID);
    SmartDashboard.putNumber("AprilTag/pose/X", m_currentAprilTag.pose.getX());
    SmartDashboard.putNumber("AprilTag/pose/Y", m_currentAprilTag.pose.getY());
    SmartDashboard.putNumber("AprilTag/pose/Z", m_currentAprilTag.pose.getZ());
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

    if (poseArray.value.length < 6) return new AprilTag(-1, new Pose3d());

    Translation3d poseTranslation =
        new Translation3d(
            poseArray.value[0], // x
            poseArray.value[1], // y
            poseArray.value[2] // z
            );

    Rotation3d poseOrientation =
        new Rotation3d(
            poseArray.value[3], // roll = rotx
            poseArray.value[4], // pitch = roty
            poseArray.value[5] // yaw = rotz
            );

    Pose3d aprilTagPose =
        new Pose3d(poseTranslation, poseOrientation); // creating pose3d based off of our
    // translation3d and rot3d and tid
    return new AprilTag(aprilTagId, aprilTagPose);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("AprilTag/tagID", () -> m_currentAprilTag.ID, null);
    builder.addDoubleProperty("AprilTag/pose/X", m_currentAprilTag.pose::getX, null);
    builder.addDoubleProperty("AprilTag/pose/Y", m_currentAprilTag.pose::getY, null);
    builder.addDoubleProperty("AprilTag/pose/Z", m_currentAprilTag.pose::getZ, null);
  }
  
  public interface LimelightExplanation{
  
  }
}
