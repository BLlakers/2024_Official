package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stuff extends SubsystemBase {
    // these are global-ish
    public static Double angle = 0.0;
    public Double aligncamera;
    public boolean isAligned;
    public DoubleArraySubscriber zstuff;
    private Double heightfromfloor = 7.5;
    private Double targetheightfromfloor = 1.0; //changes with field design

    public Stuff()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        zstuff = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {0, 0, 0, 0, 0, 0});
        
    }

    public void robotInit(){
        
    }

    

    @Override
    public void periodic() {
        // camera stuff, from the documentation
        // the .lime is to look for the network table instance called lime which is the
        // refective tape thing, .april would be the april tag thing
        //private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        //private NetworkTableEntry tid = getDefault().getTable("limelight").getEntry("tid");
        //private NetworkTableEntry test = getDefault().getTable("limelight").getEntry("botpose");
        //private NetworkTableEntry botpose = getDefault().getTable("limelight").getEntry("targetpose_cameraspace");
        
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry tid = table.getEntry("tid");
        double camerax = tx.getDouble(0.0);
        double cameray = ty.getDouble(0.0);
        
        //jank distance method
        double angleradians = cameray * (3.14159/180.0);
        double jankdistance = (heightfromfloor - targetheightfromfloor) / Math.tan(angleradians) + 10;
        System.out.println(jankdistance);

        //double[] s = zstuff.get();
        //SmartDashboard.putNumber("Limelight X", camerax);
        //SmartDashboard.putNumber("Limelight Y", cameray);
        //System.out.println(test1);

        //testing
        //DoubleArraySubscriber posesub = table.getDoubleArrayTopic("botpose");
        //double[] result = posesub.get();
        //System.out.println(result);

        TimestampedDoubleArray poseArray =  zstuff.getAtomic(); // (x, y, z, rotx, roty, rotz)
        Translation3d poseTranslation = new Translation3d(
            poseArray.value[0],  // x
            poseArray.value[1],  // y
            poseArray.value[2] // z
        );
        
        Rotation3d poseOrientation = new Rotation3d(
            poseArray.value[3], // roll = rotx
            poseArray.value[4], // pitch = roty
            poseArray.value[5] // yaw = rotz
        );

        Pose3d aprilTagPose = new Pose3d(poseTranslation, poseOrientation);
        int aprilTagId = (int) tid.getInteger(-1);
        AprilTag aprilTag = new AprilTag(
            aprilTagId,
            aprilTagPose
        );

        // finding if it is within he perfect angles. perfect angles are from 13.5 to
        // 5.7, with 9.6 being perfectly centered
        if (3 >= camerax && -3 <= camerax) {
            isAligned = true;
            angle = 9.6;
        } else if (camerax == 0 && cameray == 0) {
            isAligned = false;
            angle = 9.6;

        } else {
            isAligned = false;
            // finds where the robot should move to in terms of degreese, which i will
            // eventuly translate into motr rpm (example: 1 degree = 0.2 motor rpm) and i
            // will tell the motrs
            // to move at the rpm need to make it be as close to 9.6 as possible
            angle = camerax;
            SmartDashboard.putNumber("subsystemangle", angle);
        }
        SmartDashboard.putNumber("subsystemangle", angle);
        SmartDashboard.putBoolean("Aligned?", isAligned);

        SmartDashboard.putNumber("AprilTag/tagID", aprilTag.ID);
        SmartDashboard.putNumber("AprilTag/pose/X", aprilTag.pose.getX());
        SmartDashboard.putNumber("AprilTag/pose/Y", aprilTag.pose.getY());
        SmartDashboard.putNumber("AprilTag/pose/Z", aprilTag.pose.getZ());
    }
}
