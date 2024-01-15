package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightTags extends SubsystemBase {
    boolean distance;
    public static Double tx2 = 0.0;

    @Override
    public void periodic() {
        // camera stuff, from the documentation
        // the .lime is to look for the network table instance called lime which is the
        // refective tape thing, .april would be the april tag thing
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-april");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry tid = table.getEntry("tid");
        double tx2 = tx.getDouble(0.0);
        double ty2 = ty.getDouble(0.0);
        double s7 = tid.getDouble(0.0);

        // tid is the tag id
        SmartDashboard.putNumber("Tag Number", s7);
        SmartDashboard.putNumber("Limelight2 tx", tx2);
        SmartDashboard.putNumber("Limelight2 ty", ty2);

        // distance, mabeye?????
        // if (s7 < 279){
        // distance = false;
        //
        // if (s7)

    }

}
