package frc.robot.Other;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SmartDashboardSetup {
    private SubsystemGetter Get = new SubsystemGetter();

    public void SmartDashboardConfig() {
    SmartDashboard.putData(Get.DriveTrainSub());
    SmartDashboard.putData(Get.ShooterSub());
    SmartDashboard.putData(Get.HangSub());
    SmartDashboard.putData(Get.IntakeSub());
    SmartDashboard.putData(Get.IntakeWheelsSub());
    SmartDashboard.putData(Get.LimelightSub());
    SmartDashboard.putData(Get.DriveTrainSub().getName() + "/Reset Pose 2D", Get.DriveTrainSub().resetPose2d());
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
  }
}
