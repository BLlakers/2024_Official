package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainPID;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {
    private final DriveTrainPID m_drivetrainSubsystem;
    private final Supplier<AprilTag> m_aprilTagProvider;
    private final Shooter m_Shooter;

    private static final Transform3d APRILTAG_TO_SHOOTINGTARGET = new Transform3d(); // TODO: update

    public AutoShooter(Supplier<AprilTag> aprilTagSupplier, Shooter m_ShooterSub, DriveTrainPID drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_aprilTagProvider = aprilTagSupplier;
        m_Shooter = m_ShooterSub;
    }

    public void initialize() {
        var robotPose = m_drivetrainSubsystem.getPose2d();
    }

    public void execute() {
        AprilTag aprilTag = m_aprilTagProvider.get();

        Pose3d botToTag = aprilTag.pose;
        Pose3d botToTarget = botToTag.transformBy(APRILTAG_TO_SHOOTINGTARGET);
        Translation2d BotPoseT = aprilTag.pose.getTranslation().toTranslation2d();
        Rotation2d BotPoseR = aprilTag.pose.getRotation().toRotation2d();
        Pose2d BotPose2d = new Pose2d(BotPoseT, BotPoseR);

        Translation2d botToTargetTranslation = botToTag.getTranslation().toTranslation2d();
        Rotation2d targetDirection = botToTargetTranslation.getAngle();
        System.out.println("3D:" + botToTag.toString());
        System.out.println("2D:" + botToTargetTranslation.toString());
        Rotation2d currentAngle = m_Shooter.GetShooterAngle();
        Rotation2d DesiredAngle = new Rotation2d(); // TODO
    }
}
