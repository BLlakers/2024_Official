package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.DriveTrainPID;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {
    private final Supplier<AprilTag> m_aprilTagProvider;
    private final Shooter m_shooter;

    private final OrientShooterAngle m_orientShooterAngleCommand;
    private final AprilAlignCommand m_aprilAlignCommand;

    private static final Transform3d APRILTAG_TO_SHOOTINGTARGET = new Transform3d(); // TODO: update should be top of shooter 

    public AutoShooter(Supplier<AprilTag> aprilTagSupplier, Shooter m_ShooterSub, DriveTrainPID drivetrainSubsystem) {
        m_aprilTagProvider = aprilTagSupplier;
        m_shooter = m_ShooterSub;

        m_aprilAlignCommand = new AprilAlignCommand(aprilTagSupplier, drivetrainSubsystem);
        m_orientShooterAngleCommand = new OrientShooterAngle(m_ShooterSub, this::CalculateShooterAngle);

        m_orientShooterAngleCommand.unless(m_aprilAlignCommand::isFinished);
    }

    @Override
    public void initialize() {
        m_aprilAlignCommand.initialize();
        m_orientShooterAngleCommand.initialize();
    }

    @Override
    public void execute() {
        m_aprilAlignCommand.execute();
        m_orientShooterAngleCommand.execute();
    }

    public double CalculateShooterAngle() {
        Pose3d botToTargetPose = m_aprilTagProvider.get().pose.transformBy(APRILTAG_TO_SHOOTINGTARGET);
        double desiredShooterAngle = Math.atan2(
                botToTargetPose.getZ(),
                botToTargetPose.getTranslation().toTranslation2d().getNorm());

        return desiredShooterAngle;
    }

    @Override
    public void end(boolean interrupted) {
        m_aprilAlignCommand.end(interrupted);
        m_orientShooterAngleCommand.end(interrupted);

        if (!interrupted)
            m_shooter.Shoot(); // shoot if we actually reached the target
    }

    @Override
    public boolean isFinished() {
        return m_aprilAlignCommand.isFinished() && m_orientShooterAngleCommand.getController().atGoal();
    }
}
