package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.HangCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Hanger extends SubsystemBase {
    private DigitalInput hangRightMagSwitch = new DigitalInput(8);
    private DigitalInput hangLeftMagSwitch = new DigitalInput(7);
    private CANSparkMax hangerLeftMtr = new CANSparkMax(Constants.Hanger.LeftMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax hangerRightMtr = new CANSparkMax(Constants.Hanger.RightMtrC,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    private RelativeEncoder hangerLeftMtrEnc = hangerLeftMtr.getEncoder();
    private RelativeEncoder hangerRightMtrEnc = hangerRightMtr.getEncoder();

    public Hanger() { // Limelight - get Rotation3d rel to tag. / get navx pose (Can navx get 3d?)
        ResetHangEnc();
    }

    @Override
    public void periodic() {
        // navx.getRotation3d();
        // armRotationMtr1.follow(armRotationMtr2);

        SmartDashboard.putNumber("Hanger/Left/Hang Pos", hangerLeftMtrEnc.getPosition());
        SmartDashboard.putNumber("Hanger/Right/Hang Pos", hangerRightMtrEnc.getPosition());

        SmartDashboard.putNumber("Hanger/Left/Current", hangerLeftMtr.getOutputCurrent());
        SmartDashboard.putNumber("Hanger/Right/Current", hangerRightMtr.getOutputCurrent());

        SmartDashboard.putNumber("Hanger/Left/Hang Velo", hangerLeftMtrEnc.getVelocity());
        SmartDashboard.putNumber("Hanger/Right/Hang Velo", hangerRightMtrEnc.getVelocity());

        SmartDashboard.putBoolean("Hanger/Left/MagSwitch Engaged", LeftHangIsDown());
        SmartDashboard.putBoolean("Hanger/Right/MagSwitch Engaged", RightHangIsDown());

    }

    public void ResetHangEnc() {
        hangerLeftMtrEnc.setPosition(0.0);
        hangerRightMtrEnc.setPosition(0.0);
    }

    public Command ResetHangCmd() {
        return runOnce(this::ResetHangEnc);
    }

    public void LeftHangUp() {
        hangerLeftMtr.set(0.25);
    }

    public void LeftHangDown() {
        hangerLeftMtr.set(-0.25);
    }

    public void LeftHangStop() {
        hangerLeftMtr.set(0);
    }
    public void HangStop() {
        hangerLeftMtr.set(0);
        hangerRightMtr.set(0);
    }
    public Command HangStopCommand(){
        return run(()-> {
            HangStop();
        });
    }

    public void RightHangUp() {
        hangerRightMtr.set(0.25);
    }

    public void RightHangDown() {
        hangerRightMtr.set(-0.25);
    }

    public void RightHangStop() {
        hangerRightMtr.set(0);
    }

    public boolean LeftHangIsDown() {
        return !hangLeftMagSwitch.get();
    }

    public boolean RightHangIsDown() {
        return !hangRightMagSwitch.get();
    }

    public double GetLeftPosition() {
        return hangerLeftMtrEnc.getPosition();
    }

    public double GetRightPosition() {
        return hangerRightMtrEnc.getPosition();
    }

}
