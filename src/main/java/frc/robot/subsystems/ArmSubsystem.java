package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import maniplib.ManipArm;
import maniplib.motors.ManipSparkMax;

public class ArmSubsystem extends SubsystemBase {

    ManipSparkMax armMotor = new ManipSparkMax(12);
    ManipArm arm = new ManipArm(armMotor, Constants.ArmConstants.armConfig);

    public ArmSubsystem() {

    }

    public void setAutoStow() {
        arm.setDefaultCommand(
                arm.autoStowWithOverride(
                        Constants.ArmConstants.kStowSetpoint
        ));
    }

    public Command armToL1() {
        return arm.setGoal(Constants.ArmConstants.kL1Setpoint);
    }

    public Command armToL2() {
        return arm.setGoal(Constants.ArmConstants.kL2Setpoint);
    }

    public Command armToL3() {
        return arm.setGoal(Constants.ArmConstants.kL3Setpoint);
    }

    public Command armToL4() {
        return arm.setGoal(Constants.ArmConstants.kL4Setpoint);
    }

    public Command armUp() {
        return arm.runArmSpeedCommand(Constants.ArmConstants.kArmSpeed);
    }

    public Command armDown() {
        return arm.runArmSpeedCommand(-Constants.ArmConstants.kArmSpeed);
    }

    public void stopArm() {
        arm.stopArm();
    }

    @Override
    public void simulationPeriodic() {
        // Update the arm mechanism simulation.
        Constants.kArmMech.setAngle(arm.getMechAngle());
    }
}
