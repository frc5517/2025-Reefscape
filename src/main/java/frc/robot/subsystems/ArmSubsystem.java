package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import maniplib.ManipArm;
import maniplib.motors.ManipSparkMax;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

public class ArmSubsystem extends SubsystemBase {

    ManipSparkMax armMotor = new ManipSparkMax(12);
    ManipArm arm = new ManipArm(armMotor, Constants.ArmConstants.armConfig);

    DigitalInput armABSInput = new DigitalInput(ArmConstants.kArmABSID);
    DutyCycleEncoder armABS = new DutyCycleEncoder(armABSInput);

    public ArmSubsystem() {
        arm.addAbsoluteEncoderValue(Degrees.convertFrom(armABS.get(), Rotations));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm ABS Raw", Degrees.convertFrom(armABS.get(), Rotations));
        SmartDashboard.putNumber("Arm ABS Adjusted", Degrees.convertFrom(armABS.get(), Rotations) -
                ArmConstants.armConfig.kArmOffsetToHorizantalZero.in(Degrees));
    }

    public void setAutoStow() {
        arm.setDefaultCommand(
                arm.autoStowWithOverride(
                        Constants.ArmConstants.kStowSetpoint
                ));
    }

    public void toggleAutoStow() {
        arm.toggleAutoStow();
    }

    public void setArmStow(boolean armStow) {
        arm.setAutoStow(armStow);
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
