package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import maniplib.ManipArm;
import maniplib.motors.ManipSparkMax;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

public class ArmSubsystem extends SubsystemBase {

    private final ManipSparkMax armMotor = new ManipSparkMax(12);
    private final ManipArm arm = new ManipArm(armMotor, Constants.ArmConstants.armConfig);

    private final DutyCycleEncoder armABS = new DutyCycleEncoder(ArmConstants.kArmABSID);

    private boolean isDealgaeLockedBoolean = false;
    private final Trigger isDealgaeLocked = new Trigger(() -> isDealgaeLockedBoolean);

    public ArmSubsystem() {
        arm.enableDefaultLimits();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Raw Raw", -armABS.get());
        SmartDashboard.putNumber("Arm ABS Raw", Degrees.convertFrom(-armABS.get(), Rotations));
        SmartDashboard.putNumber("Arm ABS Adjusted", Degrees.convertFrom(-armABS.get(), Rotations) -
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

    public Command armToProcessor() {
        return arm.setGoal(ArmConstants.kProcessorSetpoint);
    }

    public Command armToStation() {
        return arm.setGoal(Constants.ArmConstants.kStationSetpoint);
    }

    public Command armToDealgaeHigh() {
        return arm.setGoal(ArmConstants.kDealgae);
    }

    public Command armToDealgaeLow() {
        return arm.setGoal(ArmConstants.kDealgae);
    }

    public Command armToStow() {
        return arm.setGoal(Constants.ArmConstants.kStowSetpoint);
    }

    public Command armUp() {
        return arm.runArmSpeedCommand(Constants.ArmConstants.kArmSpeed);
    }

    public Command armDown() {
        return arm.runArmSpeedCommand(-Constants.ArmConstants.kArmSpeed);
    }

    public Command runSys() {
        return arm.runSysIdRoutine();
    }

    public Trigger atAngle(double angle, double tolerance) {
        return arm.atAngle(angle, tolerance);
    }

    public void stopArm() {
        arm.stopArm();
    }

    public void seedArmEncoder() {
        arm.addAbsoluteEncoderValue(armABS.get() * -1);
    }

    public Angle getAngle() {
        return arm.getAngle();
    }

    @Override
    public void simulationPeriodic() {
        // Update the arm mechanism simulation.
        Constants.kArmMech.setAngle(arm.getMechAngle());
    }
}
