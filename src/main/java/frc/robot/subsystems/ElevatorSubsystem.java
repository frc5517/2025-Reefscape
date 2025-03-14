package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import maniplib.ManipElevator;
import maniplib.motors.ManipSparkMax;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final ManipSparkMax rightElevatorMotor = new ManipSparkMax(14);
    private final ManipSparkMax leftElevatorMotor = new ManipSparkMax(13);
    private final ManipElevator elevator = new ManipElevator(leftElevatorMotor, Constants.ElevatorConstants.elevatorConfig);
    private final Trigger topLimit = new Trigger(() -> elevator.nearMax(Millimeters.convertFrom(.5, Inches)));
    private final DigitalInput elevatorLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitPort);
    private final Trigger bottomLimit = new Trigger(() -> !elevatorLimitSwitch.get());
    private final Trigger scaleHeightHit = new Trigger(() -> scaleForDrive(1) < Constants.DrivebaseConstants.kScaleSpeedMax);

    public ElevatorSubsystem() {
        rightElevatorMotor.setMotorBrake(true);
        elevator.addFollower(rightElevatorMotor, false);

        bottomLimit.or(topLimit).onTrue(Commands.runOnce(elevator::stopElevator, elevator).andThen(() -> elevator.runkG()));
        bottomLimit.onTrue(runOnce(() -> elevator.setHeight(Inches.of(0))));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator Limit Switch", !elevatorLimitSwitch.get());
        SmartDashboard.putNumber("Elev Motor Rotations", leftElevatorMotor.getPosition());
        SmartDashboard.putNumber("Scale Height", scaleForDrive(1));
        SmartDashboard.putBoolean("Scale Height Hit", scaleHeightHit.getAsBoolean());
    }

    public void setAutoStow() {
        elevator.setDefaultCommand(
                elevator.autoStowWithOverride(
                        Constants.ElevatorConstants.kStowSetpoint
                )
        );
    }

    public void toggleAutoStow() {
        elevator.toggleAutoStow();
    }

    public void setElevatorStow(boolean elevatorStow) {
        elevator.setAutoStow(elevatorStow);
    }

    public Command elevatorToL1() {
        return elevator.setGoal(Constants.ElevatorConstants.kL1Setpoint);
    }

    public Command elevatorToL2() {
        return elevator.setGoal(Constants.ElevatorConstants.kL2Setpoint);
    }

    public Command elevatorToL3() {
        return elevator.setGoal(Constants.ElevatorConstants.kL3Setpoint);
    }

    public Command elevatorToL4() {
        return elevator.setGoal(Constants.ElevatorConstants.kL4Setpoint);
    }

    public Command elevatorToStation() {
        return elevator.setGoal(Constants.ElevatorConstants.kStationSetpoint);
    }

    public Command elevatorToDealgaeHigh() {
        return elevator.setGoal(Constants.ElevatorConstants.kDealgaeHigh);
    }

    public Command elevatorToDealgaeLow() {
        return elevator.setGoal(Constants.ElevatorConstants.kDealgaeLow);
    }

    public Command elevatorToStow() {
        return elevator.setGoal(Constants.ElevatorConstants.kStowSetpoint);
    }

    public Command elevatorUp() {
        return Commands.runEnd(() -> elevator.runElevatorSpeed(Constants.ElevatorConstants.kElevatorSpeed),
                () -> elevator.runkG(), elevator);
    }

    public Command elevatorDown() {
        return Commands.runEnd(() -> elevator.runElevatorSpeed(-Constants.ElevatorConstants.kElevatorSpeed),
                () -> elevator.runkG(), elevator);
    }

    public double scaleForDrive(double inputSpeed) {
        double multiple =
                1.0 - (elevator.getHeightMeters() - Constants.ElevatorConstants.elevatorConfig.kMinHeight.in(Meters))
                        / (Constants.ElevatorConstants.elevatorConfig.kMaxHeight.in(Meters) - Constants.ElevatorConstants.elevatorConfig.kMinHeight.in(Meters));
        return
                multiple > Constants.DrivebaseConstants.kScaleSpeedMax ? inputSpeed :
                        multiple < Constants.DrivebaseConstants.kScaleSpeedMin ? Constants.DrivebaseConstants.kScaleSpeedMin : inputSpeed * multiple;
    }

    public Trigger scaleHeightHit() {
        return scaleHeightHit;
    }

    public void stopElevator() {
        elevator.stopElevator();
    }

    public double getHeight() {
        return Units.metersToInches(elevator.getHeightMeters());
    }

    public Trigger atHeight(double height, double tolerance) {
        return elevator.atHeight(height, tolerance);
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation Mechanism.
        Constants.kElevatorCarriage.setPosition(Constants.ArmConstants.armConfig.kArmLength, elevator.getMechLength());
        Constants.kElevatorTower.setLength(elevator.getMechLength());
    }
}
