package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import maniplib.ManipElevator;
import maniplib.motors.ManipSparkMax;

import static edu.wpi.first.units.Units.Volts;

public class ElevatorSubsystem extends SubsystemBase {

    ManipSparkMax rightElevatorMotor = new ManipSparkMax(14);
    ManipSparkMax leftElevatorMotor = new ManipSparkMax(13);
    ManipElevator elevator = new ManipElevator(rightElevatorMotor, Constants.ElevatorConstants.elevatorConfig);

    DigitalInput elevatorLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitPort);

    public ElevatorSubsystem() {
        leftElevatorMotor.setMotorBrake(true);
        elevator.addFollower(leftElevatorMotor, true);

    }

    @Override
    public void periodic() {
        elevator.setBottomLimitSwitch(elevatorLimitSwitch.get());
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

    public Command elevatorUp() {
        return runEnd(() -> elevator.runElevatorSpeed(Constants.ElevatorConstants.kElevatorSpeed),
                () -> elevator.runElevatorVoltage(Volts.of(-Constants.ElevatorConstants.elevatorConfig.kElevatorkG)));
    }

    public Command elevatorDown() {
        return runEnd(() -> elevator.runElevatorSpeed(-Constants.ElevatorConstants.kElevatorSpeed),
                () -> elevator.runElevatorVoltage(Volts.of(-Constants.ElevatorConstants.elevatorConfig.kElevatorkG)));
    }

    public void stopElevator() {
        elevator.stopElevator();
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation Mechanism.
        Constants.kElevatorCarriage.setPosition(Constants.ArmConstants.armConfig.kArmLength, elevator.getMechLength());
        Constants.kElevatorTower.setLength(elevator.getMechLength());
    }
}
