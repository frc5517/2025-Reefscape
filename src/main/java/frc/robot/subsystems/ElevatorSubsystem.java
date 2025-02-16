package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import maniplib.ManipElevator;
import maniplib.motors.ManipSparkMax;

public class ElevatorSubsystem extends SubsystemBase {

    ManipSparkMax rightElevatorMotor = new ManipSparkMax(13);
    ManipSparkMax leftElevatorMotor = new ManipSparkMax(14);
    ManipElevator elevator = new ManipElevator(rightElevatorMotor, Constants.ElevatorConstants.elevatorConfig);


    public ElevatorSubsystem() {

        leftElevatorMotor.setMotorBrake(true);
        elevator.addFollower(leftElevatorMotor, true);

    }

    public void setAutoStow() {
        elevator.setDefaultCommand(
                elevator.autoStowWithOverride(
                        Constants.ElevatorConstants.kStowSetpoint
                )
        );
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
        return elevator.runElevatorSpeedCommand(Constants.ElevatorConstants.kElevatorSpeed);
    }

    public Command elevatorDown() {
        return elevator.runElevatorSpeedCommand(-Constants.ElevatorConstants.kElevatorSpeed);
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
