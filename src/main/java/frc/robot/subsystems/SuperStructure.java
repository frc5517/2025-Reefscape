package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SuperStructure extends SubsystemBase {

    ArmSubsystem arm;
    ElevatorSubsystem elevator;
    IntakeShooterSubsystem intakeShooter;

    /**
     * Initialize the robot control {@link SuperStructure}
     */
    public SuperStructure(
        ArmSubsystem arm,
        ElevatorSubsystem elevator,
        IntakeShooterSubsystem intakeShooter) {
        this.arm = arm;
        this.elevator = elevator;
        this.intakeShooter = intakeShooter;
    }

    
    /**
     * A command to stop all manipulator motors.
     * 
     * @return a command to stop all manipulator motors.
     */
    public Command stopAllManipulators() {
        return run(() -> {
            intakeShooter.stopIntakeShooter();
            elevator.stopElevator();
            arm.stopArm();
        });
    }

}
