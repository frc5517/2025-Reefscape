package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import maniplib.ManipArm;
import maniplib.ManipElevator;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;


public class SuperStructure extends SubsystemBase {

    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final IntakeShooterSubsystem intakeShooter;

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

        SmartDashboard.putData("Side View", Constants.sideRobotView);
    }

    public Command structureToL1() {
        return
                arm.armToL1()
                        .alongWith(elevator.elevatorToL1());
    }

    public Command structureToL2() {
        return
                arm.armToL2()
                        .alongWith(elevator.elevatorToL2());
    }

    public Command structureToL3() {
        return
                arm.armToL3()
                        .alongWith(elevator.elevatorToL3());
    }

    public Command structureToL4() {
        return
                arm.armToL4()
                        .alongWith(elevator.elevatorToL4());
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
