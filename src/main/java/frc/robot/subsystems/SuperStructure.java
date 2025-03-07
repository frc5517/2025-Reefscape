package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;


public class SuperStructure extends SubsystemBase {

    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final IntakeShooterSubsystem intakeShooter;

    private boolean isOperatorManualBoolean = true;
    private final Trigger isOperatorManual = new Trigger(() -> isOperatorManualBoolean);

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

        updateAutoStow();

        SmartDashboard.putData("Side View", Constants.sideRobotView);
    }

    public void updateAutoStow() {
        arm.setArmStow(isOperatorManualBoolean);
        elevator.setElevatorStow(isOperatorManualBoolean);
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

    public Command structureToStation() {
        return
                arm.armToStation()
                        .alongWith(elevator.elevatorToStation());
    }

    public Command toggleOperatorControls() {
        return runOnce(() -> {
            isOperatorManualBoolean = !isOperatorManualBoolean;
        });
    }

    public Command updateStowCommand() {
        return runOnce(this::updateAutoStow);
    }

    public Trigger isOperatorManual() {
        return isOperatorManual;
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
