package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.sql.Driver;


public class SuperStructure extends SubsystemBase {

    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final IntakeShooterSubsystem intakeShooter;
    private final SwerveSubsystem drivebase;
    private final PoseSelector poseSelector;

    private boolean isOperatorManualBoolean = true;
    private final Trigger isOperatorManual = new Trigger(() -> isOperatorManualBoolean);
    private boolean isOperatorScoreBoolean = false;
    private final Trigger isOperatorScore = new Trigger(() -> isOperatorScoreBoolean);

    /**
     * Initialize the robot control {@link SuperStructure}
     */
    public SuperStructure(
            ArmSubsystem arm,
            ElevatorSubsystem elevator,
            IntakeShooterSubsystem intakeShooter,
            SwerveSubsystem drivebase,
            PoseSelector poseSelector) {

        this.arm = arm;
        this.elevator = elevator;
        this.intakeShooter = intakeShooter;
        this.drivebase = drivebase;
        this.poseSelector = poseSelector;

        updateAutoStow();

        if (RobotBase.isSimulation()) {
            SmartDashboard.putData("Side View", Constants.sideRobotView);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isPID", isOperatorManual.negate().getAsBoolean());
        SmartDashboard.putBoolean("isAutoScore", isOperatorScore.getAsBoolean()); 
    }

    public void updateAutoStow() {
        arm.setArmStow(isOperatorManualBoolean);
        elevator.setElevatorStow(isOperatorManualBoolean);
    }

    public Command getCoral() {
        return RobotBase.isSimulation() ?
                drivebase.driveToStation(poseSelector)
                        .alongWith(structureToStation())
                        .until(drivebase.atStation(poseSelector))
                        .andThen(intakeShooter.intakeUntilSensed()
                                .withTimeout(4))
                        .andThen(drivebase.driveBackwards()
                                .alongWith(forceStow())
                                .withTimeout(.5)) :
                drivebase.driveToStation(poseSelector)
                        .alongWith(intakeShooter.intakeUntilSensed())
                        .andThen(safeStow()
                                .withTimeout(.5));
    }

    public Command scoreL1() {
        return
                intakeShooter.pullBackIntake()
                        .alongWith(Commands.waitSeconds(.25)
                                .andThen(drivebase.driveToReef(poseSelector)))
                        .alongWith(
                                Commands.waitSeconds(.5)
                                        .andThen(structureToL1()))
                        .until(drivebase.atReef(poseSelector)
                                .and(structureAtL1()))
                        .andThen(
                                structureToL1()
                                        .alongWith(
                                                Commands.waitSeconds(.5)
                                                        .andThen(intakeShooter.shoot()))
                                        .withTimeout(1.5))
                        .andThen(
                                safeStow());
    }

    public Command scoreL2() {
        return
                intakeShooter.pullBackIntake()
                        .alongWith(Commands.waitSeconds(.25)
                                .andThen(drivebase.driveToReef(poseSelector)))
                        .alongWith(
                                Commands.waitSeconds(.5)
                                        .andThen(structureToL2()))
                        .until(drivebase.atReef(poseSelector)
                                .and(structureAtL2()))
                        .andThen(
                                structureToL2()
                                        .alongWith(
                                                Commands.waitSeconds(.5)
                                                        .andThen(intakeShooter.shoot()))
                                        .withTimeout(1.5))
                        .andThen(
                                safeStow());
    }

    public Command scoreL3() {
        return
                intakeShooter.pullBackIntake()
                        .alongWith(Commands.waitSeconds(.25)
                                .andThen(drivebase.driveToReef(poseSelector)))
                        .alongWith(
                                Commands.waitSeconds(.5)
                                        .andThen(structureToL3()))
                        .until(drivebase.atReef(poseSelector)
                                .and(structureAtL3()))
                        .andThen(
                                structureToL3()
                                        .alongWith(
                                                Commands.waitSeconds(.5)
                                                        .andThen(intakeShooter.shoot()))
                                        .withTimeout(1.5))
                        .andThen(
                                safeStow());
    }

    public Command scoreL4() {
        return
                intakeShooter.pullBackIntake()
                        .alongWith(Commands.waitSeconds(.25)
                                .andThen(drivebase.driveToReef(poseSelector)))
                        .alongWith(
                                Commands.waitSeconds(.5)
                                        .andThen(structureToL4()))
                        .until(drivebase.atReef(poseSelector)
                                .and(structureAtL4()))
                        .andThen(
                                structureToL4()
                                        .alongWith(
                                                Commands.waitSeconds(.5)
                                                        .andThen(intakeShooter.shoot()))
                                        .withTimeout(1.5))
                        .andThen(safeStow());
    }

    public Command autonScoreL4() {
        return
                structureToL4()
                        .alongWith(intakeShooter.pullBackIntake())
                        .until(structureAtL4())
                        .andThen(
                                structureToL4()
                                        .alongWith(
                                                Commands.waitSeconds(1)
                                                        .andThen(intakeShooter.shoot())))
                        .withTimeout(4);
    }

    public Command dealgaeLowAndScore() {
        return
                drivebase.driveToAlgae(poseSelector)
                        .alongWith(structureToDealgaeLow())
                        .alongWith(intakeShooter.intakeAlgaeUntilSensed())
                        .withTimeout(5)
                        .until(intakeShooter.getAlgaeTrigger())
                        .andThen(
                                structureToProcessor()
                                        .alongWith(drivebase.driveBackwards())
                                        .withTimeout(0.5))
                        .andThen(
                                structureToProcessor()
                                        .alongWith(drivebase.driveToProcessor(poseSelector)))
                        .until(drivebase.atProcessor(poseSelector))
                        .andThen(intakeShooter.shootAlgaeUntilGone())
                        .andThen(
                                structureToProcessor()
                                        .alongWith(drivebase.driveBackwards())
                                        .withTimeout(0.5))
                        .andThen(
                                structureToProcessor()
                                        .alongWith(drivebase.driveBackwards())
                                        .withTimeout(0.5));
    }

    public Command dealgaeHighAndScore() {
        return
                drivebase.driveToAlgae(poseSelector)
                        .alongWith(structureToDealgaeHigh())
                        .alongWith(intakeShooter.intakeAlgaeUntilSensed())
                        .withTimeout(5)
                        .until(intakeShooter.getAlgaeTrigger())
                        .andThen(
                                structureToProcessor()
                                        .alongWith(drivebase.driveBackwards())
                                        .withTimeout(0.5))
                        .andThen(
                                structureToProcessor()
                                        .alongWith(drivebase.driveToProcessor(poseSelector)))
                        .until(drivebase.atProcessor(poseSelector))
                        .andThen(intakeShooter.shootAlgaeUntilGone())
                        .andThen(
                                structureToProcessor()
                                        .alongWith(drivebase.driveBackwards())
                                        .withTimeout(0.5))
                        .andThen(
                                structureToProcessor()
                                        .alongWith(drivebase.driveBackwards())
                                        .withTimeout(0.5));
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

    public Command structureToProcessor() {
        return
                arm.armToProcessor()
                        .alongWith(elevator.elevatorToProcessor());
    }

    public Command structureToStation() {
        return
                arm.armToStation()
                        .alongWith(elevator.elevatorToStation());
    }

    public Command structureToDealgaeHigh() {
        return
                arm.armToDealgaeHigh()
                        .alongWith(elevator.elevatorToDealgaeHigh());
    }

    public Command structureToDealgaeLow() {
        return
                arm.armToDealgaeLow()
                        .alongWith(elevator.elevatorToDealgaeLow());
    }

    public Command forceStow() {
        return
                arm.armToStow()
                        .alongWith(elevator.elevatorToStow());
    }

    public Trigger structureAtL1() {
        return
                elevator.atHeight(
                                Units.inchesToMeters(Constants.ElevatorConstants.kL1Setpoint),
                                Units.inchesToMeters(Constants.ElevatorConstants.kAutoScoreToleranceInches))
                        .and(arm.atAngle(
                                Constants.ArmConstants.kL1Setpoint,
                                Constants.ArmConstants.kAutoScoreToleranceDegrees
                        ));
    }

    public Trigger structureAtL2() {
        return
                elevator.atHeight(
                                Units.inchesToMeters(Constants.ElevatorConstants.kL2Setpoint),
                                Units.inchesToMeters(Constants.ElevatorConstants.kAutoScoreToleranceInches))
                        .and(arm.atAngle(
                                Constants.ArmConstants.kL2Setpoint,
                                Constants.ArmConstants.kAutoScoreToleranceDegrees
                        ));
    }

    public Trigger structureAtL3() {
        return
                elevator.atHeight(
                                Units.inchesToMeters(Constants.ElevatorConstants.kL3Setpoint),
                                Units.inchesToMeters(Constants.ElevatorConstants.kAutoScoreToleranceInches))
                        .and(arm.atAngle(
                                Constants.ArmConstants.kL3Setpoint,
                                Constants.ArmConstants.kAutoScoreToleranceDegrees
                        ));
    }

    public Trigger structureAtL4() {
        return
                elevator.atHeight(
                                Units.inchesToMeters(Constants.ElevatorConstants.kL4Setpoint),
                                Units.inchesToMeters(Constants.ElevatorConstants.kAutoScoreToleranceInches))
                        .and(arm.atAngle(
                                Constants.ArmConstants.kL4Setpoint,
                                Constants.ArmConstants.kAutoScoreToleranceDegrees
                        ));
    }

    public Trigger structureAtDealgaeLow() {
        return
                elevator.atHeight(
                                Units.inchesToMeters(Constants.ElevatorConstants.kDealgaeLow),
                                Units.inchesToMeters(Constants.ElevatorConstants.kAutoScoreToleranceInches))
                        .and(arm.atAngle(
                                Constants.ArmConstants.kDealgae,
                                Constants.ArmConstants.kAutoScoreToleranceDegrees
                        ));
    }

    public Trigger structureAtDealgaeHigh() {
        return
                elevator.atHeight(
                                Units.inchesToMeters(Constants.ElevatorConstants.kDealgaeHigh),
                                Units.inchesToMeters(Constants.ElevatorConstants.kAutoScoreToleranceInches))
                        .and(arm.atAngle(
                                Constants.ArmConstants.kDealgae,
                                Constants.ArmConstants.kAutoScoreToleranceDegrees
                        ));
    }

    public Command safeStow() {
        return
                drivebase.driveBackwards()
                        .alongWith(forceStow())
                        .withTimeout(0.5);
    }

    public Command enablePID() {
        return runOnce(() -> {
            isOperatorManualBoolean = false;
        });
    }

    public Command disablePID() {
        return runOnce(() -> {
            isOperatorManualBoolean = true;
        });
    }

    public Command toggleOperatorControls() {
        return runOnce(() -> {
            isOperatorManualBoolean = !isOperatorManualBoolean;
        });
    }

    public Trigger isOperatorManual() {
        return isOperatorManual;
    }

    public Command updateStowCommand() {
        return runOnce(this::updateAutoStow);
    }

    public Command toggleOpScore() {
        return runOnce(() -> {
            isOperatorScoreBoolean = !isOperatorScoreBoolean;
        });
    }

    public Trigger isOperatorScore() {
        return isOperatorScore;
    }

    /**
     * A command to stop all manipulator motors.
     */
    public void stopAllManipulators() {
        run(() -> {
            intakeShooter.stopIntakeShooter();
            elevator.stopElevator();
            arm.stopArm();
        });
    }
}
