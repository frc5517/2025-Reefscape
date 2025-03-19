package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import static edu.wpi.first.units.Units.*;

public class IntakeShooterSubsystem extends SubsystemBase {

    private final ManipSparkMax intakeMotor = new ManipSparkMax(IntakeShooterConstants.kIntakeShooterMotorID);
    private final ManipShooterIntake intakeShooter = new ManipShooterIntake(intakeMotor,
            IntakeShooterConstants.intakeShooterConfig);

    private final DigitalInput coralSensor = new DigitalInput(IntakeShooterConstants.kIntakeShooterCoralSensorID);

    private Trigger coralTrigger = new Trigger(coralSensor::get);

    // Maple sim stuff
    private final SwerveSubsystem swerve;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final IntakeSimulation intakeSimulation;

    StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getStructArrayTopic("Coral Array",
                    Pose3d.struct)
            .publish();
    StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getStructArrayTopic("Algae Array",
                    Pose3d.struct)
            .publish();

    public IntakeShooterSubsystem(SwerveSubsystem drivebase, ElevatorSubsystem elevator, ArmSubsystem arm) {
        intakeMotor.setMotorBrake(true);

        if (RobotBase.isSimulation()) {
            coralTrigger = new Trigger(() -> false);
        }

        this.swerve = drivebase;
        this.elevator = elevator;
        this.arm = arm;
        this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
                "Coral",
                drivebase.getMapleDrive(),
                Inches.of(5),
                IntakeSimulation.IntakeSide.FRONT,
                1
        );
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Coral Trigger", coralTrigger.getAsBoolean());
        SmartDashboard.putNumber("Intake Applied Output", intakeMotor.getAppliedOutput());
    }

    public Command intake() {
        return RobotBase.isSimulation() ?
                Commands.runOnce(this::addSimCoralToIntake, this) :
                Commands.runEnd(() -> intakeShooter.runSpeed(-Constants.IntakeShooterConstants.kIntakeSpeed),
                        () -> intakeShooter.runVoltage(-IntakeShooterConstants.kIntakekG), intakeShooter);
    }

    public Command intakeUntilSensed() {
        return intake()
                .until(coralTrigger);
    }

    public Trigger getCoralTrigger() {
        return coralTrigger;
    }

    public Command shoot() {
        return RobotBase.isSimulation() ?
                simShoot() :
                intakeShooter.runSpeedCommand(Constants.IntakeShooterConstants.kShootSpeed);
    }

    public Command shootUntilGone() {
        return shoot()
                .until(coralTrigger.negate());
    }

    public void stopIntakeShooter() {
        intakeShooter.stopShooterCommand();
    }

    @Override
    public void simulationPeriodic() {
        // Get the positions of the notes (both on the field and in the air);
        coralPoses.set(SimulatedArena.getInstance()
                .getGamePiecesByType("Coral")
                .toArray(Pose3d[]::new)
        );
        algaePoses.set(SimulatedArena.getInstance()
                .getGamePiecesByType("Algae")
                .toArray(Pose3d[]::new)
        );
    }

    public void addSimCoralToIntake() {
        intakeSimulation.addGamePieceToIntake();
    }

    public Command simShoot() {
        return runOnce(() -> {
                if (intakeSimulation.getGamePiecesAmount() > 0) {
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            swerve.getMapleDrive().getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                            new Translation2d(
                                    (Constants.ArmConstants.armConfig.kArmLength / 2 +
                                            Constants.ElevatorConstants.kCenterToElevator) - Math.abs(arm.getAngle().in(Rotations) * 1.2),
                                    0),
                            // Obtain robot speed from drive simulation
                            swerve.getMapleDrive().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            swerve.getMapleDrive().getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            Inches.of(elevator.getHeight() +
                                    Constants.ElevatorConstants.kBottomCarriageToArmInches),
                            // The initial speed of the coral
                            MetersPerSecond.of(4),
                            // The coral is ejected at a 35-degree slope
                            arm.getAngle()));
            intakeSimulation.setGamePiecesCount(0);
                }});
    }
}