// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.Set;

public class RobotContainer {

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final PoseSelector poseSelector = new PoseSelector(drivebase);
    private final AddressableLEDSubsystem ledSubsystem = new AddressableLEDSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem(
            drivebase,
            elevatorSubsystem,
            armSubsystem);
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final SuperStructure superStructure = new SuperStructure(
            armSubsystem,
            elevatorSubsystem,
            intakeShooterSubsystem,
            drivebase,
            poseSelector
    );
    SwerveInputStream swerveStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1)
            .cubeTranslationControllerAxis(true)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
            .deadband(Constants.OperatorConstants.DEADBAND)
            .scaleTranslation(.8)
            .scaleRotation(.4)
            .robotRelative(true)
            .allianceRelativeControl(false)
            .translationHeadingOffset(Rotation2d.k180deg);
    SwerveInputStream opSwerveStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> operatorXbox.getLeftY() * -1,
                    () -> operatorXbox.getLeftX() * -1)
            .cubeTranslationControllerAxis(true)
            .withControllerRotationAxis(() -> operatorXbox.getRightX() * -1)
            .deadband(Constants.OperatorConstants.DEADBAND)
            .scaleTranslation(.8)
            .scaleRotation(.4)
            .robotRelative(true)
            .allianceRelativeControl(false)
            .translationHeadingOffset(Rotation2d.k180deg);
    private SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        setupAutonomous();
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

    }

    private void configureBindings() {
        Command driveRobotOriented = drivebase.driveFieldOriented(swerveStream);
        Command opDriveRobotOriented = drivebase.driveFieldOriented(opSwerveStream);

        //
        ledSubsystem.setDefaultCommand(ledSubsystem.runPatternBoth(
                LEDPattern.solid(ledSubsystem.editColor(Color.kFirstRed)),
                LEDPattern.solid(ledSubsystem.editColor(Color.kCadetBlue))
        ));

        // Sets default drive command
        drivebase.setDefaultCommand(driveRobotOriented);

        // Setup auto stow
        armSubsystem.setAutoStow();
        elevatorSubsystem.setAutoStow();

        // enable auto slow with elevator height
        elevatorSubsystem.scaleHeightHit().whileTrue(Commands.runEnd(
                () -> swerveStream.scaleTranslation(elevatorSubsystem.scaleForDrive(0.8))
                        .scaleRotation(elevatorSubsystem.scaleForDrive(0.4)),
                () -> swerveStream.scaleTranslation(0.8)
                        .scaleRotation(0.6)
        ));

        // Driver Controls
        // Cardinal direction selector for reef pose
        driverXbox.povUp().onTrue(Commands.runOnce(poseSelector::selectNorth));
        driverXbox.povUpRight().onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        driverXbox.povDownRight().onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        driverXbox.povDown().onTrue(Commands.runOnce(poseSelector::selectSouth));
        driverXbox.povDownLeft().onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        driverXbox.povUpLeft().onTrue(Commands.runOnce(poseSelector::selectNorthWest));

        // Choose left or right pose, used in station and reef
        driverXbox.povRight().onTrue(Commands.runOnce(poseSelector::selectRight));
        driverXbox.povLeft().onTrue(Commands.runOnce(poseSelector::selectLeft));

        // Cycle cage and station poses
        driverXbox.leftBumper().onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        driverXbox.rightBumper().onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));

        // Slow speed while holding left trigger
        driverXbox.leftTrigger().whileTrue(Commands.runEnd(
                () -> swerveStream.scaleTranslation(0.3)
                        .scaleRotation(0.2),
                () -> swerveStream.scaleTranslation(0.8)
                        .scaleRotation(0.6)
        ));
        // Boost speed while holding right trigger
        driverXbox.rightTrigger().whileTrue(Commands.runEnd(
                () -> swerveStream.scaleTranslation(1)
                        .scaleRotation(.75),
                () -> swerveStream.scaleTranslation(.8)
                        .scaleRotation(.6)
        ));

        // Toggle to invert controls
        driverXbox.back().toggleOnTrue(Commands.runEnd(
                () -> swerveStream.translationHeadingOffset(true),
                () -> swerveStream.translationHeadingOffset(false)
        ));

        // Toggle field or robot relative speeds
        driverXbox.start().toggleOnTrue(Commands.runEnd(
                () -> swerveStream.robotRelative(false)
                        .allianceRelativeControl(true),
                () -> swerveStream.robotRelative(true)
                        .allianceRelativeControl(false)
        ));

        // Drive to reef
        driverXbox.a().whileTrue(Commands.defer(() -> drivebase.driveToPose(
                poseSelector::flippedReefPose,
                .6), Set.of(drivebase)));
        // Drive to station
        driverXbox.b().whileTrue(Commands.defer(() -> drivebase.driveToPose(
                poseSelector::flippedStationPose,
                .6), Set.of(drivebase)));
        // Drive into climb
        driverXbox.y().whileTrue(Commands.defer(() -> drivebase.driveToPose(
                        poseSelector::flippedCagePose,
                        .7), Set.of(drivebase))
                .andThen(drivebase.driveBackwards()
                        .withTimeout(.5)));

        // Operator Auto Controls
        // Advanced control toggles
        // Toggle between PID auto controls and manual
        operatorXbox.leftTrigger()
                .and(operatorXbox.rightTrigger())
                .onTrue(superStructure.toggleOperatorControls()
                        .andThen(superStructure.updateStowCommand())
                        .andThen(superStructure.runOnce(superStructure::stopAllManipulators)));
        // Toggle between score commands and pid setpoint control
        operatorXbox.povLeft().and(superStructure.isOperatorManual().negate()).onTrue(superStructure.toggleOpScore());

        Trigger manual = superStructure.isOperatorManual();
        Trigger score = superStructure.isOperatorScore();
        Trigger notManual = superStructure.isOperatorManual().negate();
        Trigger notScore = superStructure.isOperatorScore().negate();
        Trigger manualScore = manual.and(score);
        Trigger notManualScore = notManual.and(score);
        Trigger notManualNotScore = notManual.and(notScore);

        // Move structure with pid while holding povLeft
        operatorXbox.a().and(notManualNotScore).whileTrue(superStructure.getCoral());
        operatorXbox.b().and(notManualNotScore).whileTrue(superStructure.scoreL2());
        operatorXbox.x().and(notManualNotScore).whileTrue(superStructure.scoreL3());
        operatorXbox.y().and(notManualNotScore).whileTrue(superStructure.scoreL4());

        // Operator basic PID
        // Auto score and collect while holding face buttons
        operatorXbox.a().and(notManualScore).whileTrue(superStructure.structureToStation());
        operatorXbox.b().and(notManualScore).whileTrue(superStructure.structureToL2());
        operatorXbox.x().and(notManualScore).whileTrue(superStructure.structureToL3());
        operatorXbox.y().and(notManualScore).whileTrue(superStructure.structureToL4());

        // Move structure to dealgae while holding povUp or povDown
        operatorXbox.povUp().and(notManual).whileTrue(superStructure.structureToDealgaeHigh());
        operatorXbox.povDown().and(notManual).whileTrue(superStructure.structureToDealgaeLow());

        // Intake and shoot with bumpers
        operatorXbox.leftBumper().and(notManual).whileTrue(intakeShooterSubsystem.intakeUntilSensed());
        operatorXbox.rightBumper().and(notManual).whileTrue(intakeShooterSubsystem.shoot());

        // Operator Manual Controls
        // Elevator manual up and down while holding A or B.
        operatorXbox.a().and(superStructure.isOperatorManual()).whileTrue(elevatorSubsystem.elevatorUp());
        operatorXbox.b().and(superStructure.isOperatorManual()).whileTrue(elevatorSubsystem.elevatorDown());

        // Arm manual up and down while holding X or Y
        operatorXbox.x().and(superStructure.isOperatorManual()).whileTrue(armSubsystem.armUp());
        operatorXbox.y().and(superStructure.isOperatorManual()).whileTrue(armSubsystem.armDown());

        // Manual intake and shoot without auto stop while holding bumpers
        operatorXbox.leftBumper().and(superStructure.isOperatorManual()).whileTrue(intakeShooterSubsystem.intake());
        operatorXbox.rightBumper().and(superStructure.isOperatorManual()).whileTrue(intakeShooterSubsystem.shoot());

        // Climb anytime while holding start or back
        operatorXbox.start().whileTrue(climbSubsystem.climbUp());
        operatorXbox.back().whileTrue(climbSubsystem.climbDown());
    }

    public void seedEncoders() {
        armSubsystem.seedArmEncoder();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setupAutonomous() {
        NamedCommands.registerCommand("structuresToL1", superStructure.structureToL1());
        NamedCommands.registerCommand("structuresToL2", superStructure.structureToL2());
        NamedCommands.registerCommand("structuresToL3", superStructure.structureToL3());
        NamedCommands.registerCommand("structuresToL4", superStructure.structureToL4());
        NamedCommands.registerCommand("intake", intakeShooterSubsystem.intake());
        NamedCommands.registerCommand("shoot", intakeShooterSubsystem.shoot());

        NamedCommands.registerCommand("enablePID", superStructure.enablePID()
                .andThen(superStructure.updateStowCommand()));
        NamedCommands.registerCommand("disablePID", superStructure.disablePID()
                .andThen(superStructure.updateStowCommand()));

        NamedCommands.registerCommand("getCoral", superStructure.getCoral());
        NamedCommands.registerCommand("scoreL1", superStructure.scoreL1());
        NamedCommands.registerCommand("scoreL2", superStructure.scoreL2());
        NamedCommands.registerCommand("scoreL3", superStructure.scoreL3());
        NamedCommands.registerCommand("scoreL4", superStructure.scoreL4());

        NamedCommands.registerCommand("cycleStationUp", Commands.runOnce(poseSelector::cycleStationSlotUp));
        NamedCommands.registerCommand("cycleStationDown", Commands.runOnce(poseSelector::cycleStationSlotDown));
        NamedCommands.registerCommand("selectSlot1", Commands.runOnce(poseSelector::selectSlot1));
        NamedCommands.registerCommand("selectSlot2", Commands.runOnce(poseSelector::selectSlot2));
        NamedCommands.registerCommand("selectSlot3", Commands.runOnce(poseSelector::selectSlot3));

        NamedCommands.registerCommand("selectSouth", Commands.runOnce(poseSelector::selectSouth));
        NamedCommands.registerCommand("selectSoutheast", Commands.runOnce(poseSelector::selectSouthEast));
        NamedCommands.registerCommand("selectSouthwest", Commands.runOnce(poseSelector::selectSouthWest));
        NamedCommands.registerCommand("selectNorth", Commands.runOnce(poseSelector::selectNorth));
        NamedCommands.registerCommand("selectNortheast", Commands.runOnce(poseSelector::selectNorthEast));
        NamedCommands.registerCommand("selectNorthwest", Commands.runOnce(poseSelector::selectNorthWest));
        NamedCommands.registerCommand("selectLeft", Commands.runOnce(poseSelector::selectLeft));
        NamedCommands.registerCommand("selectRight", Commands.runOnce(poseSelector::selectRight));
        NamedCommands.registerCommand("driveToReef", Commands.defer(() -> drivebase.driveToPose(poseSelector::flippedReefPose, elevatorSubsystem.scaleForDrive(.8)), Set.of(drivebase)));
        NamedCommands.registerCommand("driveToStation", Commands.defer(() -> drivebase.driveToPose(poseSelector::flippedStationPose, elevatorSubsystem.scaleForDrive(.8)), Set.of(drivebase)));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData(autoChooser);
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
    public void lockDrive() {
        Commands.run(drivebase::lock, drivebase);
    }
}
