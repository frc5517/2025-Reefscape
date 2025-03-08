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
import maniplib.utils.AllianceFlipUtil;
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
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final SuperStructure superStructure = new SuperStructure(
            armSubsystem,
            elevatorSubsystem,
            intakeShooterSubsystem
    );

    private final Trigger elevatorSlow = new Trigger(() -> elevatorSubsystem.getHeight() > 30);
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
            .deadband(Constants.OperatorConstants.DEADBAND)
            .scaleTranslation(0.6)
            .scaleRotation(0.3)
            .allianceRelativeControl(true);
    SwerveInputStream robotOriented = driveAngularVelocity.copy()
            .robotRelative(true)
            .allianceRelativeControl(false)
            .translationHeadingOffset(Rotation2d.k180deg);
    Command drive = drivebase.driveCommand(
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1,
            () -> driverXbox.getRightX() * -1
    );
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
        Command driveRobotOriented = drivebase.driveFieldOriented(robotOriented);
        Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);

        ledSubsystem.setDefaultCommand(ledSubsystem.runPatternBoth(
                LEDPattern.solid(ledSubsystem.editColor(Color.kFirstRed)),
                LEDPattern.solid(ledSubsystem.editColor(Color.kCadetBlue))
        ));

        // Default Commands
        drivebase.setDefaultCommand(driveRobotOriented);

        armSubsystem.setAutoStow();
        elevatorSubsystem.setAutoStow();
        operatorXbox.leftTrigger()
                .and(operatorXbox.rightTrigger())
                .onTrue(superStructure.toggleOperatorControls()
                        .andThen(superStructure.updateStowCommand())
                        .andThen(superStructure.runOnce(superStructure::stopAllManipulators)));

        // Driver Controls
        driverXbox.pov(0).onTrue(Commands.runOnce(poseSelector::selectNorth));
        driverXbox.pov(45).onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        driverXbox.pov(135).onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        driverXbox.pov(180).onTrue(Commands.runOnce(poseSelector::selectSouth));
        driverXbox.pov(225).onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        driverXbox.pov(315).onTrue(Commands.runOnce(poseSelector::selectNorthWest));

        driverXbox.pov(90).onTrue(Commands.runOnce(poseSelector::selectRight));
        driverXbox.pov(270).onTrue(Commands.runOnce(poseSelector::selectLeft));

        driverXbox.leftBumper().onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        driverXbox.rightBumper().onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));

        elevatorSlow.whileTrue(Commands.runEnd(
                () -> robotOriented.scaleTranslation(.3)
                        .scaleRotation(.15),
                () -> robotOriented.scaleRotation(.6)
                        .scaleRotation(.3)
        ));

        driverXbox.rightTrigger().whileTrue(Commands.runEnd(
                () -> robotOriented.scaleTranslation(.8)
                        .scaleRotation(.4),
                () -> robotOriented.scaleTranslation(.6)
                        .scaleRotation(.3)
        ));
        driverXbox.leftTrigger().whileTrue(Commands.runEnd(
                () -> robotOriented.scaleTranslation(.3)
                        .scaleTranslation(.15),
                () -> robotOriented.scaleTranslation(.6)
                        .scaleTranslation(.3)
        ));

        driverXbox.back().toggleOnTrue(Commands.runEnd(
                () -> robotOriented.translationHeadingOffset(true),
                () -> robotOriented.translationHeadingOffset(false)
        ));

        driverXbox.start().toggleOnTrue(Commands.runEnd(
                () -> robotOriented.robotRelative(false)
                        .allianceRelativeControl(true),
                () -> robotOriented.robotRelative(true)
                        .allianceRelativeControl(false)
        ));

        driverXbox.a().whileTrue(Commands.defer(() -> drivebase.driveToPose(
                AllianceFlipUtil.shouldFlip() ?
                        AllianceFlipUtil.flip(poseSelector.reefPose()) :
                        poseSelector.reefPose()), Set.of(drivebase)));
        driverXbox.b().whileTrue(Commands.defer(() -> drivebase.driveToPose(
                AllianceFlipUtil.shouldFlip() ?
                        AllianceFlipUtil.flip(poseSelector.stationPose()) :
                        poseSelector.stationPose()), Set.of(drivebase)));

        // Operator Auto Controls
        operatorXbox.a().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToStation());
        operatorXbox.b().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToL2());
        operatorXbox.x().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToL3());
        operatorXbox.y().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToL4());

        operatorXbox.povUp().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToDealgaeHigh());
        operatorXbox.povDown().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToDealgaeLow());

        operatorXbox.leftBumper().and(superStructure.isOperatorManual().negate()).whileTrue(intakeShooterSubsystem.intake());
        operatorXbox.rightBumper().and(superStructure.isOperatorManual().negate()).whileTrue(intakeShooterSubsystem.shoot());

        operatorXbox.start().and(superStructure.isOperatorManual().negate()).onTrue(superStructure.stopAllManipulators());

        // Operator Manual Controls
        operatorXbox.a().and(superStructure.isOperatorManual()).whileTrue(elevatorSubsystem.elevatorUp());
        operatorXbox.b().and(superStructure.isOperatorManual()).whileTrue(elevatorSubsystem.elevatorDown());

        operatorXbox.x().and(superStructure.isOperatorManual()).whileTrue(armSubsystem.armUp());
        operatorXbox.y().and(superStructure.isOperatorManual()).whileTrue(armSubsystem.armDown());

        operatorXbox.leftBumper().and(superStructure.isOperatorManual()).whileTrue(intakeShooterSubsystem.intake());
        operatorXbox.rightBumper().and(superStructure.isOperatorManual()).whileTrue(intakeShooterSubsystem.shoot());

        operatorXbox.start().and(superStructure.isOperatorManual()).whileTrue(climbSubsystem.climbUp());
        operatorXbox.back().and(superStructure.isOperatorManual()).whileTrue(climbSubsystem.climbDown());
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

        NamedCommands.registerCommand("togglePID", superStructure.toggleOperatorControls().andThen(superStructure.updateStowCommand()));

        NamedCommands.registerCommand("selectSouth", Commands.runOnce(poseSelector::selectSouth));
        NamedCommands.registerCommand("selectSoutheast", Commands.runOnce(poseSelector::selectSouthEast));
        NamedCommands.registerCommand("selectSouthwest", Commands.runOnce(poseSelector::selectSouthWest));
        NamedCommands.registerCommand("selectNorth", Commands.runOnce(poseSelector::selectNorth));
        NamedCommands.registerCommand("selectNortheast", Commands.runOnce(poseSelector::selectNorthEast));
        NamedCommands.registerCommand("selectNorthwest", Commands.runOnce(poseSelector::selectNorthWest));
        NamedCommands.registerCommand("cycleStationUp", Commands.runOnce(poseSelector::cycleStationSlotUp));
        NamedCommands.registerCommand("cycleStationDown", Commands.runOnce(poseSelector::cycleStationSlotDown));
        NamedCommands.registerCommand("selectLeft", Commands.runOnce(poseSelector::selectLeft));
        NamedCommands.registerCommand("selectRight", Commands.runOnce(poseSelector::selectRight));
        NamedCommands.registerCommand("driveToReef", Commands.defer(() -> drivebase.driveToPose(poseSelector.reefPose()), Set.of(drivebase)));
        NamedCommands.registerCommand("driveToStation", Commands.defer(() -> drivebase.driveToPose(poseSelector.stationPose()), Set.of(drivebase)));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData(autoChooser);
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
