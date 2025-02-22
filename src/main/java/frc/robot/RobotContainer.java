// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

public class RobotContainer {

    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final DriverStructure driverStructure = new DriverStructure(drivebase);

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
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
            .deadband(Constants.OperatorConstants.DEADBAND)
            .cubeTranslationControllerAxis(true)
            .cubeRotationControllerAxis(true)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    SwerveInputStream robotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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

        drivebase.setDefaultCommand(driveRobotOriented);
        driverXbox.start().toggleOnTrue(driveFieldOriented);

        //ledSubsystem.setPattern(LEDPattern.solid(ledSubsystem.editColor(Color.kMagenta))  );

        armSubsystem.setAutoStow();
        elevatorSubsystem.setAutoStow();
        operatorXbox.leftTrigger().and(operatorXbox.rightTrigger()).onTrue(superStructure.toggleOperatorControls().andThen(superStructure.updateStowCommand()));

        driverXbox.leftBumper().onTrue(Commands.runOnce(driverStructure::cycleReefPoseDown));
        driverXbox.rightBumper().onTrue(Commands.runOnce(driverStructure::cycleReefPoseUp));

        // Operator Auto Controls
        operatorXbox.a().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToL1());
        operatorXbox.b().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToL2());
        operatorXbox.x().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToL3());
        operatorXbox.y().and(superStructure.isOperatorManual().negate()).whileTrue(superStructure.structureToL4());

        operatorXbox.pov(0).and(superStructure.isOperatorManual().negate()).whileTrue(elevatorSubsystem.elevatorUp());
        operatorXbox.pov(180).and(superStructure.isOperatorManual().negate()).whileTrue(elevatorSubsystem.elevatorDown());

        operatorXbox.pov(90).and(superStructure.isOperatorManual().negate()).whileTrue(armSubsystem.armDown());
        operatorXbox.pov(270).and(superStructure.isOperatorManual().negate()).whileTrue(armSubsystem.armUp());

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

    }

    public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand("New Auto");
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
