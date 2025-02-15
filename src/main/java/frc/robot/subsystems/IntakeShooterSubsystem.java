package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;

public class IntakeShooterSubsystem extends SubsystemBase {

    ManipSparkMax intakeMotor = new ManipSparkMax(11);
    ManipShooterIntake intakeShooter = new ManipShooterIntake(intakeMotor);

    public IntakeShooterSubsystem() {
        intakeMotor.setMotorBrake(true);
    }

    public Command intake() {
        return intakeShooter.setSpeed(-Constants.IntakeShooterConstants.kIntakeSpeed);
    }

    public Command shoot() {
        return intakeShooter.setSpeed(Constants.IntakeShooterConstants.kShootSpeed);
    }

    public Command stopIntakeShooter() {
        return intakeShooter.stopShooterCommand();
    }
}
