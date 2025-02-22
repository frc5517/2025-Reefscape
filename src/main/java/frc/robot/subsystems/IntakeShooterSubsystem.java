package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeShooterConstants;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;

public class IntakeShooterSubsystem extends SubsystemBase {

    ManipSparkMax intakeMotor = new ManipSparkMax(11);
    ManipShooterIntake intakeShooter = new ManipShooterIntake(intakeMotor);

    DigitalInput coralSensor = new DigitalInput(IntakeShooterConstants.kIntakeShooterCoralSensorID);

    public IntakeShooterSubsystem() {
        intakeMotor.setMotorBrake(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Coral Sensor", coralSensor.get());
    }

    public Command intake() {
        return intakeShooter.setSpeed(-Constants.IntakeShooterConstants.kIntakeSpeed);
    }

    public Command shoot() {
        return intakeShooter.setSpeed(Constants.IntakeShooterConstants.kShootSpeed);
    }

    public void stopIntakeShooter() {
        intakeShooter.stopShooterCommand();
    }
}
