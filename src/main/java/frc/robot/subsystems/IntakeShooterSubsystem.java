package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeShooterConstants;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;

public class IntakeShooterSubsystem extends SubsystemBase {

    private final ManipSparkMax intakeMotor = new ManipSparkMax(11);
    private final ManipShooterIntake intakeShooter = new ManipShooterIntake(intakeMotor);

    private final DigitalInput coralSensor = new DigitalInput(IntakeShooterConstants.kIntakeShooterCoralSensorID);

    private final Trigger coralTrigger = new Trigger(coralSensor::get);

    public IntakeShooterSubsystem() {
        intakeMotor.setMotorBrake(true);
        coralTrigger.onTrue(intakeShooter.stopShooterCommand());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Coral Sensor", coralSensor.get());
    }

    public Command intake() {
        return intakeShooter.runSpeedCommand(-Constants.IntakeShooterConstants.kIntakeSpeed)
                .finallyDo(() -> intakeShooter.runVoltageCommand(-0.2));
    }

    public Command shoot() {
        return intakeShooter.runSpeedCommand(Constants.IntakeShooterConstants.kShootSpeed);
    }

    public void stopIntakeShooter() {
        intakeShooter.stopShooterCommand();
    }
}