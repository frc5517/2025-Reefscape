package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeShooterConstants;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;

public class IntakeShooterSubsystem extends SubsystemBase {

    private final ManipSparkMax intakeMotor = new ManipSparkMax(IntakeShooterConstants.kIntakeShooterMotorID);
    private final ManipShooterIntake intakeShooter = new ManipShooterIntake(intakeMotor,
            IntakeShooterConstants.intakeShooterConfig);

    private final DigitalInput coralSensor = new DigitalInput(IntakeShooterConstants.kIntakeShooterCoralSensorID);

    private Trigger coralTrigger = new Trigger(coralSensor::get);

    public IntakeShooterSubsystem() {
        intakeMotor.setMotorBrake(true);

        if (RobotBase.isSimulation()) {
            coralTrigger = new Trigger(() -> false);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Coral Trigger", coralTrigger.getAsBoolean());
        SmartDashboard.putNumber("Intake Applied Output", intakeMotor.getAppliedOutput());
    }

    public Command intake() {
        return Commands.runEnd(() -> intakeShooter.runSpeed(-Constants.IntakeShooterConstants.kIntakeSpeed),
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
        return intakeShooter.runSpeedCommand(Constants.IntakeShooterConstants.kShootSpeed);
    }

    public Command shootUntilGone() {
        return shoot()
                .until(coralTrigger.negate());
    }

    public void stopIntakeShooter() {
        intakeShooter.stopShooterCommand();
    }
}