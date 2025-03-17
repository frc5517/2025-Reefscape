package maniplib;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import maniplib.motors.ManipMotor;
import maniplib.utils.ManipIntakeShooterConstants;
import maniplib.utils.PIDControlType;

public class ManipShooterIntake extends SubsystemBase {

    private final ManipMotor motor;
    private final ManipIntakeShooterConstants constants;

    private final FlywheelSim flywheelSim;

    /**
     * Initialize the {@link ManipShooterIntake} to be used.
     *
     * @param motor motor to set as the lead motor for this {@link ManipShooterIntake}
     */
    public ManipShooterIntake(ManipMotor motor, ManipIntakeShooterConstants constants) {
        this.motor = motor;
        this.constants = constants;

        motor.setGearbox(
                constants.gearbox);

        motor.setPIDControlType(PIDControlType.ControlType.VELOCITY);

        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        constants.gearbox,
                        constants.MOI,
                        constants.MOI),
                constants.gearbox,
                0.02 / 4096.0);
    }

    @Override
    public void periodic() {
        if (Telemetry.manipVerbosity.ordinal() <= Telemetry.ManipTelemetry.LOW.ordinal()) {
        }
        if (Telemetry.manipVerbosity.ordinal() <= Telemetry.ManipTelemetry.HIGH.ordinal()) {
            SmartDashboard.putNumber("Intake Shooter Applied Output", motor.getAppliedOutput());
        }
    }

    @Override
    public void simulationPeriodic() {
        // Set simulated input
        flywheelSim.setInput(motor.getAppliedOutput() * RoboRioSim.getVInVoltage());
        // Then update it
        flywheelSim.update(0.02);

        // Then iterate rev devices from sim values
        motor.iterateRevSim(
                flywheelSim.getAngularVelocityRPM(),
                RoboRioSim.getVInVoltage(),
                0.02);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        flywheelSim.getCurrentDrawAmps())
        );
    }

    /**
     * Sets the {@link ManipMotor} to follow another {@link ManipMotor}.
     *
     * @param followerMotor {@link ManipMotor} to follow the lead motor.
     * @param isInverted    whether to invert the follower or not.
     */
    public void addFollower(ManipMotor followerMotor, boolean isInverted) {
        followerMotor.setAsFollower(motor, isInverted);
    }

    /**
     * Set the motor voltage.
     *
     * @param voltage to run the motor at.
     */
    public void runVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * Set the motor voltage as a command.
     *
     * @param voltage to run the motor at.
     */
    public Command runVoltageCommand(double voltage) {
        return runEnd(
                () -> {
                    runVoltage(voltage);
                }, this::stopShooter
        );
    }

    /**
     * Set the percentage output as a command.
     *
     * @param speed percent out for the motor controller.
     */
    public void runSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Set the percentage output.
     *
     * @param speed percent out for the motor controller as a command.
     */
    public Command runSpeedCommand(double speed) {
        return runEnd(
                () -> {
                    runSpeed(speed);
                },
                motor::stopMotor);
    }

    /**
     * Set the closed loop PID controller reference point.
     *
     * @param setpoint Setpoint, value type changes with ControlType.
     */
    public Command setReference(double setpoint) {
        return runEnd(
                () -> {
                    motor.setReference(setpoint);
                },
                motor::stopMotor
        );
    }

    /**
     * Stop the {@link ManipShooterIntake}
     */
    public void stopShooter() {
        motor.stopMotor();
    }

    /**
     * A command that stops the {@link ManipShooterIntake}
     *
     * @return a command that stops the {@link ManipShooterIntake}
     */
    public Command stopShooterCommand() {
        return motor.stopMotorCommand();
    }

}
