package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import maniplib.AdvancedManipArm;
import maniplib.ManipArm;
import maniplib.ManipElevator;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;


public class SuperStructure extends SubsystemBase {

    ManipSparkMax intakeMotor = new ManipSparkMax(11, DCMotor.getNEO(1));
    ManipShooterIntake intakeShooter = new ManipShooterIntake(intakeMotor);

    ManipSparkMax armMotor = new ManipSparkMax(12, DCMotor.getNEO(1));
    AdvancedManipArm arm = new AdvancedManipArm(armMotor, Constants.ArmConstants.armConfig);

    DigitalInput throughboreInput = new DigitalInput(2);
    DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(throughboreInput);

    /**
     * Initialize the robot control {@link SuperStructure}
     */
    public SuperStructure() {
        intakeMotor.setMotorBrake(true);

        //arm.addAbsoluteEncoder(throughboreEncoder);
        arm.setupArmMech(Constants.ArmConstants.armMech);
    }

    /**
     * A command to run {@link ManipArm} at speed.
     * 
     * @param speed percent to run {@link ManipArm} at.
     * @return a command to run {@link ManipArm} at speed.
     */
    public Command runArm(double speed) {
        return run(() -> {
            arm.runArm(speed);
        });
    }

    /**
     * A command to run {@link ManipShooterIntake} at speed.
     * 
     * @param speed percent to run {@link ManipShooterIntake} at.
     * @return a command to run {@link ManipShooterIntake} at speed.
     */
    public Command runShooterIntake(double speed) {
        return run(() -> {
            intakeShooter.setSpeed(speed);
        });
    }
    
    /**
     * A command to stop all manipulator motors.
     * 
     * @return a command to stop all manipulator motors.
     */
    public Command stopAllManipulators() {
        return run(() -> {
            intakeShooter.stopShooter();
            arm.stopArm();
        });
    }

}
