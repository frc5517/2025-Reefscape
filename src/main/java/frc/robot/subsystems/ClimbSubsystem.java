// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    /**
     * Creates a new ClimbSubsystem.
     */
    //creates climb limit switch
    public static final DigitalInput climbBottomLimit = new DigitalInput(3);
    public static final Trigger climbBottomTrigger = new Trigger(() -> !climbBottomLimit.get());

    private final SparkMax climbMotor = new SparkMax(15, MotorType.kBrushless);

    public ClimbSubsystem() {
        climbBottomTrigger.onTrue(run(this::stopClimb));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Encoder", climbMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Climb Bottom Limit", !climbBottomLimit.get());
    }

    public Command climbUp() {
        return runEnd(() -> {
            climbMotor.set(Constants.ClimberConstants.kClimberSpeed);
        }, () -> {
            climbMotor.set(0.0);
        });
    }

    public Command climbDown() {
        return runEnd(() -> {
            climbMotor.set(-Constants.ClimberConstants.kClimberSpeed);
        }, () -> {
            climbMotor.set(0.0);
        });
    }

    public void stopClimb() {
        climbMotor.set(0.0);
    }
}
