// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    /**
     * Creates a new ClimbSubsystem.
     */
    private SparkMax climbMotor = new SparkMax(15, MotorType.kBrushless);

    public ClimbSubsystem() {
        SmartDashboard.putNumber("Climb Encoder", climbMotor.getEncoder().getPosition());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command climbUp(double speed) {
        return runEnd(() -> {
            climbMotor.set(speed);
        }, () -> {
            climbMotor.set(0.0);
        });
    }

    public Command climbDown(double speed) {
        return runEnd(() -> {
            climbMotor.set(-speed);
        }, () -> {
            climbMotor.set(0.0);
        });
    }
}
