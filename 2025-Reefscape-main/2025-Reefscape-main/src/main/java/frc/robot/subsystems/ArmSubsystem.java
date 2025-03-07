// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import maniplib.ManipArm;
import maniplib.motors.ManipSparkMax;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  ManipSparkMax armMotor = new ManipSparkMax(ArmConstants.kArmMotorID);
  ManipArm arm = new ManipArm(armMotor, ArmConstants.armConfig);
  
  public ArmSubsystem() {
    // Not yet plugged in
    //arm.addAbsoluteEncoder(armABS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command armToL1() {
    return run(() -> {
      arm.reachSetpoint(-75);
    });
  }

  public Command armToL2() {
    return run(() -> {
      arm.reachSetpoint(75);
    });
  }

  public Command armUp() {
    return runEnd(() -> {
      arm.runArm(ArmConstants.kArmSpeed);
    }, () -> {
      arm.runArm(0);;
    });
  }

  public Command armDown() {
    return runEnd(() -> {
      arm.runArm(-ArmConstants.kArmSpeed);
    }, () -> {
      arm.runArm(0);
    });
  }

  public Command stopArm() {
    return run(() -> {
      arm.stopArm();
    });
  }

}
