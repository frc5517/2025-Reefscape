// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;

public class IntakeShooterSubsystem extends SubsystemBase {
  /** Creates a new IntakeShooterSubsystem. */
  ManipSparkMax intakeMotor = new ManipSparkMax(IntakeShooterConstants.kIntakeShooterMotorID);
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
    return runEnd(() -> {
      //intakeShooter.setSpeed(IntakeShooterConstants.kIntakeSpeed);
      intakeMotor.set(-IntakeShooterConstants.kIntakeSpeed);
    }, () -> {
      //intakeShooter.stopShooter();
      intakeMotor.stopMotor();
    });
  }
  
  public Command shoot() {
    return runEnd(() -> {
      //intakeShooter.setSpeed(IntakeShooterConstants.kShootSpeed);
      intakeMotor.set(IntakeShooterConstants.kShootSpeed);
    }, () -> {
      //intakeShooter.stopShooter();
      intakeMotor.stopMotor();
    });
  }

  public Command stopIntakeShooter() {
    return intakeShooter.stopShooterCommand();
  }

}
