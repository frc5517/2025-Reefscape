// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import maniplib.ManipElevator;
import maniplib.motors.ManipSparkMax;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  ManipSparkMax rightElevatorMotor = new ManipSparkMax(ElevatorConstants.kRightElevatorMotorID);
  ManipSparkMax leftElevatorMotor = new ManipSparkMax(ElevatorConstants.kLeftElevatorMotorID);
  ManipElevator elevator = new ManipElevator(rightElevatorMotor);

  DigitalInput elevatorABSInput = new DigitalInput(ElevatorConstants.kElevatorABSID);
  DutyCycleEncoder elevatorABS = new DutyCycleEncoder(elevatorABSInput);

  public ElevatorSubsystem() {
    elevator.addFollower(leftElevatorMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Value not yet seeded..
    SmartDashboard.putNumber("Elevator ABS", elevatorABS.get());
  }

  public Command elevatorUp() {
    return runEnd(() -> {
      elevator.setSpeed(ElevatorConstants.kElevatorSpeed);
    }, () -> {
      elevator.stopElevator();
    });
  }

  public Command elevatorDown() {
    return runEnd(() -> {
      elevator.setSpeed(-ElevatorConstants.kElevatorSpeed);
    }, () -> {
      elevator.stopElevator();
    });
  }

  public Command stopElevator() {
    return elevator.stopElevatorCommand();
  }

}
