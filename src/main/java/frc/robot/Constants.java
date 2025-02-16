// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import maniplib.utils.ManipArmConstants;
import maniplib.utils.ManipElevatorConstants;
import swervelib.math.Matter;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final Mechanism2d sideRobotView = new Mechanism2d(ArmConstants.armConfig.kArmLength * 2,
            ElevatorConstants.elevatorConfig.kMaxHeight.in(
                    Meters) +
                    ArmConstants.armConfig.kArmLength);
    public static final MechanismRoot2d kElevatorCarriage;
    public static final MechanismLigament2d kArmMech;
    public static final MechanismLigament2d kElevatorTower;

    static {
        kElevatorCarriage = Constants.sideRobotView.getRoot("ElevatorCarriage",
                ArmConstants.armConfig.kArmLength,
                ElevatorConstants.elevatorConfig.kStartingHeightSim.in(
                        Meters));
        kArmMech = kElevatorCarriage.append(
                new MechanismLigament2d(
                        "Arm",
                        ArmConstants.armConfig.kArmLength,
                        ArmConstants.armConfig.kArmStartingAngle.in(Degrees),
                        6,
                        new Color8Bit(Color.kYellow)));
        kElevatorTower = kElevatorCarriage.append(new MechanismLigament2d(
                "Elevator",
                ElevatorConstants.elevatorConfig.kStartingHeightSim.in(Meters),
                -90,
                6,
                new Color8Bit(Color.kRed)));
    }

    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {
        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static final class ArmConstants {
        public static final int kArmABSID = 1; // DIO

        public static final double kArmSpeed = 0.3;
        public static final double kL1Setpoint = -20;
        public static final double kL2Setpoint = -10;
        public static final double kL3Setpoint = 0;
        public static final double kL4Setpoint = 10;

        public static final double kStowSetpoint = 0;

        public static final ManipArmConstants armConfig =
                new ManipArmConstants(
                        DCMotor.getNEO(1),
                        2.026,
                        0,
                        0.092372,
                        0.012394,
                        0.019009,
                        0.12269,
                        0.0042372,
                        140,
                        3,
                        17,
                        0,
                        -90,
                        90,
                        true,
                        .5,
                        171.35,
                        0.01,
                        40,
                        20,
                        40,
                        true
                );
    }

    public static final class ElevatorConstants {
        public static final int kBottomLimitPort = 5;
        public static final double kElevatorSpeed = 1;
        public static final double kL1Setpoint = 10;
        public static final double kL2Setpoint = 20;
        public static final double kL3Setpoint = 30;
        public static final double kL4Setpoint = 40;

        public static final double kStowSetpoint = 0;

        public static final ManipElevatorConstants elevatorConfig =
                new ManipElevatorConstants(
                        DCMotor.getNEO(2),
                        28.475,
                        0,
                        1.2353,
                        0.08,
                        23.47,
                        0.45118,
                        0.044757,
                        50,
                        2,
                        6,
                        0,
                        72,
                        0,
                        true,
                        0.2,
                        40,
                        15,
                        30,
                        0,
                        true
                );
    }

    public static final class IntakeShooterConstants {
        public static final int kIntakeShooterCoralSensorID = 0; // DIO

        public static final double kIntakeSpeed = .3;
        public static final double kShootSpeed = .5;
    }

    public static final class AddressableConstants {
        public static final int kLedPort = 0; // PWM
        public static final int kLedLength = 200;

        public static final Distance kLedSpacing = Meter.of(1 / 60);
    }

}
