// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
                        -75,
                        75,
                        false,
                        .5,
                        351.6,
                        0.01,
                        40,
                        20,
                        40,
                        true
                );
    }

    public static final class ElevatorConstants {
        public static final int kBottomLimitPort = 2;
        public static final double kElevatorSpeed = .5;
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
                        0.25,
                        15,
                        2,
                        10,
                        0,
                        50,
                        0,
                        true,
                        0.6,
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

        public static final Distance kLedSpacing = Meter.of(1.0 / 60);
    }

    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final ProfiledPIDController driveToPosePID =
                new ProfiledPIDController(0, 0, 0,
                        new TrapezoidProfile.Constraints(10, 10));

        public static final ProfiledPIDController driveToPoseOmegePID =
                new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(10, 10));

        public static final Transform2d kBranchOffsetLeft = new Transform2d(
                Units.inchesToMeters(30), // Offset away from reef.
                Units.inchesToMeters(13 / 2.0), // Offset to left branch.
                Rotation2d.kZero);
        public static final Transform2d kBranchOffsetRight = new Transform2d(
                Units.inchesToMeters(30), // Offset away from reef.
                Units.inchesToMeters(-13 / 2.0), // Offset to right branch.
                Rotation2d.kZero);

        public static final Pose2d LEFT_STATION_POSE_1 = new Pose2d(
                new Translation2d(0.75, 6.6),
                new Rotation2d(Math.toRadians(125))
        );
        public static final Pose2d LEFT_STATION_POSE_2 = new Pose2d(
                new Translation2d(1.15, 6.9),
                new Rotation2d(Math.toRadians(125))
        );
        public static final Pose2d LEFT_STATION_POSE_3 = new Pose2d(
                new Translation2d(1.45, 6.2),
                new Rotation2d(Math.toRadians(125))
        );
        public static final Pose2d LEFT_STATION_POSE_4 = new Pose2d(
                new Translation2d(1.7, 7.3),
                new Rotation2d(Math.toRadians(125))
        );
        public static final Pose2d RIGHT_STATION_POSE_1 = new Pose2d(
                new Translation2d(5, 5),
                new Rotation2d(Math.toRadians(90))
        );
        public static final Pose2d RIGHT_STATION_POSE_2 = new Pose2d(
                new Translation2d(5, 5),
                new Rotation2d(Math.toRadians(90))
        );
        public static final Pose2d RIGHT_STATION_POSE_3 = new Pose2d(
                new Translation2d(5, 5),
                new Rotation2d(Math.toRadians(90))
        );
        public static final Pose2d RIGHT_STATION_POSE_4 = new Pose2d(
                new Translation2d(5, 5),
                new Rotation2d(Math.toRadians(90))
        );

        public static final Pose2d SOUTH_FACE_POSE = new Pose2d(
                Units.inchesToMeters(144.003),
                Units.inchesToMeters(158.500),
                Rotation2d.fromDegrees(180));
        public static final Pose2d SOUTHWEST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(160.373),
                Units.inchesToMeters(186.857),
                Rotation2d.fromDegrees(120));
        public static final Pose2d NORTHWEST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(193.116),
                Units.inchesToMeters(186.858),
                Rotation2d.fromDegrees(60));
        public static final Pose2d NORTH_FACE_POSE = new Pose2d(
                Units.inchesToMeters(209.489),
                Units.inchesToMeters(158.502),
                Rotation2d.fromDegrees(0));
        public static final Pose2d NORTHEAST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(193.118),
                Units.inchesToMeters(130.145),
                Rotation2d.fromDegrees(-60));
        public static final Pose2d SOUTHEAST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(160.375),
                Units.inchesToMeters(130.144),
                Rotation2d.fromDegrees(-120));

        public static final Pose2d REEF_NORTH_LEFT_POSE =
                NORTH_FACE_POSE.plus(kBranchOffsetLeft);
        public static final Pose2d REEF_NORTH_RIGHT_POSE =
                NORTH_FACE_POSE.plus(kBranchOffsetRight);
        public static final Pose2d REEF_NORTHEAST_LEFT_POSE =
                NORTHEAST_FACE_POSE.plus(kBranchOffsetLeft);
        public static final Pose2d REEF_NORTHEAST_RIGHT_POSE =
                NORTHEAST_FACE_POSE.plus(kBranchOffsetRight);
        public static final Pose2d REEF_NORTHWEST_LEFT_POSE =
                NORTHWEST_FACE_POSE.plus(kBranchOffsetLeft);
        public static final Pose2d REEF_NORTHWEST_RIGHT_POSE =
                NORTHWEST_FACE_POSE.plus(kBranchOffsetRight);
        public static final Pose2d REEF_SOUTH_LEFT_POSE =
                SOUTH_FACE_POSE.plus(kBranchOffsetLeft);
        public static final Pose2d REEF_SOUTH_RIGHT_POSE =
                SOUTH_FACE_POSE.plus(kBranchOffsetRight);
        public static final Pose2d REEF_SOUTHEAST_LEFT_POSE =
                SOUTHEAST_FACE_POSE.plus(kBranchOffsetLeft);
        public static final Pose2d REEF_SOUTHEAST_RIGHT_POSE =
                SOUTHEAST_FACE_POSE.plus(kBranchOffsetRight);
        public static final Pose2d REEF_SOUTHWEST_LEFT_POSE =
                SOUTHWEST_FACE_POSE.plus(kBranchOffsetLeft);
        public static final Pose2d REEF_SOUTHWEST_RIGHT_POSE =
                SOUTHWEST_FACE_POSE.plus(kBranchOffsetRight);
    }

}
