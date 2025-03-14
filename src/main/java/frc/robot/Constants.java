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
    public static class OperatorConstants {
        public static final double DEADBAND = 0.1;
    }

    public static final class ArmConstants {
        public static final int kArmABSID = 1; // DIO
        public static final double kArmSpeed = 0.3;
        public static final double kL1Setpoint = -20;
        public static final double kL2Setpoint = -20;
        public static final double kL3Setpoint = -20;
        public static final double kL4Setpoint = -35;
        public static final double kStationSetpoint = 32;
        public static final double kDealgaeHigh = -15;
        public static final double kDealgaeLow = -15;
        public static final double kStowSetpoint = 70;
        public static final double kAutoScoreToleranceDegrees = .5;
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
                        75,
                        false,
                        .5,
                        -173,
                        0.01,
                        40,
                        20,
                        40,
                        true
                );
    }

    public static final class ElevatorConstants {
        public static final int kBottomLimitPort = 2;
        public static final double kElevatorSpeed = .4;
        public static final double kL1Setpoint = 10;
        public static final double kL2Setpoint = 6;
        public static final double kL3Setpoint = 25;
        public static final double kL4Setpoint = 60.5;
        public static final double kStationSetpoint = 1;
        public static final double kDealgaeHigh = 37.5;
        public static final double kDealgaeLow = 20;
        public static final double kStowSetpoint = 0;
        public static final double kSlowElevatorHeightInches = 30;
        public static final double kAutoScoreToleranceInches = .5;
        public static final ManipElevatorConstants elevatorConfig =
                new ManipElevatorConstants(
                        DCMotor.getNEO(2),
                        38,
                        1,
                        0.01,
                        0.08,
                        3.07,
                        0.01,
                        0.25,
                        15,
                        2,
                        10,
                        0,
                        63.5,
                        0,
                        true,
                        0.6,
                        40,
                        15,
                        20,
                        0,
                        true
                );
    }

    public static final class IntakeShooterConstants {
        public static final int kIntakeShooterCoralSensorID = 0; // DIO
        public static final double kIntakeSpeed = .3;
        public static final double kShootSpeed = .5;
        public static final double kIntakekG = .2;
    }

    public static final class ClimberConstants {
        public static final double kClimberSpeed = 1;
        public static final int kClimbLimitPort = 3;
    }

    public static final class AddressableConstants {
        public static final int kLedPort = 0; // PWM
        public static final int kLedLength = 200;

        public static final Distance kLedSpacing = Meter.of(1.0 / 60);
    }

    public static final class DrivebaseConstants {
        public static final double MAX_SPEED = Units.feetToMeters(14.5);
        public static final double kScaleSpeedMax = 0.8;
        public static final double kScaleSpeedMin = 0.3;
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        // Drive to pose constants
        public static final double kDistanceUntilPID = Units.inchesToMeters(12);
        public static final double kAtReefTolerance = Units.inchesToMeters(.2);
        public static final double kAtStationTolerance = Units.inchesToMeters(.2);


        // Drive to pose from SwerveInputStream constants.
        public static final ProfiledPIDController driveToPoseXPID =
                new ProfiledPIDController(
                        150,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                360,
                                720));

        public static final ProfiledPIDController driveToPoseOmegaPID =
                new ProfiledPIDController(
                        50,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                360,
                                720));

        //
        // Pose offsets below. Proceed with caution!
        //

        public static final Transform2d BRANCH_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(-30), // Offset away from reef.
                Units.inchesToMeters(13 / 2.0), // Offset to left branch.
                Rotation2d.kZero);
        public static final Transform2d BRANCH_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(-30), // Offset away from reef.
                Units.inchesToMeters(-13 / 2.0), // Offset to right branch.
                Rotation2d.kZero);

        public static final Transform2d STATION_OFFSET = new Transform2d(
                Units.inchesToMeters(-20),
                Units.inchesToMeters(0),
                Rotation2d.kZero
        );

        public static final Transform2d SLOT_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(10), // Added several times to achieve all 5 poses.
                Rotation2d.kZero
        );
        public static final Transform2d SLOT_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(-10), // Added several times to achieve all 5 poses.
                Rotation2d.kZero
        );

        public static final Transform2d CAGE_OFFSET = new Transform2d(
                Units.inchesToMeters(-30),
                Units.inchesToMeters(0),
                Rotation2d.k180deg
        );

        //
        // Constant poses below, proceed with EXTREME CAUTION!!
        //

        public static final Pose2d LEFT_STATION_CENTER_POSE =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(125.989));
        public static final Pose2d RIGHT_STATION_CENTER_POSE =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(234.011));

        public static final Pose2d kLeftCage =
                new Pose2d(
                        Units.inchesToMeters(345.428),
                        Units.inchesToMeters(286.779),
                        Rotation2d.kZero);
        public static final Pose2d kMiddleCage =
                new Pose2d(
                        Units.inchesToMeters(345.428),
                        Units.inchesToMeters(242.855),
                        Rotation2d.kZero);
        public static final Pose2d kRightCage =
                new Pose2d(
                        Units.inchesToMeters(345.428),
                        Units.inchesToMeters(199.947),
                        Rotation2d.kZero);

        public static final Pose2d LEFT_STATION_POSE_1 = LEFT_STATION_CENTER_POSE
                .plus(STATION_OFFSET)
                .plus(SLOT_OFFSET_LEFT);
        public static final Pose2d LEFT_STATION_POSE_2 = LEFT_STATION_CENTER_POSE
                .plus(STATION_OFFSET);
        public static final Pose2d LEFT_STATION_POSE_3 = LEFT_STATION_CENTER_POSE
                .plus(STATION_OFFSET)
                .plus(SLOT_OFFSET_RIGHT);

        public static final Pose2d RIGHT_STATION_POSE_1 = RIGHT_STATION_CENTER_POSE
                .plus(STATION_OFFSET)
                .plus(SLOT_OFFSET_RIGHT);
        public static final Pose2d RIGHT_STATION_POSE_2 = RIGHT_STATION_CENTER_POSE
                .plus(STATION_OFFSET);
        public static final Pose2d RIGHT_STATION_POSE_3 = RIGHT_STATION_CENTER_POSE
                .plus(STATION_OFFSET)
                .plus(SLOT_OFFSET_LEFT);

        public static final Pose2d LEFT_CAGE_POSE = kLeftCage
                .plus(CAGE_OFFSET);
        public static final Pose2d MIDDLE_CAGE_POSE = kMiddleCage
                .plus(CAGE_OFFSET);
        public static final Pose2d RIGHT_CAGE_POSE = kRightCage
                .plus(CAGE_OFFSET);

        public static final Pose2d SOUTH_FACE_POSE = new Pose2d(
                Units.inchesToMeters(144.003),
                Units.inchesToMeters(158.500),
                Rotation2d.fromDegrees(0));
        public static final Pose2d SOUTHWEST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(160.373),
                Units.inchesToMeters(186.857),
                Rotation2d.fromDegrees(300));
        public static final Pose2d NORTHWEST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(193.116),
                Units.inchesToMeters(186.858),
                Rotation2d.fromDegrees(240));
        public static final Pose2d NORTH_FACE_POSE = new Pose2d(
                Units.inchesToMeters(209.489),
                Units.inchesToMeters(158.502),
                Rotation2d.fromDegrees(180));
        public static final Pose2d NORTHEAST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(193.118),
                Units.inchesToMeters(130.145),
                Rotation2d.fromDegrees(120));
        public static final Pose2d SOUTHEAST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(160.375),
                Units.inchesToMeters(130.144),
                Rotation2d.fromDegrees(60));

        public static final Pose2d REEF_NORTH_LEFT_POSE =
                NORTH_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTH_RIGHT_POSE =
                NORTH_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_NORTHEAST_LEFT_POSE =
                NORTHEAST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTHEAST_RIGHT_POSE =
                NORTHEAST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_NORTHWEST_LEFT_POSE =
                NORTHWEST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTHWEST_RIGHT_POSE =
                NORTHWEST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTH_LEFT_POSE =
                SOUTH_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTH_RIGHT_POSE =
                SOUTH_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTHEAST_LEFT_POSE =
                SOUTHEAST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTHEAST_RIGHT_POSE =
                SOUTHEAST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTHWEST_LEFT_POSE =
                SOUTHWEST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTHWEST_RIGHT_POSE =
                SOUTHWEST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
    }

    public static final class VisionConstants {
        // Coordinate system, makes x, y, and z easy.
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        // Bottom Camera Constants
        public static final Rotation3d kBottomCamRotation = new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(0),
                Units.degreesToRadians(0));
        public static final Translation3d kBottomCamPosition = new Translation3d(
                Units.inchesToMeters(2),
                Units.inchesToMeters(0),
                Units.inchesToMeters(8));

        // Top Camera Constants
        public static final Rotation3d kTopCamRotation = new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-40),
                Units.degreesToRadians(0));
        public static final Translation3d kTopCamPosition = new Translation3d(
                Units.inchesToMeters(2),
                Units.inchesToMeters(0),
                Units.inchesToMeters(35));
    }

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

}
