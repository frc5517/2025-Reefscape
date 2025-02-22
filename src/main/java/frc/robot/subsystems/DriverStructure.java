package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.util.Arrays;

public class DriverStructure extends SubsystemBase {

    SwerveSubsystem swerve;

    private ReefPose reefPose = ReefPose.REEF_POSE_1;
    private final ReefPose[] poses = ReefPose.values();

    public DriverStructure(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Reef Pose", reefPose.ordinal() + 1);
    }

    public enum ReefPose {
        REEF_POSE_1,
        REEF_POSE_2,
        REEF_POSE_3,
        REEF_POSE_4,
        REEF_POSE_5,
        REEF_POSE_6,
        REEF_POSE_7,
        REEF_POSE_8,
        REEF_POSE_9,
        REEF_POSE_10,
        REEF_POSE_11,
        REEF_POSE_12;
    }

    public void cycleReefPoseUp() {
        int ordinalPoseUp = (reefPose.ordinal() + 1) % poses.length;
        reefPose = poses[ordinalPoseUp];
    }

    public void cycleReefPoseDown() {
        int ordinalPoseDown = (reefPose.ordinal() - 1) % poses.length;
        if (ordinalPoseDown == -1) {
            ordinalPoseDown = 11;
        }
        reefPose = poses[ordinalPoseDown];
    }

    public void driveToCoralStation() {

    }

    public Pose2d reefPose() {
        Pose2d pose;
        if (reefPose == ReefPose.REEF_POSE_1) {
            pose = Constants.DrivebaseConstants.REEF_POSE_1;
        } else if (reefPose == ReefPose.REEF_POSE_2) {
            pose = Constants.DrivebaseConstants.REEF_POSE_2;
        } else if (reefPose == ReefPose.REEF_POSE_3) {
            pose = Constants.DrivebaseConstants.REEF_POSE_3;
        } else if (reefPose == ReefPose.REEF_POSE_4) {
            pose = Constants.DrivebaseConstants.REEF_POSE_4;
        } else if (reefPose == ReefPose.REEF_POSE_5) {
            pose = Constants.DrivebaseConstants.REEF_POSE_5;
        } else if (reefPose == ReefPose.REEF_POSE_6) {
            pose = Constants.DrivebaseConstants.REEF_POSE_6;
        } else if (reefPose == ReefPose.REEF_POSE_7) {
            pose = Constants.DrivebaseConstants.REEF_POSE_7;
        } else if (reefPose == ReefPose.REEF_POSE_8) {
            pose = Constants.DrivebaseConstants.REEF_POSE_8;
        } else if (reefPose == ReefPose.REEF_POSE_9) {
            pose = Constants.DrivebaseConstants.REEF_POSE_9;
        } else if (reefPose == ReefPose.REEF_POSE_10) {
            pose = Constants.DrivebaseConstants.REEF_POSE_10;
        } else if (reefPose == ReefPose.REEF_POSE_11) {
            pose = Constants.DrivebaseConstants.REEF_POSE_11;
        } else if (reefPose == ReefPose.REEF_POSE_12) {
            pose = Constants.DrivebaseConstants.REEF_POSE_12;
        } else {
            pose = swerve.getPose();
        }
        return pose;
    }

}
