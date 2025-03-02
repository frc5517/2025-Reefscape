package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.units.Units.Degrees;

public class PoseSelector extends SubsystemBase {

    private final ReefPose[] poses = ReefPose.values();
    private final ReefSide[] sides = ReefSide.values();
    private final LeftOrRight[] leftOrRights = LeftOrRight.values();
    private final StationSlot[] stationSlots = StationSlot.values();
    private final MutAngle selectedPose = Degrees.mutable(0);
    SwerveSubsystem swerve;
    private ReefPose reefPose = ReefPose.NORTH_LEFT;
    private ReefSide reefSide = ReefSide.NORTH;
    private LeftOrRight leftOrRight = LeftOrRight._LEFT;
    private StationSlot stationSlot = StationSlot.POSE_1;
    private StationPose stationPose = StationPose.POSE_1_LEFT;

    public PoseSelector(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Reef Pose", reefPose.toString());
        SmartDashboard.putString("Reef Side", reefSide.toString());
        SmartDashboard.putString("Reef Branch", leftOrRight.toString());
        SmartDashboard.putNumber("Reef Selected Pose", selectedPose.in(Degrees));

        SmartDashboard.putString("Station Slot", stationSlot.toString());
        SmartDashboard.putString("Station Pose", stationPose.toString());

        SmartDashboard.putString("Reef Pose2d", reefPose().toString());
    }

    /**
     * Cycles poses right.
     */
    public void cycleReefPoseRight() {
        int ordinalPoseUp = (reefPose.ordinal() + 1) % poses.length;
        reefPose = poses[ordinalPoseUp];
        updatePoseData(false);
    }

    /**
     * Cycles poses right.
     */
    public void cycleReefPoseLeft() {
        int ordinalPoseDown = (reefPose.ordinal() - 1) % poses.length;
        if (ordinalPoseDown == -1) {
            ordinalPoseDown = poses.length - 1;
        }
        reefPose = poses[ordinalPoseDown];
        updatePoseData(false);
    }

    /**
     * Cycles where to go to on the coral station.
     */
    public void cycleStationSlotUp() {
        int ordinalSlotUp = (stationSlot.ordinal() + 1) % stationSlots.length;
        stationSlot = stationSlots[ordinalSlotUp];
        updateStationPose();
    }

    /**
     * Cycles where to go to on the coral station.
     */
    public void cycleStationSlotDown() {
        int ordinalSlotDown = (stationSlot.ordinal() - 1) % stationSlots.length;
        if (ordinalSlotDown == -1) {
            ordinalSlotDown = stationSlots.length - 1;
        }
        stationSlot = stationSlots[ordinalSlotDown];
        updateStationPose();
    }

    /**
     * Selects north reef side.
     */
    public void selectNorth() {
        reefSide = ReefSide.NORTH;
        updatePoseData(true);
    }

    /**
     * Selects northeast reef side.
     */
    public void selectNorthEast() {
        reefSide = ReefSide.NORTHEAST;
        updatePoseData(true);
    }

    /**
     * Selects northwest reef side.
     */
    public void selectNorthWest() {
        reefSide = ReefSide.NORTHWEST;
        updatePoseData(true);
    }

    /**
     * Selects south reef side.
     */
    public void selectSouth() {
        reefSide = ReefSide.SOUTH;
        updatePoseData(true);
    }

    /**
     * Selects southeast reef side.
     */
    public void selectSouthEast() {
        reefSide = ReefSide.SOUTHEAST;
        updatePoseData(true);
    }

    /**
     * Selects southwest reef side.
     */
    public void selectSouthWest() {
        reefSide = ReefSide.SOUTHWEST;
        updatePoseData(true);
    }

    /**
     * Selects left branch of reef side and left coral station.
     */
    public void selectLeft() {
        leftOrRight = LeftOrRight._LEFT;
        updatePoseData(true);
        updateStationPose();
    }

    /**
     * Selects right branch of reef side and right coral station.
     */
    public void selectRight() {
        leftOrRight = LeftOrRight._RIGHT;
        updatePoseData(true);
        updateStationPose();
    }

    /**
     * Updates the selectedPose angle and current selected pose if fed separate side and branch data.
     *
     * @param together whether to update from branch and side data or directly from current selected pose.
     */
    public void updatePoseData(boolean together) {
        if (together) {
            reefPose = ReefPose.valueOf(reefSide.toString() + leftOrRight.toString());
        } else {
            // Determines which side pose is on.
            reefSide = sides[reefPose.ordinal() / 2];
            // Determines if selected pose is left or right.
            leftOrRight = leftOrRights[reefPose.ordinal() % 2 == 0 ? 0 : 1];
        }

        selectedPose.mut_replace(((reefSide.ordinal() * 360) / 6.0) +
                (leftOrRight.ordinal() == 0 ? -10 : 10), Degrees);
    }

    public void updateStationPose() {
        stationPose = StationPose.valueOf(stationSlot.toString() + leftOrRight.toString());
    }

    /**
     * @return selected reef {@link Pose2d}.
     */
    public Pose2d reefPose() {
        return switch (reefPose) {
            case NORTH_LEFT -> Constants.DrivebaseConstants.REEF_NORTH_LEFT_POSE;
            case NORTH_RIGHT -> Constants.DrivebaseConstants.REEF_NORTH_RIGHT_POSE;
            case NORTHEAST_LEFT -> Constants.DrivebaseConstants.REEF_NORTHEAST_LEFT_POSE;
            case NORTHEAST_RIGHT -> Constants.DrivebaseConstants.REEF_NORTHEAST_RIGHT_POSE;
            case NORTHWEST_LEFT -> Constants.DrivebaseConstants.REEF_NORTHWEST_LEFT_POSE;
            case NORTHWEST_RIGHT -> Constants.DrivebaseConstants.REEF_NORTHWEST_RIGHT_POSE;
            case SOUTH_LEFT -> Constants.DrivebaseConstants.REEF_SOUTH_LEFT_POSE;
            case SOUTH_RIGHT -> Constants.DrivebaseConstants.REEF_SOUTH_RIGHT_POSE;
            case SOUTHEAST_LEFT -> Constants.DrivebaseConstants.REEF_SOUTHEAST_LEFT_POSE;
            case SOUTHEAST_RIGHT -> Constants.DrivebaseConstants.REEF_SOUTHEAST_RIGHT_POSE;
            case SOUTHWEST_LEFT -> Constants.DrivebaseConstants.REEF_SOUTHWEST_LEFT_POSE;
            case SOUTHWEST_RIGHT -> Constants.DrivebaseConstants.REEF_SOUTHWEST_RIGHT_POSE;
            default -> swerve.getPose();
        };
    }

    public Pose2d stationPose() {
        return switch (stationPose) {
            case POSE_1_LEFT -> Constants.DrivebaseConstants.LEFT_STATION_POSE_1;
            case POSE_2_LEFT -> Constants.DrivebaseConstants.LEFT_STATION_POSE_2;
            case POSE_3_LEFT -> Constants.DrivebaseConstants.LEFT_STATION_POSE_3;
            case POSE_4_LEFT -> Constants.DrivebaseConstants.LEFT_STATION_POSE_4;
            case POSE_1_RIGHT -> Constants.DrivebaseConstants.RIGHT_STATION_POSE_1;
            case POSE_2_RIGHT -> Constants.DrivebaseConstants.RIGHT_STATION_POSE_2;
            case POSE_3_RIGHT -> Constants.DrivebaseConstants.RIGHT_STATION_POSE_3;
            case POSE_4_RIGHT -> Constants.DrivebaseConstants.RIGHT_STATION_POSE_4;
            default -> swerve.getPose();
        };
    }

    public enum ReefSide {
        NORTH,
        NORTHEAST,
        SOUTHEAST,
        SOUTH,
        SOUTHWEST,
        NORTHWEST,
    }

    public enum LeftOrRight {
        _RIGHT,
        _LEFT,
    }

    public enum ReefPose {
        NORTH_RIGHT,
        NORTH_LEFT,
        NORTHEAST_RIGHT,
        NORTHEAST_LEFT,
        SOUTHEAST_RIGHT,
        SOUTHEAST_LEFT,
        SOUTH_RIGHT,
        SOUTH_LEFT,
        SOUTHWEST_RIGHT,
        SOUTHWEST_LEFT,
        NORTHWEST_RIGHT,
        NORTHWEST_LEFT,
    }

    public enum StationSlot {
        POSE_1,
        POSE_2,
        POSE_3,
        POSE_4,
    }

    public enum StationPose {
        POSE_1_LEFT,
        POSE_2_LEFT,
        POSE_3_LEFT,
        POSE_4_LEFT,

        POSE_1_RIGHT,
        POSE_2_RIGHT,
        POSE_3_RIGHT,
        POSE_4_RIGHT,
    }

}
