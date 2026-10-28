package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    private static final Translation2d kDefaultBlueHubPosition = new Translation2d(Inches.of(182.105), Inches.of(158.845));
    private static final Translation2d kDefaultRedHubPosition = new Translation2d(Inches.of(469.115), Inches.of(158.845));

    private static final int[] kBlueHubTagIds = {20, 21, 26, 18};
    private static final int[] kRedHubTagIds = {5, 10, 2, 4};

    private static Alliance cachedAlliance = null;

    /**
     * Returns the cached alliance, polling DriverStation until it becomes available.
     * Once set, the alliance is locked for the rest of the match.
     */
    public static Alliance getAlliance() {
        if (cachedAlliance == null) {
            DriverStation.getAlliance().ifPresent(a -> {
                cachedAlliance = a;
                DriverStation.reportWarning("Landmarks: alliance locked to " + a, false);
            });
        }
        return cachedAlliance != null ? cachedAlliance : Alliance.Blue;
    }

    public static Translation2d hubPosition() {
        //final Alliance alliance = getAlliance();
        //return alliance == Alliance.Blue ? computeHubPosition(kBlueHubTagIds, kDefaultBlueHubPosition) : computeHubPosition(kRedHubTagIds, kDefaultRedHubPosition);
        return kDefaultBlueHubPosition;
    }

    public static final AprilTagFieldLayout layout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = layout.getFieldLength();
    public static final double fieldWidth = layout.getFieldWidth();

    //red outpost
    public static final Translation2d redOutpostCenter = getTagPosition(13);
    public static final Translation2d redOutpostOffset = getTagPosition(14);

    //red ds 1
    public static final Translation2d redDSOffset = redOutpostCenter.plus(new Translation2d(0,fieldWidth - 0.5));

    //red hub (backside)
    public static final Translation2d redHub = computeHubPosition(kRedHubTagIds, kDefaultRedHubPosition);
    public static final Translation2d redHubOffset = getTagPosition(4);

    //blue outpost
    public static final Translation2d blueOutpostCenter  = getTagPosition(29);
    public static final Translation2d blueOutpostOffset  = getTagPosition(30);

    //blue ds 1
    public static final Translation2d blueDSOffset = blueOutpostCenter.minus(new Translation2d(0, fieldWidth -0.5));

    //blue hub (backside)
    public static final Translation2d blueHub = computeHubPosition(kBlueHubTagIds, kDefaultBlueHubPosition);
    public static final Translation2d blueHubOffset = getTagPosition(26);

    private static Translation2d computeHubPosition(int[] tagIds, Translation2d fallback) {
        Translation2d sum = new Translation2d();
        int count = 0;
        for (int id : tagIds) {
            Optional<Pose3d> pose = layout.getTagPose(id);
            if (pose.isPresent()) {
                sum = sum.plus(pose.get().getTranslation().toTranslation2d());
                count++;
            }
        }
        return count > 0 ? sum.div(count) : fallback;
    }

    private static Translation2d getTagPosition(int id) {
        return layout.getTagPose(id)
            .map(pose3d -> pose3d.getTranslation().toTranslation2d())
            .orElse(new Translation2d(0, 0));
    }
    
    public static Pose2d climbPose() {
        final Alliance alliance = getAlliance();
        return alliance == Alliance.Blue ? Constants.ClimbAlignment.kBlueAllianceTargetPose : Constants.ClimbAlignment.kRedAllianceTargetPose;
    }

    public static Translation2d getClosestTag(Translation2d robotPosition) {
        final int[] hubTagIds = getAlliance() == Alliance.Blue ? kBlueHubTagIds : kRedHubTagIds;
        Translation2d closest = null;
        double closestDistance = Double.MAX_VALUE;

        for (int id : hubTagIds) {
            Optional<Pose3d> tagPose = layout.getTagPose(id);
            if (tagPose.isPresent()) {
                Translation2d tagPosition = tagPose.get().getTranslation().toTranslation2d();
                double distance = robotPosition.getDistance(tagPosition);
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closest = tagPosition;
                }
            }
        }

        return closest != null ? closest : hubPosition();
    }
}