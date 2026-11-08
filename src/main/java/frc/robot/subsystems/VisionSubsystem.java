package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Landmarks;
import frc.robot.LimelightHelpers; // Note your specific path

public class VisionSubsystem extends SubsystemBase {
    private final String[] llNames = {"limelight-shooter"};
    private final String primaryLL = "limelight";
    private static final double kSingleTagAmbiguityThreshold = 0.2;

    // 1. Make the list an instance variable (re-use it to save memory)
    private final List<LimelightHelpers.PoseEstimate> cachedEstimates = new ArrayList<>();

    public VisionSubsystem() {}

    // Get the horizontal offset to the target
    public double getTX() {
        return LimelightHelpers.getTX(primaryLL);
    }

    // Get the vertical offset (useful for distance estimation)
    public double getTY() {
        return LimelightHelpers.getTY(primaryLL);
    }

    // Identify the YOLOv8 class currently being seen
    public String getDetectedClass() {
        return !LimelightHelpers.getNeuralClassID(primaryLL).isEmpty() ?
                LimelightHelpers.getLatestResults(primaryLL).targetingResults.classifierResults.get(0).className :
                "None";
    }

    @Override
    public void periodic() {
        // This runs 50 times a second on the robot
        // 2. Clear the list at the start of every loop (50Hz)
        cachedEstimates.clear();

        for (String llName : llNames) {
            // MegaTag2 requires orientation. Assuming 0 here, 
            // but Drivetrain should ideally call a "refresh" method with gyro data.
            //LimelightHelpers.SetRobotOrientation(llName, 0, 0, 0, 0, 0, 0);
            
            // 3. This is the expensive part—we only do it ONCE per camera
            LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            
            if (est != null && est.tagCount > 0) {
                cachedEstimates.add(est);
            }
        }
    }

    // 4. Return the already-filled list
    public List<LimelightHelpers.PoseEstimate> getAllPoseEstimates() {
        return cachedEstimates;
    }

    //kalman filter to see how trustworthy the vision code is
    public Matrix<N3, N1> getVisionStdDevsForEstimate(LimelightHelpers.PoseEstimate estimate) { 
        if (estimate == null || estimate.tagCount == 0) { //if 0 tags seen or no pose estimate, IGNORE ALL VISION
            return VecBuilder.fill(1e9, 1e9, 1e9);
        }
        
        double distance = estimate.avgTagDist;
        int tagCount = estimate.tagCount;
        
        double xyStdDev = 0.02 + (0.02 * distance * distance);
        // MegaTag2 already uses the gyro heading to solve for translation, so we
        // must not let vision correct the heading — that creates a feedback loop.
        double thetaStdDev = 999999.0;

        if (tagCount == 1) { //if we only see 1 tag trust it less
            xyStdDev *= 2.0;
        } else if (tagCount >= 5) { //if we see 5 or more (in our alliance zone basically) trust it more
            xyStdDev *= 0.7;
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public Pose2d getEstimatedGlobalPose() {
        LimelightHelpers.PoseEstimate bestEstimate = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        
        for (String llName : llNames) {
            if (!LimelightHelpers.getTV(llName)) {
                continue; //if not confident skip
            }
            
            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            
            if (estimate == null || estimate.tagCount == 0 || !isPoseInsideField(estimate.pose)
                || hasHighSingleTagAmbiguity(estimate)) {
                continue;
            }
            
            //more tags = better, closer = better
            double score = estimate.tagCount * 10.0 - estimate.avgTagDist;
            
            if (score > bestScore) {
                bestScore = score;
                bestEstimate = estimate;
            }
        }
        
        return bestEstimate != null ? bestEstimate.pose : null;
    }

    public Double getPoseTimestampSeconds() {
        Pose2d bestPose = getEstimatedGlobalPose();
        if (bestPose == null) { //if not there js ignore the rest
            return null;
        }
        
        //timestamp of best estimate we have
        for (String llName : llNames) {
            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            
            if (estimate != null && estimate.pose.equals(bestPose)) {
                return estimate.timestampSeconds;
            }
        }
        return null;
    }


    public int getTagCount() {
        int totalTags = 0;
        
        for (String llName : llNames) {
            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            
            if (estimate != null) {
                totalTags += estimate.tagCount;
            }
        }
        
        return totalTags;
    }

    private boolean isPoseInsideField(Pose2d pose) {
        if (pose == null) {
            return false;
        }
        final double x = pose.getX();
        final double y = pose.getY();
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return false;
        }
        final double margin = 0.1;
        return x >= -margin
            && x <= Landmarks.fieldLength + margin
            && y >= -margin
            && y <= Landmarks.fieldWidth + margin;
    }

    public boolean hasHighSingleTagAmbiguity(LimelightHelpers.PoseEstimate estimate) {
        if (estimate == null) {
            return true;
        }
        if (estimate.tagCount > 1 || estimate.rawFiducials == null
            || estimate.rawFiducials.length == 0) {
            return false;
        }
        return estimate.rawFiducials[0].ambiguity > kSingleTagAmbiguityThreshold;
    }
}
