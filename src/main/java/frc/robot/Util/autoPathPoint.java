package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class autoPathPoint {
    public final Pose2d pose;
    public final boolean isCameraTracking;
    public final boolean isIntakeOn;
    public final double angleTolerance;
    public final double posTolerance;
    public autoPathPoint(Pose2d pose, boolean isCameraTracking, boolean isIntakeOn){
        this.pose = pose;
        this.isCameraTracking = isCameraTracking;
        this.isIntakeOn = isIntakeOn;
        posTolerance = Constants.xToPosTolerance;
        angleTolerance = Constants.thetaToPosTolerance;
    }
    public autoPathPoint(Pose2d pose, boolean isCameraTracking, boolean isIntakeOn, double coordTol, double angleTol){
        this.pose = pose;
        this.isCameraTracking = isCameraTracking;
        this.isIntakeOn = isIntakeOn;
        posTolerance = coordTol;
        angleTolerance = angleTol;
    }
}
