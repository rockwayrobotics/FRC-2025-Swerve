package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands;
import java.util.List;
import java.util.Set;
import java.util.function.IntConsumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  private int lastSeenTagID = -1;
  private int lastAcknowledgedTag = -1;
  private static final Set<Integer> reefTags = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
  private final IntConsumer onLastSeenTagChange;
  private final DoubleSubscriber seenTag;
  private IntegerPublisher lastSeenTagPublisher;
  PhotonCamera flame1 = new PhotonCamera("Flame 1");
  PhotonPoseEstimator a1Pose =
      new PhotonPoseEstimator(
          AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)));
  // PhotonCamera flame2 = new PhotonCamera("Flame 2");
  // PhotonCamera igneous = new PhotonCamera("Igneous");

  public void visionPV() {
    PhotonPipelineResult a1Result = flame1.getLatestResult();
    // PhotonPipelineResult a2Result = flame2.getLatestResult();
    // PhotonPipelineResult igResult = igneous.getLatestResult();
    List<PhotonTrackedTarget> a1Targets = a1Result.getTargets();
    // List<PhotonTrackedTarget> a2Targets = a2Result.getTargets();
    // List<PhotonTrackedTarget> oDTargets = igResult.getTargets();
    // List<PhotonTrackedTarget> aToTargets = new LinkedList<PhotonTrackedTarget>();
    boolean f1HasTargets = a1Result.hasTargets();
    // boolean igHasTargets = igResult.hasTargets();
    // boolean f2HasTargets = a2Result.hasTargets();

    /*if(f1HasTargets && f2HasTargets){
      if(a1Targets.size()>a2Targets.size()){
        for(int i=0;i<=a2Targets.size();i++){
          int a=a2Targets.get(i).getFiducialId();
          boolean b = a1Targets.contains(a);
          if(b){
            aToTargets.add(a2Targets.get(i));
          }
        }
      }else{
        if(a1Targets.size()<=a2Targets.size()){
          for(int i=0;i>=a1Targets.size();i++){
            int a=a1Targets.get(i).getFiducialId();
            boolean b = a2Targets.contains(a);
            if(b){
              aToTargets.add(a1Targets.get(i));
            }
          }
        }
      }
    }
    if(aToTargets.size()==0){
      PhotonTrackedTarget a = a1Result.getBestTarget();
      PhotonTrackedTarget b = a2Result.getBestTarget();
      if()
    }*/

  }

  public Vision(IntConsumer onLastSeenTagChange) {
    var nt = NetworkTableInstance.getDefault();
    this.onLastSeenTagChange = onLastSeenTagChange;
    this.seenTag = nt.getDoubleTopic("/limelight/tid").subscribe(-1);
    this.lastSeenTagPublisher = nt.getIntegerTopic("/AdvantageKit/Vision/lastSeenTagId").publish();
  }

  public int getLastSeenTagID() {
    return this.lastSeenTagID;
  }

  @Override
  public void periodic() {
    if (this.lastSeenTagID >= 1) {
      switch (DriveCommands.targetSide) {
        case 1: // center
          Logger.recordOutput(
              "Vision/targetPose", DriveCommands.getLandingPose(this.lastSeenTagID));
          break;
        case 2: // right
          Logger.recordOutput(
              "Vision/targetPose", DriveCommands.getRightLandingPose(this.lastSeenTagID));
          break;
        case 0: // left
        default:
          Logger.recordOutput(
              "Vision/targetPose", DriveCommands.getLeftLandingPose(this.lastSeenTagID));
          break;
      }
    }
    var tid = (int) seenTag.get();
    if (tid == this.lastAcknowledgedTag) {
      return;
    }
    this.lastAcknowledgedTag = tid;
    if (reefTags.contains(tid)) {
      this.lastSeenTagID = tid;
      this.onLastSeenTagChange.accept(tid);
      this.lastSeenTagPublisher.accept(tid);
      Logger.recordOutput("Vision/targetPose", DriveCommands.getLeftLandingPose(tid));
    }
  }
}
