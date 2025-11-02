package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands;
import java.util.Set;
import java.util.function.IntConsumer;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private int lastSeenTagID = -1;
  private int lastAcknowledgedTag = -1;
  private static final Set<Integer> reefTags = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
  private final IntConsumer onLastSeenTagChange;
  private final DoubleSubscriber seenTag;
  private IntegerPublisher lastSeenTagPublisher;

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
