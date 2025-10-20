package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.*;

public class VisionSimulator {
  private final VisionSystemSim visionSim;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;

  public VisionSimulator() {
    visionSim = new VisionSystemSim("main");

    SimCameraProperties camProps = new SimCameraProperties();
    camProps.setCalibration(960, 720, Rotation2d.fromDegrees(70));
    camProps.setFPS(30);
    camProps.setAvgLatencyMs(25);
    camProps.setCalibError(0.3, 0.1);

    camera = new PhotonCamera("limelight");
    cameraSim = new PhotonCameraSim(camera, camProps);

    Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(Centimeters.of(9), Centimeters.of(-25), Centimeters.of(31)),
            new Rotation3d(Degrees.of(90), Degrees.of(180), Degrees.of(90)));
    visionSim.addCamera(cameraSim, robotToCamera);
    visionSim.addAprilTags(AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField());
    cameraSim.enableDrawWireframe(true);
  }

  public void update(Pose2d robotPose) {
    visionSim.update(robotPose);
  }
}
