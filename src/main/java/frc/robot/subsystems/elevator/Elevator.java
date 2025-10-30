package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double goalHeightMillimeters = 0;
  private double heightMillimeters = 0;
  private boolean homeBeamBroken = false;
  private boolean isHomed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    heightMillimeters = inputs.positionMillimeters;
    homeBeamBroken = inputs.homeBeamBroken;
    Logger.recordOutput("Elevator/goal_height_mm", goalHeightMillimeters);
    Logger.recordOutput("Elevator/isHomed", isHomed);

    if (DriverStation.isDisabled() || !isHomed) {
      io.stop();
    } else {
      io.moveTowardsGoal(goalHeightMillimeters, heightMillimeters);
    }
  }

  public void setGoalHeightMillimeters(double heightMillimeters) {
    goalHeightMillimeters = heightMillimeters;
  }

  public double getHeightMillimeters() {
    return heightMillimeters;
  }

  public double getGoalHeightMillimeters() {
    return goalHeightMillimeters;
  }

  public boolean atGoal() {
    // within 1 centimeter
    return Math.abs(heightMillimeters - goalHeightMillimeters) < 10;
  }

  public void stop() {
    io.stop();
  }

  public void setMaxNormalizedSpeedTuner(double speed) {
    io.setMaxNormalizedSpeedTuner(speed);
  }

  public double getMaxNormalizedSpeedTuner() {
    return io.getMaxNormalizedSpeedTuner();
  }

  public void home() {
    io.zeroEncoder();
    setGoalHeightMillimeters(0);
    isHomed = true;
  }

  public void stayStill() {
    setGoalHeightMillimeters(heightMillimeters);
  }
}
