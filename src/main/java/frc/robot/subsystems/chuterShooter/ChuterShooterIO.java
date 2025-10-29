package frc.robot.subsystems.chuterShooter;

import org.littletonrobotics.junction.AutoLog;

public interface ChuterShooterIO {
  @AutoLog
  public static class ChuterShooterIOInputs {
    public double shooterVelocityRadPerSec = 0.0;
    public double appliedOutput = 0.0;
  }

  /**
   * Set the shooter to a normalized speed in [-1, 1]. FIXME: This is probably not what we want
   *
   * @param speed
   */
  public default void setShooterSpeed(double speed) {}

  public default void scheduleShoot(double speed, double delaySeconds) {}

  public default void stopShooting() {}

  public default void updateInputs(ChuterShooterIOInputs inputs) {}

  public default void updateParams(boolean resetSafe) {}
}
