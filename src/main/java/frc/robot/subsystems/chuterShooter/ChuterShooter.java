package frc.robot.subsystems.chuterShooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Tuner;
import org.littletonrobotics.junction.Logger;

public class ChuterShooter extends SubsystemBase {
  private final ChuterShooterIO io;
  private final ChuterShooterIOInputsAutoLogged inputs = new ChuterShooterIOInputsAutoLogged();

  final Tuner shooterSpeedTuner = new Tuner("ShooterSpeed", 0.3, true);

  public ChuterShooter(ChuterShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ChuterShooter", inputs);

    if (DriverStation.isDisabled()) {
      io.setShooterSpeed(0);
    }
  }

  public void startShooting() {
    io.setShooterSpeed(shooterSpeedTuner.get());
  }

  public void setShooterMotor(double speed) {
    io.setShooterSpeed(speed);
  }

  public void stopShooting() {
    io.stopShooting();
  }

  public void scheduleShoot(double speed, double delaySeconds) {
    io.scheduleShoot(speed, delaySeconds);
  }
}
