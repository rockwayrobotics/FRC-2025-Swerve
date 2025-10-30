package frc.robot.subsystems.chuterShooter;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.CAN;

public class ChuterShooterIOReal implements ChuterShooterIO {
  private final Object motorLock = new Object();
  protected final SparkMax shooterMotor = new SparkMax(CAN.SHOOTER, MotorType.kBrushless);
  protected final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

  protected double shooterSpeed = 0;

  private Notifier delayedShootNotifier = new Notifier(() -> {});

  @Override
  public void updateInputs(ChuterShooterIOInputs inputs) {
    synchronized (motorLock) {
      shooterMotor.set(this.shooterSpeed);
    }

    ifOk(
        shooterMotor,
        shooterEncoder::getVelocity,
        (value) -> inputs.shooterVelocityRadPerSec = value);
    ifOk(shooterMotor, shooterMotor::getAppliedOutput, (value) -> inputs.appliedOutput = value);
  }

  @Override
  public void setShooterSpeed(double speed) {
    this.shooterSpeed = speed;
  }

  @Override
  public void stopShooting() {
    this.shooterSpeed = 0;
    delayedShootNotifier.stop();
  }

  @Override
  public void scheduleShoot(double speed, double delaySeconds) {
    delayedShootNotifier.setCallback(
        () -> {
          synchronized (motorLock) {
            shooterMotor.set(speed);
          }
        });
    delayedShootNotifier.startSingle(delaySeconds);
    setShooterSpeed(speed);
  }

  @Override
  public void updateParams(boolean resetSafe) {
    ResetMode resetMode =
        resetSafe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(38)
        .voltageCompensation(12.0)
        .inverted(true);
    tryUntilOk(
        shooterMotor,
        5,
        () -> shooterMotor.configure(shooterConfig, resetMode, PersistMode.kPersistParameters));
  }
}
