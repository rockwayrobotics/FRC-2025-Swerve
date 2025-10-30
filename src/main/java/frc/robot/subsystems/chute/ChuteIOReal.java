package frc.robot.subsystems.chute;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Chute;
import frc.robot.util.Tuner;

public class ChuteIOReal implements ChuteIO {
  protected final SparkMax pivotMotor = new SparkMax(CAN.PIVOT, MotorType.kBrushless);
  protected final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  protected final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  final Tuner pivotFeedforwardkS = new Tuner("Chute/pivot_feedforward_Ks", 0.07, true);
  // FIXME: why is this negative, it shouldn't be negative
  final Tuner pivotFeedforwardkG = new Tuner("Chute/pivot_feedforward_Kg", -0.125, true);
  final Tuner pivotPID_P = new Tuner("Chute/pivot_Kp", 0.3, true);
  final Tuner pivotPID_D = new Tuner("Chute/pivot_Kd", 0, true);
  final Tuner pivotMaxNormalizedSpeed = new Tuner("Chute/pivot_normalized_speed_max", 0.4, true);
  final Tuner pivotMinNormalizedSpeed = new Tuner("Chute/pivot_normalized_speed_min", -0.4, true);
  final Tuner pivotSoftLimitMinAngleRads =
      new Tuner("Chute/pivot_soft_limit_min_angle_rads", Units.degreesToRadians(-90), true);
  final Tuner pivotSoftLimitMaxAngleRads =
      new Tuner("Chute/pivot_soft_limit_max_angle_rads", Units.degreesToRadians(90), true);

  protected AsynchronousInterrupt inte = null;

  protected ArmFeedforward pivotFeedforward;

  public ChuteIOReal() {
    updateParams(true);

    // At creation time, set encoder positions to our initial position
    tryUntilOk(pivotMotor, 5, () -> pivotEncoder.setPosition(Chute.PIVOT_INITIAL_ANGLE_RADS));

    pivotFeedforwardkS.addListener((_e) -> updateParams(false));
    pivotFeedforwardkG.addListener((_e) -> updateParams(false));
    pivotPID_P.addListener((_e) -> updateParams(false));
    pivotPID_D.addListener((_e) -> updateParams(false));
    pivotMaxNormalizedSpeed.addListener((_e) -> updateParams(false));
    pivotMinNormalizedSpeed.addListener((_e) -> updateParams(false));
    pivotSoftLimitMinAngleRads.addListener((_e) -> updateParams(false));
    pivotSoftLimitMaxAngleRads.addListener((_e) -> updateParams(false));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    ifOk(pivotMotor, pivotEncoder::getPosition, (value) -> inputs.pivotAngleRadians = value);
    ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotVelocityRadPerSec = value);
    ifOk(pivotMotor, pivotMotor::getAppliedOutput, (value) -> inputs.appliedOutput = value);
  }

  @Override
  public void moveTowardsPivotGoal(double goalAngleRadians, double currentAngleRadians) {
    // Arm feed forward expects 0 to be parallel to the floor, but for us, 0 is
    // pointed straight down.
    var ff =
        pivotFeedforward.calculate(
            Math.PI - currentAngleRadians + Units.degreesToRadians(90),
            Math.signum(goalAngleRadians - currentAngleRadians));
    pivotController.setReference(
        goalAngleRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  @Override
  public void stopPivot() {
    pivotMotor.set(0);
  }

  private void updateParams(boolean resetSafe) {
    ResetMode resetMode =
        resetSafe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    pivotFeedforward = new ArmFeedforward(pivotFeedforwardkS.get(), pivotFeedforwardkG.get(), 0);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    if (resetSafe) {
      pivotConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(38)
          .voltageCompensation(12.0)
          .inverted(true);
      pivotConfig
          .encoder
          .positionConversionFactor(2 * Math.PI / Chute.PIVOT_GEAR_RATIO)
          .velocityConversionFactor(2 * Math.PI / Chute.PIVOT_GEAR_RATIO / 60);
    }
    // No ff term here because we want position control not velocity
    pivotConfig.closedLoop.pidf(pivotPID_P.get(), 0, pivotPID_D.get(), 0);
    pivotConfig.closedLoop.outputRange(
        pivotMinNormalizedSpeed.get(), pivotMaxNormalizedSpeed.get());
    pivotConfig
        .softLimit
        .forwardSoftLimit(pivotSoftLimitMaxAngleRads.get())
        .reverseSoftLimit(pivotSoftLimitMinAngleRads.get())
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    tryUntilOk(
        pivotMotor,
        5,
        () -> pivotMotor.configure(pivotConfig, resetMode, PersistMode.kPersistParameters));
  }

  public void setBrakeMode(boolean mode) {
    pivotMotor.configure(
        new SparkMaxConfig().idleMode(mode ? IdleMode.kBrake : IdleMode.kCoast),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void setEncoder(double position) {
    tryUntilOk(pivotMotor, 5, () -> pivotEncoder.setPosition(position));
  }
}
