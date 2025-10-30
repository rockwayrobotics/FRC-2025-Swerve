package frc.robot.subsystems.elevator;

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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Elevator;
import frc.robot.util.Tuner;
import java.util.function.DoubleSupplier;

public class ElevatorIOReal implements ElevatorIO {
  // Note that we may eventually have a second motor on the elevator
  protected final SparkMax motorMain = new SparkMax(CAN.ELEVATOR_FRONT, MotorType.kBrushless);
  protected final SparkMax motorFollower = new SparkMax(CAN.ELEVATOR_BACK, MotorType.kBrushless);

  final Tuner elevatorFeedforwardkS = new Tuner("Elevator/feedforward_Ks", 0.1, true);
  final Tuner elevatorFeedforwardkG = new Tuner("Elevator/feedforward_Kg", 0.4, true);
  final Tuner elevatorFeedforwardkV = new Tuner("Elevator/feedforward_Kv", 0.01427, true);
  final Tuner elevatorPID_P = new Tuner("Elevator/Kp", 0.00136, true);
  final Tuner elevatorPID_I = new Tuner("Elevator/KI", 0, true);
  final Tuner elevatorPID_D = new Tuner("Elevator/Kd", 0.005, true);
  final Tuner elevatorMaxNormalizedSpeed = new Tuner("Elevator/normalized_speed_max", 0.6, true);
  final Tuner elevatorMinNormalizedSpeed = new Tuner("Elevator/normalized_speed_min", -0.5, true);
  final Tuner elevatorSoftLimitMin = new Tuner("Elevator/soft_limit_min_mm", 20, true);
  final Tuner elevatorSoftLimitMax = new Tuner("Elevator/soft_limit_max_mm", 1200, true);
  final Tuner elevatorRampRate = new Tuner("Elevator/ramp_rate_s", 1, true);

  protected final RelativeEncoder encoder = motorMain.getEncoder();
  protected final SparkClosedLoopController controller = motorMain.getClosedLoopController();
  protected ElevatorFeedforward feedforward;

  // FIXME: Not used and not measured - only needs to be arbitrary positive
  protected final double SPEED_MM_PER_SEC = 0.5;

  public ElevatorIOReal() {
    updateParams(true);

    tryUntilOk(motorMain, 5, () -> encoder.setPosition(0.0));
    var followConfig = new SparkMaxConfig().follow(motorMain);
    tryUntilOk(
        motorFollower,
        5,
        () ->
            motorFollower.configure(
                followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    elevatorFeedforwardkS.addListener((_e) -> updateParams(false));
    elevatorFeedforwardkG.addListener((_e) -> updateParams(false));
    elevatorFeedforwardkV.addListener((_e) -> updateParams(false));
    elevatorPID_P.addListener((_e) -> updateParams(false));
    elevatorPID_I.addListener((_e) -> updateParams(false));
    elevatorPID_D.addListener((_e) -> updateParams(false));
    elevatorMaxNormalizedSpeed.addListener((_e) -> updateParams(false));
    elevatorMinNormalizedSpeed.addListener((_e) -> updateParams(false));
    elevatorSoftLimitMin.addListener((_e) -> updateParams(false));
    elevatorSoftLimitMax.addListener((_e) -> updateParams(false));
    elevatorRampRate.addListener((_e) -> updateParams(false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // inputs.homeBeamBroken = Sensors.getInstance().getElevatorHomeBeambroken();
    // if (inputs.homeBeamBroken) {
    // // REVUtils.tryUntilOk(() -> encoder.setPosition(0.0));
    // }

    // FIXME: Measure CAN bus usage with all these queries?
    ifOk(motorMain, encoder::getPosition, (value) -> inputs.positionMillimeters = value);
    ifOk(motorMain, encoder::getVelocity, (value) -> inputs.velocityMillimetersPerSec = value);
    ifOk(
        motorMain,
        new DoubleSupplier[] {motorMain::getAppliedOutput, motorMain::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);
    ifOk(motorMain, motorMain::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
    // FIXME: Could ask for temperature?
    // REVUtils.ifOk(motor, motor::getMotorTemperature, (value) ->
    // inputs.tempCelsius = value);
  }

  @Override
  public void moveTowardsGoal(double goalHeightMillimeters, double currentHeightMillimeters) {
    var velocity = SPEED_MM_PER_SEC * Math.signum(goalHeightMillimeters - currentHeightMillimeters);
    var ff = feedforward.calculate(velocity);
    controller.setReference(
        goalHeightMillimeters, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  public void setVoltage(double volts) {
    controller.setReference(volts, ControlType.kVoltage);
  }

  @Override
  public void stop() {
    motorMain.set(0);
  }

  @Override
  public void setMaxNormalizedSpeedTuner(double speed) {
    elevatorMaxNormalizedSpeed.set(speed);
  }

  @Override
  public double getMaxNormalizedSpeedTuner() {
    return elevatorMaxNormalizedSpeed.get();
  }

  public void updateParams(boolean resetSafe) {
    ResetMode resetMode =
        resetSafe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    feedforward =
        new ElevatorFeedforward(
            elevatorFeedforwardkS.get(), elevatorFeedforwardkG.get(), elevatorFeedforwardkV.get());
    SparkMaxConfig config = new SparkMaxConfig();
    if (resetSafe) {
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(60)
          .voltageCompensation(12.0)
          .inverted(true);
      // FIXME: apply config to both motors

      config
          .encoder
          .positionConversionFactor(Elevator.ELEVATOR_CONVERSION_FACTOR)
          .velocityConversionFactor(Elevator.ELEVATOR_CONVERSION_FACTOR / 60);
    }

    config
        .softLimit
        .forwardSoftLimit(elevatorSoftLimitMax.get())
        .reverseSoftLimit(elevatorSoftLimitMin.get())
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    // No ff term here because we want position control not velocity
    config.closedLoop.pidf(elevatorPID_P.get(), elevatorPID_I.get(), elevatorPID_D.get(), 0);
    config.closedLoop.outputRange(
        elevatorMinNormalizedSpeed.get(), elevatorMaxNormalizedSpeed.get());
    config.closedLoopRampRate(elevatorRampRate.get());
    tryUntilOk(
        motorMain, 5, () -> motorMain.configure(config, resetMode, PersistMode.kPersistParameters));
  }

  public void zeroEncoder() {
    encoder.setPosition(0);
  }
}
