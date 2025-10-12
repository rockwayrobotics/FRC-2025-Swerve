package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.backLeftDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.backLeftTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.backLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.backRightDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.backRightTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.backRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.driveEncoderPositionFactor;
import static frc.robot.subsystems.drive.DriveConstants.driveEncoderVelocityFactor;
import static frc.robot.subsystems.drive.DriveConstants.driveGearbox;
import static frc.robot.subsystems.drive.DriveConstants.driveKd;
import static frc.robot.subsystems.drive.DriveConstants.driveKp;
import static frc.robot.subsystems.drive.DriveConstants.driveKs;
import static frc.robot.subsystems.drive.DriveConstants.driveKv;
import static frc.robot.subsystems.drive.DriveConstants.driveMotorCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.driveMotorReduction;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.frontRightDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontRightTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.subsystems.drive.DriveConstants.turnEncoderPositionFactor;
import static frc.robot.subsystems.drive.DriveConstants.turnEncoderVelocityFactor;
import static frc.robot.subsystems.drive.DriveConstants.turnGearRatio;
import static frc.robot.subsystems.drive.DriveConstants.turnGearbox;
import static frc.robot.subsystems.drive.DriveConstants.turnInverted;
import static frc.robot.subsystems.drive.DriveConstants.turnKd;
import static frc.robot.subsystems.drive.DriveConstants.turnKp;
import static frc.robot.subsystems.drive.DriveConstants.turnMotorCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.turnMotorReduction;
import static frc.robot.subsystems.drive.DriveConstants.turnPIDMaxInput;
import static frc.robot.subsystems.drive.DriveConstants.turnPIDMinInput;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sensors.ThriftyEncoder;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ModuleIORealSim implements ModuleIO {
  private final Rotation2d zeroRotation;
  private final int moduleId;

  // Hardware objects
  private final SparkFlex driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  private final ThriftyEncoder turnEncoder;

  // Sim objects
  private final SparkSim driveSparkSim;
  private final SparkSim turnSparkSim;
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIORealSim(int module) {
    this.moduleId = module;
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox);

    zeroRotation =
        switch (module) {
          case 0 -> backRightZeroRotation;
          case 1 -> backLeftZeroRotation;
          case 2 -> frontLeftZeroRotation;
          case 3 -> frontRightZeroRotation;
          default -> new Rotation2d();
        };
    driveSpark =
        new SparkFlex(
            switch (module) {
              case 0 -> backRightDriveCanId;
              case 1 -> backLeftDriveCanId;
              case 2 -> frontLeftDriveCanId;
              case 3 -> frontRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    driveSparkSim = new SparkFlexSim(driveSpark, DCMotor.getNeoVortex(1));
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> backRightTurnCanId;
              case 1 -> backLeftTurnCanId;
              case 2 -> frontLeftTurnCanId;
              case 3 -> frontRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSparkSim = new SparkMaxSim(turnSpark, DCMotor.getNEO(1));
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = new ThriftyEncoder(module % 4);
    turnEncoder.setInverted(true);
    turnEncoder.setPositionOffset(0); // -zeroRotation.getRadians());
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new SparkFlexConfig();

    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0);

    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor) // Converts from rotations to radians
        .velocityConversionFactor(
            driveEncoderVelocityFactor) // Converts from rotations to radians/second
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            driveKp, 0.0,
            driveKd, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(turnEncoderPositionFactor / turnGearRatio)
        .velocityConversionFactor(turnEncoderVelocityFactor / turnGearRatio)
        // .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    // .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .outputRange(-.5, .5) // FIXME: REMOVE THIS LATER
        .pidf(turnKp, 0.0, turnKd, 0.0);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        // .absoluteEncoderPositionAlwaysOn(true)
        // .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        // .absoluteEncoderVelocityAlwaysOn(true)
        // .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(turnSpark, turnEncoder::getVirtualPosition);

    turnSpark.getEncoder().setPosition(turnEncoder.getVirtualPosition());
    System.out.println("Set encoder " + module + ", " + turnEncoder.getVirtualPosition());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    simulationUpdate();

    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getVirtualPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value));
    ifOk(
        turnSpark,
        () -> turnSpark.getEncoder().getPosition(),
        (value) -> inputs.turnRelativePosition = new Rotation2d(value));
    ifOk(
        turnSpark,
        () -> turnSpark.getEncoder().getVelocity(),
        (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  private void simulationUpdate() {
    // Compute applied voltages from Spark simulation
    double driveVoltage = driveSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    double turnVoltage = turnSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage();

    driveSim.setInputVoltage(MathUtil.clamp(driveVoltage, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnVoltage, -12.0, 12.0));

    // Step the physical simulation forward
    driveSim.update(0.02);
    turnSim.update(0.02);

    driveSparkSim.iterate(
        driveSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
    turnSparkSim.iterate(
        turnSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

    double turnAngleRad = turnSim.getAngularPositionRotations() * 2 * Math.PI / turnGearRatio;
    turnAngleRad = MathUtil.inputModulus(turnAngleRad, 0, 2 * Math.PI);
    turnSparkSim.setPosition(turnAngleRad);

    System.out.printf(
        "turnPos=%.3f rad, turnApplied=%.2fV%n",
        turnSparkSim.getPosition(),
        turnSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            driveSim.getCurrentDrawAmps() + turnSim.getCurrentDrawAmps()));
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(rotation.getRadians(), turnPIDMinInput, turnPIDMaxInput);
    Logger.recordOutput("/Drive/TurnSetpoint" + this.moduleId, setpoint);
    turnController.setReference(setpoint, ControlType.kPosition);
  }
}
