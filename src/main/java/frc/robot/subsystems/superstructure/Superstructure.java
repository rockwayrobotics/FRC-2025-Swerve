package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.Side;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.TunableSetpoints;
import frc.robot.util.Tuner;

public class Superstructure extends SubsystemBase {
  public final Elevator elevator;
  public final Chute chute;

  protected final Tuner pivotAngleTweakTuner = new Tuner("Chute/pivot_angle_tweak_deg", 0, true);

  protected TunableSetpoints setpoints = new TunableSetpoints();

  public Superstructure(Elevator elevator, Chute chute) {
    this.elevator = elevator;
    this.chute = chute;
  }

  @Override
  public void periodic() {
    if (elevator.getGoalHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
      elevator.periodic();
    } else if ((Math.abs(chute.getPivotAngleRads()) - Units.degreesToRadians(90)) < 0.1) {
      elevator.periodic();
    }

    // FIXME: make this more robust
    chute.periodic(
        elevator.getHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM - 50);
  }

  public void setElevatorGoalHeightMillimeters(double heightMillimeters) {
    // FIXME: Check if elevator is homed. If not... do nothing?
    // FIXME done?????: Check chute state, if not safe, move it first? Or do
    // nothing?
    if (heightMillimeters < Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
      if (chute.getPivotAngleRads() <= 0) {
        chute.setPivotGoalRads(Units.degreesToRadians(-90));
      } else {
        chute.setPivotGoalRads(Units.degreesToRadians(90));
      }
    }
    // ; // FIXME: make constant
    elevator.setGoalHeightMillimeters(MathUtil.clamp(heightMillimeters, 0, 1200 - 1));
  }

  public double getElevatorHeightMillimeters() {
    return elevator.getHeightMillimeters();
  }

  public void setElevatorMaxSpeed(double speed) {
    elevator.setMaxNormalizedSpeedTuner(speed);
  }

  public double getElevatorMaxSpeed() {
    return elevator.getMaxNormalizedSpeedTuner();
  }

  public boolean isElevatorAtGoal() {
    return elevator.atGoal();
  }

  public boolean isPivotAtGoal() {
    return chute.isPivotAtGoal();
  }

  public void setChutePivotGoalRads(double pivotAngleRads) {
    // FIXME: Check if chute is homed. If not... do nothing?
    // FIXME: Check elevator height, if not safe, move it first? Or do nothing?
    chute.setPivotGoalRads(pivotAngleRads);
  }

  public double getPivotAngleRads() {
    return chute.getPivotAngleRads();
  }

  /** Schedules the superstructure homing sequence. */
  public void home() {
    // var command = Commands.parallel(
    // Commands.runOnce(() -> grabber.home()),
    // Commands.runOnce(() -> {
    // elevator.home();
    // elevator.setGoalHeightMillimeters(400);
    // }),
    // Commands.sequence(
    // Commands
    // .waitUntil(() -> elevator.getHeightMillimeters() >
    // Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM),
    // Commands.runOnce(() -> chute.home())))
    // .finallyDo(() -> elevator.setGoalHeightMillimeters(0));

    // command.addRequirements(this);
    // command.schedule();
    var command =
        Commands.parallel(
            Commands.runOnce(
                () -> {
                  elevator.home();
                }),
            Commands.runOnce(
                () -> {
                  chute.home();
                }));

    command.addRequirements(this);
    command.schedule();
  }

  /**
   * Schedules a command to shut down the superstructure carefully. This is intended to be used to
   * get ready for climp.
   */
  public void foldForClimp() {
    Commands.parallel(
            Commands.runOnce(
                () -> {
                  if (chute.getPivotAngleRads() <= 0) {
                    chute.setPivotGoalRads(Units.degreesToRadians(-90));
                  } else {
                    chute.setPivotGoalRads(Units.degreesToRadians(90));
                  }
                }),
            Commands.runOnce(
                () -> {
                  if (elevator.getGoalHeightMillimeters()
                      > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM + 1) {
                    elevator.setGoalHeightMillimeters(
                        Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM + 1);
                  }
                }),
            Commands.sequence(
                Commands.waitUntil(
                    () -> Math.abs(chute.getPivotAngleRads()) > Units.degreesToRadians(87)),
                Commands.runOnce(() -> elevator.setGoalHeightMillimeters(0))))
        .schedule();
  }

  public void gotoSetpoint(CoralLevel level, Side side) {
    int sideMultiplier = (side == Side.LEFT) ? -1 : 1;
    double pivotAngleTweak =
        Units.degreesToRadians(MathUtil.clamp(pivotAngleTweakTuner.get(), -10, 10));
    switch (level) {
      case L1:
        setChutePivotGoalRads(
            sideMultiplier * setpoints.L1_chute_pivot_angle_rads() + pivotAngleTweak);
        setElevatorGoalHeightMillimeters(setpoints.L1_elevator_height_mm());
        break;
      case L2:
        setChutePivotGoalRads(
            sideMultiplier * setpoints.L2_chute_pivot_angle_rads() + pivotAngleTweak);
        setElevatorGoalHeightMillimeters(setpoints.L2_elevator_height_mm());
        break;
      case L3:
        setChutePivotGoalRads(
            sideMultiplier * setpoints.L3_chute_pivot_angle_rads() + pivotAngleTweak);
        setElevatorGoalHeightMillimeters(setpoints.L3_elevator_height_mm());
        break;
      case Intake:
        setChutePivotGoalRads(
            sideMultiplier * setpoints.intake_chute_pivot_angle_rads() + pivotAngleTweak);
        setElevatorGoalHeightMillimeters(setpoints.intake_elevator_height_mm());
        break;
    }
  }

  public boolean atGoal() {
    return elevator.atGoal() && chute.isPivotAtGoal();
  }

  public void stayStill() {
    elevator.stayStill();
    chute.stayStill();
  }
}
