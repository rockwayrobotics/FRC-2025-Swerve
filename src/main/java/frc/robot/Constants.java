// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class CAN {
    // Dev drivebase CAN IDs
    public static final int FL_DRIVE_DEV = 2;
    public static final int FL_TURN_DEV = 6;
    public static final int FR_DRIVE_DEV = 3;
    public static final int FR_TURN_DEV = 7;
    public static final int BL_DRIVE_DEV = 9;
    public static final int BL_TURN_DEV = 5;
    public static final int BR_DRIVE_DEV = 4;
    public static final int BR_TURN_DEV = 8;

    // Comp drivebase CAN IDs
    public static final int FL_DRIVE_COMP = 3;
    public static final int FL_TURN_COMP = 7;
    public static final int FR_DRIVE_COMP = 4;
    public static final int FR_TURN_COMP = 8;
    public static final int BL_DRIVE_COMP = 2;
    public static final int BL_TURN_COMP = 6;
    public static final int BR_DRIVE_COMP = 9;
    public static final int BR_TURN_COMP = 5;

    // Common CAN IDs
    public static final int ELEVATOR_BACK = 11;
    public static final int ELEVATOR_FRONT = 12;
    public static final int PIVOT = 10;
    public static final int SHOOTER = 13;
  }

  public static final class Chute {
    // We start at -90 degrees with the shooter facing to starboard
    // 0 degrees is vertically down
    // 90 degrees is the other limit, facing flat to port
    public static final double PIVOT_INITIAL_ANGLE_RADS = Units.degreesToRadians(-90.0);

    public static final double PIVOT_FLAT = Units.degreesToRadians(-90);

    public static final double PIVOT_GEAR_RATIO = 45; // 60/12 and 9:1 planetary

    // FIXME: Complete guess
    public static final double MOI = 0.1;
    public static final double CHUTE_LENGTH_METERS = 0.66;
    // Distance from loading end of chute to center of wheel, measured on y-axis
    // only
    public static final double CHUTE_WHEEL_POSITION_METERS = 0.568;
    public static final double SHOOTER_WHEEL_RADIUS_METERS = 0.05715 / 2.0;

    public static final double CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM = 256;
  }

  public static final class Elevator {
    public static final double GEAR_RATIO = 2;

    public static final double ELEVATOR_CONVERSION_FACTOR = 26.3472615411;

    public static final double SPROCKET_DIAMETER_METERS = 0.0448060837;
    // March 1st 2025:
    // (375/1.33203125) = 281.5249266862
    // (1192/4.251) = 280.4046106798

    public static final double CARRIAGE_MASS_KG = 5;
    // Minimum height of pivot center
    public static final double MIN_PIVOT_HEIGHT_METERS = 0.24;
    // Maximum extension height of elevator
    public static final double MAX_HEIGHT_METERS = 1.320;
  }

  public static enum CoralLevel {
    L1,
    L2,
    L3,
    Intake
  }

  public static enum Side {
    LEFT,
    RIGHT
  }
}
