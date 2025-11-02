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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.NavX.AHRS;
import java.util.Queue;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX =
      new AHRS(Port.kMXP, /* NavXComType.kMXP_SPI, */ (byte) odometryFrequency);
  // private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte)
  // odometryFrequency);
  private boolean initialYawSet = false;
  private double initialYaw = 0;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  // IDEA: Use magnetometer reading to initialize gyro position after calibrating on field
  // so that we know our heading at startup?
  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    if (!initialYawSet && inputs.connected) {
      initialYaw = -navX.getAngle();
      initialYawSet = true;
    }
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle() - initialYaw);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  public void zeroGyro() {
    navX.zeroYaw();
    initialYaw = 0;
    initialYawSet = true;
  }

  public double getAngle() {
    return navX.getAngle();
  }
}
