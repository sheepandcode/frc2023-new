// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  private AHRS ahrs;

  private Gyro() {
    ahrs = new AHRS();
  }

  private static Gyro gyro = null;

  public static Gyro getInstance() {
    if (gyro == null) {
      gyro = new Gyro();
    }
    return gyro;
  }

  public void reset() {
    ahrs.reset();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
