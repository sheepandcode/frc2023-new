// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turncontroller extends SubsystemBase {
  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;
  
  
  private static final double kToleranceDegrees = 0;
  private static final double kToleranceAngularVelocity = 0;
  
  
  
  /** Creates a new Turncontroller. */
  private PIDController turnController;

  private Turncontroller() {
    turnController = new PIDController(kP, kI, kD);
  }

  private static Turncontroller tcontroller = null;

  public static Turncontroller getInstance() {
    if (tcontroller == null) {
      tcontroller = new Turncontroller();

    }
    return tcontroller;
  }

  public void setSetpoint(double x) {
    turnController.setSetpoint(x);
  }

  public void reset() {
    turnController.reset();
  }
  public void enableContinuousInput(double a, double b) {
    turnController.enableContinuousInput(a, b);
  }
  public void setIntegratorRange(double m, double n) {
    turnController.setIntegratorRange(m, n);
  }
  public void setTolerance() {
    turnController.setTolerance(kToleranceDegrees, kToleranceAngularVelocity);
  }
  public double calculate(double c) {
    return turnController.calculate(c);
  }
  public boolean atSetpoint() {
    return turnController.atSetpoint();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
