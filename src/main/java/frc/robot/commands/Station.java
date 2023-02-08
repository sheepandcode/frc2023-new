// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Turncontroller;

public class Station extends CommandBase {
  /** Creates a new Station. */
  private final Drivebase m_drivebase;
  private final Gyro m_gyro;
  private final Turncontroller m_Turncontroller;

  public Station(Drivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = drivebase;
    m_gyro = Gyro.getInstance();
    m_Turncontroller = Turncontroller.getInstance();
    m_Turncontroller.setSetpoint(0);
    m_Turncontroller.enableContinuousInput(-11, 11);
    m_Turncontroller.setIntegratorRange(0, 0);
    m_Turncontroller.setTolerance();

    addRequirements(m_gyro);
    addRequirements(m_drivebase);
    addRequirements(m_Turncontroller);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Turncontroller.reset();
    m_gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("start", true);
    SmartDashboard.putNumber("angle", m_gyro.getPitch());
    double actualangle = m_gyro.getPitch();
    double speed = m_Turncontroller.calculate(Math.abs(actualangle));
    speed += Math.signum(speed) * 0.1;
    if (actualangle > 0) {
      m_drivebase.drive(speed, speed);
      
    } else if (actualangle < 0) {
      m_drivebase.drive(-speed, -speed);
    }
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.drive(0, 0);
    SmartDashboard.putBoolean("start", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Turncontroller.atSetpoint();
  }
}
