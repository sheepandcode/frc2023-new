// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.CAN_ID.*;
import static frc.robot.Constants.KinematicsMeters.*;

import java.util.ArrayList;
import java.util.List;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivebase. */
  public WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  public WPI_TalonSRX rightFollow = new WPI_TalonSRX(2);
  public WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  public WPI_TalonSRX leftFollow = new WPI_TalonSRX(4);

  private MecanumDrive mecanum = new MecanumDrive(leftMaster, rightMaster, leftFollow, rightFollow);
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(LEFT_FRONT_CENTER, RIGHT_FRONT_CENTER,
      LEFT_BACK_CENTER, RIGHT_BACK_CENTER);

  private List<Double> encoder = new ArrayList<>();

  public Drivebase() {
    leftMaster.setInverted(false);
    leftFollow.setInverted(false);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftFollow.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFollow.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollow.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollow.setNeutralMode(NeutralMode.Brake);

  }

  public void drive(double x, double y, double zR) {
    SmartDashboard.putNumber("xSpeed", x);
    SmartDashboard.putNumber("ySpeed", y);
    SmartDashboard.putNumber("zRotation", zR);
    mecanum.driveCartesian(x, y, zR);
  }

  public List<Double> getEncoder() {
    return encoder;
  }

  public MecanumDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoder.add(0, leftMaster.getSelectedSensorPosition());
    encoder.add(1, rightMaster.getSelectedSensorPosition());
    encoder.add(2, leftFollow.getSelectedSensorPosition());
    encoder.add(3, rightFollow.getSelectedSensorPosition());

    // Put to SmartDashboard
    SmartDashboard.putNumber("Left front Talon temp", leftMaster.getTemperature());
    SmartDashboard.putNumber("Left back Talon temp", leftFollow.getTemperature());
    SmartDashboard.putNumber("Right front Talon temp", rightMaster.getTemperature());
    SmartDashboard.putNumber("Right back Talon temp", rightFollow.getTemperature());
  }
  }

