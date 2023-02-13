// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class PID{
    public static final double kP = 1;
    public static final double kI = 1;
    public static final double kD = 1;


    public static final int kToleranceDegrees = 0;
    public static final int kToleranceAngularVelocity = 0;
  }
  public static class CAN_ID{
    public static final int LEFT_FRONT = 1;
    /** Left Back Motor */
    public static final int LEFT_BACK = 2;
    /** Right Front Motor */
    public static final int RIGHT_FRONT = 3;
    /** Right Back Motor */
    public static final int RIGHT_BACK = 4;
  }

  public static class KinematicsMeters{
    public static Translation2d LEFT_FRONT_CENTER = new Translation2d();
    /** Left Back Wheel */
    public static Translation2d LEFT_BACK_CENTER = new Translation2d();
    /** Right Front Wheel */
    public static Translation2d RIGHT_FRONT_CENTER = new Translation2d();
    /** Right Back Wheel */
    public static Translation2d RIGHT_BACK_CENTER = new Translation2d();
  }
}
  