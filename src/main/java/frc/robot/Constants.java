// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final int JoystickPort = 0;
    public static final int JoystickTranslationAxis = 1;
    public static final int JoystickStrafeAxis = 0;
    public static final int JoystickRotationAxis = 2;
  }

  public static class SwerveConstants {

    // flipped because originally there was too much slipping
    public static final double trackWidth = Units.inchesToMeters(24.5);
    public static final double wheelBase = Units.inchesToMeters(18.5);

    // Kinematics gets each module relative to center. X is forward/backward and Y is left/right, left is positive
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left 
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // back left
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // back right
    );

    public static final double kPTurning = 0.25;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double rotationGearRatio = (150.0 / 7.0);

    public static final double maxMetersPerSecond = 5;

    // front left
    public static final int frontLeftDriveMotorId = 8;
    public static final int frontLeftRotationMotorId = 7;
    public static final int frontLeftCanCoderId = 12;
    public static final double frontLeftOffsetRad = Units.degreesToRadians(104);

    // front right
    public static final int frontRightDriveMotorId = 6;
    public static final int frontRightRotationMotorId = 5;
    public static final int frontRightCanCoderId = 13;
    public static final double frontRightOffsetRad = Units.degreesToRadians(44.209);

    // back left
    public static final int backLeftDriveMotorId = 4;
    public static final int backLeftRotationMotorId = 3;
    public static final int backLeftCanCoderId = 14;
    public static final double backLeftOffsetRad = Units.degreesToRadians(144);

    // back right
    public static final int backRightDriveMotorId = 2;
    public static final int backRightRotationMotorId = 1;
    public static final int backRightCanCoderId = 11;
    public static final double backRightOffsetRad = Units.degreesToRadians(1.5);

    public static final double kTeleDriveMaxAcceleration = 3;
    public static final double kTeleDriveMaxAngularAcceleration = 3;
  }
}
