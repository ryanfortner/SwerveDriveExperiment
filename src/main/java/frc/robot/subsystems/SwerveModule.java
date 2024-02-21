// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax turningMotor;
  private final CANSparkMax driveMotor;

  private final RelativeEncoder turningEncoder;
  private final RelativeEncoder driveEncoder;

  private final PIDController turningPidController;

  private final CANcoder canCoder;
  
  private final boolean canCoderReversed;
  private final double canCoderOffsetRad;

  /** Creates a new SwerveModule. */
  public SwerveModule(
    int driveMotorId,
    int turningMotorId,
    boolean driveMotorReversed,
    boolean turningMotorReversed,
    int canCoderId,
    double canCoderOffset,
    boolean canCoderReversed) {
      this.canCoderOffsetRad = canCoderOffset;
      this.canCoderReversed = canCoderReversed;
      canCoder = new CANcoder(canCoderId);

      driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
      turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      turningMotor.setInverted(turningMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      turningEncoder = turningMotor.getEncoder();

      driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
      driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
      turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
      turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

      turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
      turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public double getDrivePosition() {
      return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
      return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
      return turningEncoder.getVelocity();
    }

    public double getCanCoderRad() {
      return canCoder.getAbsolutePosition().getValueAsDouble();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
