package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANcoder canCoder;
    private final double canCoderOffsetRadians;

    private final PIDController rotationPidController;

    public SwerveModule(
        int driveMotorId,
        int rotationMotorId,
        int canCoderId,
        double canCoderOffsetRadians
    ) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        canCoder = new CANcoder(canCoderId);
        this.canCoderOffsetRadians = canCoderOffsetRadians;

        rotationPidController = new PIDController(SwerveConstants.kPTurning, 0, 0);

        // Position conversion factors (revolutions (gear ratio) to radians)
        driveEncoder.setPositionConversionFactor(2.0 * Math.PI / SwerveConstants.driveGearRatio);
        rotationEncoder.setPositionConversionFactor(2.0 * Math.PI / SwerveConstants.rotationGearRatio);
        
        // Velocity conversion factors (rpm to m/s)
        driveEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60 / SwerveConstants.driveGearRatio);
        rotationEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60 / SwerveConstants.rotationGearRatio);

        // tells pidcontroller that -pi is the same as +pi
        // can calculate shorter path to setpoint
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

        // Makes the range of the sensor 0-1 so that radians can be calculated
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        
        // Makes turning ccw positive
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        // Apply cancoder configuration
        canCoder.getConfigurator().apply(canCoderConfig);
    }

    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    public Rotation2d getRotationEncoderPosition() {
        double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

        if (unsignedAngle < 0) unsignedAngle += 2 * Math.PI;

        return new Rotation2d(unsignedAngle);
    }

    public CANcoder getCANcoder() {
        return canCoder;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getRotationVelocity() {
        return rotationEncoder.getVelocity();
    }

    public CANSparkMax getDriveMotor() {
        return driveMotor;
    }

    public CANSparkMax getRotationMotor() {
        return rotationMotor;
    }

    // Gets actual amount of rotation from each motor by getting cancoder position offset (from center)
    // and subtracting from current rotation
    public Rotation2d getCANcoderRad() {
        double canCoderRad = (Math.PI * 2 * canCoder.getAbsolutePosition().getValueAsDouble()) - canCoderOffsetRadians % (2 * Math.PI);
        return new Rotation2d(canCoderRad);
    }

    // Resets encoders and makes the rotation motor equal to the offset
    // so that we account for the offset
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(getCANcoderRad().getRadians());
    }

    // Takes in velocity and angle which calculates how much it needs to turn and apply forward motion.
    // This will come in use throughout other code
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getCANcoderRad());
    }

    // Actually applies a SwerveModuleState, but uses scaling rather than PID for the drive motor
    public void setDesiredStates(SwerveModuleState state) {
        // Optimize finds the closest angle to the target
        state = SwerveModuleState.optimize(state, getRotationEncoderPosition());

        // TODO: convert this to PID
        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxMetersPerSecond);

        // use PID for turning to avoid overshooting
        rotationMotor.set(rotationPidController.calculate(getRotationEncoderPosition().getRadians(), state.angle.getRadians()));
    }

    @Override
    public void periodic() {
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}


