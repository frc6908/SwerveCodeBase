package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase {
    private final SparkMax driveMotor;
    private final SparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANcoder canCoder;
    private final double canCoderOffsetRadians;
    
    // PID
    private final PIDController rotationPIDController;

    /* CREATE SWERVE MODULE */
    public SwerveModule(
      int driveMotorID,
      int rotationMotorID,
      int canCoderID,
      double canCoderOffsetRadians,
      boolean isDriveInverted
    ) {
      // motor controllers + configuration
      driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
      rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);
      configureMotor(driveMotor, 
                      isDriveInverted, 
                      IdleMode.kBrake,
                      DrivetrainConstants.drivePositionConversionFactor, 
                      DrivetrainConstants.driveVelocityConversionFactor
                    );
      configureMotor(rotationMotor, 
                      isDriveInverted, 
                      IdleMode.kBrake,
                      DrivetrainConstants.rotationPositionConversionFactor, 
                      DrivetrainConstants.rotationVelocityConversionFactor
                    );
      // PID
      rotationPIDController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
      rotationPIDController.setTolerance(DrivetrainConstants.kTolerance);
      rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

      // Initializations
      canCoder = new CANcoder(canCoderID);
      this.canCoderOffsetRadians = canCoderOffsetRadians;
      driveEncoder = driveMotor.getEncoder();
      rotationEncoder = rotationMotor.getEncoder();
    }

    public void configureMotor(
      SparkMax motor,
      boolean isInverted,
      IdleMode idleMode,
      double positionConversionFactor,
      double velocityConversionFactor
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
          .inverted(isInverted)
          .idleMode(idleMode);
        config.encoder
          .positionConversionFactor(positionConversionFactor)
          .velocityConversionFactor(velocityConversionFactor);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // get current position of canCoder
    public double getCANCoderRad() {
      double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
      double angle = (2 * Math.PI * absolutePosition) - canCoderOffsetRadians;
      return angle % (2 * Math.PI);
    }

    // need to reset encoders upon startup
    public void resetEncoder() {
      driveEncoder.setPosition(0);
      rotationEncoder.setPosition(0);
    }

    // return current drive velocity
    public double getDriveVelocity(){
      return driveEncoder.getVelocity();
    }

    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCANCoderRad()));
    }

    // stops the robot
    public void stop(){
      driveMotor.stopMotor();
      rotationMotor.stopMotor();
    }

    // opitimize function
    // helps prevent motors from turning more than 90 degrees
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90){
        return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
      }
      else{
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    // sets the desired robot state
    public void setState(SwerveModuleState state){
      if (Math.abs(state.speedMetersPerSecond) < 0.0001){
        stop();
        return;
      }

      state = optimize(state, getState().angle);
      // set rotation motor with PID controller
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), state.angle.getRadians()));
      // set drive motor speed
      driveMotor.set(state.speedMetersPerSecond/DrivetrainConstants.maxVelocity);
    }

    public double getDrivePosition() {
      return driveEncoder.getPosition();
    }
  
    public double getRotationPosition() {
      return rotationEncoder.getPosition();
    }
  
    public void initRotationOffset() {
      rotationEncoder.setPosition(getCANCoderRad());
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

  //   /**
  //  * Example command factory method.
  //  *
  //  * @return a command
  //  */
  // public Command exampleMethodCommand() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         /* one-time action goes here */
  //       });
  // }
  // 
  // /**
  //  * An example method querying a boolean state of the subsystem (for example, a digital sensor).
  //  *
  //  * @return value of some boolean subsystem state, such as a digital sensor.
  //  */
  // public boolean exampleCondition() {
  //   // Query some boolean state, such as a digital sensor.
  //   return false;
  // }

  // @Override
  // public void simulationPeriodic() {
  //   // This method will be called once per scheduler run during simulation
  // }
}
