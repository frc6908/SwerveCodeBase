package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class InitialSwerveSubsystem extends SubsystemBase {
    
    // SwerveModule frontLeftModule = new SwerveModule(SwerveConstants.kFLDrive, SwerveConstants.kFLAngle, SwerveConstants.kFLCanCoder);
    // SwerveModule frontRightModule = new SwerveModule(SwerveConstants.kFRDrive, SwerveConstants.kFRAngle, SwerveConstants.kFRCanCoder);
    // SwerveModule backLeftModule = new SwerveModule(SwerveConstants.kBLDrive, SwerveConstants.kBLAngle, SwerveConstants.kBLCanCoder);
    // SwerveModule backRightModule = new SwerveModule(SwerveConstants.kBRDrive, SwerveConstants.kBRAngle, SwerveConstants.kBRCanCoder);

    // public SwerveSubsystem() {

    //     // Front left Module Construction
    // }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            /* one-time action goes here */
            });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // FL FR BL BR
        double loggingState[] = {
            // frontLeftModule.getState().angle.getDegrees(),
            // frontLeftModule.getState().speedMetersPerSecond,
            // frontRightModule.getState().angle.getDegrees(),
            // frontRightModule.getState().speedMetersPerSecond,
            // backLeftModule.getState().angle.getDegrees(),
            // backLeftModule.getState().speedMetersPerSecond,
            // backRightModule.getState().angle.getDegrees(),
            // backRightModule.getState().speedMetersPerSecond
            0, 0,0,0,0,0,0,0
        };
        // sending data to smart dashboard
        SmartDashboard.putNumberArray("Swerve Module States", loggingState);
        System.out.println("wegflueiwbhfjewf");
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
