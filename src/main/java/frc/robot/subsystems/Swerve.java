package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Swerve extends SubsystemBase {

    private final CANSparkMax mDrive;
    private final CANSparkMax mAngle;
    private final CANcoder encAngle;

    private final RelativeEncoder encDrive;

    private final PIDController drivePidController;
    private final PIDController anglePIDController;

    private final SimpleMotorFeedforward driveFeedForward;
    private final SimpleMotorFeedforward angleFeedForward;

    private final ProfiledPIDController angleProfiledPIDController;

    private final TrapezoidProfile.Constraints angleTrapezoidProfile;

    private SwerveModuleState desiredState;

    public Swerve(int driveMotorID, int angleMotorID, int angleMotorEncID) {

        // init motors
        mDrive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mAngle = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        encAngle = new CANcoder(angleMotorEncID);//angle and drive encoders may be swapped

        encDrive = mDrive.getEncoder();

        // Drive rate Control
        drivePidController = new PIDController(
                Constants.driveSwerveP,
                Constants.driveSwerveI,
                Constants.driveSwerveD);

        driveFeedForward = new SimpleMotorFeedforward(
                Constants.driveSwerveSFF,
                Constants.driveSwerveVFF,
                Constants.driveSwerveAFF);

        // Angle position Control

        // angleTrapezoidProfile = new TrapezoidProfile.Constraints(
        //         Constants.maxSwerveAngleSpeed, // Look at 2024 Fitz code
        //         Constants.maxSwerveAngleAccel);

        // angleProfiledPIDController = new ProfiledPIDController(
        //         Constants.angleSwerveP,
        //         Constants.angleSwerveI,
        //         Constants.angleSwerveD,
        //         angleTrapezoidProfile);

        anglePIDController = new PIDController(
                Constants.angleSwerveP,
                Constants.angleSwerveI,
                Constants.angleSwerveD);

        angleFeedForward = new SimpleMotorFeedforward(
                Constants.angleSwerveSFF,
                Constants.angleSwerveVFF,
                Constants.angleSwerveAFF);

        // Configure anglePID
        // angleProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // init desired state
        desiredState = new SwerveModuleState();
    }

    /**
     * Gets the current module angle
     * 
     * @return current swerve module angle
     */
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(encAngle.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAnglePos(){
        return Rotation2d.fromRadians(encAngle.getPosition());
    }

    /**
     * Gets the current drive speed
     * 
     * @return current swerve module drive speed
     */
    public double getCurrentVelocity() {
        return (encDrive.getVelocity() / Constants.driveGearRatio) * Constants.wheelCirc;
    }

    /**
     * Gets the current drive distance
     * 
     * @return current swerve module drive distance
     */
    public double getCurrentDrivePos() {
        return (encDrive.getPosition() / Constants.driveGearRatio * Constants.wheelCirc);
    }

    /**
     * Gets the current swerve module positions
     * 
     * @return current swerve module positions
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getCurrentDrivePos(), getCurrentAngle());
    }

    /**
     * Sets the desired swerve module state
     * 
     * @param desiredState desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }

    /**
     * Subsystem period update
     */
    @Override
    public void periodic() {
        var encoderRotation = getCurrentAngle();

        // Optimize
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        updateDrive(state);
        updateAngle(state);
        updateUI();
    }

    /**
     * Updates the drive rate controllers
     * 
     * @param state Swerve module state
     */
    private void updateDrive(SwerveModuleState state){
        double pidOutput = drivePidController.calculate(
                getCurrentVelocity(),
                state.speedMetersPerSecond);

        double ffOutput = driveFeedForward.calculate(state.speedMetersPerSecond);

        mDrive.setVoltage(pidOutput * ffOutput);
    }

    private void updateAngle(SwerveModuleState state){
        //Get current module angle
        Rotation2d encoderRotation = getAnglePos();

        //Calculate target rate
        double error = encoderRotation.getRadians() - state.angle.getRadians();
        double compError = 2 * Math.PI - (Math.abs(error));
        double compareError = Math.min(Math.abs(error), compError);
        double direction = error > 0 ? 1 : -1;
        double rampRate = 20;

        if (compareError == compError) {
            direction *= -1;
        }

        double targetRate = Math.min(1.0, compareError / Constants.swerveAngleRampDist.getRadians())
    }
        double curMetersPerSec = getCurrentVelocity();

        

        final double outputDriveFF = driveFeedForward.calculate(state.speedMetersPerSecond);

        double turnOutput = angleProfiledPIDController.calculate(
                encoderRotation.getRadians(),
                state.angle.getRadians());

        

        double finalError = compareError * direction;
        double angleVelocity = finalError * rampRate;
        double turnOutput0 = anglePIDController.calculate(angleVelocity);
        double outputAngleFF = angleFeedForward.calculate(angleVelocity);

        mDrive.setVoltage(driveOutput + outputAngleFF);
        mAngle.setVoltage(turnOutput0 + outputAngleFF);

        SmartDashboard.putNumber("driveVelocity", encDrive.getVelocity());
        SmartDashboard.putNumber("state.SpeedMeters/second", state.speedMetersPerSecond);
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("Desired Angle", state.angle.getRadians());
        SmartDashboard.putNumber("Current Angle", encoderRotation.getRadians());
    }

}
