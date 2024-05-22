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

    public final String name;

    private SwerveModuleState desiredState;

    public Swerve(int driveMotorID, int angleMotorID, int angleMotorEncID, String name) {

        // init motors
        mDrive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mAngle = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        encDrive = mDrive.getEncoder();

        encAngle = new CANcoder(angleMotorEncID);

        this.name = name;

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
        // Constants.maxSwerveAngleSpeed, // Look at 2024 Fitz code
        // Constants.maxSwerveAngleAccel);

        // angleProfiledPIDController = new ProfiledPIDController(
        // Constants.angleSwerveP,
        // Constants.angleSwerveI,
        // Constants.angleSwerveD,
        // angleTrapezoidProfile);

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
    public Rotation2d getAnglePos() {
        return Rotation2d.fromRotations(encAngle.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * gets the current swerve module angle rate
     * 
     * @return angle rate
     */
    public double getAngleRate() {
        return encAngle.getVelocity().getValueAsDouble();
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
     * Gets the current module state
     * 
     * @return current swerve module positions
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getCurrentVelocity(), getAnglePos());
    }

    /**
     * Gets the current swerve module positions
     * 
     * @return current swerve module positions
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getCurrentDrivePos(), getAnglePos());
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

        // Optimize
        //SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAnglePos());

        updateDrive(desiredState);
        updateAngle(desiredState);
    }

    /**
     * Updates the drive rate controllers
     * 
     * @param state Swerve module state
     */
    private void updateDrive(SwerveModuleState state) {
        double pidOutput = drivePidController.calculate(
                getCurrentVelocity(),
                state.speedMetersPerSecond);

        double ffOutput = driveFeedForward.calculate(state.speedMetersPerSecond);

        //mDrive.setVoltage(0);// TODO remove
        mDrive.setVoltage(pidOutput + ffOutput);
    }

    /**
     * Updates the angle position and the rate controllers
     * 
     * @param state
     */
    private void updateAngle(SwerveModuleState state) {
        // Get current module angle
        Rotation2d encoderRotation = getAnglePos();

        // Calculate target rate
        double error = encoderRotation.getRadians() - state.angle.getRadians();
        double compError = 2 * Math.PI - (Math.abs(error));
        double compareError = Math.min(Math.abs(error), Math.abs(compError));
        double direction = error > 0 ? 1 : -1;

        if (compareError == Math.abs(compError)) {
            direction *= -1;
        }

        double targetRate = Math.min(.9, compareError / Constants.swerveAngleRampDist.getRadians())
                * Constants.maxSwerveAngleSpeed;
        double angleVelocity = targetRate * direction;

        // Calculate motor output
        double pidOutput = anglePIDController.calculate(getAngleRate(), angleVelocity);
        double ffOutput = angleFeedForward.calculate(angleVelocity);

        // set motor output
        //mAngle.setVoltage(0);
        mAngle.setVoltage(-(pidOutput + ffOutput));

        SmartDashboard.putNumber(name + " driveVelocity", encAngle.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(name + " AngleVelocity", angleVelocity);
        SmartDashboard.putNumber(name + " state.SpeedMeters/second", state.speedMetersPerSecond);
        SmartDashboard.putNumber(name + " Error", error);
        SmartDashboard.putNumber(name + " compError", compError);
        SmartDashboard.putNumber(name + " compareError", compareError);
        SmartDashboard.putNumber(name + " Desired Angle", state.angle.getDegrees());
        SmartDashboard.putNumber(name + " Current Angle", encoderRotation.getDegrees());
        SmartDashboard.putNumber(name + " RampDistance", Constants.swerveAngleRampDist.getDegrees());
    }

}
