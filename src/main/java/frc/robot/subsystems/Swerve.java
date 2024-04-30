package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Swerve extends SubsystemBase {

    private final CANSparkMax mDrive;
    private final CANSparkMax mAngle;
    private final CANcoder encAngle;

    private final RelativeEncoder driveEnc;
    private final RelativeEncoder angleEnc;

    private final PIDController drivePidController;
    private final PIDController anglePIDController;
    private final SimpleMotorFeedforward driveFeedForward;
    private final SimpleMotorFeedforward angleFeedForward;

    private final ProfiledPIDController angleProfiledPIDController;

    private final TrapezoidProfile.Constraints angleTrapezoidProfile;

    private SwerveModuleState desiredState;

    private String swerveName;

    public Swerve(int driveMotorID, int angleMotorID, int angleMotorEncID, String swerveName) {

        mDrive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mAngle = new CANSparkMax(angleMotorEncID, MotorType.kBrushless);
        encAngle = new CANcoder(angleMotorEncID);

        driveEnc = mDrive.getEncoder();//TODO this will most likely cause an error
        angleEnc = mAngle.getEncoder();

        this.swerveName = swerveName;

        // Drive Control
        drivePidController = new PIDController(
                Constants.driveSwerveP,
                Constants.driveSwerveI,
                Constants.driveSwerveD);

        driveFeedForward = new SimpleMotorFeedforward(
                Constants.driveSwerveSFF,
                Constants.driveSwerveVFF,
                Constants.driveSwerveAFF);

        // Angle Control
        angleTrapezoidProfile = new TrapezoidProfile.Constraints(
                Constants.maxSwerveAngleSpeed,
                Constants.maxSwerveAngleAccel);

        angleProfiledPIDController = new ProfiledPIDController(
                Constants.angleSwerveP,
                Constants.angleSwerveI,
                Constants.angleSwerveD,
                angleTrapezoidProfile);

        anglePIDController = new PIDController(
                Constants.angleSwerveP,
                Constants.angleSwerveI,
                Constants.angleSwerveD);

        angleFeedForward = new SimpleMotorFeedforward(
                Constants.angleSwerveSFF,
                Constants.angleSwerveVFF,
                Constants.angleSwerveAFF);

        // Configure anglePID
        angleProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // init desired state
        desiredState = new SwerveModuleState();
    }

    public Rotation2d getCurrentAngle() {
        return new Rotation2d(encAngle.getAbsolutePosition().getValueAsDouble() * (2 * Math.PI));
    }

    public double getCurrentVelocity() {
        return (driveEnc.getVelocity()/* .getValueAsDouble()*/ / Constants.driveGearRatio) * Constants.wheelCirc;//TODO ?
    }

    public double getCurrentDrivePos() {
        return (driveEnc.getPosition()/* .getValueAsDouble()*/ / Constants.driveGearRatio * Constants.wheelCirc);//TODO ?
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getCurrentDrivePos(), getCurrentAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }

    @Override
    public void periodic() {
        var encoderRotation = getCurrentAngle();

        // Optimize
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        double curMetersPerSec = getCurrentVelocity();

        double driveOutput = drivePidController.calculate(
                curMetersPerSec,
                state.speedMetersPerSecond);

        final double outputDriveFF = driveFeedForward.calculate(state.speedMetersPerSecond);

        double turnOutput = angleProfiledPIDController.calculate(
                encoderRotation.getRadians(),
                state.angle.getRadians());

        double error = encoderRotation.getRadians() - state.angle.getRadians();
        double compError = 2 * Math.PI - (Math.abs(error));
        double compareError = Math.min(Math.abs(error), compError);
        double direction;
        double rampRate = 20;

        if (error < 0) {
            direction = -1;
        } else {
            direction = 1;
        }

        if (compareError == compError) {
            direction *= -1;
        }

        double finalError = compareError * direction;
        double angleVelocity = finalError * rampRate;
        double turnOutput0 = anglePIDController.calculate(angleVelocity);
        double outputAngleFF = angleFeedForward.calculate(angleVelocity);

        mDrive.setVoltage(driveOutput + outputAngleFF);
        mAngle.setVoltage(turnOutput0 + outputAngleFF);
    }

}
