package frc.robot.subsystems;

import org.ejml.dense.row.misc.TransposeAlgs_DDRM;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS;

public class Drive extends SubsystemBase {
    private static Drive drive = null;

    private final Translation2d frontLeftLocation;
    private final Translation2d frontRightLocation;
    private final Translation2d backLeftLocation;
    private final Translation2d backRightLocation;

    final Swerve frontLeft;
    private final Swerve frontRight;
    private final Swerve backLeft;
    private final Swerve backRight;

    private final AHRS navx;

    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final SwerveDriveKinematics kinematics;
    private Pose2d getPosition;

    private Drive() {
        frontLeftLocation = new Translation2d((Constants.robotLength / 2 - Constants.wheelInset),
                -(Constants.robotLength / 2 - Constants.wheelInset));
        frontRightLocation = new Translation2d((Constants.robotLength / 2 - Constants.wheelInset),
                (Constants.robotLength / 2 - Constants.wheelInset));
        backLeftLocation = new Translation2d(-(Constants.robotLength / 2 - Constants.wheelInset),
                -(Constants.robotLength / 2 - Constants.wheelInset));
        backRightLocation = new Translation2d(-(Constants.robotLength / 2 - Constants.wheelInset),
                (Constants.robotLength / 2 - Constants.wheelInset));

        kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        frontLeft = new Swerve(
                Constants.FLDriveM,
                Constants.FLAngleM,
                Constants.FLAngleEnc);

        frontRight = new Swerve(
                Constants.FRDriveM,
                Constants.FRAngleM,
                Constants.FRAngleEnc);

        backLeft = new Swerve(
                Constants.BLDriveM,
                Constants.BLAngleM,
                Constants.BLAngleEnc);

        backRight = new Swerve(
                Constants.BRDriveM,
                Constants.BRAngleM,
                Constants.BRAngleEnc);

        navx = new AHRS(SPI.Port.kMXP);
        navx.reset();

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()

                },
                new Pose2d(),
                VecBuilder.fill(1, 1, Units.degreesToRadians(5)),
                VecBuilder.fill(1, 1, Units.degreesToRadians(5)));// I don't know what this does
    }

    public void setSpeed(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, navx.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
        }

        speeds = ChassisSpeeds.discretize(speeds, Constants.updatePeriod);

        var swerveModuleState = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState, Constants.kMaxSpeed);

        frontLeft.setDesiredState(swerveModuleState[0]);
        frontRight.setDesiredState(swerveModuleState[1]);
        backLeft.setDesiredState(swerveModuleState[2]);
        backRight.setDesiredState(swerveModuleState[3]);
    }

    public void setVector(double speed, Rotation2d heading, double rSpeed) {
        double xSpeed = Math.cos(heading.getRadians()) * speed;
        double ySpeed = Math.sin(heading.getRadians()) * speed;
        setSpeed(xSpeed, ySpeed, rSpeed, false);
    }

    public Pose2d getEstimatedPos() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setVisionPos(Pose2d pose, double timeStamp) {
        getPosition = swerveDrivePoseEstimator.update(navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
        swerveDrivePoseEstimator.addVisionMeasurement(pose, timeStamp);
        
    }

    public static Drive getInstance() {
        if (drive == null) {
            drive = new Drive();
        }
        return drive;
    }
}
