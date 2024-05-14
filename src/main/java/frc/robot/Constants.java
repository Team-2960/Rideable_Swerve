package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    // Robot constants
    public final static double updatePeriod = 0.02;// in seconds
    // Motor IDs TODO motor IDs
    public static int FLDriveM = 2;
    public static int FLAngleM = 1;
    public static int FLAngleEnc = 9;

    public static int FRDriveM = 4;
    public static int FRAngleM = 3;
    public static int FRAngleEnc = 12;

    public static int BLDriveM = 6;
    public static int BLAngleM = 5;
    public static int BLAngleEnc = 10;

    public static int BRDriveM = 8;
    public static int BRAngleM = 7;
    public static int BRAngleEnc = 11;
    // PID/FF values TODO PID values
    public static double driveSwerveP = .2;
    public static double driveSwerveI = 0;
    public static double driveSwerveD = 0;

    public static double angleSwerveP = .2;
    public static double angleSwerveI = 0;
    public static double angleSwerveD = 0;

    public static double driveSwerveSFF = 0;
    public static double driveSwerveVFF = 0;
    public static double driveSwerveAFF = 0;

    public static double angleSwerveSFF = 0;
    public static double angleSwerveVFF = 0;
    public static double angleSwerveAFF = 0;
    // Speeds
    public static double maxSwerveAngleSpeed = 2;
    public static double maxSwerveAngleAccel = 2;

    public final static double kMaxSpeed = 5;
    public final static double kMaxAngularSpeed = 3;

    // Gear Ratios
    public static double driveGearRatio = 6.86;// TODO change this to gear ratio
    public static double wheelCirc = 12.56637;

    //wheel position
    public static double robotLength = 0.7493;
    public static double wheelInset = 0.0873125;

    //drive
    public static final Rotation2d swerveAngleRampDist = Rotation2d.fromDegrees(30);

}
