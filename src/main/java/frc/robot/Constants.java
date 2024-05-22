package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    // Robot constants
    public final static double updatePeriod = 0.02;// in seconds
    // Motor IDs TODO motor IDs
    public static int FLDriveM = 2;
    public static int FLAngleM = 1;
    public static int FLAngleEnc = 9;

    public static int FRDriveM = 8;
    public static int FRAngleM = 7;
    public static int FRAngleEnc = 12;

    public static int BLDriveM = 4;
    public static int BLAngleM = 3;
    public static int BLAngleEnc = 11;

    public static int BRDriveM = 6;
    public static int BRAngleM = 5;
    public static int BRAngleEnc = 10;
    // PID/FF values TODO PID values
    public static double driveSwerveP = 0;//.000014912;
    public static double driveSwerveI = 0;
    public static double driveSwerveD = 0;

    public static double angleSwerveP = 0.01;//0.00000017;
    public static double angleSwerveI = 0;
    public static double angleSwerveD = 0;//0.001;

    public static double driveSwerveSFF = 0;
    public static double driveSwerveVFF = .95;
    public static double driveSwerveAFF = 0;

    public static double angleSwerveSFF = 0.01;
    public static double angleSwerveVFF = 0.55;
    public static double angleSwerveAFF = 0;
    // Speeds
    public static final double maxSwerveAngleSpeed = 2 * Math.PI;
    public static final double maxSwerveAngleAccel = 2 * Math.PI;

    public final static double kMaxSpeed = 4.5;
    public final static double kMaxAngularSpeed = 2 * 2 * Math.PI;

    // Gear Ratios
    public static double driveGearRatio = 6.86;// TODO change this to gear ratio
    public static double wheelCirc = 12.56637;

    //wheel position
    public static double robotLength = 0.7493;
    public static double wheelInset = 0.0873125;

    //drive
    public static final Rotation2d swerveAngleRampDist = Rotation2d.fromDegrees(30);

}
