package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase {
    // instance
    public static OI oi = null;

    // joysticks
    private static Joystick driverController;

    // create OI
    public static OI getOI() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }

    private OI() {
        driverController = new Joystick(0);
    }

    @Override
    public void periodic() {
        if (DriverStation.isTeleop()) {
            boolean fieldRelative = false;

            double xSpeed = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.1) * Constants.kMaxSpeed;
            double ySpeed = -MathUtil.applyDeadband(driverController.getRawAxis(1), 0.1) * Constants.kMaxSpeed;
            double rSpeed = MathUtil.applyDeadband(driverController.getRawAxis(2), 0.1) * Constants.kMaxAngularSpeed;

            Drive.getInstance().setSpeed(xSpeed, ySpeed, rSpeed, fieldRelative);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("rSpeed", rSpeed);
            SmartDashboard.putBoolean("FieldRelative", fieldRelative);
        }
    }

    public static OI getInstance() {
        if (oi == null) {
            oi = new OI();
        }

        return oi;
    }

}
