package frc.robot.Commands.Swerve;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.SwervePoint;
import frc.robot.Utils.Vector2d;

public class DriveToPoint extends Command implements Consts{

    private SwervePoint m_target;
    private PIDController m_xPidController;
    private PIDController m_yPidController;
    private boolean m_isFinished;

    public DriveToPoint(SwervePoint target) {
        addRequirements(Swerve.getInstance(ChassisValues.USES_ABS_ENCODER));
        m_target = target;    
    }

    @Override
    public void initialize() {
        m_xPidController = new PIDController(PIDValues.POS_KP, PIDValues.POS_KI, PIDValues.POS_KD);
        m_yPidController = new PIDController(PIDValues.POS_KP, PIDValues.POS_KI, PIDValues.POS_KD);
        m_isFinished = false;
        SmartDashboard.putNumber("max drive speed", ChassisValues.AUTO_DRIVE_SPEED);
    }

    @Override
    public void execute() {

        // get current values
        double xCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getY();

        // calculate outputs
        double xOutput = MathUtil.clamp(m_xPidController.calculate(xCurrent, m_target.getX()),
                -ChassisValues.AUTO_DRIVE_SPEED,
                ChassisValues.AUTO_DRIVE_SPEED);
        double yOutput = MathUtil.clamp(m_yPidController.calculate(yCurrent, m_target.getY()),
                -ChassisValues.AUTO_DRIVE_SPEED,
                ChassisValues.AUTO_DRIVE_SPEED);

        // apply outputs
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveFieldOrientedAngle(new Vector2d(xOutput, yOutput));
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnToFieldOriented(m_target.getAngle());
    }

    @Override
    public boolean isFinished() {
           
        // get current state of robot
        double xCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getY();
        double headingCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getFieldOrientedAngle();
        return // check if entered x threshold
        Math.abs(m_target.getX() - xCurrent) < PIDValues.X_TOLERANCE &&
        // check if entered y threshold
                Math.abs(m_target.getY() - yCurrent) < PIDValues.Y_TOLERANCE &&
                // check if entered heading threshold
                (Funcs.modulo(m_target.getAngle() - Funcs.modulo(headingCurrent, 360), 360)) < PIDValues.HEADING_TOLERANCE;
            
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();
        SmartDashboard.putNumber("max drive speed", ChassisValues.DRIVE_SPEED);
    }

    public boolean getIsFinished(){
        return m_isFinished;
    }



    
}
