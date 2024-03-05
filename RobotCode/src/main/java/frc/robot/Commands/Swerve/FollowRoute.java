package frc.robot.Commands.Swerve;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.SwervePoint;
import frc.robot.Utils.Vector2d;

public class FollowRoute extends Command implements Consts {

    private PIDController m_xPidController;
    private PIDController m_yPidController;
    private ArrayList<SwervePoint> m_posList;
    private int current;
    private boolean m_isFinished;
    public FollowRoute(ArrayList<SwervePoint> posList) {
        addRequirements(Swerve.getInstance(ChassisValues.USES_ABS_ENCODER));
        m_posList = posList;
    }

    @Override
    public void initialize() {
        m_xPidController = new PIDController(PIDValues.POS_KP, PIDValues.POS_KI, PIDValues.POS_KD);
        m_yPidController = new PIDController(PIDValues.POS_KP, PIDValues.POS_KI, PIDValues.POS_KD);
        current = 0;
        m_isFinished = false;
    }

    @Override
    public void execute() {
        if (m_posList.isEmpty())
            return;
        // get current values
        double xCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getY();

        // calculate outputs
        double xOutput = MathUtil.clamp(m_xPidController.calculate(xCurrent, m_posList.get(current).getX()),
                -Consts.ChassisValues.MAX_AUTO_DRIVE_SPEED,
                Consts.ChassisValues.MAX_AUTO_DRIVE_SPEED);
        double yOutput = MathUtil.clamp(m_yPidController.calculate(yCurrent, m_posList.get(current).getY()),
                -Consts.ChassisValues.MAX_AUTO_DRIVE_SPEED,
                Consts.ChassisValues.MAX_AUTO_DRIVE_SPEED);

        // apply outputs
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveFieldOrientedAngle(new Vector2d(xOutput, yOutput));
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnToFieldOriented(m_posList.get(current).getAngle());
    }

    @Override
    public boolean isFinished() {
        if (m_posList.isEmpty())
            return true;
        // get current state of robot
        double xCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getX();
        double yCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getY();
        double headingCurrent = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getFieldOrientedAngle();
        if (( // check if entered x threshold
        Math.abs(m_posList.get(current).getX() - xCurrent) < PIDValues.X_TOLERANCE &&
        // check if entered y threshold
                Math.abs(m_posList.get(current).getY() - yCurrent) < PIDValues.Y_TOLERANCE &&
                // check if entered heading threshold
                (Math.abs(m_posList.get(current).getAngle() - headingCurrent) % 360) < PIDValues.HEADING_TOLERANCE)) {
            current++;
        }
        m_isFinished = current >= m_posList.size();
        return current >= m_posList.size();
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();
        current = 0;
    }

    public boolean getIsFinished(){
        return m_isFinished;
    }
}
