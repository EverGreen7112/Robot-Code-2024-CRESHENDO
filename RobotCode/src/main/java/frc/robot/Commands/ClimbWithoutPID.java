package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Climber.ClimberSide;
import frc.robot.Utils.Consts;

public class ClimbWithoutPID extends Command implements Consts {

    private ClimberSide m_climbSide;
    private double m_speed;
    
    public ClimbWithoutPID(double speed, ClimberSide climberSide){
        this.m_speed = speed;
        this.m_climbSide = climberSide;
    }

    @Override
    public void initialize(){
        switch(m_climbSide){
            case CLIMB_WITH_RIGHT_SIDE:
                Climber.getInstance().climbRightSideWithoutPid(MathUtil.applyDeadband(m_speed, JoystickValues.JOYSTICK_DEADZONE));
                break;
            case CLIMB_WITH_LEFT_SIDE:
                Climber.getInstance().climbLeftSideWithoutPid(MathUtil.applyDeadband(m_speed, JoystickValues.JOYSTICK_DEADZONE));
                break;
            case CLIMB_WITH_BOTH_SIDES:
                Climber.getInstance().climbBothSidesWithoutPid(MathUtil.applyDeadband(m_speed, JoystickValues.JOYSTICK_DEADZONE));
                break;
        }
    } 
    
    @Override
    public void execute(){
         
    } 

    @Override
    public boolean isFinished(){
        return false;
    } 

    @Override
    public void end(boolean interrupted){
        Climber.getInstance().stopClimberMotors();
    } 
}