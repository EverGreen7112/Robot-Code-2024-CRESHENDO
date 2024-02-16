package frc.robot.Commands;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Climber.ClimberSide;
import frc.robot.Utils.Consts;

public class ClimbWithPID extends Command implements Consts {

    private double rpm;
    private ClimberSide ClimbSide;

    public ClimbWithPID(double rpm, ClimberSide climberSide){
        this.rpm = rpm;
        this.ClimbSide = climberSide;
    }

    @Override
    public void initialize(){
        addRequirements(Climber.getInstance());
        switch(ClimbSide){
            case CLIMB_WITH_RIGHT_SIDE:
                Climber.getInstance().climbRightWithPID(rpm);
                break;
            case CLIMB_WITH_LEFT_SIDE:
                Climber.getInstance().climbLeftWithPID(rpm);
                break;
            case CLIMB_WITH_BOTH_SIDES:
                Climber.getInstance().climbBothWithPID(rpm);
                break;
        }
    } 
    
    @Override
    public void execute(){} 

    @Override
    public boolean isFinished(){
        return false;
    } 

    @Override
    public void end(boolean interrupted){
        Climber.getInstance().stopClimberMotors();
    } 
}
