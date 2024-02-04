package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class TurnToAngle extends Command{
    
    private double m_targetAngle;

    /**
     * turn shooter to given angle
     * @param targetAngle - target angle in degrees
     */
    public TurnToAngle(double targetAngle){
        m_targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        //turn shooter to angle
        Shooter.getInstance().turnToAngle(m_targetAngle);
    }
}
