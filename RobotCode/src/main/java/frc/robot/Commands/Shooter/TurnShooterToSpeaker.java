package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class TurnShooterToSpeaker extends Command{
    
    //turn the shooter to the correct angle to the speaker

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Shooter.getInstance().turnToAngle(Shooter.getInstance().getShooterAngleToSpeaker());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopAimMotor();
    }
}
