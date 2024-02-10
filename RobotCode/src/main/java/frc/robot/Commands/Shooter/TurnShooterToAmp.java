package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import frc.robot.Utils.Consts;

public class TurnShooterToAmp extends Command implements Consts{
      @Override
    public void initialize() {
        Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_AMP_ANGLE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
