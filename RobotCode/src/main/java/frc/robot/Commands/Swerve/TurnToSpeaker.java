package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.Vector3d;

public class TurnToSpeaker extends TurnToPoint {

    public TurnToSpeaker(){
        super(new Vector2d(0,0));
        updateAlliance();
    }

    private void updateAlliance() {
        Vector3d speaker3d = new Vector3d();
        if (Robot.getAlliance() == Alliance.Blue) {
          speaker3d = ShooterValues.BLUE_SPAKER_POS;
        } 
        else if (Robot.getAlliance() == Alliance.Red) {
          speaker3d = ShooterValues.RED_SPAKER_POS;
        }
        Vector2d speaker2d = new Vector2d(speaker3d.m_x, speaker3d.m_z);
        this.setTarget(speaker2d);
    }

    @Override
    public void execute() {
        updateAlliance();
        super.execute();
    }
}
