package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.Vector3d;

public class TurnToSpeakerAuto extends TurnToPoint {

    private Vector2d m_speaker2d;
    public TurnToSpeakerAuto(){
        super(new Vector2d(0,0));
        updateAlliance();
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    private void updateAlliance() {
        Vector3d speaker3d = new Vector3d();
        if (Robot.getAlliance() == Alliance.Blue) {
          speaker3d = ShooterValues.BLUE_SPAKER_POS;
        } 
        else if (Robot.getAlliance() == Alliance.Red) {
          speaker3d = ShooterValues.RED_SPAKER_POS;
        }
        m_speaker2d = new Vector2d(speaker3d.m_x, speaker3d.m_z);
        this.setTarget(m_speaker2d);
    }

    @Override
    public void execute() {
        updateAlliance();
        super.execute();
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0,0), true);
        SmartDashboard.putNumber("heading target", Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getHeadingTargetAngle());
    }
    @Override
    public boolean isFinished() {
        execute();
        return Math.abs(Funcs.modulo(Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getGyro().getAngle(), 360)
         - Funcs.modulo(Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getHeadingTargetAngle(), 360))< 2;
    }
}
