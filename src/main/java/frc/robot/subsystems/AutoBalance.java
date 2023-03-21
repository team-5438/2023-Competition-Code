package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;

public class AutoBalance extends CommandBase 
{
    private drivetrain drive;
    private double speed = 0.3;
    private int OffBalance = 7;

    public AutoBalance(drivetrain m_drive)
    {
        m_drive = drive;
    }

    @Override
    public void initialize()
    {
        drive.arcadeDrive(0, 0);
    }

    @Override
    public void execute()
    {
        speed = drive.getPitch() / 150;

        if (drive.getPitch() >= OffBalance)
        {
            drive.arcadeDrive(speed, 0);
        }
        else if (drive.getPitch() <= OffBalance)
        {
            drive.arcadeDrive(speed, 0);
        }
        else 
        {
            drive.arcadeDrive(0, 0);
        }
    }
    
    public void end()
    {
        drive.arcadeDrive(0, 0);
    }

    public boolean Finish()
    {
        return false;
    }
}
