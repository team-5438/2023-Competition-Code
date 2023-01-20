/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
	private final DoubleSupplier m_fwd, m_rot;
	private final drivetrain m_Drivetrain;

	/**
	 * Creates a new DefaultDrive.
	 */
	public DefaultDrive(drivetrain drivetrain, DoubleSupplier fwdValue, DoubleSupplier rotValue) {
		// define requirements
		m_Drivetrain = drivetrain;
		m_fwd = fwdValue;
		m_rot = rotValue;
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// m_Drivetrain.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble());
		m_Drivetrain.curvatureDrive(m_fwd.getAsDouble(), m_rot.getAsDouble(), true);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}