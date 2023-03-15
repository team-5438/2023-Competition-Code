package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class PIDSendable implements Sendable
{
  private final PIDController pid;
  String name;

  public PIDSendable(String name, PIDController pid)
  {
    this.pid = pid;
    this.name = name;
    SendableRegistry.addChild(this, pid);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.setActuator(true);
    builder.setSafeState(() -> pid.reset());
    builder.addDoubleProperty("p", pid::getP, pid::setP);
    builder.addDoubleProperty("i", pid::getI, pid::setI);
    builder.addDoubleProperty("d", pid::getD, pid::setD);
    builder.addDoubleProperty("setpoint", pid::getSetpoint, pid::setSetpoint);
    builder.addDoubleProperty("error", pid::getError, pid::setError);
    builder.addDoubleProperty("result", pid::get, null);
    builder.addStringProperty(".name", () -> name, null);
  }
}