// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;

public class Claw extends SubsystemBase {
  private SparkFlex tilt = new SparkFlex(Ports.tilt, MotorType.kBrushless);
  private SparkFlex claw = new SparkFlex(Ports.claw, MotorType.kBrushless);

  private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
  private SparkClosedLoopController controller = tilt.getClosedLoopController();

  public Claw() 
  {
    tilt.configure(Configs.tilt, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    claw.configure(Configs.claw, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public enum TiltState
  {
    TRANSITION(90),
    CUBEINTAKEFRONT(100),
    CUBEINTAKEBACK(100),
    CONEINTAKEFRONT(150),
    CONEINTAKEBACK(150),
    CUBESCORE(20),
    CONESCORE(20);

    public double position;
    private TiltState(double position)
    {
      this.position = position;
    } 
  }
  private TiltState state = TiltState.TRANSITION;

  @Override
	public void periodic() 
	{
		controller.setSetpoint(
      state.position, 
      ControlType.kPosition, ClosedLoopSlot.kSlot0, 
      ff.calculate(Math.toRadians(tilt.getAbsoluteEncoder().getPosition() - RobotContainer.getPivotPose()),
      0));

		SmartDashboard.putNumber("tilt pos",tilt.getAbsoluteEncoder().getPosition());
	}

  public void intake()
  {
    claw.set(0.5);
  }

  public void outtake()
  {
    claw.set(-0.5);
  }

  public void stop()
  {
    claw.stopMotor();
  }

  public void transitionState()
  {
    state = TiltState.TRANSITION;
    claw.stopMotor();
  }
  public void scoreConeState()
  {
    state = TiltState.CONESCORE;
  }
  public void cubeIntakeFront()
  {
    state = TiltState.CUBEINTAKEFRONT;
  }
  public void cubeIntakeBack()
  {
    state = TiltState.CUBEINTAKEBACK;
  }
  public void coneIntakeFront()
  {
    state = TiltState.CONEINTAKEFRONT;
  }
  public void coneIntakeBack()
  {
    state = TiltState.CONEINTAKEBACK;
  }
  public void coneScoreState()
  {
    state = TiltState.CONESCORE;
  }
  public void cubeScoreState()
  {
    state = TiltState.CUBESCORE;
  }
}
