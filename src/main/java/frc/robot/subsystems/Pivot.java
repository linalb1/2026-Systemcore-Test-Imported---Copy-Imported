// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Ports;

public class Pivot extends SubsystemBase {
  private SparkFlex motorL = new SparkFlex(Ports.pivotL, MotorType.kBrushless);
  private SparkFlex motorR = new SparkFlex(Ports.pivotR, MotorType.kBrushless);

  private ArmFeedforward ff = new ArmFeedforward(0, 0.1, 0);
  private SparkClosedLoopController controller = motorL.getClosedLoopController();

  public Pivot() 
  {
    motorL.configure(Configs.pivotL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorR.configure(Configs.pivotR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //angle fron zero to horizontal
  private static final double offset = 24;

  public enum PivotState
  {
    TRANSITION(90),
    CONESCORE(60),
    CUBESCORE(60),
    CONEINTAKEFRONT(-45),
    CONEINTAKEBACK(225),
    CUBEINTAKEFRONT(-45),
    CUBEINTAKEBACK(225);

    public double position;
    private PivotState(double position)
    {
      this.position = position + offset;
    } 
  }
  private PivotState state = PivotState.TRANSITION;

  @Override
	public void periodic() 
	{
		controller.setSetpoint(state.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff.calculate(Math.toRadians(motorL.getAbsoluteEncoder().getPosition() -offset), 0));

		SmartDashboard.putNumber("pivot pos", motorL.getAbsoluteEncoder().getPosition());
	}

  public double getState()
  {
    return state.position;
  }

  public double getPose()
  {
    return motorL.getAbsoluteEncoder().getPosition()-offset;
  }

  public void transitionState()
  {
    state = PivotState.TRANSITION;
  }
  public void scoreConeState()
  {
    state = PivotState.CONESCORE;
  }
  public void cubeIntakeFront()
  {
    state = PivotState.CUBEINTAKEFRONT;
  }
  public void cubeIntakeBack()
  {
    state = PivotState.CUBEINTAKEBACK;
  }
  public void coneIntakeFront()
  {
    state = PivotState.CONEINTAKEFRONT;
  }
  public void coneIntakeBack()
  {
    state = PivotState.CONEINTAKEBACK;
  }
  public void coneScoreState()
  {
    state = PivotState.CONESCORE;
  }
  public void cubeScoreState()
  {
    state = PivotState.CUBESCORE;
  }
}
