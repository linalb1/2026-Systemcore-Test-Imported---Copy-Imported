// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Robot;
import frc.robot.Constants.Ports;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase {
  private SparkFlex motorL = new SparkFlex(Ports.pivotL, MotorType.kBrushless);
  private SparkFlex motorR = new SparkFlex(Ports.pivotR, MotorType.kBrushless);

  private SparkClosedLoopController controller = motorR.getClosedLoopController();
  private PIDController pid = new PIDController(2, 0, 0);

  public Pivot() 
  {
	pid.enableContinuousInput(-1, 1);
    motorL.configure(Configs.pivotL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorR.configure(Configs.pivotR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //angle fron zero to horizontal
  private static final double offset = 32;

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


  	private static final double kG = 0.47;
  
  @Override
	public void periodic() 
	{
		double pidVolts = pid.calculate(getPose()-offset, 45 + offset);
		double ff = -kG * Math.cos(Math.toRadians(getPose())) * RobotContainer.getLength()/32.64786;
		motorR.setVoltage(pidVolts+ff);
		SmartDashboard.putNumber("pivot encoder", motorL.getAbsoluteEncoder().getPosition());
		SmartDashboard.putNumber("Pivot Pos", getPose());
		SmartDashboard.putNumber("setpoint", pid.getSetpoint());
		SmartDashboard.putNumber("pidvolts", pidVolts);
		SmartDashboard.putNumber("ff", ff);
		SmartDashboard.putNumber("output", ff+pidVolts);

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
