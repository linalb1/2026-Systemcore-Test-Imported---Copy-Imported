// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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

	private PIDController pid = new PIDController(0.085, 0, 0);

	public Pivot() 
	{
		pid.disableContinuousInput();
		pid.setTolerance(2);
		
		motorL.configure(Configs.pivotL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		motorR.configure(Configs.pivotR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	
	private boolean hasReachedTarget = false;
	public boolean hasReachedTarget()
	{
		return hasReachedTarget;
	}

	//angle fron zero to horizontal
	private static final double offset = 32;

	public enum PivotState
	{
		START(90),
		TRANSITION(212),
		CONESCORE(45),
		CUBESCORE(45),
		CONEINTAKEFRONT(0),
		CONEINTAKEBACK(180),
		CUBEINTAKEFRONT(-30),
		CUBEINTAKEBACK(210);

		public double position;
		private PivotState(double position)
		{
			this.position = position + offset;
		}
	}
  	private PivotState state = PivotState.START;


  	private static final double kG = 0.47;
	private static double pidClamp = 8;
  
  @Override
	public void periodic() 
	{

		pid.setSetpoint(state.position);
		double pidVolts = pid.calculate(getPose() + offset);
		double ff = kG * Math.cos(Math.toRadians(getPose())) * RobotContainer.getLength()/32.64786;
		
		motorR.setVoltage(MathUtil.clamp(pidVolts+ff, -pidClamp,pidClamp));
		
		if(Math.abs(state.position-(getPose()+offset)) < 30)
			hasReachedTarget = true;
		else
			hasReachedTarget = false;

		pidClamp = RobotContainer.getElevatorPose() > 80? 3.5: 8;

		SmartDashboard.putNumber("Pivot/pivot encoder", motorL.getAbsoluteEncoder().getPosition());
		SmartDashboard.putNumber("Pivot/Pivot Pos", getPose());
		SmartDashboard.putNumber("Pivot/setpoint", pid.getSetpoint());
		SmartDashboard.putNumber("Pivot/pidvolts", pidVolts);
		SmartDashboard.putNumber("Pivot/ff", ff);
		SmartDashboard.putNumber("Pivot/output", ff+pidVolts); 
		SmartDashboard.putBoolean("Pivot/hasreached", hasReachedTarget);
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
		hasReachedTarget = false;
	}
	public void startState()
	{
		state = PivotState.START;
		hasReachedTarget = false;
	}
	public void scoreConeState()
	{
		state = PivotState.CONESCORE;
		hasReachedTarget = false;
	}
	public void cubeIntakeFront()
	{
		state = PivotState.CUBEINTAKEFRONT;
		hasReachedTarget = false;
	}
	public void cubeIntakeBack()
	{
		state = PivotState.CUBEINTAKEBACK;
		hasReachedTarget = false;
	}
	public void coneIntakeFront()
	{
		state = PivotState.CONEINTAKEFRONT;
		hasReachedTarget = false;
	}
	public void coneIntakeBack()
	{
		state = PivotState.CONEINTAKEBACK;
		hasReachedTarget = false;
	}
	public void coneScoreState()
	{
		state = PivotState.CONESCORE;
		hasReachedTarget = false;
	}
	public void cubeScoreState()
	{
		state = PivotState.CUBESCORE;
		hasReachedTarget = false;
	}
}
