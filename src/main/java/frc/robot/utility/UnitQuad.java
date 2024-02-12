// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Stole from 2023 */
public class UnitQuad {
    public static final double calculate (double value) {
        return Math.copySign(value * value, value);
    }
}