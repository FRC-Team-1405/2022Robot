// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;

// FeatureToggle.<enum>.isEnabled
// i.e. FeatureToggle.FalseFlag.isEnabled()
//
// Example flags:
//  FalseFlag( false )
//
//  TrueFlag( true )
//
//  PrefenceFlag( "PrefFlag")
//
//  DigitalFlag( 0 )
//
//  ComplexFlag( () -> {
//     return (Math.random() > 0.4) ? true : false;
//  })

public enum FeatureToggle {
    // initialize Flags with a boolean or BooleanSupplier
    DeadStop( "DeadStop", true )
    ;

    public boolean isEnabled(){
        return enabled;
    }

    private boolean enabled;
    private FeatureToggle(BooleanSupplier enabled){
        this.enabled = enabled.getAsBoolean();
    }
    private FeatureToggle(boolean enabled){
        this.enabled = enabled;
    }
    private FeatureToggle(String key){
        this(key, false);
    }
    private FeatureToggle(String key, boolean value){
        key = String.format("FeatureToggle/%s", key);
        Preferences.initBoolean(key, value);
        this.enabled = Preferences.getBoolean(key, value);
    }
    private FeatureToggle(int digitialChannel){
        DigitalInput input = new DigitalInput(digitialChannel);
        this.enabled = input.get();
    }
}