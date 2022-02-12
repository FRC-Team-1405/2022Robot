package frc.robot.lib;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;

public enum FeatureToggle {
    // initialize Flags with a boolean or BooleanSupplier
    FalseFlag( false ),
    TrueFlag( true ),
    UndeclaredFlag( false ),
    PrerenceFlag( "PrefFlag" ),
    DigitalInputFlag( 0 ),
    ComplexFlag( () -> {
        return Math.random() > 0.5 ? true : false;
    });

    public boolean isEnabled(){
        return enabled;
    }

    private boolean enabled;
    // simple On/Off flag
    private FeatureToggle(boolean enabled){
        this.enabled = enabled;
    }
    // Complex On/Off flag
    private FeatureToggle(BooleanSupplier enabled){
        this.enabled = enabled.getAsBoolean();
    }
    // Preference input flag
    private FeatureToggle(String key){
        key = String.format("FeatureFlag/%s", key);
        Preferences.initBoolean(key, false);
        this.enabled = Preferences.getBoolean(key, false);
    }
    private FeatureToggle(int digitalChannel){
        this.enabled = new DigitalInput(digitalChannel).get() ? true : false;
    }
}     
