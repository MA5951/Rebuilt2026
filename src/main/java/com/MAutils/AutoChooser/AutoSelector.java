
package com.MAutils.AutoChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoSelector {
    private final SendableChooser<Autonomus> autoChooser = new SendableChooser<>();

    public void setAutoOptions(Autonomus... options) {
        autoChooser.setDefaultOption(options[0].getName(), options[0]);
        for (int i = 1; i < options.length; i++) {
            autoChooser.addOption(options[i].getName(), options[i]);
        }
    }

    public Autonomus getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
