
package com.MAutils.Utils;

import java.util.function.DoubleFunction;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.ParentDevice.MapGenerator;

//TODO delete it, its was befor the motor sim right?
public class SimStatusSignal<T> extends StatusSignal<T> {

    private Supplier<T> valueSupplier;

    public SimStatusSignal(Supplier<T> valueSupplier) {
        super(new DeviceIdentifier(), 0, () -> {}, null, null, null);
        this.valueSupplier = valueSupplier;
    }

    private SimStatusSignal(
        DeviceIdentifier deviceIdentifier,
        int spn,
        Runnable reportIfOldFunc,
        Class<T> classOfSignal,
        DoubleFunction<T> toValue,
        String signalName
    ) {
        super(deviceIdentifier, spn, reportIfOldFunc, classOfSignal, toValue, signalName);
    }

    private SimStatusSignal(
        DeviceIdentifier deviceIdentifier,
        int spn,
        Runnable reportIfOldFunc,
        Class<T> classOfSignal,
        DoubleFunction<T> toValue,
        MapGenerator<T> generator,
        String signalName
    ) {
        super(deviceIdentifier, spn, reportIfOldFunc, classOfSignal, toValue, generator, signalName);
    }

    private SimStatusSignal(
        Class<T> classOfSignal,
        DoubleFunction<T> toValue,
        StatusCode error
    ) {
        super(classOfSignal, toValue, error);
    }

    public void setValueSupplier(Supplier<T> valueSupplier) {
        this.valueSupplier = valueSupplier;
    }

    @Override
    public double getValueAsDouble() {
        return (double) valueSupplier.get();
    }

    @Override
    public StatusSignal<T> refresh(boolean reportError) {
        return this;
    }

}
