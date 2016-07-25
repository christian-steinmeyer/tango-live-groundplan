package com.favendo.steinmeyer.christian.tango.groundplan.status;

/**
 * An interface for updating the status view.
 *
 * Created by Christian Steinmeyer on 25.07.2016.
 */
public interface IStatusUpdater {

    void clear();

    void addCircle(float x, float y, boolean valid);

    void invalidate();
}
