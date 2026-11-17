package com.nxp.example.smartgreenhouse.controllers;
/** Stub UI controller: hanya mencatat event ke log. */
public class ActuatorDetailController {
    public void refreshIfOpen() { }
    public void onValveControlPublished(int valveId, boolean on) { System.out.println("[UI] valve " + valveId + " published " + on); }
    public void onValveControlPublishFailed(int valveId, boolean on) { System.out.println("[UI] valve " + valveId + " FAILED " + on); }
}
