within VirtualFCS.ComponentTesting;

package ECMSTest "Package containing models and blocks indepentent from the main library for for an test block for the ECMS control block. This also includes removing slow parts of the main model for faster and smoother debugging. Models/blocks that are indepented from main library: Vehicle, ECMS_control, powertrain, fuelCellSystem, fuel cell subsystems, " // Important to notice that the subsystems have been simply removed to reduce runtime for debugging efficiency
  extends Modelica.Icons.ExamplesPackage;
end ECMSTest;
