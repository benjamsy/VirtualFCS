within VirtualFCS.Powertrains;

model ParallelHybridPowerTrain

  parameter Real m_powertrain(unit = "kg") = 100+ 50;

  parameter Real V_HV_Bus(unit="V") = 343 "Voltage of the HV Bus";

// H2 Subsystem Paramters
  parameter Real V_tank_H2(unit="m3") = 0.13 "H2 tank volume";
  parameter Real p_tank_H2(unit="Pa") = 3500000 "H2 tank initial pressure";

// Fuel Cell Stack Paramters
  parameter Real m_FC_stack(unit = "kg") = 14.3 "FC stack mass";
  parameter Real L_FC_stack(unit = "m") = 0.255 "FC stack length";
  parameter Real W_FC_stack(unit = "m") = 0.760 "FC stack length";
  parameter Real H_FC_stack(unit = "m") = 0.060 "FC stack length";
  parameter Real vol_FC_stack(unit = "m3") = L_FC_stack * W_FC_stack * H_FC_stack "FC stack volume";
  parameter Real V_rated_FC_stack(unit="V") = 57.9 "FC stack maximum operating voltage";
  parameter Real I_rated_FC_stack(unit="A") = 300 "FC stack minimum operating voltage";
  parameter Real i_L_FC_stack(unit = "A") = 3 * I_rated_FC_stack "FC stack maximum limiting current";
  parameter Real I_nom_FC_stack(unit = "A") = 0.25 * I_rated_FC_stack "FC stack maximum limiting current";
  parameter Real N_FC_stack(unit = "1") = floor(V_rated_FC_stack/0.6433) "FC stack number of cells";

// Battery Pack Parameters
  parameter Real m_bat_pack(unit = "kg") = 100 "Mass of the pack";
  parameter Real L_bat_pack(unit = "m") = 0.6 "Battery pack length";
  parameter Real W_bat_pack(unit = "m") = 0.45 "Battery pack width";
  parameter Real H_bat_pack(unit = "m") = 0.1 "Battery pack height";
  parameter Real Cp_bat_pack(unit = "J/(kg.K)") = 1000 "Specific Heat Capacity";
  parameter Real V_min_bat_pack(unit = "V") = 37.5 "Battery pack minimum voltage";
  parameter Real V_nom_bat_pack(unit = "V") = 48 "Battery pack nominal voltage";
  parameter Real V_max_bat_pack(unit = "V") = 54.75 "Battery pack maximum voltage";
  parameter Real C_bat_pack(unit = "A.h") = 2700 "Battery pack nominal capacity";
  parameter Real SOC_init = 0.5 "Battery pack initial state of charge";
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(
    Placement(visible = true, transformation(origin = {40, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
    Placement(visible = true, transformation(origin = {-40, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Fluid.System system annotation(
    Placement(visible = true, transformation(origin = {-94, 94}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(visible = true, transformation(origin = {-68, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrochemical.Hydrogen.FuelCellSystem fuelCellSystem annotation(
    Placement(visible = true, transformation(origin = {72, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrochemical.Battery.BatterySystem batterySystem annotation(
    Placement(visible = true, transformation(origin = {-28, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrical.DCConverter converter(vDCref = V_HV_Bus)  annotation(
    Placement(visible = true, transformation(origin = { -28, 28}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VirtualFCS.Electrical.DCConverterSwitch converter1(vDCref = V_HV_Bus)  annotation(
    Placement(visible = true, transformation(origin = {72, 34}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VirtualFCS.Electrical.DC_converter dC_converter annotation(
    Placement(visible = true, transformation(origin = {22, -44}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
  VirtualFCS.Control.EnergyManagementSystem energyManagementSystem(ramp_up = 1)  annotation(
    Placement(visible = true, transformation(origin = {-68, -72}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant annotation(
    Placement(visible = true, transformation(origin = { 36, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(pin_n, ground.p) annotation(
    Line(points = {{-40, 96}, {-68, 96}, {-68, 14}}, color = {0, 0, 255}));
  connect(converter.dc_n2, batterySystem.pin_n) annotation(
    Line(points = {{-34, 18}, {-34, -22}, {-32, -22}, {-32, -62}}, color = {0, 0, 255}));
  connect(converter.dc_p2, batterySystem.pin_p) annotation(
    Line(points = {{-22, 18}, {-22, -36}, {-24, -36}, {-24, -62}}, color = {0, 0, 255}));
  connect(converter.dc_n1, pin_n) annotation(
    Line(points = {{-34, 38}, {-38, 38}, {-38, 96}, {-40, 96}}, color = {0, 0, 255}));
  connect(converter.dc_p1, pin_p) annotation(
    Line(points = {{-22, 38}, {-22, 80}, {40, 80}, {40, 96}}, color = {0, 0, 255}));
  connect(converter1.dc_n2, fuelCellSystem.pin_n) annotation(
    Line(points = {{66, 24}, {68, 24}, {68, -62}, {68, -62}}, color = {0, 0, 255}));
  connect(converter1.dc_p2, fuelCellSystem.pin_p) annotation(
    Line(points = {{78, 24}, {78, 24}, {78, -62}, {78, -62}}, color = {0, 0, 255}));
  connect(converter1.dc_n1, pin_n) annotation(
    Line(points = {{66, 44}, {64, 44}, {64, 64}, {-38, 64}, {-38, 96}, {-40, 96}}, color = {0, 0, 255}));
  connect(pin_p, converter1.dc_p1) annotation(
    Line(points = {{40, 96}, {78, 96}, {78, 44}, {78, 44}}, color = {0, 0, 255}));
  connect(energyManagementSystem.sensorInterface, batterySystem.sensorOutput) annotation(
    Line(points = {{-56, -72}, {-38, -72}, {-38, -72}, {-38, -72}}, color = {0, 0, 127}));
  connect(energyManagementSystem.controlInterface, dC_converter.I_Ref) annotation(
    Line(points = {{-80, -72}, {-90, -72}, {-90, -18}, {22, -18}, {22, -32.5}}, color = {0, 0, 127}));
  connect(booleanConstant.y, converter1.u) annotation(
    Line(points = {{48, 32}, {58, 32}, {58, 32}, {60, 32}}, color = {255, 0, 255}));
  connect(dC_converter.pin_nBus, fuelCellSystem.pin_n) annotation(
    Line(points = {{32, -54}, {68, -54}, {68, -62}, {68, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_pBus, fuelCellSystem.pin_p) annotation(
    Line(points = {{32, -34}, {78, -34}, {78, -62}, {78, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_nFC, batterySystem.pin_n) annotation(
    Line(points = {{12, -54}, {-32, -54}, {-32, -62}, {-32, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_pFC, batterySystem.pin_p) annotation(
    Line(points = {{12, -34}, {-24, -34}, {-24, -62}, {-24, -62}}, color = {0, 0, 255}));
protected
  annotation(
    Icon(graphics = {Text(origin = {-4, -12}, lineColor = {0, 0, 255}, extent = {{-150, 120}, {150, 150}}, textString = "%name"), Bitmap(origin = {-4, 3}, extent = {{-96, 97}, {104, -103}}, imageSource = "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"), Text(origin = {17, 123}, extent = {{3, 5}, {-3, -5}}, textString = "text")}, coordinateSystem(initialScale = 0.1)));
end ParallelHybridPowerTrain;
