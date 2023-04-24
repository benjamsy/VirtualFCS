within VirtualFCS.EMS_experiment.EMS_testrun.RB;

model PowerTrain_RB_testrun
  // System
  outer Modelica.Fluid.System system "System properties";
  // Powertrain parameters
  parameter Modelica.Units.SI.Mass m_powertrain = fuelCellSystem_testrun.m_FC_system + batterySystem.m_bat_pack annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter Modelica.Units.SI.Voltage V_HV_Bus = 343 "Voltage of the HV Bus" annotation(
    Dialog(group = "Powertrain Parameters"));
  // H2 Subsystem Paramters
  parameter Modelica.Units.SI.Volume V_tank_H2 = 0.13 "H2 tank volume" annotation(
    Dialog(group = "Hydrogen storage Parameters"));
  parameter Modelica.Units.SI.Pressure p_tank_H2 = 35000000 "H2 tank initial pressure" annotation(
    Dialog(group = "Hydrogen storage Parameters"));
  // Fuel Cell Stack Paramters
  parameter Modelica.Units.SI.Mass m_FC_stack = 42 "FC stack mass" annotation(
    Dialog(group = "Fuel Cell Stack Parameters"));
  parameter Modelica.Units.SI.Length L_FC_stack = 0.420 "FC stack length" annotation(
    Dialog(group = "Fuel Cell Stack Parameters"));
  parameter Modelica.Units.SI.Breadth W_FC_stack = 0.582 "FC stack width" annotation(
    Dialog(group = "Fuel Cell Stack Parameters"));
  parameter Modelica.Units.SI.Height H_FC_stack = 0.156 "FC stack height" annotation(
    Dialog(group = "Fuel Cell Stack Parameters"));
  parameter Modelica.Units.SI.Volume vol_FC_stack = L_FC_stack*W_FC_stack*H_FC_stack "FC stack volume" annotation(
    Dialog(group = "Fuel Cell Stack Parameters"));
  parameter Modelica.Units.SI.ElectricCurrent I_nom_FC_stack = 280 "FC stack nominal current" annotation(
    Dialog(group = "Fuel Cell Stack Parameters"));
  parameter Modelica.Units.SI.ElectricCurrent I_rated_FC_stack = 320 "FC stack maximum operating current" annotation(
    Dialog(group = "Fuel Cell Stack Parameters"));
  parameter Real N_FC_stack(unit = "1") = 180 "FC stack number of cells" annotation(
    Dialog(group = "Fuel Cell Stack Parameters"));
  // Battery Pack Parameters
  parameter Modelica.Units.SI.Mass m_bat_pack = 100 "Mass of the pack" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Modelica.Units.SI.Length L_bat_pack = 0.6 "Battery pack length" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Modelica.Units.SI.Breadth W_bat_pack = 0.45 "Battery pack width" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Modelica.Units.SI.Height H_bat_pack = 0.1 "Battery pack height" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Modelica.Units.SI.SpecificHeatCapacity Cp_bat_pack = 1000 "Specific Heat Capacity" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Modelica.Units.SI.Voltage V_min_bat_pack = 240 "Battery pack minimum voltage" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Modelica.Units.SI.Voltage V_nom_bat_pack = 336 "Battery pack nominal voltage" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Modelica.Units.SI.Voltage V_max_bat_pack = 403.2 "Battery pack maximum voltage" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Modelica.Units.NonSI.ElectricCharge_Ah C_bat_pack = 200 "Battery pack nominal capacity" annotation(
    Dialog(group = "Battery Pack Parameters"));
  parameter Real SOC_init = 0.5 "Battery pack initial state of charge" annotation(
    Dialog(group = "Battery Pack Parameters"));
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(
    Placement(visible = true, transformation(origin = {42, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
    Placement(visible = true, transformation(origin = {-40, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(visible = true, transformation(origin = {-68, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrochemical.Battery.BatterySystem batterySystem(C_bat_pack = C_bat_pack, Cp_bat_pack = Cp_bat_pack, H_bat_pack = H_bat_pack, L_bat_pack = L_bat_pack, SOC_init = SOC_init, V_max_bat_pack = V_max_bat_pack, V_min_bat_pack = V_min_bat_pack, V_nom_bat_pack = V_nom_bat_pack, W_bat_pack = W_bat_pack, m_bat_pack = m_bat_pack) annotation(
    Placement(visible = true, transformation(origin = {-28, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrical.DCConverter converter(vDCref = V_HV_Bus) annotation(
    Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VirtualFCS.Electrical.DC_converter dC_converter annotation(
    Placement(visible = true, transformation(origin = {22, -44}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
  // Power & Efficiencies
  Modelica.Units.SI.Power Power_del_DC_DC "Power delivered from the DC/DC converter";
  Modelica.Units.SI.Power Power_FC "Power delivered from the FC system";
  Modelica.Units.SI.Power Power_batt "Power delivered from the batt system";
  Modelica.Units.SI.Efficiency eta_drivetrain "Efficiency of the drivetrain";
  Modelica.Units.SI.Efficiency eta_DC_DC = 1 "Efficiency of the DC/DC converter";
  VirtualFCS.EMS_experiment.EMS_RB ems_rb(SOC_min = 0.3)  annotation(
    Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getFCVoltage(y = fuelCellSystem_testrun.pin_p.v)  annotation(
    Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getBattVoltage(y = batterySystem.pin_p.v)  annotation(
    Placement(visible = true, transformation(origin = {-110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression batt_V(y = batterySystem.pin_p.v)  annotation(
    Placement(visible = true, transformation(origin = {122, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.RealExpression fc_V(y = fuelCellSystem_testrun.pin_p.v)  annotation(
    Placement(visible = true, transformation(origin = {122, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  VirtualFCS.EMS_experiment.EMS_testrun.FuelCellSystem_testrun fuelCellSystem_testrun(H_FC_stack = H_FC_stack, I_nom_FC_stack = I_nom_FC_stack, I_rated_FC_stack = I_rated_FC_stack, L_FC_stack = L_FC_stack, N_FC_stack = N_FC_stack, V_tank_H2 = V_tank_H2, W_FC_stack = W_FC_stack, m_FC_stack = m_FC_stack, p_tank_H2 = p_tank_H2)  annotation(
    Placement(visible = true, transformation(origin = {60, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
    Placement(visible = true, transformation(origin = {38, -16}, extent = {{-8, -8}, {8, 8}}, rotation = 180)));
  Modelica.Blocks.Math.Division division annotation(
    Placement(visible = true, transformation(origin = {72, -20}, extent = {{8, -8}, {-8, 8}}, rotation = 0)));
equation
  Power_del_DC_DC = converter.dc_p1.i*converter.dc_p1.v;
  Power_FC = fuelCellSystem_testrun.pin_n.i*fuelCellSystem_testrun.pin_p.v;
  Power_batt = batterySystem.pin_n.i*batterySystem.pin_p.v;
  if Power_del_DC_DC > 0 then
    eta_drivetrain = min(max(((Power_del_DC_DC)/max((Power_FC + Power_batt + ((Power_FC*(1 - (fuelCellSystem_testrun.eta_FC_sys))) + (Power_batt*(1 - (batterySystem.eta_batt))) + (Power_del_DC_DC*(1 - (eta_DC_DC))))), 0.000001)), 0), 1);
  else
    eta_drivetrain = min(max(((-Power_del_DC_DC)/max((Power_FC - Power_batt + ((Power_FC*(1 - (fuelCellSystem_testrun.eta_FC_sys))) - (Power_batt*(1 - (batterySystem.eta_batt))) - (Power_del_DC_DC*(1 - (eta_DC_DC))))), 0.000001)), 0), 1);
  end if;
  connect(pin_n, ground.p) annotation(
    Line(points = {{-40, 96}, {-68, 96}, {-68, 14}}, color = {0, 0, 255}));
  connect(converter.dc_n2, batterySystem.pin_n) annotation(
    Line(points = {{-6, 20}, {-6, -22}, {-32, -22}, {-32, -62}}, color = {0, 0, 255}));
  connect(converter.dc_p2, batterySystem.pin_p) annotation(
    Line(points = {{6, 20}, {6, -36}, {-24, -36}, {-24, -62}}, color = {0, 0, 255}));
  connect(converter.dc_n1, pin_n) annotation(
    Line(points = {{-6, 40}, {-38, 40}, {-38, 96}, {-40, 96}}, color = {0, 0, 255}));
  connect(dC_converter.pin_nFC, batterySystem.pin_n) annotation(
    Line(points = {{12, -54}, {-32, -54}, {-32, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_pFC, batterySystem.pin_p) annotation(
    Line(points = {{12, -34}, {-24, -34}, {-24, -62}}, color = {0, 0, 255}));
  connect(converter.dc_p1, pin_p) annotation(
    Line(points = {{6, 40}, {42, 40}, {42, 96}}, color = {0, 0, 255}));
  connect(batterySystem.sensorOutput, ems_rb.sensorSOC) annotation(
    Line(points = {{-38, -72}, {-92, -72}, {-92, -30}, {-82, -30}}, color = {0, 0, 127}));
  connect(fuelCellSystem_testrun.pin_n, dC_converter.pin_nBus) annotation(
    Line(points = {{56, -62}, {54, -62}, {54, -54}, {32, -54}}, color = {0, 0, 255}));
  connect(dC_converter.pin_pBus, fuelCellSystem_testrun.pin_p) annotation(
    Line(points = {{32, -34}, {66, -34}, {66, -62}}, color = {0, 0, 255}));
  connect(getFCVoltage.y, ems_rb.sensorFCVoltage) annotation(
    Line(points = {{-98, 0}, {-90, 0}, {-90, -22}, {-82, -22}}, color = {0, 0, 127}));
  connect(getBattVoltage.y, ems_rb.sensorBattVoltage) annotation(
    Line(points = {{-98, -60}, {-90, -60}, {-90, -38}, {-82, -38}}, color = {0, 0, 127}));
  connect(ems_rb.controlInterface, product.u2) annotation(
    Line(points = {{-58, -30}, {-44, -30}, {-44, 0}, {56, 0}, {56, -12}, {48, -12}}, color = {0, 0, 127}));
  connect(batt_V.y, division.u1) annotation(
    Line(points = {{111, 0}, {100, 0}, {100, -16}, {82, -16}}, color = {0, 0, 127}));
  connect(fc_V.y, division.u2) annotation(
    Line(points = {{111, -40}, {100, -40}, {100, -24}, {82, -24}}, color = {0, 0, 127}));
  connect(division.y, product.u1) annotation(
    Line(points = {{64, -20}, {48, -20}}, color = {0, 0, 127}));
  connect(product.y, dC_converter.I_Ref) annotation(
    Line(points = {{30, -16}, {22, -16}, {22, -32}}, color = {0, 0, 127}));
protected
  annotation(
    Icon(graphics = {Text(origin = {-4, -12}, textColor = {0, 0, 255}, extent = {{-150, 120}, {150, 150}}, textString = "%name"), Text(origin = {17, 123}, extent = {{3, 5}, {-3, -5}}, textString = "text"), Rectangle(fillColor = {0, 60, 101}, fillPattern = FillPattern.Solid, lineThickness = 1.5, extent = {{-100, 100}, {100, -100}}, radius = 35), Polygon(fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-16.7, 56.9}, {19.3, 56.9}, {5, 11.8}, {28.4, 11.8}, {-18.7, -56.5}, {-20, -56}, {-5.3, -6}, {-28.5, -6}, {-16.7, 56.9}})}, coordinateSystem(initialScale = 0.1)));
end PowerTrain_RB_testrun;
