within VirtualFCS.Powertrains;

model HybridPowerTrainECMS
  // System
  outer Modelica.Fluid.System system "System properties";
  // Powertraom parameters
  parameter Real m_powertrain(unit = "kg") = fuelCellSystem.m_FC_system + batterySystem.m_bat_pack;
  parameter Real V_HV_Bus(unit = "V") = 343 "Voltage of the HV Bus";
  // H2 Subsystem Paramters
  parameter Real V_tank_H2(unit = "m3") = 0.13 "H2 tank volume";
  parameter Real p_tank_H2(unit = "Pa") = 35000000 "H2 tank initial pressure";
  // Fuel Cell Stack Paramters
  parameter Real m_FC_stack(unit = "kg") = 14.3 "FC stack mass";
  parameter Real L_FC_stack(unit = "m") = 0.255 "FC stack length";
  parameter Real W_FC_stack(unit = "m") = 0.760 "FC stack width";
  parameter Real H_FC_stack(unit = "m") = 0.060 "FC stack height";
  parameter Real vol_FC_stack(unit = "m3") = L_FC_stack * W_FC_stack * H_FC_stack "FC stack volume";
  parameter Real V_rated_FC_stack(unit = "V") = 57.9 "FC stack maximum operating voltage";
  parameter Real I_rated_FC_stack(unit = "A") = 300 "FC stack minimum operating current";
  parameter Real i_L_FC_stack(unit = "A") = 1.7 * I_rated_FC_stack "FC stack limiting current (max)";
  parameter Real I_nom_FC_stack(unit = "A") = 0.25 * I_rated_FC_stack "FC stack nominal current";
  parameter Real N_FC_stack(unit = "1") = floor(V_rated_FC_stack / 0.6433) "FC stack number of cells";
  // Battery Pack Parameters
  parameter Real m_bat_pack(unit = "kg") = 100 "Mass of the pack";
  parameter Real L_bat_pack(unit = "m") = 0.6 "Battery pack length";
  parameter Real W_bat_pack(unit = "m") = 0.45 "Battery pack width";
  parameter Real H_bat_pack(unit = "m") = 0.1 "Battery pack height";
  parameter Real Cp_bat_pack(unit = "J/(kg.K)") = 1000 "Specific Heat Capacity";
  parameter Real V_min_bat_pack(unit = "V") = 240 "Battery pack minimum voltage";
  parameter Real V_nom_bat_pack(unit = "V") = 336 "Battery pack nominal voltage";
  parameter Real V_max_bat_pack(unit = "V") = 403.2 "Battery pack maximum voltage";
  parameter Real C_bat_pack(unit = "A.h") = 200 "Battery pack nominal capacity";
  parameter Real SOC_init = 0.5 "Battery pack initial state of charge";
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(
    Placement(visible = true, transformation(origin = {40, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
    Placement(visible = true, transformation(origin = {-40, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrochemical.Hydrogen.FuelCellSystem fuelCellSystem(H_FC_stack = H_FC_stack, I_rated_FC_stack = I_rated_FC_stack, L_FC_stack = L_FC_stack, V_tank_H2 = V_tank_H2, W_FC_stack = W_FC_stack, m_FC_stack = m_FC_stack, p_tank_H2 = p_tank_H2) annotation(
    Placement(visible = true, transformation(origin = {72, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrochemical.Battery.BatterySystem batterySystem(C_bat_pack = C_bat_pack, Cp_bat_pack = Cp_bat_pack, H_bat_pack = H_bat_pack, L_bat_pack = L_bat_pack, SOC_init = SOC_init, V_max_bat_pack = V_max_bat_pack, V_min_bat_pack = V_min_bat_pack, V_nom_bat_pack = V_nom_bat_pack, W_bat_pack = W_bat_pack) annotation(
    Placement(visible = true, transformation(origin = {-28, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrical.DCConverter converter(vDCref = V_HV_Bus) annotation(
    Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VirtualFCS.Electrical.DC_converter dC_converter annotation(
    Placement(visible = true, transformation(origin = {22, -44}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
  // Power & Efficiencies
  Real Power_del_DC_DC(unit = "W") "Power delivered from the DC/DC converter";
  Real Power_FC(unit = "W") "Power delivered from the FC system";
  Real Power_batt(unit = "W") "Power delivered from the batt system";
  Real eta_drivetrain(unit = "100") "Efficiency of the drivetrain";
  Real eta_DC_DC(unit = "100") = 100 "Efficiency of the DC/DC converter";
  Modelica.Blocks.Sources.RealExpression getPowerRequest(y = converter.powerDC2)  annotation(
    Placement(visible = true, transformation(origin = {-114, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Control.EMS_ECMS ems_ecms(I_max_FC_stack = i_L_FC_stack,I_min_FC_stack = I_nom_FC_stack- 70, I_nom_FC_stack = I_rated_FC_stack, n_cell = N_FC_stack)  annotation(
    Placement(visible = true, transformation(origin = {-64, -28}, extent = {{-20, -10}, {20, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getOCV(y = batterySystem.batteryPack.OCV.v)  annotation(
    Placement(visible = true, transformation(origin = {-114, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getBopPower(y = fuelCellSystem.Power_BOP)  annotation(
    Placement(visible = true, transformation(origin = {-114, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getFCTemp(y = fuelCellSystem.fuelCellStack.temperatureSensor.T)  annotation(
    Placement(visible = true, transformation(origin = {-114, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getpH2(y = fuelCellSystem.fuelCellStack.p_H2)  annotation(
    Placement(visible = true, transformation(origin = {-114, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getBattVoltage(y = batterySystem.pin_p.v)  annotation(
    Placement(visible = true, transformation(origin = {-114, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getBattCurrent(y = batterySystem.pin_n.i)  annotation(
    Placement(visible = true, transformation(origin = {-114, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getpO2(y = fuelCellSystem.fuelCellStack.p_O2)  annotation(
    Placement(visible = true, transformation(origin = {-114, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  Power_del_DC_DC = converter.dc_p1.i * converter.dc_p1.v;
  Power_FC = fuelCellSystem.pin_n.i * fuelCellSystem.pin_p.v;
  Power_batt = batterySystem.pin_n.i * batterySystem.pin_p.v;
  if Power_del_DC_DC > 0 then
    eta_drivetrain = min(max(((Power_del_DC_DC)/max((Power_FC + Power_batt + ((Power_FC * (1-(fuelCellSystem.eta_FC_sys*0.01)))+ (Power_batt * (1-(batterySystem.eta_batt*0.01)))+ (Power_del_DC_DC * (1-(eta_DC_DC*0.01))))), 0.000001)) * 100, 0),100);
  else
    eta_drivetrain = min(max(((-Power_del_DC_DC)/max((Power_FC -Power_batt + ((Power_FC * (1-(fuelCellSystem.eta_FC_sys*0.01))) - (Power_batt * (1-(batterySystem.eta_batt*0.01))) - (Power_del_DC_DC * (1-(eta_DC_DC*0.01))))), 0.000001)) * 100, 0), 100);
  end if;
  connect(pin_n, ground.p) annotation(
    Line(points = {{-40, 96}, {-70, 96}, {-70, 80}}, color = {0, 0, 255}));
  connect(converter.dc_n2, batterySystem.pin_n) annotation(
    Line(points = {{-6, 20}, {-6, -22}, {-32, -22}, {-32, -62}}, color = {0, 0, 255}));
  connect(converter.dc_p2, batterySystem.pin_p) annotation(
    Line(points = {{6, 20}, {6, -36}, {-24, -36}, {-24, -62}}, color = {0, 0, 255}));
  connect(converter.dc_n1, pin_n) annotation(
    Line(points = {{-6, 40}, {-38, 40}, {-38, 96}, {-40, 96}}, color = {0, 0, 255}));
  connect(converter.dc_p1, pin_p) annotation(
    Line(points = {{6, 40}, {6, 80}, {40, 80}, {40, 96}}, color = {0, 0, 255}));
  connect(dC_converter.pin_pBus, fuelCellSystem.pin_p) annotation(
    Line(points = {{32, -34}, {78, -34}, {78, -62}, {78, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_nBus, fuelCellSystem.pin_n) annotation(
    Line(points = {{32, -54}, {68, -54}, {68, -62}, {68, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_nFC, batterySystem.pin_n) annotation(
    Line(points = {{12, -54}, {-32, -54}, {-32, -62}, {-32, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_pFC, batterySystem.pin_p) annotation(
    Line(points = {{12, -34}, {-24, -34}, {-24, -62}, {-24, -62}}, color = {0, 0, 255}));
  connect(batterySystem.sensorOutput, ems_ecms.sensorInputSOC) annotation(
    Line(points = {{-38, -72}, {-96, -72}, {-96, -32}, {-86, -32}}, color = {0, 0, 127}));
  connect(getBattVoltage.y, ems_ecms.batteryVoltage) annotation(
    Line(points = {{-102, -56}, {-74, -56}, {-74, -40}}, color = {0, 0, 127}));
  connect(getBattCurrent.y, ems_ecms.batteryCurrent) annotation(
    Line(points = {{-102, -76}, {-64, -76}, {-64, -40}}, color = {0, 0, 127}));
  connect(getpO2.y, ems_ecms.p_O2) annotation(
    Line(points = {{-102, -96}, {-54, -96}, {-54, -40}}, color = {0, 0, 127}));
  connect(getOCV.y, ems_ecms.OCV) annotation(
    Line(points = {{-102, -2}, {-78, -2}, {-78, -16}}, color = {0, 0, 127}));
  connect(getBopPower.y, ems_ecms.bopPower) annotation(
    Line(points = {{-102, 16}, {-70, 16}, {-70, -16}}, color = {0, 0, 127}));
  connect(getFCTemp.y, ems_ecms.fcTemperature) annotation(
    Line(points = {{-102, 36}, {-58, 36}, {-58, -16}}, color = {0, 0, 127}));
  connect(getpH2.y, ems_ecms.p_H2) annotation(
    Line(points = {{-102, 58}, {-50, 58}, {-50, -16}}, color = {0, 0, 127}));
  connect(ems_ecms.controlOutputFuelCellCurrent, dC_converter.I_Ref) annotation(
    Line(points = {{-42, -28}, {22, -28}, {22, -32}}, color = {0, 0, 127}));
  connect(getPowerRequest.y, ems_ecms.powerRequest) annotation(
    Line(points = {{-102, -24}, {-86, -24}}, color = {0, 0, 127}));
protected
  annotation(
    Icon(graphics = {Text(origin = {-4, -12}, textColor = {0, 0, 255}, extent = {{-150, 120}, {150, 150}}, textString = "%name"), Text(origin = {17, 123}, extent = {{3, 5}, {-3, -5}}, textString = "text"), Rectangle(fillColor = {0, 60, 101}, fillPattern = FillPattern.Solid, lineThickness = 1.5, extent = {{-100, 100}, {100, -100}}, radius = 35), Polygon(fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-16.7, 56.9}, {19.3, 56.9}, {5, 11.8}, {28.4, 11.8}, {-18.7, -56.5}, {-20, -56}, {-5.3, -6}, {-28.5, -6}, {-16.7, 56.9}})}, coordinateSystem(initialScale = 0.1)));
end HybridPowerTrainECMS;
