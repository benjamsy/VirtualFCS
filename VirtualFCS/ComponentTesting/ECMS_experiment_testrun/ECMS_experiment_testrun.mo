within VirtualFCS.ComponentTesting.ECMS_experiment_testrun;

model ECMS_experiment_testrun

  //---- Imports ----
  import OMCallPythonAdvanced;
  import OMCallPythonAdvanced.Py;
  import OMCallPythonAdvanced.Py.finalize;
  import OMCallPythonAdvanced.Py.initialize;
  import OMCallPythonAdvanced.Py.PythonHandle;
  import OMCallPythonAdvanced.Py.run;
  import OMCallPythonAdvanced.Py.nineRealArgumentsReturnReal;
  import Modelica.Utilities.Streams.print;
  import SI = Modelica.Units.SI;
  import NonSI = Modelica.Units.NonSI;
  //---- Parameters ----
  parameter Real ramp_up(unit = "1/s") = 20 "FC stack current ramp up rate" annotation(
    Dialog(group = "Control Parameters"));
  //---- Fuel cell ----
  parameter SI.Current I_min_FC_stack "FC stack minimum operating current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter SI.Current I_max_FC_stack "FC stack maximum current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter Real n_cell(unit = "1") "Number of cells in fuel cell stack" annotation(
    Dialog(group = "Powertrain Parameters"));
  Real LHV_H2(unit = "Ws/g") = 119959.2 "Hydrogen lower heating value";
  import R = Modelica.Constants.R; // Universal gas constant
  import F = Modelica.Constants.F; // Faradays number
  SI.AbsolutePressure p_0 = 100000;
  SI.Current i_0_FC_stack = 0.0000193 "FC stack cell exchange current";
  SI.Current i_x_FC_stack = 0.001 "FC stack cell cross-over current";
  SI.Resistance R_O_FC_stack = 0.0923 "FC stack cell ohmic resistance";
  NonSI.Area_cm A_FC_surf = 285 "FC stack surface area";
  Real i_L_FC_stack(unit"A/cm2") = 1.12 "FC stack cell maximum limiting current A/cm2";
  SI.Voltage U_0 = 1.229 "Theoretical maximum voltage for a single cell";
  SI.Voltage fc_V "FC voltage calculated from FC current output of optimization";
  final SI.Efficiency eta_fc_sys_estimate "Polynomial fit fuel cell system efficiency";
  final NonSI.Pressure_bar pH2 "Fuel inlet pressure";
  final NonSI.Pressure_bar pO2 "Air inlet pressure";
  // Used to calculate test /and control parameters. Will not matter if removed.
  //---- Battery ----
  parameter Real SOC_min(unit = "1") = 0.4 "Minimum allowed SOC of battery" annotation(
    Dialog(group = "Control Parameters"));
  parameter Real SOC_max(unit = "1") = 0.7 "Maximum allowed SOC of battery" annotation(
    Dialog(group = "Control Parameters"));
  parameter SI.Current I_min_batt = 500 "Battery min current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter SI.Current I_max_batt = 500 "Battery max current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter SI.Voltage OCV = 49.17 "Open circuit voltage battery" annotation(
    Dialog(group = "Powertrain Parameters")); // Define this prior to experiment!
  Real SOC(unit = "1") "State of charge of battery";
  SI.Voltage batt_V "Battery voltage real";
  SI.Current batt_I "Battery Current real";
  parameter SI.Resistance internalResistance = 0.028 "Internal resistance of battery real time" annotation(
    Dialog(group = "Powertrain Parameters")); //Define this prior to experiment!!
  SI.Current batterySetCurrent "Battery set current calculated by the optimization solver";
  //---- Other side of DCDC ----
  final SI.Current I_min_fc_dcdc_out "The minimum allowed current out of the DCDC converter";
  final SI.Current I_max_fc_dcdc_out "The maximum allowed current out of the DCDC converter";
  final SI.Current set_current_dcdc_out "The set current out of the DCDC converter";
  // -- Battery charging/discharging efficiencies --
  final SI.Efficiency eta_b_chr(start = 1) "Battery charge efficiency";
  final SI.Efficiency eta_b_dischr(start = 1) "Battery discharge efficiency";
  final Integer eta_b_chr_counter(start = 1) "Counts the number of charge efficiencies calculated";
  final Real eta_b_chr_sum(start = 1);
  final Real eta_b_chr_avg(start = 1) "Battery charge efficiency average";
  final Integer eta_b_dischr_counter(start = 1) "Counts the number of discharge efficiencies calculated";
  final Real eta_b_dischr_sum(start = 1);
  final Real eta_b_dischr_avg(start = 1) "Battery discharge efficiency average";
  //---- Time
  Real t = time;
  final Real t_comp(start = 0);
  parameter Integer t_const = 50 "FC should not start before this time" annotation(
    Dialog(group = "Control Parameters"));
  // Assumtion on start-up procedure. This is not modelled, so FC is just turned off this time.
  //---- Necessary for ECMS ----
  final Real sigma(start = 1);
  Real mu(unit = "1") = 1;
  final SI.Power powerD(start = 0) "Power request";
  final SI.Current currentD(start = 0) "Current request";
  final Real k_b_f(start = 1);
  final NonSI.MassFlowRate_gps C_fc(start = 0);
  final NonSI.MassFlowRate_gps C_batt(start = 0);
  final NonSI.MassFlowRate_gps C_batt_2(start = 0); // C_batt*k_b
  final NonSI.MassFlowRate_gps C(start = 0);
  final Real C_sum(start = 0);
  Modelica.Blocks.Math.ContinuousMean C_a(u = C); // Average total consumption
  Modelica.Blocks.Math.ContinuousMean C_a_fc(u = C_fc); // Average fc consumption
  Modelica.Blocks.Math.ContinuousMean C_a_batt(u = C_batt); // Average batt consumption
  final NonSI.MassFlowRate_gps C_avg;
  final NonSI.MassFlowRate_gps C_avg_fc;
  final NonSI.MassFlowRate_gps C_avg_batt;
  //---- Set current variable ---- Three is necessary to ensure constraints are maintained
  final SI.Current set_current_1(start = 0) "Fuel cell set current";
  final SI.Current set_current_2(start = 0) "Fuel cell set current";
  final SI.Current set_current_3(start = 0) "Fuel cell set current";
  final Boolean turn_off(start = false);
  // Boolean to control if the FC is on or off.
  //---- Python Handle ----
  Py.PythonHandle pyHandle = Py.PythonHandle() "Define Python Handle";
  //---- Python minimization solver program ----
  String pyProgram = "
def ECMS_main(powerDemand, SoC, empty, b_V, sig, empty1, fc_temp, pH2, pO2):
\t import numpy
\t import scipy
\t import scipy.optimize
\t from scipy.optimize import Bounds
\t import math
\t #import time

\t #t1 = time.time()

\t i_fc_guess = 0
\t i_fc_guess_temp = 0
\t i_batt_guess = 0
\t i_batt_guess_temp = 0
\t C_guess_pre = 100

\t for i in range(0,int(float(" + String(I_max_FC_stack) + ")), int(float(" + String(I_max_FC_stack) + ")/int(float(" + String(I_max_FC_stack) + ")/4))):
\t \t i_fc_guess_temp = i

\t \t i_batt_guess_temp = (i_fc_guess_temp*(float(" + String(n_cell) + ")*(float(" + String(U_0) + ") - float(" + String(R) + ")*fc_temp/(2*float(" + String(F) + "))*math.log(pH2*(pO2**0.5)) - (0.85*0.001)*(fc_temp - 298.15) - float(" + String(R_O_FC_stack) + ")*(i_fc_guess_temp/float(" + String(A_FC_surf) + ")) - (float(" + String(R) + ")*fc_temp)/(2*float(" + String(F) + ")*0.3419)*math.log(abs(max(i_fc_guess_temp, 0.001)/float(" + String(A_FC_surf) + "))/float(" + String(i_0_FC_stack) + ")) + (float(" + String(R) + ")*1.4672*fc_temp)/(2*float(" + String(F) + "))*math.log(1-(abs(i_fc_guess_temp/float(" + String(A_FC_surf) + "))/float(" + String(i_L_FC_stack) + "))))))/(b_V)

\t \t C_guess = (((float(" + String(n_cell) + ")*(float(" + String(U_0) + ") - float(" + String(R) + ")*fc_temp/(2*float(" + String(F) + "))*math.log(pH2*(pO2**0.5)) - (0.85*0.001)*(fc_temp - 298.15) - float(" + String(R_O_FC_stack) + ")*(i_fc_guess_temp/float(" + String(A_FC_surf) + ")) - (float(" + String(R) + ")*fc_temp)/(2*float(" + String(F) + ")*0.3419)*math.log(abs(abs(max(i_fc_guess_temp, 0.001)/float(" + String(A_FC_surf) + "))/float(" + String(i_0_FC_stack) + ")) + (float(" + String(R) + ")*1.4672*fc_temp)/(2*float(" + String(F) + "))*math.log(1-(abs(i_fc_guess_temp/float(" + String(A_FC_surf) + "))/float(" + String(i_L_FC_stack) + "))))) * i_fc_guess_temp) / (float(" + String(LHV_H2) + ") * (-2.28032634348 * 10**(-16) * i_fc_guess_temp**(6) + 5.29567212538 * 10**(-13) * i_fc_guess_temp**(5) - 4.83584705561 * 10**(-10) * i_fc_guess_temp**(4) + 2.19811746997 * 10**(-7) * i_fc_guess_temp**(3) -5.08697130216 * 10**(-5) * i_fc_guess_temp**(2) + 4.7311314221 * 10**(-3) * i_fc_guess_temp + 0.427083655202))) + (1 - 2*float(" + String(mu) + ")*(SoC - 0.5*(float(" + String(SOC_max) + ") + float(" + String(SOC_min) + ")))/(float(" + String(SOC_max) + ") - float(" + String(SOC_min) + "))) * ((float(sig) * float(b_V) * i_batt_guess_temp) / (float(" + String(LHV_H2) + "))))

\t \t if C_guess < C_guess_pre:
\t \t \t C_guess_pre = C_guess
\t \t \t i_fc_guess = i_fc_guess_temp
\t \t \t i_batt_guess = i_batt_guess_temp

\t bounds = Bounds([(i_fc_guess - 5), (i_batt_guess - 5)], [(i_fc_guess + 5), (i_batt_guess + 5)])

\t cons = {'type':'eq', 'fun' : lambda y : (float(" + String(n_cell) + ")*(float(" + String(U_0) + ") - float(" + String(R) + ")*fc_temp/(2*float(" + String(F) + "))*math.log(pH2*(pO2**0.5)) - (0.85*0.001)*(fc_temp - 298.15) - float(" + String(R_O_FC_stack) + ")*(y[0]/float(" + String(A_FC_surf) + ")) - (float(" + String(R) + ")*fc_temp)/(2*float(" + String(F) + ")*0.3419)*math.log(abs(max(y[0], 0.001)/float(" + String(A_FC_surf) + "))/float(" + String(i_0_FC_stack) + ")) + (float(" + String(R) + ")*1.4672*fc_temp)/(2*float(" + String(F) + "))*math.log(1-(abs(y[0]/float(" + String(A_FC_surf) + "))/float(" + String(i_L_FC_stack) + ")))) * y[0]) + (b_V * y[1]) - powerDemand}


\t res = scipy.optimize.minimize(lambda x : (((float(" + String(n_cell) + ")*(float(" + String(U_0) + ") - float(" + String(R) + ")*fc_temp/(2*float(" + String(F) + "))*math.log(pH2*(pO2**0.5)) - (0.85*0.001)*(fc_temp - 298.15) - float(" + String(R_O_FC_stack) + ")*(x[0]/float(" + String(A_FC_surf) + ")) - (float(" + String(R) + ")*fc_temp)/(2*float(" + String(F) + ")*0.3419)*math.log(abs(abs(max(x[0], 0.001)/float(" + String(A_FC_surf) + "))/float(" + String(i_0_FC_stack) + ")) + (float(" + String(R) + ")*1.4672*fc_temp)/(2*float(" + String(F) + "))*math.log(1-(abs(x[0]/float(" + String(A_FC_surf) + "))/float(" + String(i_L_FC_stack) + "))))) * x[0]) / (float(" + String(LHV_H2) + ") * (-2.28032634348 * 10**(-16) * x[0]**(6) + 5.29567212538 * 10**(-13) * x[0]**(5) - 4.83584705561 * 10**(-10) * x[0]**(4) + 2.19811746997 * 10**(-7) * x[0]**(3) -5.08697130216 * 10**(-5) * x[0]**(2) + 4.7311314221 * 10**(-3) * x[0] + 0.427083655202))) + (1 - 2*float(" + String(mu) + ")*(SoC - 0.5*(float(" + String(SOC_max) + ") + float(" + String(SOC_min) + ")))/(float(" + String(SOC_max) + ") - float(" + String(SOC_min) + "))) * ((float(sig) * float(b_V) * x[1]) / (float(" + String(LHV_H2) + ")))), numpy.array([i_fc_guess, i_batt_guess]), method='trust-constr', bounds=bounds, constraints=cons, options = {'maxiter':25})

\t #t2 = time.time()

\t #print('Function call time =', (t2-t1))

\t return float(res.x[0])

  ";
  //---- Names ----
  constant String pyModuleName = "__main__";
  constant String pyFunctionName = "ECMS_main";
  //---- Connections ----
  Modelica.Blocks.Interfaces.RealInput sensorInputSOC annotation(
    Placement(visible = true, transformation(origin = {-220, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-220, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput controlOutputFuelCellCurrent annotation(
    Placement(visible = true, transformation(origin = {210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising = ramp_up) annotation(
    Placement(visible = true, transformation(origin = {62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Abs abs1 annotation(
    Placement(visible = true, transformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression setCurrent(y = set_current_3) annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput powerRequest annotation(
    Placement(visible = true, transformation(origin = {-100, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-100, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput batteryVoltage annotation(
    Placement(visible = true, transformation(origin = {-100, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-100, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput batteryCurrent annotation(
    Placement(visible = true, transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput fcTemperature annotation(
    Placement(visible = true, transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput p_H2 annotation(
    Placement(visible = true, transformation(origin = {100, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {100, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput p_O2 annotation(
    Placement(visible = true, transformation(origin = {100, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {100, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Continuous.Filter filter(analogFilter = Modelica.Blocks.Types.AnalogFilter.CriticalDamping, f_cut = 0.7, order = 5)  annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.Power battPowerV2;
  SI.Power fcPowerV2;
  SI.Power battFcPower;
  Real testConstrain; // Should be zero
  
initial equation
//---- Import necessary Python libraries ----
  Py.initialize(pyHandle);
  Py.run(pyHandle, "import sys");
  Py.run(pyHandle, "import os");
equation
//---- Redefine variables ----
  SOC = sensorInputSOC;
  batt_V = max(batteryVoltage, 1);
  powerD = powerRequest;
  batt_I = - batteryCurrent;
  eta_fc_sys_estimate = -2.28032634348 * 10^(-16)* set_current_3^6 + 5.29567212538 * 10^(-13) * set_current_3^5 - 4.83584705561 * 10^(-10) * set_current_3^4 + 2.19811746997 * 10^(-7) * set_current_3^3 -5.08697130216 * 10^(-5) * set_current_3^2 + 4.7311314221 * 10^(-3) * set_current_3 + 0.427083655202;
  pH2 = max(p_H2 * 0.00001, 0.000001);
  pO2 = max(0.2 * p_O2 * 0.00001, 0.000001);

//---- Recalculations ----
  
  if turn_off then
    fc_V = n_cell*(U_0 - R*fcTemperature/(2*F)*log(pH2*(pO2^0.5)) - (0.85*0.001)*(fcTemperature - 298.15) - R_O_FC_stack*(0/A_FC_surf) - (R*fcTemperature)/(2*F*0.3419)*log(abs(max(0,0.001)/A_FC_surf)/i_0_FC_stack) + (R*1.4672*fcTemperature)/(2*F)*log(1-(abs(0/A_FC_surf)/i_L_FC_stack)));
  else
    fc_V = n_cell*(U_0 - R*fcTemperature/(2*F)*log(pH2*(pO2^0.5)) - (0.85*0.001)*(fcTemperature - 298.15) - R_O_FC_stack*(max(set_current_1, I_min_FC_stack)/A_FC_surf) - (R*fcTemperature)/(2*F*0.3419)*log(abs(max(max(set_current_1, I_min_FC_stack),0.001)/A_FC_surf)/i_0_FC_stack) + (R*1.4672*fcTemperature)/(2*F)*log(1-(abs(max(set_current_1, I_min_FC_stack)/A_FC_surf)/i_L_FC_stack)));
  end if; // This if-else recalculates what the fuel cell voltage should be. It will not be completly accurate due to dynamic mechanics
  
  currentD = powerD/batt_V; // Calculates the current demand (on the battery side of converters)
  batterySetCurrent = currentD - (fcPowerV2/batt_V); // Calculates the "real" battery set current inside the optimization solver
  battPowerV2 = batterySetCurrent * batt_V; // Calculates the battery power assumed by the optimization solver, is not the same as real battery power
  fcPowerV2 = set_current_1 * fc_V; // Calculates the fuel cell set power decided by the optimization solver
  battFcPower = fcPowerV2 + battPowerV2; // Calculates the combined power from FC and batt inside the optimization solver
  testConstrain = (fc_V * set_current_1) + (batt_V * batterySetCurrent) - powerD; // Calculates the constrain inside the optimization solver (Should be close to zero)
//---- Consumptions ----
  C_fc = (((n_cell*(U_0 - R*fcTemperature/(2*F)*log(pH2*(pO2^0.5)) - (0.85*0.001)*(fcTemperature - 298.15) - R_O_FC_stack*(set_current_1/A_FC_surf) - (R*fcTemperature)/(2*F*0.3419)*log(abs(max(set_current_1, 0.001)/A_FC_surf)/i_0_FC_stack) + (R*1.4672*fcTemperature)/(2*F)*log(1-(abs(set_current_1/A_FC_surf)/i_L_FC_stack))))*set_current_1)/(LHV_H2*(-2.28032634348 * 10^(-16)* set_current_3^6 + 5.29567212538 * 10^(-13) * set_current_3^5 - 4.83584705561 * 10^(-10) * set_current_3^4 + 2.19811746997 * 10^(-7) * set_current_3^3 -5.08697130216 * 10^(-5) * set_current_3^2 + 4.7311314221 * 10^(-3) * set_current_3 + 0.427083655202))); // Calculates C_fc the same as the optimization solver. This will vary from the actual FC since set_current_1 is used, and no dynamic behavious is considered.
  C_batt = ((sigma*batt_V*batterySetCurrent)/(LHV_H2)); // Calculates C_batt same as the optimization solver. Will vary from actual batt.
  k_b_f = 1-2*mu*((SOC - 0.5*(SOC_max + SOC_min))/(SOC_max - SOC_min)); // Calculates the k_b term
  C_batt_2 = k_b_f*C_batt; // Calculates k_b * C_batt
  C = C_fc + k_b_f*C_batt; // Calcualtes C_tot
  C_avg = C_a.y; // Average total consumption
  C_avg_fc = C_a_fc.y; // Average fc consumption
  C_avg_batt = C_a_batt.y; // Average batt consumption
 
  //---- Recalculate from FC side of DCDC converter to battery side ----
  I_min_fc_dcdc_out = (I_min_FC_stack * n_cell*(U_0 - R*fcTemperature/(2*F)*log(pH2*(pO2^0.5)) - (0.85*0.001)*(fcTemperature - 298.15) - R_O_FC_stack*(I_min_FC_stack/A_FC_surf) - (R*fcTemperature)/(2*F*0.3419)*log(abs(I_min_FC_stack/A_FC_surf)/i_0_FC_stack) + (R*1.4672*fcTemperature)/(2*F)*log(1-(abs(I_min_FC_stack/A_FC_surf)/i_L_FC_stack)))) / (batt_V); // Calculates the minimum set current for the fuel cell - on the battery side of the DCDC converter
  
  I_max_fc_dcdc_out = ((I_max_FC_stack - 1) * n_cell*(U_0 - R*fcTemperature/(2*F)*log(pH2*(pO2^0.5)) - (0.85*0.001)*(fcTemperature - 298.15) - R_O_FC_stack*(I_max_FC_stack/A_FC_surf) - (R*fcTemperature)/(2*F*0.3419)*log(abs(I_max_FC_stack/A_FC_surf)/i_0_FC_stack) + (R*1.4672*fcTemperature)/(2*F)*log(1-(abs(I_max_FC_stack/A_FC_surf)/i_L_FC_stack)))) / (batt_V); // Calculates the maximum set current for the fuel cell - on the battery side of the DCDC converter
  
  set_current_dcdc_out = (set_current_1 * fc_V) / (batt_V); // Calculates the set current for the fuel cell - on the battery side of the DCDC converter
  
  //---- If-else logic to calculate battery charging/discharing efficiencies ----
  if batt_I >= 0 then
// battery is discharging, from: https://doi.org/10.1016/j.conengprac.2011.06.008
    eta_b_dischr = 0.5*(1 + sqrt(1 - 4*internalResistance*batt_I*batt_V/OCV^2));
    eta_b_chr = 1;
  elseif batt_I < 0 then
// battery is charging
    eta_b_chr = 2/(1 + sqrt(1 - 4*internalResistance*batt_I*batt_V/OCV^2));
    eta_b_dischr = 1;
  else
    eta_b_chr = 1;
    eta_b_dischr = 1;
  end if;
  
  //---- When statement to avoide long loops ----
  when t >= pre(t_comp) + 2 then
    t_comp = pre(t_comp) + 2;
    if batt_I < 0 then
      // Battery is Charging
      eta_b_chr_counter = pre(eta_b_chr_counter) + 1;
      eta_b_chr_sum = pre(eta_b_chr_sum) + eta_b_chr;
      eta_b_chr_avg = eta_b_chr_sum/eta_b_chr_counter;
      eta_b_dischr_counter = pre(eta_b_dischr_counter);
      eta_b_dischr_sum = pre(eta_b_dischr_sum);
      eta_b_dischr_avg = pre(eta_b_dischr_avg);
      sigma = eta_b_dischr_avg*eta_b_chr;
    else
      // Battery is Discharging
      eta_b_dischr_counter = pre(eta_b_dischr_counter) + 1;
      eta_b_dischr_sum = pre(eta_b_dischr_sum) + eta_b_dischr;
      eta_b_dischr_avg = eta_b_dischr_sum/eta_b_dischr_counter;
      eta_b_chr_counter = pre(eta_b_chr_counter);
      eta_b_chr_sum = pre(eta_b_chr_sum);
      eta_b_chr_avg = pre(eta_b_chr_avg);
      sigma = 1/(eta_b_chr_avg*eta_b_dischr);
    end if;
    //---- The core of ECMS, the cost function is ran ----
    //set_current_1 = Py.nineRealArgumentsReturnReal(pyHandle, powerD, SOC, 1, batt_V, sigma, 1, fcTemperature, pH2, pO2, pyProgram, pyModuleName, pyFunctionName); //Cost function is ran.
    set_current_1 = noEvent(Py.nineRealArgumentsReturnReal(pyHandle, powerD, SOC, 1, batt_V, sigma, 1, fcTemperature, pH2, pO2, pyProgram, pyModuleName, pyFunctionName)); //Cost function is ran
    C_sum = pre(C_sum) + C;
  end when;
// ---- Final control sequence ----
  if SOC > SOC_max then // If the SOC of the battery is above the maximum allowed limit, the fuel cell is shut down until 2/3 of the allowed interval is reached
    turn_off = true;
  elseif SOC < (SOC_min + (2/3)*(SOC_max-SOC_min)) then
    turn_off = false;
  else
    turn_off = pre(turn_off);
  end if;
  if turn_off == false then
    set_current_2 = max(set_current_dcdc_out, I_min_fc_dcdc_out); //The fuel cell is minimum operated on the lower limited, no matter what the optimization solver outputs
  else
    set_current_2 = 0;
  end if;
  if set_current_2 < I_min_fc_dcdc_out or t < t_const then // If time is less then t_const, the fuel cell remains off
    set_current_3 = 0;
  else
    set_current_3 = set_current_2;
  end if;

  
  connect(slewRateLimiter.y, abs1.u) annotation(
    Line(points = {{74, 0}, {90, 0}}, color = {0, 0, 127}));
  connect(abs1.y, controlOutputFuelCellCurrent) annotation(
    Line(points = {{114, 0}, {210, 0}}, color = {0, 0, 127}));
  connect(setCurrent.y, filter.u) annotation(
    Line(points = {{-38, 0}, {-12, 0}}, color = {0, 0, 127}));
  connect(filter.y, slewRateLimiter.u) annotation(
    Line(points = {{12, 0}, {50, 0}}, color = {0, 0, 127}));
  annotation( //Update documentation. 
    Icon(graphics = {Rectangle(fillColor = {0, 70, 40}, fillPattern = FillPattern.Solid, lineThickness = 1.5, extent = {{-200, 100}, {200, -100}}, radius = 35), Rectangle(fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-43.2, 41.4}, {43, -45}}, radius = 8), Rectangle(origin = {-35, -3}, fillColor = {255, 0, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-22.2, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-35, -3}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, lineThickness = 0, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-9.5, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {3.3, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {16.1, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {28.85, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {0, 30}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, 17.2}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, 4.35}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -8.45}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -21.25}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -34.05}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(fillColor = {0, 70, 40}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{39.8, 38.3}, {-40, -41.8}}, radius = 5), Rectangle( fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-27.3, 25.5}, {27, -29}}), Rectangle(fillColor = {0, 70, 40}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-24.1, 22.25}, {23.85, -25.8}})}, coordinateSystem(extent = {{-200, -100}, {200, 100}}, initialScale = 0.1)), Diagram(coordinateSystem(extent = {{-200, -100}, {200, 100}}, initialScale = 0.1)),
    Documentation(info = "<html><head></head><body><div>The EnergyManagementSystem component is designed to manage the flow of power between the fuel cell stack, battery, vehicle load, and balance-of-plant load. It splits the load according to pre-defined energy management rules, which are implemented within the bounds of the battery management system and the fuel cell control unit.</div><div><br></div></body></html>"));

algorithm
  //if t > pre(t_comp) + 1 then
    //set_current_1 := delay(Py.nineRealArgumentsReturnReal(pyHandle, powerD, SOC, 1, batt_V, sigma, 1, fcTemperature, pH2, pO2, pyProgram, pyModuleName, pyFunctionName), 0.5, 1); //Cost function is ran.
  //end if;
  

end ECMS_experiment_testrun;
