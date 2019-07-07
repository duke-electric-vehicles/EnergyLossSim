clear; clc; close all;

%all loss terms are in watts

v = 6.706; %nominal race speed in m/s
massCar = 26; %mass of car in kg
massDriver = 45; %mass of driver in kg
g = 9.807; %acceleration of gravity

massTotal = massCar + massDriver;

totalLosses = [];
totalLossesLabels = {};

%air drag------------------------------------------------
airCdA = 0.0438; %coefficient of drag
airDensity = 1.225; %density of air, kg/m^3

% notes:
%   the speed of the car is not constant, as it accelerates from a
%   standstill at the beginning. This variable speed is compressed into a
%   single "RMS" multiplier
airRMSMultiplier = 1.0149; %rms of the velocity profile, world record attempt

airForce = 0.5 * airDensity * airCdA * (v * airRMSMultiplier)^2;
airPower = airForce * v;

totalLosses = [totalLosses, airPower];
totalLossesLabels{end+1} = 'External air drag';

%rolling resistance--------------------------------------
rrCoeff = 0.0015; %coefficient of rolling resistance
rrForce = massTotal * g * rrCoeff; %drag force of rolling resistance in newtons
rrPower = rrForce * v; %power loss of rolling resistance in watts

totalLosses = [totalLosses, rrPower];
totalLossesLabels{end+1} = 'Tire rolling resistance';

%cornering losses----------------------------------------
ca = 120; %tire cornering stiffness, newtons per degree
cornerRadius = [25 25 25 25]; %turn radius in meters. assuming each corner turns 90 degrees
cornerVelocity = [v v v v]; %speed taken through each corner
trackLength = 1947.1; %track length in meters. Galot raceway in Benson, NC

alpha = (massTotal .* cornerVelocity.^2 ./ cornerRadius) ./ ca; %tire slip angle, degrees
corneringDragForce = ca .* alpha .^ 2 .* pi ./ 180; %traction force needed

corneringPower = corneringDragForce .* v;
corneringAveragePower = sum(corneringPower .* 0.5 .* pi .* cornerRadius ./ trackLength);

totalLosses = [totalLosses, corneringAveragePower];
totalLossesLabels{end+1} = 'Tire cornering losses';

%wheel air drag------------------------------------------
wheelDia = 0.475; %diameter of the wheel in m
wheelOmega = v / (wheelDia / 2);
wheelCdA = 1.1e-3;
wheelAirLoss = 3 * 0.5 * airDensity * v^3 * wheelCdA; %three wheels in the car

totalLosses = [totalLosses, wheelAirLoss];
totalLossesLabels{end+1} = 'Internal wheel air drag';
%bearing drag--------------------------------------------
Tb = 4.9e-3; %bearing frictional moment, Nm
bearingLoss = 3 * v * Tb /  (wheelDia/2); %three wheels in the car

totalLosses = [totalLosses, bearingLoss];
totalLossesLabels{end+1} = 'Wheel bearing drag';

%motor losses--------------------------------------------
motorGearRatio = 120/14;
motorRPM = wheelOmega * motorGearRatio * 60/(2*pi);
motorCurrent = 5;
motorVoltage = 16;

modelKv = 189;
modelLs = 0.160e-3 * 1.75;
modelRs = 0.186 * 1.34;
PvsERPM = [-9.5593e-13   4.6815e-08   3.3604e-04    0];
modelKt = 1./(modelKv*2*pi/60);

noLoadTorque =  polyval(PvsERPM,motorRPM*2)./(motorRPM/60*2*pi);
modelTorque = motorCurrent*modelKt  - noLoadTorque;
modelRsTmp = sqrt(modelRs.^2 + (modelLs*motorRPM*2 / 60 * 2 * pi).^2);
V = motorRPM / modelKv + motorCurrent.*modelRsTmp;

% notes:
%   controller in world record car was older than the dyno motor controller
%       and drew 600mW instead of 300mW

modelLosses(:,:,1) = motorCurrent.^2.*modelRs;     % I2R
modelLosses(:,:,2) = polyval(PvsERPM,motorRPM*2);  % nonelectrical
modelLosses(:,:,3) = 0.6*ones(size(motorRPM));     % controller
modelLosses(:,:,4) = 6e-3*modelTorque.^2.*motorRPM;% transmission

motorDutyCycle = sum(totalLosses) / (motorCurrent * motorVoltage);

totalLosses = [totalLosses, sum(modelLosses)*motorDutyCycle];
totalLossesLabels{end+1} = 'Motor losses';

motorEff = 1 - (sum(modelLosses)*motorDutyCycle) / sum(totalLosses);

%pre-hydrogen pie chart----------------------------------
figure;

H = pie(totalLosses);
T = H(strcmpi(get(H,'Type'),'text'));
P = cell2mat(get(T,'Position'));
set(T,{'Position'},num2cell(P*0.6,2));
P(1, 1) = -1.7;%hack to make text better
text(P(:,1),P(:,2),totalLossesLabels(:));

%fuel cell-----------------------------------------------
h2Eff = 0.584;
electricalPower = sum(totalLosses);
fuelCellLoss = electricalPower / h2Eff - electricalPower;

totalLosses = [totalLosses, fuelCellLoss];
totalLossesLabels{end+1} = 'Fuel cell losses';

%plotting and scorekeeping------------------------------
figure;
H = pie(totalLosses);
T = H(strcmpi(get(H,'Type'),'text'));
P = cell2mat(get(T,'Position'));
set(T,{'Position'},num2cell(P*0.6,2));
text(P(:,1),P(:,2),totalLossesLabels(:));


totalPower = sum(totalLosses);
electricScoreMetric = v ./ (electricalPower ./ 3600); %electric score in km per kWh
electricScoreEnglish = electricScoreMetric ./ 1.609; %electric score in miles per kWh
joulesPerLiterGas = 42.9e6 * 0.7646; %using constants from 2018 Eco-Marathon rules
scoreMetric = joulesPerLiterGas .* v ./ (totalPower .* 1000); %hydrogen score in km per liter of gas
scoreEnglish = scoreMetric ./ 1.609 .* 3.78541; % miles per gallon

%note, actual world record hydrogen score was 14,573 MPG, 6196 km/L

fprintf('Predicted electric score: %.1f mi/kWh, Hydrogen score: %.1f km/L\n', electricScoreEnglish, scoreMetric);