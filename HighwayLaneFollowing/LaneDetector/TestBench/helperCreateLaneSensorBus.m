% helperCreateLaneSensorBus creates bus objects used by LaneMarkerDetectorTestBench.slx
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

% Copyright 2020 The MathWorks, Inc.

%%

LaneSensor = Simulink.Bus;
LaneSensor.Description = ['Describes sensor structure interface for lan' ...
                          'e sensor'];
LaneSensor.DataScope = 'Auto';
LaneSensor.HeaderFile = '';
LaneSensor.Alignment = -1;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'Left';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = 1;
saveVarsTmp{1}.DataType = 'Bus: LaneSensorBoundaries';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.SampleTime = -1;
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'Right';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = 1;
saveVarsTmp{1}(2, 1).DataType = 'Bus: LaneSensorBoundaries';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).SampleTime = -1;
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
LaneSensor.Elements = saveVarsTmp{1};
clear saveVarsTmp;

LaneSensorBoundaries = Simulink.Bus;
LaneSensorBoundaries.Description = ['Describes sensor structure interfa' ...
                                    'ce for lane boundaries from lane s' ...
                                    'ensor'];
LaneSensorBoundaries.DataScope = 'Auto';
LaneSensorBoundaries.HeaderFile = '';
LaneSensorBoundaries.Alignment = -1;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'Curvature';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = 1;
saveVarsTmp{1}.DataType = 'single';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.SampleTime = -1;
saveVarsTmp{1}.DocUnits = 'rad/m';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'CurvatureDerivative';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = 1;
saveVarsTmp{1}(2, 1).DataType = 'single';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).SampleTime = -1;
saveVarsTmp{1}(2, 1).DocUnits = 'rad/m^2';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'HeadingAngle';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = 1;
saveVarsTmp{1}(3, 1).DataType = 'single';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).SampleTime = -1;
saveVarsTmp{1}(3, 1).DocUnits = 'rad';
saveVarsTmp{1}(3, 1).Description = '';
saveVarsTmp{1}(4, 1) = Simulink.BusElement;
saveVarsTmp{1}(4, 1).Name = 'LateralOffset';
saveVarsTmp{1}(4, 1).Complexity = 'real';
saveVarsTmp{1}(4, 1).Dimensions = 1;
saveVarsTmp{1}(4, 1).DataType = 'single';
saveVarsTmp{1}(4, 1).Min = [];
saveVarsTmp{1}(4, 1).Max = [];
saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(4, 1).SampleTime = -1;
saveVarsTmp{1}(4, 1).DocUnits = 'm';
saveVarsTmp{1}(4, 1).Description = '';
saveVarsTmp{1}(5, 1) = Simulink.BusElement;
saveVarsTmp{1}(5, 1).Name = 'Strength';
saveVarsTmp{1}(5, 1).Complexity = 'real';
saveVarsTmp{1}(5, 1).Dimensions = 1;
saveVarsTmp{1}(5, 1).DataType = 'single';
saveVarsTmp{1}(5, 1).Min = [];
saveVarsTmp{1}(5, 1).Max = [];
saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(5, 1).SampleTime = -1;
saveVarsTmp{1}(5, 1).DocUnits = '';
saveVarsTmp{1}(5, 1).Description = '';
saveVarsTmp{1}(6, 1) = Simulink.BusElement;
saveVarsTmp{1}(6, 1).Name = 'XExtent';
saveVarsTmp{1}(6, 1).Complexity = 'real';
saveVarsTmp{1}(6, 1).Dimensions = [1 2];
saveVarsTmp{1}(6, 1).DataType = 'single';
saveVarsTmp{1}(6, 1).Min = [];
saveVarsTmp{1}(6, 1).Max = [];
saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(6, 1).SampleTime = -1;
saveVarsTmp{1}(6, 1).DocUnits = '';
saveVarsTmp{1}(6, 1).Description = '';
saveVarsTmp{1}(7, 1) = Simulink.BusElement;
saveVarsTmp{1}(7, 1).Name = 'BoundaryType';
saveVarsTmp{1}(7, 1).Complexity = 'real';
saveVarsTmp{1}(7, 1).Dimensions = 1;
saveVarsTmp{1}(7, 1).DataType = 'Enum: LaneBoundaryType';
saveVarsTmp{1}(7, 1).Min = []; 
saveVarsTmp{1}(7, 1).Max = [];
saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(7, 1).SampleTime = -1;
saveVarsTmp{1}(7, 1).DocUnits = '';
saveVarsTmp{1}(7, 1).Description = '';
LaneSensorBoundaries.Elements = saveVarsTmp{1};
clear saveVarsTmp;

