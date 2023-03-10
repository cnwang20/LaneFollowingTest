% helperCreateVehicleDetectorBusObjects creates bus objects used by
% VisionVehicleDetectorTestBench.slx
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

% Copyright 2020-2021 The MathWorks, Inc.
           
%% Create BusVision
BusVision = Simulink.Bus;
BusVision.Description = '';
BusVision.DataScope = 'Auto';
BusVision.HeaderFile = '';
BusVision.Alignment = -1;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'NumDetections';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.SampleTime = -1;
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'IsValidTime';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [1 1];
saveVarsTmp{1}(2, 1).DataType = 'boolean';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).SampleTime = -1;
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'Detections';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = [20 1];
saveVarsTmp{1}(3, 1).DataType = 'Bus: BusVisionDetections';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).SampleTime = -1;
saveVarsTmp{1}(3, 1).DocUnits = '';
saveVarsTmp{1}(3, 1).Description = '';
BusVision.Elements = saveVarsTmp{1};
clear saveVarsTmp;

%% Create BusVisionDetections
BusVisionDetections = Simulink.Bus;
BusVisionDetections.Description = '';
BusVisionDetections.DataScope = 'Auto';
BusVisionDetections.HeaderFile = '';
BusVisionDetections.Alignment = -1;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'Time';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.SampleTime = -1;
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'Measurement';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [3 1];
saveVarsTmp{1}(2, 1).DataType = 'double';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).SampleTime = -1;
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'MeasurementNoise';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = [3 3];
saveVarsTmp{1}(3, 1).DataType = 'double';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).SampleTime = -1;
saveVarsTmp{1}(3, 1).DocUnits = '';
saveVarsTmp{1}(3, 1).Description = '';
saveVarsTmp{1}(4, 1) = Simulink.BusElement;
saveVarsTmp{1}(4, 1).Name = 'SensorIndex';
saveVarsTmp{1}(4, 1).Complexity = 'real';
saveVarsTmp{1}(4, 1).Dimensions = [1 1];
saveVarsTmp{1}(4, 1).DataType = 'double';
saveVarsTmp{1}(4, 1).Min = [];
saveVarsTmp{1}(4, 1).Max = [];
saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(4, 1).SampleTime = -1;
saveVarsTmp{1}(4, 1).DocUnits = '';
saveVarsTmp{1}(4, 1).Description = '';
saveVarsTmp{1}(5, 1) = Simulink.BusElement;
saveVarsTmp{1}(5, 1).Name = 'ObjectClassID';
saveVarsTmp{1}(5, 1).Complexity = 'real';
saveVarsTmp{1}(5, 1).Dimensions = [1 1];
saveVarsTmp{1}(5, 1).DataType = 'double';
saveVarsTmp{1}(5, 1).Min = [];
saveVarsTmp{1}(5, 1).Max = [];
saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(5, 1).SampleTime = -1;
saveVarsTmp{1}(5, 1).DocUnits = '';
saveVarsTmp{1}(5, 1).Description = '';
saveVarsTmp{1}(6, 1) = Simulink.BusElement;
saveVarsTmp{1}(6, 1).Name = 'ObjectAttributes';
saveVarsTmp{1}(6, 1).Complexity = 'real';
saveVarsTmp{1}(6, 1).Dimensions = [1 1];
saveVarsTmp{1}(6, 1).DataType = ['Bus: BusVisionDetectionsObjectAttribu' ...
                                 'tes'];
saveVarsTmp{1}(6, 1).Min = [];
saveVarsTmp{1}(6, 1).Max = [];
saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(6, 1).SampleTime = -1;
saveVarsTmp{1}(6, 1).DocUnits = '';
saveVarsTmp{1}(6, 1).Description = '';
saveVarsTmp{1}(7, 1) = Simulink.BusElement;
saveVarsTmp{1}(7, 1).Name = 'MeasurementParameters';
saveVarsTmp{1}(7, 1).Complexity = 'real';
saveVarsTmp{1}(7, 1).Dimensions = [1 1];
saveVarsTmp{1}(7, 1).DataType = ['Bus: BusVisionDetectionsMeasurementPa' ...
                                 'rameters'];
saveVarsTmp{1}(7, 1).Min = [];
saveVarsTmp{1}(7, 1).Max = [];
saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(7, 1).SampleTime = -1;
saveVarsTmp{1}(7, 1).DocUnits = '';
saveVarsTmp{1}(7, 1).Description = '';
BusVisionDetections.Elements = saveVarsTmp{1};
clear saveVarsTmp;

%% Create BusVisionDetectionsMeasurementParameters
BusVisionDetectionsMeasurementParameters = Simulink.Bus;
BusVisionDetectionsMeasurementParameters.Description = '';
BusVisionDetectionsMeasurementParameters.DataScope = 'Auto';
BusVisionDetectionsMeasurementParameters.HeaderFile = '';
BusVisionDetectionsMeasurementParameters.Alignment = -1;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'Frame';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'Enum: drivingCoordinateFrameType';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.SampleTime = -1;
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'OriginPosition';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [3 1];
saveVarsTmp{1}(2, 1).DataType = 'double';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).SampleTime = -1;
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'Orientation';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = [3 3];
saveVarsTmp{1}(3, 1).DataType = 'double';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).SampleTime = -1;
saveVarsTmp{1}(3, 1).DocUnits = '';
saveVarsTmp{1}(3, 1).Description = '';
saveVarsTmp{1}(4, 1) = Simulink.BusElement;
saveVarsTmp{1}(4, 1).Name = 'HasVelocity';
saveVarsTmp{1}(4, 1).Complexity = 'real';
saveVarsTmp{1}(4, 1).Dimensions = [1 1];
saveVarsTmp{1}(4, 1).DataType = 'boolean';
saveVarsTmp{1}(4, 1).Min = [];
saveVarsTmp{1}(4, 1).Max = [];
saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(4, 1).SampleTime = -1;
saveVarsTmp{1}(4, 1).DocUnits = '';
saveVarsTmp{1}(4, 1).Description = '';
BusVisionDetectionsMeasurementParameters.Elements = saveVarsTmp{1};
clear saveVarsTmp;

%% Create BusVisionDetectionsObjectAttributes
BusVisionDetectionsObjectAttributes = Simulink.Bus;
BusVisionDetectionsObjectAttributes.Description = '';
BusVisionDetectionsObjectAttributes.DataScope = 'Auto';
BusVisionDetectionsObjectAttributes.HeaderFile = '';
BusVisionDetectionsObjectAttributes.Alignment = -1;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'TargetIndex';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.SampleTime = -1;
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
BusVisionDetectionsObjectAttributes.Elements = saveVarsTmp{1};
clear saveVarsTmp;

%% Create BusVision Info
BusVisionInfo = Simulink.Bus;
BusVisionInfo.Description = '';
BusVisionInfo.DataScope = 'Auto';
BusVisionInfo.HeaderFile = '';
BusVisionInfo.Alignment = -1;
BusVisionInfo.PreserveElementDimensions = false;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'NumDetections';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'Detections';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [20 1];
saveVarsTmp{1}(2, 1).DataType = 'Bus: BusVisionDetectionsInfo';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
BusVisionInfo.Elements = saveVarsTmp{1};
clear saveVarsTmp;

%% Create BusVisionDetectionsInfo
BusVisionDetectionsInfo = Simulink.Bus;
BusVisionDetectionsInfo.Description = '';
BusVisionDetectionsInfo.DataScope = 'Auto';
BusVisionDetectionsInfo.HeaderFile = '';
BusVisionDetectionsInfo.Alignment = -1;
BusVisionDetectionsInfo.PreserveElementDimensions = false;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'Boxes';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 4];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'Scores';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [1 1];
saveVarsTmp{1}(2, 1).DataType = 'double';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
BusVisionDetectionsInfo.Elements = saveVarsTmp{1};
clear saveVarsTmp;