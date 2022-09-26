function motive_new(block)

setup(block);

 
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 8;

% Override input port properties
block.InputPort(1).Dimensions        = 1;
%block.InputPort(1).DirectFeedthrough = false;

% Override output port properties
block.OutputPort(1).Dimensions       = 3;
block.OutputPort(2).Dimensions       = 3;
block.OutputPort(3).Dimensions       = 3;
block.OutputPort(4).Dimensions       = 3;
block.OutputPort(5).Dimensions       = 3;
block.OutputPort(6).Dimensions       = 3;
block.OutputPort(7).Dimensions       = 3;
block.OutputPort(8).Dimensions       = 3;

block.SampleTimes = [-1 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required
block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);

 
function DoPostPropSetup(block)
block.NumDworks = 1;
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

  
function SetInputPortSamplingMode(block, idx, fd)
block.InputPort(idx).SamplingMode = fd;
block.OutputPort(1).SamplingMode = fd;
block.OutputPort(2).SamplingMode = fd;
block.OutputPort(3).SamplingMode = fd;
block.OutputPort(4).SamplingMode = fd;
block.OutputPort(5).SamplingMode = fd;
block.OutputPort(6).SamplingMode = fd;
block.OutputPort(7).SamplingMode = fd;
block.OutputPort(8).SamplingMode = fd;

 
 
function InitializeConditions(block)
disp('NatNet Sample Begin')
global theClient;
global frameRate
disp('[NatNet] Connecting to OptiTrack Server.');
HostIP = char('131.155.144.102');
ClientIP = char('131.155.144.102');
dllPath = fullfile('D:','NatNet_SDK_4.0','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(1);
flg = theClient.Initialize(ClientIP,HostIP);
if (flg == 0)
    disp('[NatNet] Initialization Succeeded')
else
    disp('[NatNet] Initialization Failed')
end

GetDataDescriptions(theClient)
[byteArray, retCode] = theClient.SendMessageAndWait('FrameRate');
if(retCode ==0)
    byteArray = uint8(byteArray);
    frameRate = typecast(byteArray,'single');
end

function Outputs(block)
global theClient;
data = theClient.GetLastFrameOfData();
% Tractor 2
rigidBodyData1 = data.RigidBodies(1);
[ang1_tractor2,ang2_tractor2,ang3_tractor2] = quat2angle([rigidBodyData1.qw rigidBodyData1.qx rigidBodyData1.qy rigidBodyData1.qz],'YZX'); 
Pitch_tractor2 =   (ang1_tractor2 * 180.0 / pi); 
Yaw_tractor2   =   (ang2_tractor2 * 180.0 / pi);
Roll__tractor2  =   (ang3_tractor2 * 180.0 / pi);

block.OutputPort(1).Data = double([rigidBodyData1.x -rigidBodyData1.y rigidBodyData1.z ]);
block.OutputPort(2).Data = double([Pitch_tractor2 Yaw_tractor2 Roll__tractor2]);

% Tractor 3
rigidBodyData2 = data.RigidBodies(2);
[ang1_trailer2,ang2_trailer2,ang3_trailer2] = quat2angle([rigidBodyData2.qw rigidBodyData2.qx rigidBodyData2.qy rigidBodyData2.qz],'YZX'); 
Pitch_trailer2 =   (ang1_trailer2 * 180.0 / pi); 
Yaw_trailer2   =   (ang2_trailer2 * 180.0 / pi);
Roll_trailer2  =   (ang3_trailer2 * 180.0 / pi);

block.OutputPort(3).Data = double([rigidBodyData2.x -rigidBodyData2.y rigidBodyData2.z ]);
block.OutputPort(4).Data = double([Pitch_trailer2 Yaw_trailer2 Roll_trailer2]);

% Trailer 2
rigidBodyData3 = data.RigidBodies(3);
[ang1_tractor3,ang2_tractor3,ang3_tractor3] = quat2angle([rigidBodyData3.qw rigidBodyData3.qx rigidBodyData3.qy rigidBodyData3.qz],'YZX'); 
Pitch_tractor3 =   (ang1_tractor3 * 180.0 / pi); 
Yaw_tractor3  =   (ang2_tractor3 * 180.0 / pi);
Roll_tractor3  =   (ang3_tractor3 * 180.0 / pi);

block.OutputPort(5).Data = double([rigidBodyData3.x -rigidBodyData3.y rigidBodyData3.z ]);
block.OutputPort(6).Data = double([Pitch_tractor3 Yaw_tractor3 Roll_tractor3]);

% Trailer 3
rigidBodyData4 = data.RigidBodies(4);
[ang1_trailer3,ang2_trailer3,ang3_trailer3] = quat2angle([rigidBodyData4.qw rigidBodyData4.qx rigidBodyData4.qy rigidBodyData4.qz],'YZX'); 
Pitch_trailer3 =   (ang1_trailer3 * 180.0 / pi); 
Yaw_trailer3   =   (ang2_trailer3 * 180.0 / pi);
Roll_trailer3  =   (ang3_trailer3 * 180.0 / pi);

block.OutputPort(7).Data = double([rigidBodyData4.x -rigidBodyData4.y rigidBodyData4.z ]);
block.OutputPort(8).Data = double([Pitch_trailer3 Yaw_trailer3 Roll_trailer3]);






 
function Terminate(~)
global theClient;
theClient.Uninitialize();
disp('NatNet Sample End')




