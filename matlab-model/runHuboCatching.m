function runHuboCatching()
%% Begin initialize robot models

options.floating = false;

r1 = HuboBiped('urdf/hubo_minimal_contact.urdf',options);
r2 = ViconBall();
options.floating = true;
r = RigidBodyManipulator([],options);
r = r.addRobotFromURDF('../vicon/matlab-model/ball.urdf',[0;0;0],[0;0;0],options);
options.floating = false;
r = r.addRobotFromURDF('urdf/hubo_minimal_contact.urdf',[0;0;0],[0;0;0],options);

v = r.constructVisualizer;

%% Initialize all modules
%Robot Hand
fk = ForwardKin(r1);

%Filter out Rotational Coordinates
simoHand = SIMOSplitter(12,[1 , 3 , 7,9]);
misoHand = MISOJoiner(simoHand.getOutputFrame().frame);

simoBall = SIMOSplitter(12,[1 , 3 , 7,9]);  %For the ball, only if needed.
misoBall = MISOJoiner(simoBall.getOutputFrame().frame);


%simoJoints = SIMOSplitter(34,[1, 6, 7,34]);
%Primary Trajectory Merging controller
TrajControl = NewHuboQPController(0.001,100); %UnTuned.

%Inverse Kinectmatic to joint vels.
ik = VelocityIKController(r1);


%% Connect all modules
clear ins outs input_select output_select;
output_select(1).system = 2;
output_select(1).output = 1;
load HuboControllerGains
PGain = zeros(28);
DGain = zeros(28);
% %Preset
%   PGain(15,15) = 0.08;
%   DGain(15,15) = 0.00025;
%   PGain(14,14) = 0.005;
%   DGain(14,14) = 0.0002;
%   PGain(13,13) = 2;%1
%   DGain(13,13) = 0.0004;%0.0004
%   PGain(12,12) = 0.1;
%   DGain(12,12) = 0.0005;
%   PGain(11,11) = 8;
%   DGain(11,11) = 0.01;
%   PGain(10,10) = 8;
%   DGain(10,10) = 0.01;

  PGain(15,15) = 0.008;
  DGain(15,15) = 0.00025;
  PGain(14,14) = 0.005;
  DGain(14,14) = 0.0002;
  PGain(13,13) = 0.1;%1
  DGain(13,13) = 0.0004;%0.0004
  PGain(12,12) = 0.2;
  DGain(12,12) = 0.0005;
  PGain(11,11) = 1;
  DGain(11,11) = 0.001;
  PGain(10,10) = 1;
  DGain(10,10) = 0.001;
%  PGain(13,13) = 20;
%  DGain(13,13) = 1;

PGain = PGain*5;
DGain = DGain*0.5;
velocitypdr1 = VelocityRBMController(r1,PGain,DGain); %Velocity Controlled.  312.5*PGain,0.2*DGain)
velocitypdr1 = velocitypdr1.setOutputFrame(r1.getInputFrame());
velocitypdr1 = mimoFeedback(velocitypdr1,r1,[],[],[],output_select);
clear ins outs input_select output_select;
%Cascade Controllers
ins = struct();
ins(1).from_output = 1;
ins(1).to_input = 1;
input_select(1).system = 1;
input_select(1).input = 1;
output_select(1).system = 2;
output_select(1).output = 1;
output_select(2).system = 1;
output_select(2).output = 1;
sysHand = mimoCascade(velocitypdr1,fk,ins,input_select,output_select);%Hubo hand and ForwardK
clear ins outs input_select output_select;

simoHand = simoHand.setInputFrame(sysHand.getOutputFrame.frame{1});
sysHand = mimoCascade(sysHand,simoHand);
misoHand = misoHand.setInputFrame(MultiCoordinateFrame(sysHand.getOutputFrame.frame(2:3)));
ins(1).from_output = 2;
ins(1).to_input = 1;
ins(2).from_output = 3;
ins(2).to_input = 2;
sysHand = mimoCascade(sysHand,misoHand,ins);%Filter out rotational terms.

clear ins outs input_select output_select;
sysBall = r2;

ins(1).from_output = 1;
ins(1).to_input = 1;
output_select(1).system = 1;
output_select(1).output = 1;
output_select(2).system = 2;
output_select(2).output = 1;
output_select(3).system = 2;
output_select(3).output = 2;
simoBall = simoBall.setInputFrame(sysBall.getOutputFrame());
sysBall = mimoCascade(sysBall,simoBall,ins,[],output_select);
misoBall = misoBall.setInputFrame(MultiCoordinateFrame(sysBall.getOutputFrame.frame(2:3)));
sysBall = mimoCascade(sysBall,misoBall);%Filter out rotational terms.


ik_inputFrame = ik.getInputFrame().frame;
TrajControl = TrajControl.setOutputFrame(MultiCoordinateFrame.constructFrame({ik_inputFrame{1}}));
sys = mimoCascade(TrajControl,ik);

%sys = sys.setOutputFrame(simoJoints.getInputFrame());
%sys = mimoCascade(sys,simoJoints);


% Hubo and ViconBall to Controller

clear ins outs input_select output_select;
ins(1).from_output = 1;
ins(1).to_input = 3;
ins(2).from_output = 2;
ins(2).to_input = 2;
output_select(1).system = 1;
output_select(1).output = 2;
sysHand = sysHand.setOutputFrame(MultiCoordinateFrame(sys.getInputFrame.frame([3 2])));

sys_output_frame = sys.getOutputFrame();
sysHand = sysHand.setInputFrame(MultiCoordinateFrame.constructFrame({sys_output_frame}));


sys = mimoFeedback(sysHand,sys,ins,[],[],output_select);

clear ins outs input_select output_select;
ins(1).from_output = 2;
ins(1).to_input = 1;

output_select(1).system = 2;
output_select(1).output = 1;

output_select(2).system = 1;
output_select(2).output = 1;
sys = sys.setInputFrame(sysBall.getOutputFrame.frame{2});

sys = mimoCascade(sysBall,sys, ins, [],output_select);

%% Initialize Xload hubo_catching.mat;
load hubo_fp
load hubo_catching.mat
x0(9) = 2;
x0(1) = 0.0638+1;
x0(7) = -4;
x0(2) = -0.2803-0.2;
x0(3) = 1.6223+0.3;
x0(81:110) = zeros(30,1);
x0 = x0([19:46 53:80 1:6 7:12 81:109]);
%x0(57:62) = x0(1:6);
%% Simulate
tic;
[ytraj, xtraj] = sys.simulate([0 0.5],x0');
toc;


%load Hubo_Vicon_traj_adjIndex
%xtraj = MixedTrajectory(xtraj.trajs,newtrajIndex);
%% Visualize
xtraj2 = xtraj([57:62 1:28 63:68 29:56]);%[7:12 1:6 13:40 41:68]);
xtraj = MixedTrajectory({xtraj2},{[1:68]});
v = v.setInputFrame(xtraj.getOutputFrame());
playback(v,xtraj,struct('slider',true));

%Analysis
figure(2);
targetTraj = xtraj.eval(xtraj.breaks);
plot(targetTraj([40:end],:)');%53
end