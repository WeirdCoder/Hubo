function runHuboCatching()
%% Begin initialize robot models

options.floating = true;

r1 = HuboBiped();
r2 = ViconBall();

r = RigidBodyManipulator([],options);
r = r.addRobotFromURDF('../vicon/matlab-model/ball.urdf',[0;0;0],[0;0;0],options);
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


simoJoints = SIMOSplitter(34,[1, 6, 7,34]);
%Primary Trajectory Merging controller
TrajControl = HuboQPController(0.01); %UnTuned.

%Inverse Kinectmatic to joint vels.
ik = VelocityIKController(r1);


%% Connect all modules
clear ins outs input_select output_select;


velocitypdr1 = r1.pdcontrol(eye(r1.getNumInputs),eye(r1.getNumInputs)); %Velocity Controlled.
%Cascade Controllers

sysHand = mimoCascade(velocitypdr1,fk);%Hubo hand and ForwardK
sysHand = sysHand.setOutputFrame(simoHand.getInputFrame());
sysHand = mimoCascade(sysHand,simoHand);
misoHand = misoHand.setInputFrame(sysHand.getOutputFrame);
sysHand = mimoCascade(sysHand,misoHand);%Filter out rotational terms.


sysBall = r2;
sysBall = sysBall.setOutputFrame(simoBall.getInputFrame());
sysBall = mimoCascade(sysBall,simoBall);
misoBall = misoBall.setInputFrame(sysBall.getOutputFrame);
sysBall = mimoCascade(sysBall,misoBall);%Filter out rotational terms.


ik_inputFrame = ik.getInputFrame().frame;
TrajControl = TrajControl.setOutputFrame(MultiCoordinateFrame.constructFrame({ik_inputFrame{1}}));
sys = mimoCascade(TrajControl,ik);

sys = sys.setOutputFrame(simoJoints.getInputFrame());
sys = mimoCascade(sys,simoJoints);


% Hubo and ViconBall to Controller
ins(1).from_output = 1;
ins(1).to_input = 2;
outs(1).from_output = 2;
outs(1).to_input = 1;
sys_input_frame = sys.getInputFrame().frame;
sysHand = sysHand.setOutputFrame(MultiCoordinateFrame.constructFrame({sys_input_frame{2}}));

sys_output_frame = sys.getOutputFrame().frame;
sysHand = sysHand.setInputFrame(MultiCoordinateFrame.constructFrame({sys_output_frame{2}}));


sys = mimoFeedback(sysHand,sys,ins,outs,[],[]);

clear ins outs input_select output_select;
ins(1).from_output = 1;
ins(1).to_input = 1;

sys_input_frame = sys.getInputFrame().frame;
sysBall = sysBall.setOutputFrame(MultiCoordinateFrame.constructFrame({ sys_input_frame{1}}));

sys = mimoCascade(sysBall,sys,ins,[],[]);



%% Initialize Xload hubo_catching.mat;
load hubo_fp
r1.setInitialState(xstar);
load hubo_catching.mat
%x0(13:80) = xstar;
%% Simulate
[ytraj, xtraj] = sys.simulate([0 0.58],x0);
xtraj = xtraj(1:6,13:47,7:12,48:80);
%% Visualize
v = v.setInputFrame(sys.getStateFrame())
playback(v,xtraj,struct('slider',true));

end