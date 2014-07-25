function runHuboCatching()
%% Begin initialize robot models
 

r1 = HuboBiped();
r2 = ViconBall();

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
TrajControl = QPController(0.01); %UnTuned.

%Inverse Kinectmatic to joint vels.
ik = VelocityIKController(r1);


%% Connect all modules
clear ins outs input_select output_select;


r1 = r1.pdcontrol(eye(r1.getNumInputs),eye(r1.getNumInputs)); %Velocity Controlled.
%Cascade Controllers

sysHand = mimoCascade(r1,fk);%Hubo hand and ForwardK
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



%% Initialize X0
load hubo_catching.mat

%% Simulate
sys.simulate([0 1],x0);

%% Visualize

end