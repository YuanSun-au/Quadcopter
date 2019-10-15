%% Breach script for closed_loop.slx

%% Set inport switch
% We need to set the inport switch so that it reads the "From Breach"
% inports
load_system('closed_loop');
set_param('closed_loop/Manual Switch','sw','1');

%% Initialize Breach
% Disable some warnings that make the script look ugly
warning('off', 'Simulink:Engine:OutputNotConnected');
warning('off', 'Simulink:Engine:InputNotConnected');

InitBreach;
B = BreachSimulinkSystem('closed_loop');

% We can print the signals of the system
B.PrintSignals();

% We can also find specifically just the inputs of the system
inputs = B.Sys.InputList;
disp('Inputs:');
disp(inputs);



%% Create input generators
% We create input generators for all inputs of the system
% As we saw in the 'inputs' list, the input of the system are:
% - Base_Thrust
% - Ref_Roll
% - Ref_Pitch
% - Ref_YawRate

% We can generate signals just by having fixed control points. 

% NOT MANDATORY:
% You can also try out any of these 3 other input generators:
% - pulse_signal_gen
% - var_cp_signal_gen
% - var_step_signal_gen 
% To use them, you can just comment out the fixed_cp_signal_gen and instead
%   uncomment the generator you want to use. 

% NOTE: It's up to you how many control points to use - you don't
%   necessarily have to use 3. 

% Pitch reference input generator: choose one by commenting/uncommenting!
% thrust_gen    = pulse_signal_gen({'Base_Thrust'});
thrust_gen    = fixed_cp_signal_gen({'Base_Thrust'},... % Signal name
                                    3); % Number of control points
% thrust_gen    = var_cp_signal_gen({'Base_Thrust'},... % Signal name
%                                     3); % Number of control points
% thrust_gen    = var_step_signal_gen({'Base_Thrust'},... % Signal name
%                                     3); % Number of control points

% Roll reference input generator: choose one by commenting/uncommenting!
% roll_gen    = pulse_signal_gen({'Ref_Roll'});
roll_gen    = fixed_cp_signal_gen({'Ref_Roll'}, ... % Signal name
                                     3); % Number of control points
% roll_gen    = var_cp_signal_gen({'Ref_Roll'},... % Signal name
%                                     3); % Number of control points
% roll_gen    = var_step_signal_gen({'Ref_Roll'},...
%                                     3); % Number of control points

% Pitch reference input generator: choose one by commenting/uncommenting!
% pitch_gen    = pulse_signal_gen({'Ref_Pitch'});
pitch_gen    = fixed_cp_signal_gen({'Ref_Pitch'},... % Signal name
                                    3); % Number of control points
% pitch_gen    = var_cp_signal_gen({'Ref_Pitch'},... % Signal name
%                                     3); % Number of control points
% pitch_gen    = var_step_signal_gen({'Pitch_reference'},... % Signal name
%                                     3); % Number of control points

% Pitch reference input generator: choose one by commenting/uncommenting!
% yawrate_gen    = pulse_signal_gen({'Ref_YawRate'});
yawrate_gen    = fixed_cp_signal_gen({'Ref_YawRate'},... % Signal name
                                    3); % Number of control points
% yawrate_gen    = var_cp_signal_gen({'Ref_YawRate'},... % Signal name
%                                     3); % Number of control points
% yawrate_gen    = var_step_signal_gen({'Ref_YawRate'},... % Signal name
%                                     3); % Number of control points



% We have to combine all the different generators into a Breach generator
% system
InputGen = BreachSignalGen({thrust_gen, roll_gen, pitch_gen, yawrate_gen});

% We tell Breach that InputGen is the generator to use for our system
B.SetInputGen(InputGen);

% We can print all the parameters of the system after creating the input
% generators
B.PrintParams();

% Note that all the parameters have been initiated to 0. 
% NOTE: To get interesting simulations, you should change them all!

%% Set input generator parameters

% We can change the parameters manually in the following way:
B.SetParam({'Base_Thrust_u0','Base_Thrust_u1','Base_Thrust_u2'},...
                        [20000 30000 14000]);
B.SetParam({'Ref_Roll_u0', 'Ref_Roll_u1', 'Ref_Roll_u2'},...
                        [5 0 10]);          
                    
% For some input parameters we might want to assign a RANGE of values. 
% This is done in the following way:
B.SetParamRanges({'Ref_Pitch_u0', 'Ref_Pitch_u1', 'Ref_Pitch_u2'},...
                       [-10 10; -10 10; -10 10]);
                   
% Print the parameters again to see the changed values
B.PrintParams();
                    
%% Sample parameters
% For all parameters that have a range defined, we can sample parameter
% values in the following way:
B.QuasiRandomSample(3); % Samples 3 values

%% Simulate the system
TotalSimulationTime = 10;
B.SetTime(TotalSimulationTime); % Set the total simulation time to e.g. 10s
B.Sim(); % Simulate the system

%% Plot results
% We can plot each signal against its reference for the first scenario ...
%% Figure 1
B.PlotSignals({'Base_Thrust'},...% Signals to plot
	[1],... % Which scenario to plot (only the first)
	{},... % Additional options
	1); % Boolean indicating that signals should be plotted in ONE figure 

%% Figure 2
B.PlotSignals({'Ref_Roll','Roll'},... % Signals to plot
	[1],... % Which scenario to plot (only the first)
	{},... % Additional options
	1); % Boolean indicating that signals should be plotted in ONE figure

%% Figure 3
B.PlotSignals({'Ref_Pitch','Pitch'},...% Signals to plot
	[1],... % Which scenario to plot (only the first)
	{},... % Additional options
	1); % Boolean indicating that signals should be plotted in ONE figure

%% Figure 4
B.PlotSignals({'Ref_YawRate','Yaw'},...% Signals to plot
	[1],... % Which scenario to plot (only the first)
	{},... % Additional options
	1); % Boolean indicating that signals should be plotted in ONE figure      


            
% ... and we can also plot ALL signals in ALL scenarios in ONE figure
%% Figure 5
B.PlotSignals({'Roll_reference','Roll','Pitch_reference','Pitch','Yaw_reference','Yaw'});

%% Figure 6
B.PlotSignals({'Base_Thrust','x','y','z'});
            

%% Write specifications for the system
% Now we want to really test the system to see if it fulfills
% specifications

% To see which signals the system has, use the following:
B.PrintSignals();
% The signals we can use in our STL formulae are:
% - Pitch[t]
% - Roll[t]
% - Yaw[t]
% - Roll_reference[t]
% - Pitch_reference[t]
% - Yaw_reference[t]
% - Yawrate_reference[t]

% We want to see if the pitch is always close to its reference or not
% The STL specification for this could be:
pitch_close_to_ref = STL_Formula('pitch_close_to_ref','alw_[0,TotalSimulationTime](abs(Pitch[t] - Ref_Pitch[t]) < tol)');
% What should the tolerance be? Let's try with 0.1
pitch_close_to_ref = set_params(pitch_close_to_ref,{'tol'},[0.1]);

% HINT: This is where you can add other STL formulas to try to falsify!

% Check the specification against the simulations we performed
pitch_results = B.CheckSpec(pitch_close_to_ref);
disp('Spec satisfaction results:')
disp(pitch_results);

% Negative results mean that the robustness is negative, i.e., the
%   specification is not fulfilled. 
% We can plot the robustness function of the specification for a more
%   detailed view.
B.PlotRobustSat(pitch_close_to_ref,... % Specification to plot
    inf,... % Depth of formula to plot
    [],... % Time instants where to evaluate
    1);

% Ok, so the tolerance of 0.1 was probably too tight ... how about 15?
pitch_close_to_ref2 = set_params(pitch_close_to_ref,{'tol'},[15]);

% Check specification again
pitch_results2 = B.CheckSpec(pitch_close_to_ref2);
disp('Spec satisfaction results, second try:')
disp(pitch_results2);

% Positive results mean that the specification is fulfilled for all three
%   scenarios!
% To verify the results, we can plot robustness for scenario 1 again:
B.PlotRobustSat(pitch_close_to_ref2,... % Specification to plot
    inf,... % Depth of formula to plot
    [],... % Time instants where to evaluate
    1);

%% Falsification
% So the specification holds for the given three scenarios. But will it
%   hold for all different scenarios?
% To tackle this question, we use FALSIFICATION. 

% First, we create a Breach falsification problem
falsif_pb = FalsificationProblem(B, pitch_close_to_ref2);
% Now, we let Breach attempt to solve it!
res = falsif_pb.solve();

if ~isempty(res)
    % Breach managed to falsify the specification
    B_False = falsif_pb.GetBrSet_False();
    % B_False is the same as B, but containing the falsified trajectory
    
    % Plot the robustness to visualize why the specification was not held
    B_False.PlotRobustSat(pitch_close_to_ref2);
end


%% Parameter synthesis
% We falsified the property that yaw stays within tol=0.1 of its reference.
% We might also ask: for what minimum value of tol is this property true?
% To answer this, we use parameter synthesis

% Create a parameter synthesis problem
synth_pb = ParamSynthProblem(B, pitch_close_to_ref2, {'tol'},  [0 1]);

% Let Breach solve it
synth_pb.solve();

% Store the best (=lowest) possible tolerance for which the specification
%   holds. 
tol_best  = synth_pb.x_best;
