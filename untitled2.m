clc;
clear;
close all;

% =====================================
% SYMBOLIC VARIABLES
% =====================================

syms G1 G2 G3 G4 H1 H2

% =====================================
% CORRECT TRANSFER FUNCTION (FROM THEORY)
% =====================================

T_correct = (G1*(G2*G3 + G4)) / ...
            (1 + G2*H1 + G1*(G2*G3 + G4)*H2);

T_correct = simplify(T_correct);

% =====================================
% TAKE USER MANUAL ANSWER
% =====================================

disp('Enter your manual transfer function using:')
disp('G1, G2, G3, G4, H1, H2')
disp('Example:')
disp('(G1*(G2*G3 + G4)) / (1 + G2*H1 + G1*(G2*G3 + G4)*H2)')
disp(' ')

user_input = input('Enter your manual answer: ','s');

T_manual = str2sym(user_input);   % convert string to symbolic
T_manual = simplify(T_manual);

% =====================================
% COMPARISON
% =====================================

Difference = simplify(T_correct - T_manual);

disp(' ')
disp('======================================')

if Difference == 0
    disp('✅ RESULT: Your manual answer is CORRECT!');
else
    disp('❌ RESULT: Your manual answer is WRONG.');
    disp('Difference is:')
    pretty(Difference)
end

disp('======================================')
