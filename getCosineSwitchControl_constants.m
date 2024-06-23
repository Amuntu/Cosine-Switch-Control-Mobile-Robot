function [z, constantsAsStruct, constantsAsArray, n, w] = getCosineSwitchControl_constants(map, T)

%T = 9;
%{
map = [[x1 x2 ... xn]   ; % x-coordinates.
       [y1 y2 ... yn]   ; % y-coordinates.
       [q1 q2 ... qn] ] ; % theta-oreintations.
%}

%Switch row(2) witch represent Y-coor, with row(3) = (thetas) to ease the code's calculations:
temp = map(2,:);
map(2,:) = map(3,:);
map(3,:) = temp;

nPoints = size(map,2); % Number of desired Points.
n = nPoints*2 -1; % Number of switching Points (Intervals).

E = T/n;
w = 2*pi/E;

z = zeros(3,n+1); % Control variables.
z(:,1) = map(:,1).*[1; 0; 1] + tan(map(:,1)).*[0; 1; 0]; %z(1) = z(t0).
z = sym(z); % Convert to Symbols in order to solve equations to avoid any problem converting from num -> sym.
map(2,:) = tan(map(2,:));
t = (1:size(map,1))'; % Temp matrix to calculate (Odd & Even) equations without using nested loops.
c = sym('c', [n 1]); % Define n variables for 'c' constants [c1, c2, c3... cn].
for i=1:n
    if mod(i,2)==1
        z(:,i+1) = c(i)*E*(~mod(t,2)) + z(:,i);% Odd Intervals.
    else
        z(:,i+1) = c(i)*E*(mod(t,2)).*[1;0;z(2,i)] + z(:,i); % Even Intervals.
    end
end
equations = sym(zeros(size(map,1),(n-1)/2 ));
z(:,1) = [];
j = 1;
for i=1:size(z,2)
    if (i-1) == j*(2) % 2 = number of jumps to get to the point.
        equations(:,j) = z(:,i) == map(:,j+1);
        j = j + 1;
    end
end

constantsAsStruct = solve(equations(:),c);
constantsAsArray = double(struct2array(constantsAsStruct)');