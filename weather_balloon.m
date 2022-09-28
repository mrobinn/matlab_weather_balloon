
g = 9.81;                   %gravity
mass = 2;                   %we assume weight is 2 kg
vol = 4;                    %(4/3)*pi*(rad)^3;  %volume of balloon in m^3
rho = 1.293;                %air density at sea level
                            %area = 4*pi*rad^2;
D = 3.7;                    %0.5*0.9*rho*(area/2);
Fb = vol*rho*g;             %lifting force

%Euler-Cromer method

time=2;                     %time interval
dt=.01;                     %time-step, delta t

%decides how many rows will be in each matrix based on delta t
n = ceil(time/dt);

%create and set matrices to n rows and 1 column
y = zeros(n,1);             %y(position)
v = zeros(n,1);             %v(velocity)
a = zeros(n,1);             %a(acceleration)
t = zeros(n,1);             %t(time)

%set initial conditions
a(1) = Fb/mass-g-(D/mass)*abs(v(1))*v(1); %acceleration

%only acceleration is set, the other values are already 0

for i = 2:n
    v(i) = v(i-1) + a(i-1)*dt;                    %integral of acceleration
    y(i) = y(i-1) + v(i-1)*dt;                    %integral of velocity
    a(i) = Fb/mass-g-(D/mass)*abs(v(i))*v(i);     %acceleration with drag
    t(i) = t(i-1) + dt;                           %set time for each i
end

%plot displacement over time
subplot(3,1,1)
plot(t,y)
xlabel('Time (s)')
ylabel('Displacement (m)')

%plot velocity over time
subplot(3,1,2)
plot(t,v)
%set y-axis limit to 4, makes it easier to read graph
ylim([0 4])
xlabel('Time (s)')
ylabel('Velocity (m/s)')
%point when acceleration is close enough to 0 = terminal velocity
hold on
tv = plot(1.18,2.90001,'r*');
legend(tv,'Terminal velocity')
hold off
%plot acceleration over time
subplot(3,1,3)
plot(t,a,'LineWidth',1)
%set y-axis limit to 20, makes it easier to read graph
ylim([0 20])
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')