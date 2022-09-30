
g = 9.81;                   %gravity
mass = 2;                   %we assume weight is 2 kg
vol = 4;                    %we assume volume is 4 m^3
rho = 1.293;                %air density at sea level
D = 3.7;                    %0.5*0.9*rho*(area/2);
Fb = vol*rho*g;             %lifting force

%wind
%w0 = 7;
%q = 10;

%Euler-Cromer method
time=2;                     %time interval
dt=.01;                     %time-step, delta t

%decides how many rows will be in each matrix based on delta t
n = ceil(time/dt);

%create and set matrices to n rows and 1 column
y = zeros(n,1);             %y(position)
v = zeros(n,1);             %v(velocity)
a = zeros(n,1);             %a(acceleration)
%ax = zeros(n,1);            %acceleration on x
%w = zeros(n,1);             %wind
%vx = zeros(n,1);            %velocity on x

%calculate a,v,y with no drag(Nd)
aNd = zeros(n,1);           
vNd = zeros(n,1);
yNd = zeros(n,1);

t = zeros(n,1);             %t(time)

%set initial conditions
a(1) = Fb/mass-g-(D/mass)*abs(v(1))*v(1);   %set acceleration on y
%ax(1) = (D/mass)*abs(vx(1))*vx(1);          %set acceleration on x
%w(1) = 5;                                  %initial wind velocity m/s
aNd = Fb/mass;

for i = 1:n-1
    v(i+1) = v(i) + a(i)*dt;                        %integral of acceleration
    y(i+1) = y(i) + v(i)*dt;                        %integral of velocity
    a(i+1) = Fb/mass-g-(D/mass)*abs(v(i))*v(i);     %acceleration with drag
    %ax(i+1)= (D/mass)*abs(vx(i))*vx(i);            %acceleration on x-axis
    %w(i+1) = w0*(1-exp(-y(i)/q));                  %wind speed based on height
    %vx(i+1) = vx(i) + ax(i)*dt;                    %velocity on x-axis
    vNd(i+1) = vNd(i) + aNd(i)*dt;                        %integral of acceleration
    yNd(i+1) = yNd(i) + vNd(i)*dt;  
    aNd(i+1) = Fb/mass;

    t(i+1) = t(i) + dt;                             %set time for each i
end

%plot displacement over time
subplot(4,1,1)
plot(t,y,'LineWidth',1)
xlabel('Time (s)')
ylabel('Displacement (m)')
hold on
plot(t,yNd, 'Linewidth', 1)
hold off

%plot velocity over time
subplot(4,1,2)
plot(t,v,'LineWidth',1)
%set y-axis limit to 4, makes it easier to read graph
ylim([0 10])
xlabel('Time (s)')
ylabel('Velocity (m/s)')
%point when acceleration is close enough to 0 = terminal velocity
hold on
plot(t,vNd, 'Linewidth',1)
tv = plot(0.65,2.90001,'r*');
legend(tv,'Terminal velocity')
hold off

%plot acceleration over time
subplot(4,1,3)
plot(t,a,'LineWidth',1)
%set y-axis limit to 20, makes it easier to read graph
ylim([0 20])
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
hold on
tv = plot(0.65,0,'r*');
legend(tv,'Terminal velocity')
hold off

