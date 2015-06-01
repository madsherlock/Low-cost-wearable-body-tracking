function [center,normal,radius,sigmah,b_converged] = getCircle(X,b_plot,fignum)
%%
% X = m x 3
% b_plot        bool whether to plot animation or not
% fignum        plot figure number (new figure will be created)
% sigmah        Estimate of the standard deviation of the weighted 
%               residual errors.
% b_converged   bool indicating if algorithm converged to a circle.
%%
% Best fit plane (used as initial guess for center and normal to circle):
[x0, a, d, normd] = lsplane(X);
% Best fit circle:
[center, normal, radius, d, e, f, sigmah, b_converged, Vx0n, Van, urn, GNlog, a, R0, R] ...
    = ls3dcircle(X, x0, x0, 0.5*norm(X(1,:)-X(end,:)),...
    0.000001, 0.000001);

%%
if b_plot
    theta=0:0.01:2*pi;
    v=null(normal');
    points=repmat(center,1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
    
    figure(fignum);
    %axis([-1 1 -1 1 -1 1]);
    %axis(1.5*[min(u(:,1)) max(u(:,1)) min(u(:,2)) max(u(:,2)) min(u(:,3)) max(u(:,3))]);
    axis([min(X(:,1))-0.1 max(X(:,1))+0.1...
        min(X(:,2))-0.1 max(X(:,2))+0.1...
        min(X(:,3))-0.1 max(X(:,3))+0.1]);
    axis vis3d
    hold on
    %view(3)
    view(-163,22)
    grid on
    camproj orthographic
    rotate3d on
    plot3(points(1,:),points(2,:),points(3,:),'r-');
    ph6=plot3(X(1,1),X(1,2),X(1,3),'.b','MarkerFaceColor','b');
    for i=1:length(X(:,1))
        set(ph6,'XData',X(i,1),...
            'YData',X(i,2),...
            'ZData',X(i,3));
        drawnow
    end
end