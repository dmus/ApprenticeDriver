function [f, g, drdx, drdu, d2rdxdx, d2rdudu] = state_to_reward_features(s,u,A,B)
%STATE2REWARDFEATURES Summary of this function goes here
%   Detailed explanation goes here
    
    Df = 6; % 6 reward features
    Dx = 6;
    Du = 2;
    T = size(s,1);
    
    f = zeros(T,Df);

    g = zeros(T,Df,Du);
    drdx = zeros(T,Nf,Dx);
    drdu = zeros(T,Nf,Du);
    d2rdxdx = zeros(T,Nf,Dx,Dx);
    d2rdudu = zeros(T,Nf,Du,Du);
    
    % Prevent getting of road, distance from edge is rewarded
    f(:,1) = 1-s(:,5).^2;
    drdx(:,1,5) = -2*s(:,5);
    d2rdxdx(:,1,5,5) = -2;
    g(:,1,:) = permute(gradprod(A,B,permute(drdx(:,1,:),[1 3 2])),[1 3 2]);
    
    % High speed is rewarded
    f(:,2) = s(:,1);
    drdx(:,2,6) = 1;
    g(:,2,:) = permute(gradprod(A,B,permute(drdx(:,2,:),[1 3 2])),[1 3 2]);
    
    % Distance covered is rewarded    
    TRACKLENGTH = 6205.462891;
    covered = s(:,4)-s(:,12);
    special = covered < -100; % Special cases when new lap is started
    covered(special) = TRACKLENGTH-s(special,12) + s(special,4);
    f(:,3) = s(:,4);
    drdx(:,3,4) = 1;
    g(:,3,:) = permute(gradprod(A,B,permute(drdx(:,3,:),[1 3 2])),[1 3 2]);
    
    % Lateral speed
    f(:,4) = s(:,2);
    drdx(4,2) = 1;
    g(:,4,:) = permute(gradprod(A,B,permute(drdx(:,4,:),[1 3 2])),[1 3 2]);
    
    % Angle
    f(:,5) = s(:,6)^2;
    drdx(5,6) = 2*s(6);
    d2rdxdx(6,6) = 2;
    g(:,5,:) = permute(gradprod(A,B,permute(drdx(:,5,:),[1 3 2])),[1 3 2]);
    
    % Steering
    f(:,6) = u(:,2)^2;
    drdu(6,2) = 2*u(2);
    d2rdudu(2,2) = 2;
    g(:,6,:) = drdu(:,6,:);
end

